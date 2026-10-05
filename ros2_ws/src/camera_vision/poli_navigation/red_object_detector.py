import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Vector3  # [detected, x_offset, area] 전달용
import cv2
import numpy as np
import json

class RedObjectDetector(Node):
    def __init__(self):
        super().__init__('red_object_detector')

        # 1. ROS 2 Publisher 생성
        # B가 사용자에게 전달할 데이터 (JSON 문자열 형태 토픽)
        self.publisher_string = self.create_publisher(String, '/target_info_str', 10)
        # 수치 처리용 Vector3 토픽 (x: detected(1 or 0), y: x_offset, z: area)
        self.publisher_vec = self.create_publisher(Vector3, '/target_info', 10)

        # 2. 카메라 (U20CAM-720P) 연결 (기본 0번 디바이스)
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("카메라를 열 수 없습니다!")
            return

        # 카메라 해상도 설정 (720P = 1280x720)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

        # 3. 주기적 실행 타이머 (30 FPS 기준 약 0.033초마다 실행)
        self.timer = self.create_timer(0.033, self.process_frame)
        self.get_logger().info("Red Object Detector Node Started!")

    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("카메라 프레임을 읽을 수 없습니다.")
            return

        frame_height, frame_width, _ = frame.shape
        center_x = frame_width / 2.0  # 화면 중심 X좌표 (640)

        # Step 1: HSV 색상 공간 변환
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Step 2: 빨간색 영역 검출 (HSV에서 빨간색은 0 부근과 180 부근 2개 범위)
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 100, 100])
        upper_red2 = np.array([180, 255, 255])

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)

        # Step 3: Noise 제거 (MOP_OPEN: 침식 후 팽창)
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # Step 4: Contour(윤곽선) 검출
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        detected = False
        x_offset = 0.0
        area = 0.0

        if contours:
            # Step 5: 가장 큰 구조대상(Contour) 선택
            largest_contour = max(contours, key=cv2.contourArea)
            area = float(cv2.contourArea(largest_contour))

            # 노이즈 방지를 위해 최소 크기 설정 (예: 500 이상일 때만 인식)
            if area > 500:
                detected = True

                # Step 6: 중심좌표 계산 (Moments)
                M = cv2.moments(largest_contour)
                if M['m00'] != 0:
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])

                    # Step 7: x_offset 계산 (-1.0 ~ +1.0 정규화)
                    # 왼쪽: -1.0 ~ 0.0 / 오른쪽: 0.0 ~ +1.0
                    x_offset = round((cx - center_x) / center_x, 3)

                    # 화면 시각화용 드로잉
                    cv2.drawContours(frame, [largest_contour], -1, (0, 255, 0), 2)
                    cv2.circle(frame, (cx, cy), 7, (255, 0, 0), -1)
                    cv2.line(frame, (int(center_x), 0), (int(center_x), frame_height), (255, 255, 0), 1)

        # Step 8: ROS 2 결과 출력 (Publish)
        # 1) Vector3 형태 (x=detected, y=x_offset, z=area)
        vec_msg = Vector3()
        vec_msg.x = 1.0 if detected else 0.0
        vec_msg.y = float(x_offset)
        vec_msg.z = float(area)
        self.publisher_vec.publish(vec_msg)

        # 2) JSON 문자열 형태
        data_dict = {
            "detected": detected,
            "x_offset": x_offset,
            "area": area
        }
        str_msg = String()
        str_msg.data = json.dumps(data_dict)
        self.publisher_string.publish(str_msg)

        # 터미널 로깅
        self.get_logger().info(f"detected = {detected}, x_offset = {x_offset}, area = {int(area)}")

        # 화면 출력 (디버깅용)
        cv2.imshow("Red Object Detection", frame)
        cv2.imshow("Mask", mask)
        cv2.waitKey(1)

    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = RedObjectDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()