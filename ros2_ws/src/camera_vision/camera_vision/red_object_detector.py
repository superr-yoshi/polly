"""빨간 대상(큐브, 요구조자)을 찾아 /vision/target 으로 보내는 노드.

카메라: Raspberry Pi AI Camera (Sony IMX500, CSI), Picamera2 로 사용.
메시지: poli_interfaces/msg/TargetDetection (docs/interfaces.md 약속).
"""

import cv2
from libcamera import Transform
import numpy as np
from picamera2 import Picamera2
from poli_interfaces.msg import TargetDetection
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

# ---------------------------------------------------------------------------
# 카메라 설정
# ---------------------------------------------------------------------------
# area 값은 이 해상도 기준 픽셀 수. 해상도를 바꾸면 GRAB_AREA 도 다시 재야 한다.
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
CAMERA_FPS = 30.0

# 발행 주기. 약속은 10Hz 이상. 실제 값은 ros2 topic hz 로 확인한다.
PUBLISH_HZ = 20.0

# TODO_MEASURE: 빨간 물체를 로봇 기준 왼쪽에 두고 x_offset 이 음수인지 확인.
# 양수가 나오면(화면이 좌우 반전) True 로 바꾼다.
CAMERA_HFLIP = False
# TODO_MEASURE: 카메라가 거꾸로 달려 있으면 CAMERA_HFLIP, CAMERA_VFLIP 둘 다 True.
CAMERA_VFLIP = False

# ---------------------------------------------------------------------------
# 빨간색 판단 기준 (OpenCV HSV: H 0~179, S 0~255, V 0~255)
# ---------------------------------------------------------------------------
# 빨간색은 H 가 0 근처와 179 근처 두 곳에 걸쳐 있다.
RED_HUE_LOW_MAX = 10  # TODO_MEASURE: H 0 ~ 이 값까지 빨강
RED_HUE_HIGH_MIN = 170  # TODO_MEASURE: 이 값 ~ H 179 까지 빨강
# 채도(S): 색이 얼마나 진한지. 밝기와 상관없는 비율 값이라 조명에 강하다.
# 피부, 분홍, 나무색처럼 옅은 붉은색을 걸러내는 역할.
RED_SAT_MIN = 120  # TODO_MEASURE
# 밝기(V): 조명이 어두워도 잡히도록 낮게 둔다. 너무 어두운 노이즈만 버린다.
RED_VAL_MIN = 50  # TODO_MEASURE

# 이 넓이(픽셀)보다 작은 빨간 덩어리는 노이즈로 보고 무시한다.
MIN_AREA = 300.0  # TODO_MEASURE: 가장 먼 인식 거리에서 대상의 area 보다 작게

# 노이즈 제거용 필터 크기 (홀수)
BLUR_SIZE = 5
MORPH_KERNEL_SIZE = 5


def find_red_target(frame_bgr):
    """BGR 이미지에서 가장 큰 빨간 덩어리를 찾는다.

    반환값: (detected, x_offset, area, contour, mask)
    x_offset 은 화면 가운데 0, 왼쪽 끝 -1.0, 오른쪽 끝 +1.0.
    못 찾으면 detected=False, x_offset=0.0, area=0.0.
    """
    height, width = frame_bgr.shape[:2]
    center_x = width / 2.0

    blurred = cv2.GaussianBlur(frame_bgr, (BLUR_SIZE, BLUR_SIZE), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    lower1 = np.array([0, RED_SAT_MIN, RED_VAL_MIN], dtype=np.uint8)
    upper1 = np.array([RED_HUE_LOW_MAX, 255, 255], dtype=np.uint8)
    lower2 = np.array([RED_HUE_HIGH_MIN, RED_SAT_MIN, RED_VAL_MIN], dtype=np.uint8)
    upper2 = np.array([179, 255, 255], dtype=np.uint8)
    mask = cv2.bitwise_or(
        cv2.inRange(hsv, lower1, upper1),
        cv2.inRange(hsv, lower2, upper2),
    )

    kernel = np.ones((MORPH_KERNEL_SIZE, MORPH_KERNEL_SIZE), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(
        mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return False, 0.0, 0.0, None, mask

    largest = max(contours, key=cv2.contourArea)
    area = float(cv2.contourArea(largest))
    if area < MIN_AREA:
        return False, 0.0, 0.0, None, mask

    moments = cv2.moments(largest)
    if moments['m00'] == 0:
        return False, 0.0, 0.0, None, mask

    cx = moments['m10'] / moments['m00']
    x_offset = (cx - center_x) / center_x
    x_offset = max(-1.0, min(1.0, x_offset))
    return True, float(x_offset), area, largest, mask


class RedTargetDetector(Node):
    """카메라 영상에서 빨간 대상을 찾아 TargetDetection 을 발행하는 노드."""

    def __init__(self):
        """카메라를 켜고 발행 타이머를 만든다."""
        super().__init__('red_target_detector')

        # 모니터가 있을 때만 켠다. 대회(모니터 없음)에서는 반드시 False.
        # 실행 예: ros2 run camera_vision red_object_detector
        #          --ros-args -p show_debug:=true
        self.declare_parameter('show_debug', False)
        self.show_debug = self.get_parameter('show_debug').value

        self.publisher = self.create_publisher(
            TargetDetection, '/vision/target', 10)

        self.picam2 = Picamera2()
        config = self.picam2.create_video_configuration(
            # 'RGB888' 은 이름과 달리 메모리 순서가 B, G, R 이라
            # OpenCV 의 BGR 과 그대로 맞는다. (바꾸면 빨강이 파랑으로 보임)
            main={'size': (FRAME_WIDTH, FRAME_HEIGHT), 'format': 'RGB888'},
            transform=Transform(hflip=CAMERA_HFLIP, vflip=CAMERA_VFLIP),
            controls={'FrameRate': CAMERA_FPS},
        )
        self.picam2.configure(config)
        # 자동 노출, 자동 화이트밸런스는 기본으로 켜져 있다.
        # 조명 밝기/색이 바뀌어도 카메라가 알아서 맞춰 주므로 끄지 않는다.
        self.picam2.start()

        self.timer = self.create_timer(1.0 / PUBLISH_HZ, self.process_frame)
        self.get_logger().info(
            f'red_target_detector 시작: {FRAME_WIDTH}x{FRAME_HEIGHT}, '
            f'{PUBLISH_HZ:.0f}Hz, /vision/target')

    def publish(self, detected, x_offset, area):
        """TargetDetection 메시지를 발행한다."""
        msg = TargetDetection()
        msg.detected = bool(detected)
        msg.x_offset = float(x_offset)
        msg.area = float(area)
        self.publisher.publish(msg)

    def process_frame(self):
        """프레임 한 장을 처리하고 결과를 항상 발행한다."""
        try:
            frame = self.picam2.capture_array('main')
        except Exception as error:
            # 약속 1: 못 읽어도 detected=False 를 계속 보낸다.
            self.get_logger().warn(
                f'카메라 프레임을 읽지 못함: {error}',
                throttle_duration_sec=1.0)
            self.publish(False, 0.0, 0.0)
            return

        detected, x_offset, area, contour, mask = find_red_target(frame)
        self.publish(detected, x_offset, area)

        # 매 프레임 찍으면 터미널이 넘치므로 1초에 한 번만.
        self.get_logger().info(
            f'detected={detected}, x_offset={x_offset:+.3f}, '
            f'area={area:.0f}',
            throttle_duration_sec=1.0)

        if self.show_debug:
            self.show_debug_windows(frame, mask, contour)

    def show_debug_windows(self, frame, mask, contour):
        """확인용 화면을 띄운다 (모니터가 있을 때만)."""
        height, width = frame.shape[:2]
        view = frame.copy()
        cv2.line(view, (width // 2, 0), (width // 2, height),
                 (255, 255, 0), 1)
        if contour is not None:
            cv2.drawContours(view, [contour], -1, (0, 255, 0), 2)
        cv2.imshow('red_target_detector', view)
        cv2.imshow('red_mask', mask)
        cv2.waitKey(1)

    def destroy_node(self):
        """카메라와 창을 정리한다."""
        self.picam2.stop()
        self.picam2.close()
        if self.show_debug:
            cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    """노드를 실행한다."""
    rclpy.init(args=args)
    node = RedTargetDetector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()