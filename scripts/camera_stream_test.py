"""
시험용: AI 카메라 영상을 /camera/image_raw(320x240, 5 Hz)로 발행한다 (Pi에서 실행).

노트북에서 rqt_image_view로 본다 (docs/remote_view.md).
카메라는 한 프로그램만 쓸 수 있으므로 red_object_detector와 동시에 실행하지 않는다.
임무용 코드가 아니다.
"""
import time

import cv2
from picamera2 import Picamera2
import rclpy
from sensor_msgs.msg import Image

ROTATE_180 = True   # 책상 시험 때 카메라가 거꾸로 놓여 있었다 (2026-10-28)

rclpy.init()
node = rclpy.create_node('camera_stream_test')
pub = node.create_publisher(Image, '/camera/image_raw', 1)
cam = Picamera2()
cam.configure(cam.create_preview_configuration(main={'size': (640, 480), 'format': 'RGB888'}))
cam.start()
try:
    while rclpy.ok():
        frame = cv2.resize(cam.capture_array(), (320, 240))   # RGB888 = 메모리상 BGR 순서
        if ROTATE_180:
            frame = cv2.rotate(frame, cv2.ROTATE_180)
        msg = Image()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_link'
        msg.height, msg.width = frame.shape[:2]
        msg.encoding = 'bgr8'
        msg.step = msg.width * 3
        msg.data = frame.tobytes()
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.0)
        time.sleep(0.2)
except KeyboardInterrupt:
    pass
finally:
    cam.stop()
    node.destroy_node()
    rclpy.try_shutdown()
