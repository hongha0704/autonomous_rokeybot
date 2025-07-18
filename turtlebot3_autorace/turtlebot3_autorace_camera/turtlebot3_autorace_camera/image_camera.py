#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, CameraInfo
import cv2

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_raw_publisher_with_info')

        # 퍼블리셔 설정
        self.image_pub = self.create_publisher(CompressedImage, '/camera/image_raw/compressed', 10)
        self.info_pub = self.create_publisher(CameraInfo, '/camera/camera_info', 10)

        # 카메라 초기화
        self.cap = cv2.VideoCapture(2, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        if not self.cap.isOpened():
            self.get_logger().error("❌ 카메라 열기 실패 (/dev/video0)")
        else:
            self.get_logger().info("✅ 카메라 열림 성공 (/dev/video0)")

        # 타이머 설정
        self.timer = self.create_timer(0.03, self.publish)

        # 카메라 보정 정보 설정 (예시값)
        self.camera_info = CameraInfo()
        self.camera_info.width = 640
        self.camera_info.height = 480
        self.camera_info.distortion_model = 'plumb_bob'
        self.camera_info.k = [600.0, 0.0, 320.0,
                              0.0, 600.0, 240.0,
                              0.0, 0.0, 1.0]
        self.camera_info.p = [600.0, 0.0, 320.0, 0.0,
                              0.0, 600.0, 240.0, 0.0,
                              0.0, 0.0, 1.0, 0.0]
        self.camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]

        # compensation.yaml 값 참고용 로그
        self.clip_hist_percent = 1.0
        self.get_logger().info(f'[compensation] clip_hist_percent: {self.clip_hist_percent}')

        # projection.yaml 값 참고용 로그
        self.top_x = 104
        self.top_y = 0
        self.bottom_x = 217
        self.bottom_y = 120
        self.get_logger().info(f'[projection] top_x: {self.top_x}, top_y: {self.top_y}, '
                               f'bottom_x: {self.bottom_x}, bottom_y: {self.bottom_y}')

    def publish(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("⚠️ 프레임 캡처 실패")
            return

        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 80]
        success, encoded_image = cv2.imencode('.jpg', frame, encode_param)

        if not success:
            self.get_logger().warn("⚠️ 이미지 압축 실패")
            return

        # CompressedImage 메시지 생성
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera"
        msg.format = "jpeg"
        msg.data = encoded_image.tobytes()
        self.image_pub.publish(msg)

        # CameraInfo 발행
        self.camera_info.header = msg.header
        self.info_pub.publish(self.camera_info)

    def destroy_node(self):
        if self.cap.isOpened():
            self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
