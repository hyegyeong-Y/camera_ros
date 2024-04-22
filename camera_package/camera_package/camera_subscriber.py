import os
import cv2
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

class CameraSubscriber(Node):

    def __init__(self):
        super().__init__('camera_subscriber')
        self.cv_bridge = CvBridge()
        self.video_writer = None
        self.video_directory = './src/camera_package/video'  # 비디오를 저장할 디렉토리 경로

        # video 디렉토리가 없으면 생성
        if not os.path.exists(self.video_directory):
            os.makedirs(self.video_directory)

    def image_callback(self, msg):
        try:
            # ROS Image 메시지를 OpenCV 이미지로 변환
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # 비디오 녹화를 위해 비디오 라이터를 초기화
            if self.video_writer is None:
                fourcc = cv2.VideoWriter_fourcc(*'XVID')
                video_path = os.path.join(self.video_directory, 'recorded_video.avi')  # 저장할 비디오 파일 경로
                self.video_writer = cv2.VideoWriter(video_path, fourcc, 20.0, (cv_image.shape[1], cv_image.shape[0]))
            # 영상 프레임 저장
            self.video_writer.write(cv_image)
        except Exception as e:
            self.get_logger().error('Failed to process image: %s' % e)


    def start_recording(self):
        self.subscription = self.create_subscription(
            Image,
            '/camera',
            self.image_callback,
            10
        )

    def stop_recording(self):
        if self.video_writer:
            self.video_writer.release()
