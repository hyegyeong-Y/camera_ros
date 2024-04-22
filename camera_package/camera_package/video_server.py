import os
import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger

class VideoServer(Node):
    def __init__(self):
        super().__init__('video_server')
        self.start_recording_service = self.create_service(Trigger, 'start_recording', self.start_recording_callback)
        self.stop_recording_service = self.create_service(Trigger, 'stop_recording', self.stop_recording_callback)
        self.is_recording = False
        self.video_writer = None
        self.cv_bridge = CvBridge()
        self.image_subscription = self.create_subscription(Image, '/camera', self.image_callback, 10)

        # Declare storage path parameter
        self.declare_parameter('storage_path', './src/camera_package/video/')  # Default value './capture/'
        self.video_directory = self.get_parameter('storage_path').value
        self.video_filename = 'recorded_video.avi'
        self.video_count = 0  # Initialize the video count

    def start_recording_callback(self, request, response):
        if not self.is_recording:
            self.get_logger().info('Start recording request received')
            self.is_recording = True
            # Increment the video count
            self.video_count += 1
            self.video_filename = f'recorded_video_{self.video_count}.avi'  # Update the filename
            response.success = True
            response.message = 'Recording started successfully'
        else:
            response.success = False
            response.message = 'Already recording'
        return response

    def stop_recording_callback(self, request, response):
        if self.is_recording:
            self.get_logger().info('Stop recording request received')
            self.is_recording = False
            if self.video_writer is not None:
                self.video_writer.release()
                self.video_writer = None
            response.success = True
            response.message = 'Recording stopped successfully'
        else:
            response.success = False
            response.message = 'Not recording'
        return response
    
    def image_callback(self, msg):
        if self.is_recording:
            try:
                cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                if self.video_writer is None:
                    fourcc = cv2.VideoWriter_fourcc(*'XVID')
                    video_path = os.path.join(self.video_directory, self.video_filename)
                    self.get_logger().info(f'Video will be saved to: {video_path}')
                    self.video_writer = cv2.VideoWriter(video_path, fourcc, 20.0, (cv_image.shape[1], cv_image.shape[0]))
                self.video_writer.write(cv_image)
            except Exception as e:
                self.get_logger().error(f'Error processing image: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    server = VideoServer()
    rclpy.spin(server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
