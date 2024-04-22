import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import cv2

class VideoClient(Node):

    def __init__(self):
        super().__init__('video_client')
        self.start_recording_client = self.create_client(Trigger, 'start_recording')
        while not self.start_recording_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Start recording service not available, waiting again...')
        self.stop_recording_client = self.create_client(Trigger, 'stop_recording')
        while not self.stop_recording_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Stop recording service not available, waiting again...')

    def start_recording(self):
        request = Trigger.Request()
        future = self.start_recording_client.call_async(request)
        future.add_done_callback(self.start_recording_callback)

    def start_recording_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Recording started successfully')
            else:
                self.get_logger().error('Failed to start recording: %s' % response.message)
        except Exception as e:
            self.get_logger().error('Service call failed: %r' % e)

    def stop_recording(self):
        request = Trigger.Request()
        future = self.stop_recording_client.call_async(request)
        future.add_done_callback(self.stop_recording_callback)

    def stop_recording_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Recording stopped successfully')
            else:
                self.get_logger().error('Failed to stop recording: %s' % response.message)
        except Exception as e:
            self.get_logger().error('Service call failed: %r' % e)

def main(args=None):
    rclpy.init(args=args)
    client = VideoClient()
    client.start_recording()
    client.stop_recording()
    rclpy.spin(client)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
