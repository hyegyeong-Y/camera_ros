import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImgPublisher(Node):
    def __init__(self):
        super().__init__('img_publisher')
        self.publisher = self.create_publisher(
            Image,
            '/camera',
            10
        )
        self.cv_bridge = CvBridge()
        self.captured_image = None

        # Create timer to periodically publish image
        self.timer = self.create_timer(0.01, self.publish_image)

    def publish_image(self):
        if self.captured_image is not None:
            try:
                # Convert the captured image to ROS Image message
                img_msg = self.cv_bridge.cv2_to_imgmsg(self.captured_image, encoding="bgr8")

                # Publish the image message
                self.publisher.publish(img_msg)
            except Exception as e:
                self.get_logger().error(f"Failed to publish image: {str(e)}")

def main():
    rclpy.init()
    img_publisher = ImgPublisher()
    try:
        rclpy.spin(img_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        img_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
