import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import IntegerRange
from rcl_interfaces.msg import FloatingPointRange

class CannyWithCam(Node):
    def __init__(self):
        super().__init__('canny_with_cam')

        # Define parameter descriptors to specify ranges
        bright_range = IntegerRange(from_value=0, to_value=10)
        contrast_range = FloatingPointRange(from_value=0.0, to_value=2.0)
        blur_range = IntegerRange(from_value=1, to_value=10)
        thrs1_range = IntegerRange(from_value=0, to_value=1000)
        thrs2_range = IntegerRange(from_value=0, to_value=1000)

        # Declare parameters with their descriptors
        self.param_descriptor_bright = ParameterDescriptor(
            description='bright',
            integer_range=[bright_range]
        )
        self.param_descriptor_contrast = ParameterDescriptor(
            description='contrast',
            floating_point_range=[contrast_range]
        )
        self.param_descriptor_blur = ParameterDescriptor(
            description='blur',
            integer_range=[blur_range]
        )
        self.param_descriptor_thrs1 = ParameterDescriptor(
            description='thrs1',
            integer_range=[thrs1_range]
        )
        self.param_descriptor_thrs2 = ParameterDescriptor(
            description='thrs2',
            integer_range=[thrs2_range]
        )

        # Get parameter values
        self.declare_parameter('bright', 1, self.param_descriptor_bright)
        self.declare_parameter('contrast', 1.1, self.param_descriptor_contrast)
        self.declare_parameter('blur', 5, self.param_descriptor_blur)
        self.declare_parameter('thrs1', 100, self.param_descriptor_thrs1)
        self.declare_parameter('thrs2', 200, self.param_descriptor_thrs2)

        # Initialize parameters
        self.bright = self.get_parameter('bright').value
        self.contrast = self.get_parameter('contrast').value
        self.blur = self.get_parameter('blur').value
        self.thrs1 = self.get_parameter('thrs1').value
        self.thrs2 = self.get_parameter('thrs2').value

        self.subscription = self.create_subscription(
            Image,
            '/camera',
            self.image_callback,
            10)
        self.publisher = self.create_publisher(
            Image,
            '/canny_image',
            10)
        self.cv_bridge = CvBridge()

    def image_callback(self, msg):
        cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Apply brightness and contrast adjustments if needed
        adjusted_image = cv2.convertScaleAbs(cv_image, alpha=self.contrast, beta=self.bright)

        # Convert image to grayscale
        gray_image = cv2.cvtColor(adjusted_image, cv2.COLOR_BGR2GRAY)

        # Apply Canny edge detection algorithm
        edges = cv2.Canny(gray_image, self.thrs1, self.thrs2)

        # Convert edges to BGR for overlay
        edges_bgr = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)

        # Overlay edges onto original image
        overlaid_image = cv2.addWeighted(cv_image, 0.7, edges_bgr, 0.3, 0)

        # Convert the overlaid image to ROS Image message
        overlaid_msg = self.cv_bridge.cv2_to_imgmsg(overlaid_image, encoding='bgr8')

        # Publish the overlaid image
        self.publisher.publish(overlaid_msg)



def main(args=None):
    rclpy.init(args=args)
    node = CannyWithCam()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()