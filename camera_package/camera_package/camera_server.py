import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from camera_msgs.srv import CaptureCameraImage

class CameraServer(Node):
    def __init__(self):
        super().__init__('camera_server')
        self.cv_bridge = CvBridge()
        self.image_captured = False
        self.captured_image = None
        self.image_width = 0
        self.image_height = 0

        # Declare storage path parameter
        self.declare_parameter('storage_path', './src/camera_package/capture/')  # Default value './capture/'
        self.storage_path = self.get_parameter('storage_path').value

        # Create a service server to capture image
        self.capture_image_service = self.create_service(
            CaptureCameraImage,
            'capture_camera_image',
            self.capture_image_callback
        )

        # Create a subscriber to subscribe to the camera topic
        self.image_subscription = self.create_subscription(
            Image,
            '/camera',
            self.image_callback,
            10
        )

    def capture_image_callback(self, request, response):
        if not self.image_captured:
            try:
                # Find the highest image number in the storage directory
                image_files = [f for f in os.listdir(self.storage_path) if os.path.isfile(os.path.join(self.storage_path, f))]
                max_num = max([int(f.split('_')[0]) for f in image_files if f.split('_')[0].isdigit()] + [0])

                # Save the captured image to a file with the next available number
                file_name = f'{max_num + 1}_captured_image.jpg'
                file_path = os.path.join(self.storage_path, file_name)

                # Resize the captured image to the requested width and height
                captured_image_resized = cv2.resize(self.captured_image, (request.width, request.height))

                cv2.imwrite(file_path, captured_image_resized)

                response.success = True
                response.message = 'Image captured successfully'
                response.file_path = file_path

                # 이미지 캡처 상태 업데이트
                self.image_captured = True

            except Exception as e:
                response.success = False
                response.message = 'Failed to capture image: {}'.format(str(e))

        else:
            response.success = False
            response.message = 'Image has already been captured'

        # 이미지를 캡처한 후에 image_captured 플래그를 False로 설정하여 다음 이미지를 캡처할 수 있도록 합니다.
        self.image_captured = False

        return response


    def image_callback(self, msg):
        try:
            # Convert ROS image message to OpenCV image
            captured_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Check if the captured image is empty
            if captured_image is None:
                self.get_logger().error('Received an empty image')
                return

            # Get image width and height
            self.image_width = msg.width
            self.image_height = msg.height

            # Update captured image
            self.captured_image = captured_image

        except Exception as e:
            self.get_logger().error('Failed to process image: %s' % str(e))



def main():
    rclpy.init()
    camera_server = CameraServer()

    # Create a subscriber to subscribe to the camera topic
    camera_server.image_subscription = camera_server.create_subscription(
        Image,
        '/camera',
        camera_server.image_callback,
        10
    )

    try:
        rclpy.spin(camera_server)
    except KeyboardInterrupt:
        pass
    finally:
        camera_server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
