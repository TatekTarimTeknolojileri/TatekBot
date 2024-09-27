import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        
        # Subscribers to the three topics
        self.graphic_sub = self.create_subscription(Image, 'vs_nav/graphic', self.graphic_callback, 10)
        self.mask_sub = self.create_subscription(Image, 'vs_nav/mask', self.mask_callback, 10)
        self.exg_sub = self.create_subscription(Image, 'vs_nav/ExG', self.exg_callback, 10)
        
        # To convert ROS Image messages to OpenCV format
        self.bridge = CvBridge()

        # Placeholders for the images
        self.graphic_image = None
        self.mask_image = None
        self.exg_image = None
        
        self.timer = self.create_timer(0.1, self.show_images)  # Timer to update the display every 100ms

    def graphic_callback(self, msg):
        # Convert ROS Image message to OpenCV format (assuming it's a color image in BGR format)
        try:
            self.graphic_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except :
            self.get_logger().error(f"Error converting graphic image: ")

    def mask_callback(self, msg):
        # Convert ROS Image message to OpenCV format (assuming it's a grayscale image)
        try:
            self.mask_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        except:
            self.get_logger().error(f"Error converting mask image: ")

    def exg_callback(self, msg):
        # Convert ROS Image message to OpenCV format (assuming it's a color image in BGR format)
        try:
            self.exg_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        except:
            print("exg convertion error")

    def show_images(self):
        # Display the images in separate windows if available
        if self.graphic_image is not None:
            cv2.imshow('Graphic Image', self.graphic_image)
        
        if self.mask_image is not None:
            cv2.imshow('Mask Image', self.mask_image)
        
        if self.exg_image is not None:
            cv2.imshow('ExG Image', self.exg_image)
        
        # Wait for 1 ms to allow window events
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    
    # Create the image subscriber node
    image_subscriber = ImageSubscriber()

    try:
        # Spin the node so the callbacks can get called
        rclpy.spin(image_subscriber)
    except KeyboardInterrupt:
        pass
    
    # Clean up
    image_subscriber.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
