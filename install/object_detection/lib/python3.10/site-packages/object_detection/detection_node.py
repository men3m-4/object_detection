import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cvlib as cv
from cvlib.object_detection import draw_bbox
import time
from rclpy.qos import QoSProfile, qos_profile_sensor_data

class DetectorNode(Node):
    def __init__(self, node_name="detector_node"):
        super().__init__(node_name)

        # QoS profile with depth of 10
        qos_profile = QoSProfile(depth=10)

        # Create the subscription with the correct QoS
        self.subscriber = self.create_subscription(
            Image, 
            "/camera/image", 
            self.callback, 
            qos_profile
        )
        
        # Create the publisher with the correct QoS
        self.pub = self.create_publisher(
            Image, 
            "/object_detection/output", 
            qos_profile
        )

        # OpenCV bridge to convert between ROS and OpenCV images
        self.cv_bridge = CvBridge()

    def callback(self, msg):
        time_now = time.time()

        # Convert ROS image to OpenCV image
        img_opencv = self.cv_bridge.imgmsg_to_cv2(msg)

        # Perform object detection using cvlib
        bbox, label, conf = cv.detect_common_objects(
            img_opencv, enable_gpu=False
        )

        # Draw bounding boxes around detected objects
        output_image = draw_bbox(img_opencv, bbox, label, conf)

        # Convert OpenCV image back to ROS image
        img_msg = self.cv_bridge.cv2_to_imgmsg(output_image)
        img_msg.header = msg.header

        # Publish the output image
        self.pub.publish(img_msg)

        # Log the time taken for detection
        self.get_logger().info("Detection took {}s ".format(time.time() - time_now))

def main(args=None):
    rclpy.init(args=args)
    detector = DetectorNode()
    rclpy.spin(detector)
    detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

