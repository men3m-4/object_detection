     # Import necessary libraries

import rclpy                       # ROS2 client library for Python
from rclpy.node import Node        # ROS2 Node base class
from sensor_msgs.msg import Image  # Message type for image data
from std_msgs.msg import Bool      # Message type for boolean data
from cv_bridge import CvBridge     # Converts between ROS Image messages and OpenCV images
import cv2                         # OpenCV library for image capture and processing

# Define the Camera node class
class Camera(Node):
    def __init__(self):
        # Initialize the node with the name "cv_camera"
        super().__init__("cv_camera")
        
        # Create a publisher for image messages, publishing to the "camera/image" topic
        self.image_pub = self.create_publisher(Image, "camera/image", 1)
        
        # Create a publisher for status messages, publishing to the "camera/status" topic
        self.status_pub = self.create_publisher(Bool, "camera/status", 1)
        
        # Initialize CvBridge for converting between OpenCV and ROS Image messages
        self.cv_brdige = CvBridge()
        
        # Create a timer to call the `timer_clk` function at 15Hz (1/15 = 0.066 seconds)
        self.timer = self.create_timer(1/15, self.timer_clk)
        
        # Initialize video capture using OpenCV, capturing from the default camera (device 0)
        self.cap = cv2.VideoCapture(0)

    # Timer callback function that runs at 15Hz
    def timer_clk(self):
        # Capture a frame from the camera
        ret, mat = self.cap.read()

        # Check if the frame was captured successfully
        if ret:
            # Convert the captured OpenCV image (mat) to a ROS Image message
            image_msg = self.cv_brdige.cv2_to_imgmsg(mat)
            
            # Set the frame ID for the Image message header to "camera"
            image_msg.header.frame_id = "camera"
            
            # Publish the image to the "camera/image" topic
            self.image_pub.publish(image_msg)

        # Create a Bool message to publish the status of the camera capture
        status_msg = Bool()
        
        # Set the status message data to True (if the frame was captured) or False (if not)
        status_msg.data = ret
        
        # Publish the status message to the "camera/status" topic
        self.status_pub.publish(status_msg)

# Main function to start the ROS2 node
def main(args=None):
    # Initialize the ROS2 communication
    rclpy.init(args=args)
    
    # Create an instance of the Camera node
    camera = Camera()
    
    # Spin the node so it continues running and processing callbacks
    rclpy.spin(camera)
    
    # Clean up and destroy the node when shutting down
    camera.destroy_node()
    
    # Shut down ROS2 communication
    rclpy.shutdown()

# Entry point for the script
if __name__ == "__main__":
    # Run the main function
    main()
