import rclpy
import rclpy.node import  Node 
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cvlip as cv
from cvlip.object_detection import draw_bbox


class objectdetection(Node): 
    def __init__("object_detection"):
        super().__init__("object_detection")
        self.sub_ = self.create_subscription(Image,"camera/iamge",self.clk_)
        
