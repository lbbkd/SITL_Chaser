#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.srv import CommandTOL
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import time

class Controller(Node):
    def __init__(self):
        super().__init__("quad_controller")
        self.current_state = State()
        self.current_pose = PoseStamped()
        self.create_subscription(
            'mavros/state', State, self.state_cb, 10)
        self.create_subscription(
            'mavros/local_position/pose', PoseStamped, self.pose_cb, 10)
    
    def state_cb(self,state:State):
        self.current_state = state
        self.get_logger().info(str(state))
        
        
    def pose_cb(self,pose:PoseStamped):
        self.current_pose = pose
        self.get_logger().info(str(pose))
            


def main(args=None):
    rclpy.init(args=args)
    quad = Controller()
    rclpy.spin(quad)
            
    rclpy.shutdown()
        