#!/usr/bin/env python3
import rclpy
import mavros
from mavros.system import STATE_QOS
from mavros.base import SENSOR_QOS
from nav_msgs.msg import Odometry
from rclpy.node import Node
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.srv import CommandTOL
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import time
from pymavlink import mavutil

class Controller(Node):
    def __init__(self):
        super().__init__("quad_controller")
        # self.current_state = State()
        # self.current_pose = PoseStamped()
        
        self.odom_sub = self.create_subscription(mavros.local_position.Odometry,
                                                'mavros/local_position/odom', 
                                                self.pose_cb, 
                                                qos_profile=SENSOR_QOS)
        self.state_sub = self.create_subscription(
            mavros.system.State,'mavros/state', 
            self.state_cb, qos_profile=STATE_QOS)
        
        
    
        self.current_pose = 1
    
    def state_cb(self,state:State):
        self.current_state = state
        
    def pose_cb(self,odom):
        self.current_pose = odom        
        self.current_pose = odom.pose.pose.position.x
            


def main(args=None):
    rclpy.init(args=args) 
    master = mavutil.mavlink_connection('udp:127.0.0.1:14550')

    master.wait_heartbeat()
    print('heartbeat recieved')
    
    quad = Controller()
    rclpy.spin_once(quad, timeout_sec=0.01)  
    
    while rclpy.ok():        
        print(quad.current_pose)
        #print(quad.current_pose)
        
    # rclpy.spin(quad)
        
    # rclpy.shutdown()


if __name__ =='__main__':
    main()