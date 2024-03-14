import numpy as np
import rclpy
import mavros
from mavros.system import STATE_QOS
from mavros.base import SENSOR_QOS
from rclpy.node import Node
import time
from pymavlink import mavutil
from mavros_msgs.msg import State
from to_euler import euler_from_quaternion

class Chaser(Node):
    def __init__(self):
        super().__init__("quad_chaser")
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
        self.current_x = odom.pose.pose.position.x
        self.current_y = odom.pose.pose.position.y
        self.current_z = odom.pose.pose.position.z
        
        qx = odom.pose.pose.orientation.x
        qy = odom.pose.pose.orientation.y
        qz = odom.pose.pose.orientation.z
        qw = odom.pose.pose.orientation.w
        self.roll,self.pitch,self.yaw = euler_from_quaternion(qx,qy,qz,qw)
        
    
    def simple_pronav(self,heading,dt,nav_gain):
        d_los = (heading - previous_heading)/dt
        previous_heading = heading
        angular_vel = nav_gain*d_los
        return angular_vel
    