#!/usr/bin/env python

import rospy
from mavros_msgs.srv import SetMode
from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Odometry

class FollowMode():
    def __init__(self):
        # Parameters
        self.init_x = rospy.get_param("~init_x", None)
        self.init_y = rospy.get_param("~init_y", None)
        
        if self.init_x is None:
            self.init_x = input("Enter Initial X position: ")
        if self.init_y is None:
            self.init_y = input("Enter Initial Y position: ")
        
        self.ugv_pose_topic = rospy.get_param("~ugv_pose_topic", "/rovid/ekf_ugv")    
        # Publishers
        self.pub_goal_pose = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=1)
        
        # Subscribers (will be set in activate())
        self.ugv_pose_sub = None
        
        #gps offset
        self.gps_offset_sub = rospy.Subscriber("gps_offset", PoseStamped, self.gps_offset_callback)
        self.gps_offset = [0,0,0,0]
        
    def activate(self, altitude):
        '''
            Activates subscription to topics
        '''
        if self.ugv_pose_sub is None:
            self.ugv_pose_sub = rospy.Subscriber(self.ugv_pose_topic, Odometry, self.ugv_pose_callback)
        self.goal_z = altitude
        
    def deactivate(self):
        '''
            De-activates subscription to topics
        '''
        if self.ugv_pose_sub is not None:
            self.ugv_pose_sub.unregister()
            self.ugv_pose_sub = None
        
    def gps_offset_callback(self, msg):
        self.gps_offset = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, 0]
        print("gps offset added")
          
    def ugv_pose_callback(self, msg):
        goal_pose = PoseStamped()
        goal_pose.header.stamp = rospy.Time.now()
        goal_pose.pose = msg.pose.pose
        
        goal_pose.pose.position.x -= self.init_x
        goal_pose.pose.position.y -= self.init_y
        goal_pose.pose.position.z = self.goal_z
        
        goal_pose.pose.position.x += self.gps_offset[0]
        goal_pose.pose.position.y += self.gps_offset[1]
        goal_pose.pose.position.z += self.gps_offset[2]
        
        self.pub_goal_pose.publish(goal_pose)
