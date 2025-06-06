#!/usr/bin/env python

import rospy
from mavros_msgs.srv import SetMode, CommandBool, CommandBoolRequest, CommandTOL
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State

class TakeOffMode():
    def __init__(self):
        # Parameters        
        self.arm_cmd = CommandBoolRequest()
        self.arm_cmd.value = True
        self.ugv_pose_topic = rospy.get_param("~ugv_pose_topic", "/rovid/ekf_ugv")  
        
        # Publishers
        self.pub_goal_pose = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=1)
        
        # Subscribers (will be set in activate())
        self.status_sub = None
        self.ugv_pose_sub = None
        
        self.current_state = State()
        self.current_state.armed = False
    
        
    def activate(self, altitude):
        '''
            Activates subscription to topics and calls function to activate takeoff
        '''
        if (self.status_sub is None) and (self.ugv_pose_sub is None):
            self.status_sub = rospy.Subscriber("mavros/state", State, self.uav_status_callback)
            self.ugv_pose_sub = rospy.Subscriber(self.ugv_pose_topic, Odometry, self.ugv_pose_callback)
        
        self.take_off_uav(altitude)
        
    def deactivate(self):
        '''
            De-activates subscription to topics
        '''
        if (self.status_sub is not None) and (self.ugv_pose_sub is not None):
            self.status_sub.unregister()
            self.status_sub = None
            self.ugv_pose_sub.unregister()
            self.ugv_pose_sub = None
            self.current_state = State()
            self.current_state.armed = False
            
    def ugv_pose_callback(self, msg):
        self.ugv_x = msg.pose.pose.position.x
        self.ugv_y = msg.pose.pose.position.y
    
    def uav_status_callback(self, msg):
        self.current_state = msg
        
    def take_off_uav(self, altitude):
        # ARM Throttle
        rospy.wait_for_service("mavros/cmd/arming")
        arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
        rate = rospy.Rate(0.2) 
        
        while (not self.current_state.armed):
            arm_success = arming_client.call(self.arm_cmd).success
            if(arm_success):
                rospy.loginfo("Vehicle armed")
                break
            rate.sleep()
            
        while (not self.current_state.armed): #TODO - make this better
            print(self.current_state.armed)
            rate.sleep()
            
        rospy.wait_for_service("mavros/cmd/takeoff")
        takeoff_client = rospy.ServiceProxy("mavros/cmd/takeoff", CommandTOL)

        # self.takeoff_cmd = CommandTOL()
        # self.takeoff_cmd.altitude = altitude
        while True:
            print("sending take off command")
            takeoff_success = takeoff_client.call(min_pitch=0.0, yaw=0.0, latitude=0.0, longitude=0.0, altitude=altitude).success
            if(takeoff_success):
                rospy.loginfo("Takeoff started")
                break
            
            rate.sleep()
        