#!/usr/bin/env python

import rospy
from mavros_msgs.srv import CommandLong
from mavros_msgs.msg import State

GPS = 1
MARKER = 2

class EkfSource():
    def __init__(self):
        # Parameters 
        self.ekf_src_cmd = CommandLong()
        self.ekf_src_cmd.command = 42007
               
        
    def change_src(self, mode):
               
        rospy.wait_for_service("mavros/cmd/command")
        command_client = rospy.ServiceProxy("mavros/cmd/command", CommandLong)
        rate = rospy.Rate(1) 
        
        if mode != 1. and mode!= 2.:
               
            rospy.loginfo("Required mode for EKF source not valid. Service arguments must be: mode=5 for EKF source change and altitude = 1(GPS) or 2(Markers)")
            return 
        
        cmd_success = False
        while (not cmd_success):
               
            self.ekf_src_cmd.param1 = mode
                   
            cmd_success = command_client.call(False, 42007, 1, int(mode), 0, 0, 0, 0, 0, 0).success
            if(cmd_success):
                rospy.loginfo("EKF Source changed")
            
            rate.sleep()
   
