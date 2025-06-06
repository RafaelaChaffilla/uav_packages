#!/usr/bin/env python

import rospy
from mavros_msgs.srv import MessageInterval

class SetMessagesRate():
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('set_ardupilot_rate', anonymous=True)

        # Wait for the set_message_interval service to be available
        rospy.wait_for_service('mavros/set_message_interval')

        try:
            # Create a service proxy to call the set_message_interval service
            set_rate  = rospy.ServiceProxy('mavros/set_message_interval', MessageInterval)
        
            # Set the message interval for different messages
            # IMU
            response_imu = set_rate(26, 100)
            rospy.loginfo("IMU rate set: %s", response_imu)
            # RAW IMU
            response_raw_imu = set_rate(27, 100)
            rospy.loginfo("IMU rate  set: %s", response_raw_imu)
            # Orientation
            response_attitude_quat = set_rate(31, 100)
            rospy.loginfo("ATTITUDE QUATERNION rate  set: %s", response_attitude_quat)
            # Local Position (/mavros/local_position/pose)
            response_local_pose = set_rate(32, 100)
            rospy.loginfo("Local Pose rate  set: %s", response_local_pose)
            # Battery Status 
            response_bat_status = set_rate(147, 50)
            rospy.loginfo("Battery status rate  set: %s", response_bat_status)
            # mavros/state
            response_bat_info = set_rate(0, 50)
            rospy.loginfo("heartbeat rate  set: %s", response_bat_info)
            # gps raw
            response_gps_raw = set_rate(24, 50)
            rospy.loginfo("gps rate  set: %s", response_gps_raw)
            

        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)

        
if __name__=="__main__":
    try:
        set_origin = SetMessagesRate()
            
            
    except rospy.ROSInterruptException:
        pass
