#!/usr/bin/env python

import rospy
import os
from geometry_msgs.msg import Pose2D

def position_callback(msg):

    rospy.sleep(10.0)  # Wait for 10 seconds

    command = "roslaunch holybro spawn_model.launch init_x:=" + str(msg.x-0.051) + " init_y:=" + str(msg.y-0.048) + " init_th:=" + str(msg.theta)
    os.system("gnome-terminal -e 'bash -c \""+command+";bash\"'")
    
    command = "roslaunch holybro apm.launch"
    os.system("gnome-terminal -e 'bash -c \""+command+";bash\"'")
    rospy.signal_shutdown("Condition met. spawn model to gazebo node terminated.")
    
if __name__=="__main__":
    
    try:
        rospy.init_node('spawn_model_to_gazebo')
        rospy.loginfo("spawn model to gazebo node initiated")
        pose_topic = rospy.get_param("~ugv_pose_topic", "/rovid/initial_position")
        
        rospy.Subscriber(pose_topic, Pose2D, position_callback)
        
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass
    
