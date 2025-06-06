#!/usr/bin/env python

import rospy
from geographic_msgs.msg import GeoPointStamped

class SetEkfOrigin():
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('set_home_position', anonymous=True)

        # Publisher
        self.pub = rospy.Publisher('mavros/global_position/set_gp_origin', GeoPointStamped, queue_size=10)

    def publish_ekf_origin(self):
        msg = GeoPointStamped()

        msg.header.stamp = rospy.Time.now()
        
        msg.position.latitude  = 41.32086
        msg.position.longitude = -8.15127
        msg.position.altitude  = 163
        
        self.pub.publish(msg)
        
    def check_ekf_origin_status(self):
        pass
        
if __name__=="__main__":
    try:
        set_origin = SetEkfOrigin()
        rate = rospy.Rate(1)
        
        while True:
            set_origin.publish_ekf_origin()
            rate.sleep()
            
            
    except rospy.ROSInterruptException:
        pass