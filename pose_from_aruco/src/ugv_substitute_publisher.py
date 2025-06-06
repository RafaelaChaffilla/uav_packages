#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_multiply, quaternion_from_matrix, quaternion_matrix, quaternion_inverse
import tf2_ros
import tf

class UGVPoseSub():
    def __init__(self):
        # Initialize Node
        rospy.init_node('UGV_pose_substitute')
        rospy.loginfo("UGV_pose_substitute node initialized.")
        
        self.init_x = rospy.get_param("~init_x", None)
        self.init_y = rospy.get_param("~init_y", None)
        self.ugv_pose_topic = rospy.get_param("~ugv_pose_topic", "rovid/ekf_ugv")
        
        # Publishers
        self.ugv_pose_publisher = rospy.Publisher(self.ugv_pose_topic, Odometry, queue_size=10)
        
        # Subscribers
        rospy.Subscriber("mavros/imu/data", Imu, self.uav_euler_angles_imu)
        rospy.Subscriber("/uav_rel_pose", PoseStamped, self.uav_rel_pose)
        
        #TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_pub = tf.TransformBroadcaster()
        
        self.yaw_map_2_uav = 0
        self.yaw_ugv_2_uav = 0
        
    def uav_euler_angles_imu(self, msg):
        _, _, self.yaw_map_2_uav = euler_from_quaternion([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
    
    def uav_rel_pose(self, msg):
        _, _, self.yaw_ugv_2_uav = euler_from_quaternion([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
        
    def publish_pose(self):
    
        new_ugv_pose = Odometry()
        new_ugv_pose.header.stamp = rospy.Time.now()
        new_ugv_pose.pose.pose.position.x = self.init_x
        new_ugv_pose.pose.pose.position.y = self.init_y
        new_ugv_pose.pose.pose.position.z = 0
        
        ugv_yaw = self.yaw_map_2_uav - self.yaw_ugv_2_uav
        ugv_yaw = (ugv_yaw + 3.1415) % (2 * 3.1415) - 3.1415
        quat = quaternion_from_euler(0,0,ugv_yaw)
        
        new_ugv_pose.pose.pose.orientation.x = quat[0]
        new_ugv_pose.pose.pose.orientation.y = quat[1]
        new_ugv_pose.pose.pose.orientation.z = quat[2]
        new_ugv_pose.pose.pose.orientation.w = quat[3]
        
        self.ugv_pose_publisher.publish(new_ugv_pose)  
        
        self.tf_pub.sendTransform((0,0,0),
                                  (new_ugv_pose.pose.pose.orientation.x, new_ugv_pose.pose.pose.orientation.y, new_ugv_pose.pose.pose.orientation.z, new_ugv_pose.pose.pose.orientation.w),
                                  rospy.Time.now(),
                                  "rovid/base_link", "map")  
       
if __name__ == '__main__': 
    try:
        ugv_pose_substitute = UGVPoseSub()
        rate = rospy.Rate(50)
        
        while True:
            ugv_pose_substitute.publish_pose()
            rate.sleep()
            
    except rospy.ROSInterruptException:
        pass
