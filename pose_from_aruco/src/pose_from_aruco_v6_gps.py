#!/usr/bin/env python

import rospy
import tf
import tf2_ros
import tf2_geometry_msgs
import numpy as np
import PyKDL
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped, Quaternion, Transform, Vector3, TwistStamped 
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from fiducial_msgs.msg import FiducialTransformArray
from pose_from_aruco.srv import GetGPSOffset, GetGPSOffsetResponse
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_multiply, quaternion_from_matrix, quaternion_matrix, quaternion_inverse

class PoseFromAruco():
    def __init__(self):
        # Initialize Node
        rospy.init_node('Pose_from_aruco')
        rospy.loginfo("Pose_from_aruco node initialized.")
        
        # Parameters
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.camera_frame = rospy.get_param("~camera_frame", "camera_color_optical_frame")
        self.uav_frame = rospy.get_param("~uav_frame", "holybro_v1/base_link")
        self.ugv_frame = rospy.get_param("~ugv_frame", "rovid/base_link")
        self.ugv_pose_topic = rospy.get_param("~ugv_pose_topic", "rovid/ekf_ugv")
        
        self.init_x = rospy.get_param("~init_x", None)
        self.init_y = rospy.get_param("~init_y", None)
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_pub = tf.TransformBroadcaster()
           
        self.ugv_odom_time = []
        self.ugv_odom = []
        self.uav_odom_time = [0.0]
        self.uav_odom = [Quaternion(0,0,0,1)]
        self.ugv_pose = Vector3(0,0,0)
        
        self.last_fid_time = 0
        
        self.old_alt = 0
        self.gps_offset = [0., 0., 0., 0.]
        
        # Initialize Functions
        self.init_static_tfs()
        
        # Subscribers
        rospy.Subscriber('/holybro/fiducial_transforms', FiducialTransformArray, self.fiducial_transform)
        rospy.Subscriber(self.ugv_pose_topic, Odometry, self.ugv_euler_angles)
        rospy.Subscriber("mavros/local_position/pose", PoseStamped, self.uav_euler_angles_imu)
        rospy.Subscriber("mavros/state", State, self.uav_status_callback)
        rospy.Subscriber("mavros/local_position/pose", PoseStamped, self.uav_ardupilot_pose)
        
        # Service
        s = rospy.Service('get_gps_offset', GetGPSOffset, self.get_gps_offset)
    
        # Publishers
        self.rel_pose_publisher = rospy.Publisher("/uav_rel_pose", PoseStamped, queue_size=10)
        self.pose_publisher = rospy.Publisher("mavros/vision_pose/pose_cov", PoseWithCovarianceStamped, queue_size=10)
        self.speed_publisher = rospy.Publisher("mavros/vision_speed/speed_twist", TwistStamped, queue_size=10)
        self.gps_offset_publisher = rospy.Publisher("gps_offset", PoseStamped, queue_size=10)
     
    def init_static_tfs(self):
        # T_fid_C_2_fid
        T_fid_10  = self.tf_buffer.lookup_transform("fiducial_C_10", "fiducial_10", rospy.Time(0), rospy.Duration(1)) 
        T_fid_55  = self.tf_buffer.lookup_transform("fiducial_C_55", "fiducial_55", rospy.Time(0), rospy.Duration(1)) 
        T_fid_168 = self.tf_buffer.lookup_transform("fiducial_C_168", "fiducial_168", rospy.Time(0), rospy.Duration(1)) 
        T_fid_227 = self.tf_buffer.lookup_transform("fiducial_C_227", "fiducial_227", rospy.Time(0), rospy.Duration(1)) 
        T_fid_946 = self.tf_buffer.lookup_transform("fiducial_C_946", "fiducial_946", rospy.Time(0), rospy.Duration(1)) 
        
        has_cam_tf = False
        while not has_cam_tf:
            
            try:
                T_camera_2_uav = self.tf_buffer.lookup_transform(self.camera_frame, self.uav_frame, rospy.Time(0), rospy.Duration(1))
                has_cam_tf = True
                
            except:
            	 pass

        self.T_fid_10_kdl = tf2_geometry_msgs.transform_to_kdl(T_fid_10)
        self.T_fid_55_kdl = tf2_geometry_msgs.transform_to_kdl(T_fid_55)
        self.T_fid_168_kdl = tf2_geometry_msgs.transform_to_kdl(T_fid_168)
        self.T_fid_227_kdl = tf2_geometry_msgs.transform_to_kdl(T_fid_227)
        self.T_fid_946_kdl = tf2_geometry_msgs.transform_to_kdl(T_fid_946)
        self.T_camera_2_uav_kdl = tf2_geometry_msgs.transform_to_kdl(T_camera_2_uav)
        
        self.kdl_dict = {
            10:  self.T_fid_10_kdl,
            55:  self.T_fid_55_kdl,
            168: self.T_fid_168_kdl,
            227: self.T_fid_227_kdl,
            946: self.T_fid_946_kdl
        }
        
    # Transformation Calculation Functions 
    def get_static_euler_angles(self, time):
        idx_ugv        = (np.abs(np.array(self.ugv_odom_time) - time)).argmin()
        quat_map_2_ugv = self.ugv_odom[idx_ugv]
        
        idx_uav        = (np.abs(np.array(self.uav_odom_time) - time)).argmin()
        quat_map_2_uav = self.uav_odom[idx_uav]
        
        # Erase previous values from arrays
        self.ugv_odom_time = self.ugv_odom_time[idx_ugv:]
        self.ugv_odom      = self.ugv_odom[idx_ugv:]
        
        self.uav_odom_time = self.uav_odom_time[idx_uav:]
        self.uav_odom      = self.uav_odom[idx_uav:]
        
        return quat_map_2_ugv, quat_map_2_uav
  
    def get_rel_euler_angles(self, quat_map_2_ugv, quat_map_2_uav, q_marker):
        # q_cam_2_uav
        q_cam_2_uav = self.T_camera_2_uav_kdl.M.GetQuaternion()
        
        # q_uav_2_map
        _, _, yaw_map_2_ugv = euler_from_quaternion([quat_map_2_ugv.x, quat_map_2_ugv.y, quat_map_2_ugv.z, quat_map_2_ugv.w])
        _, _, yaw_cam_2_ugv = euler_from_quaternion([q_marker.x, q_marker.y, q_marker.z, q_marker.w])
        
        yaw_uav_2_ugv = - (yaw_cam_2_ugv + np.pi/2)
        yaw_map_2_uav = yaw_map_2_ugv - yaw_uav_2_ugv
        
        roll_map_2_uav, pitch_map_2_uav, _ = euler_from_quaternion([quat_map_2_uav.x, quat_map_2_uav.y, quat_map_2_uav.z, quat_map_2_uav.w])
        
        q_map_2_uav = quaternion_from_euler(roll_map_2_uav, pitch_map_2_uav, yaw_map_2_uav)
        q_uav_2_map = quaternion_inverse(q_map_2_uav)

        # q_map_2_ugv
        q_map_2_ugv = [quat_map_2_ugv.x, quat_map_2_ugv.y, quat_map_2_ugv.z, quat_map_2_ugv.w]
        
        #q_cam_2_fid = q_cam_2_uav * q_uav_2_map * q_map_2_ugv
        q_cam_2_fid = quaternion_multiply(q_cam_2_uav, q_uav_2_map)
        q_cam_2_fid = quaternion_multiply(q_cam_2_fid, q_map_2_ugv)
        
        return q_cam_2_fid

    def get_ugv_2_uav(self, fiducial_id, transformation, quat_rotation):
        tf_cam_2_fid = TransformStamped()
        tf_cam_2_fid.transform.translation = transformation.translation
        tf_cam_2_fid.transform.rotation.x  = quat_rotation[0]
        tf_cam_2_fid.transform.rotation.y  = quat_rotation[1]
        tf_cam_2_fid.transform.rotation.z  = quat_rotation[2]
        tf_cam_2_fid.transform.rotation.w  = quat_rotation[3]

        T_cam_2_fid_kdl = tf2_geometry_msgs.transform_to_kdl(tf_cam_2_fid)
        T_fid_2_cam_kdl = T_cam_2_fid_kdl.Inverse()
        tf_ugv_2_uav = self.kdl_dict[fiducial_id] * T_fid_2_cam_kdl * self.T_camera_2_uav_kdl
        
        roll, pitch, yaw = tf_ugv_2_uav.M.GetRPY()

        return [tf_ugv_2_uav.p[0], tf_ugv_2_uav.p[1], tf_ugv_2_uav.p[2], roll, pitch, yaw]        

    # Callback functions 
    def uav_status_callback(self, msg):
        self.current_state = msg
         
    def ugv_euler_angles(self, msg):
        self.ugv_odom_time.append(msg.header.stamp.to_sec())
        
        self.ugv_pose = msg.pose.pose.position
        self.ugv_odom.append(msg.pose.pose.orientation)
        
    def uav_euler_angles(self, msg):
        self.uav_odom_time.append(msg.header.stamp.to_sec())
        self.uav_odom.append(msg.pose.orientation) 
            
    def uav_ardupilot_pose(self, msg):
        (_, _, yaw) = euler_from_quaternion((msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w))
        self.current_ardupilot_pose = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, yaw]  
    
    def get_gps_offset(self, req):
        res = GetGPSOffsetResponse()
        
        res.x = self.current_ardupilot_pose[0]
        res.y = self.current_ardupilot_pose[1]
        res.z = self.current_ardupilot_pose[2]
        res.yaw = self.current_ardupilot_pose[3]
        
        self.gps_offset = self.current_ardupilot_pose
        
        res.success = True
        
        pose_offset = PoseStamped()
        pose_offset.pose.position.x = self.current_ardupilot_pose[0]
        pose_offset.pose.position.y = self.current_ardupilot_pose[1]
        pose_offset.pose.position.z = self.current_ardupilot_pose[2]
        rot = quaternion_from_euler(0,0,self.current_ardupilot_pose[3])
        pose_offset.pose.orientation.x = rot[0]
        pose_offset.pose.orientation.y = rot[1]
        pose_offset.pose.orientation.z = rot[2]
        pose_offset.pose.orientation.w = rot[3]
        
        self.gps_offset_publisher.publish(pose_offset)
        
        return res
         
    def uav_euler_angles_imu(self, msg):
        self.uav_odom_time.append(msg.header.stamp.to_sec())
        self.uav_odom.append(msg.pose.orientation) 
         
    def fiducial_transform(self, msg):
        if len(msg.transforms) == 0:
            print("NO FID SEEN")
            if len(self.uav_odom) < 0:
                uav_odom = Quaternion()
            else:
                uav_odom = self.uav_odom[0]
            
            if((self.old_alt < 0.5)): # if the last known altitude was 0.5 or bellow, consider it landed
                rospy.loginfo("Fiducials time-out. sending position as if UAV is landed")
                self.tf_pub.sendTransform((0, 0, 0.5),
                                          (uav_odom.x, uav_odom.y, uav_odom.z, uav_odom.w),
                                          msg.header.stamp,
                                          self.uav_frame, self.ugv_frame) 
            return
        
        self.last_fid_time = msg.header.stamp.to_sec()
        observations = np.zeros((len(msg.transforms), 6))
        i=0

        quat_map_2_ugv, quat_map_2_uav = self.get_static_euler_angles(msg.header.stamp.to_sec())
        
        for fiducial in msg.transforms:
            q_uav_2_ugv = self.get_rel_euler_angles(quat_map_2_ugv, quat_map_2_uav, fiducial.transform.rotation)
            T_ugv_2_uav = self.get_ugv_2_uav(fiducial.fiducial_id, fiducial.transform, q_uav_2_ugv)
            
            observations[i] = np.array(T_ugv_2_uav)
            
            i += 1
            
        if i == 1:
            uav_pose = observations[0]
        elif i > 1:
            observations[:, 5] = np.unwrap(observations[:, 5])
            uav_pose = np.mean(observations, axis=0)
        else:
            return
              
        if uav_pose[0] > 100 or uav_pose[1] > 100 or uav_pose[2] > 100:
            # outlier position
            return
          
        new_uav_pose = PoseStamped()
        new_uav_pose.header.stamp = msg.header.stamp
        new_uav_pose.header.frame_id = self.ugv_frame

        new_uav_pose.pose.position.x = uav_pose[0]
        new_uav_pose.pose.position.y = uav_pose[1]
        new_uav_pose.pose.position.z = uav_pose[2]
        (new_uav_pose.pose.orientation.x, new_uav_pose.pose.orientation.y, new_uav_pose.pose.orientation.z, new_uav_pose.pose.orientation.w) = quaternion_from_euler(uav_pose[3], uav_pose[4], uav_pose[5])
        
        self.current_uav_pose = new_uav_pose
        self.rel_pose_publisher.publish(new_uav_pose)
        self.tf_pub.sendTransform((uav_pose[0], uav_pose[1], uav_pose[2]),
                                  (new_uav_pose.pose.orientation.x, new_uav_pose.pose.orientation.y, new_uav_pose.pose.orientation.z, new_uav_pose.pose.orientation.w),
                                  msg.header.stamp,
                                  self.uav_frame, self.ugv_frame) 
        self.old_alt = uav_pose[2]
    
    def publish_pose(self):
        # SEND ABSOLUTE POSE
        try:
            TT = self.tf_buffer.lookup_transform(self.map_frame, self.uav_frame, rospy.Time(0), rospy.Duration(1)) 
            TT.transform.translation.x -= self.init_x
            TT.transform.translation.y -= self.init_y
            
            TT.transform.translation.x += self.gps_offset[0]
            TT.transform.translation.y += self.gps_offset[1]
            TT.transform.translation.z += self.gps_offset[2]
            
            q_yaw_offset = quaternion_from_euler(0,0, self.gps_offset[3])
            new_q = quaternion_multiply(q_yaw_offset, [TT.transform.rotation.x, TT.transform.rotation.y, TT.transform.rotation.z, TT.transform.rotation.w])
            
            new_uav_pose = PoseWithCovarianceStamped()
            new_uav_pose.header = TT.header
            new_uav_pose.pose.pose.position = TT.transform.translation
            new_uav_pose.pose.pose.orientation = Quaternion(new_q[0], new_q[1], new_q[2], new_q[3])

            new_uav_pose.pose.covariance = 0.001*np.eye(6).flatten()
            self.pose_publisher.publish(new_uav_pose) 
                
        except Exception as error:
            return None
       
if __name__ == '__main__': 
    try:
        pose_from_aruco = PoseFromAruco()
        rate = rospy.Rate(30)
        
        while True:
            pose_from_aruco.publish_pose()
            rate.sleep()
        
    except rospy.ROSInterruptException:
        pass
