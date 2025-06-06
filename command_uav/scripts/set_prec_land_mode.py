#!/usr/bin/env python

import rospy
import numpy as np
from mavros_msgs.msg import LandingTarget, PositionTarget
from command_uav.srv import SetFlightMode
from geometry_msgs.msg import PoseStamped, Pose, TwistStamped, Twist
from nav_msgs.msg import Odometry
from fiducial_msgs.msg import FiducialTransformArray
import tf.transformations as tf

SAFETY_ALT = 1.1 # stops precision when it reaches this altitude, corrects the position error until it's below a threshold and activates ardupilot's land mode

class LandProfile():
    def __init__(self, delta_t1_max=0, delta_t2_max=0, init_time=0, last_time = 0,
                 init_pose_x=0, init_pose_y=0, init_pose_z=0, init_pose_Y=0, 
                 max_vel_x=0, max_vel_y=0, max_vel_z=0, max_vel_Y=0, 
                 max_acc_x=0, max_acc_y=0, max_acc_z=0, max_acc_Y=0,
                 vel_int_x=0, vel_int_y=0, vel_int_z=0, vel_int_Y=0,
                 c_error_x=0, c_error_y=0, c_error_z=0, c_error_Y=0, 
                 status=False):
        
        self.delta_t1 = np.array([0,0,0,0])
        self.delta_t2 = np.array([0,0,0,0])
        # Delta 1 from Trapezoid
        self.delta_t1_max = delta_t1_max
        # Delta 2 from Trapezoid
        self.delta_t2_max = delta_t2_max
        # Time when the landing profile was designed
        self.init_time = init_time
        # Time of k-1 (to integrate)
        self.last_time = last_time
        
        # Estimated position when profile was designed
        self.init_pose = np.zeros((4,))
        
        self.init_pose[0] = init_pose_x
        self.init_pose[1] = init_pose_y
        self.init_pose[2] = init_pose_z
        self.init_pose[3] = init_pose_Y
        
        # Maximum velocity for each variable
        self.max_vel = np.zeros((4,))
        
        self.max_vel[0] = max_vel_x
        self.max_vel[1] = max_vel_y
        self.max_vel[2] = max_vel_z
        self.max_vel[3] = max_vel_Y
        
        # Maximum acceleration for each variable
        self.max_acc = np.zeros((4,))
        
        self.max_acc[0] = max_acc_x
        self.max_acc[1] = max_acc_y
        self.max_acc[2] = max_acc_z
        self.max_acc[3] = max_acc_Y
        
        # Velocity integration (to calculate what the position should be at time k)
        self.vel_int = np.zeros((4,))
        
        self.vel_int[0] = vel_int_x
        self.vel_int[1] = vel_int_y
        self.vel_int[2] = vel_int_z
        self.vel_int[3] = vel_int_Y
        
        # Cumulative error (For I in PI controler)
        self.c_error = np.zeros((4,))
        
        self.c_error[0] = c_error_x
        self.c_error[1] = c_error_y
        self.c_error[2] = c_error_z
        self.c_error[3] = c_error_Y
        
        # Boolean to store if stored information is for current landing profile or from previous one
        self.status = status
        
    def delete_profile_instance(self):
        self.status = False
        self.vel_int = np.zeros((4,))
        self.c_error = np.zeros((4,))

class PrecLandCtrl():
    def __init__(self):
        # Parameters 
        # [[v_max_x, a_max_x, k_p_x, k_i_x],
        #  [v_max_y, a_max_y, k_p_y, k_i_y],
        #  [v_max_z, a_max_z, k_p_z, k_i_z],
        #  [v_max_Y, a_max_Y, k_p_Y, k_i_Y]]
        self.parameters_matrix = np.array(rospy.get_param("~parameters_matrix",[[0.4, 0.40, 0.15, 0.0001],  
                                                                                [0.40, 0.40, 0.15, 0.0001], 
                                                                                [0.35, 0.17, 0.40, 0.01], 
                                                                                [0.15, 0.25, 0.50, 0.02]]))
        
        # Publishers
        self.pub_commands = rospy.Publisher('mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size=1)
         
        # Subscribers (will be set in activate())
        self.rel_pose_sub = None
        self.control_stage = 0 # 0:yaw, 1:x e y, 2:z

        # Initialize Variables
        self.land_profile = LandProfile(status=False)

    def activate(self):
        '''
            Activates subscription to topics
        '''
        if self.rel_pose_sub is None:
            self.rel_pose_sub = rospy.Subscriber("/uav_rel_pose", PoseStamped, self.uav_rel_pose_callback)
            
    def deactivate(self):
        '''
            De-activates subscription to topics
        '''
        if self.rel_pose_sub is not None:
            self.rel_pose_sub.unregister()
            self.rel_pose_sub = None
            self.control_stage = 0
    
    def set_land_profile(self, rel_pose):
        ''' delta_ti = [delta_ti_x, delta_ti_y, delta_ti_z, delta_ti_Y] '''
        self.delta_t1 = np.zeros((4,)) 
        self.delta_t2 = np.zeros((4,))
        pose     = np.zeros((4,))
        self.land_profile.max_vel = np.zeros((4,))
        self.land_profile.max_acc = np.zeros((4,))
        # CALCULATION IN UGV FRAME
        
        # x
        pose[0] = rel_pose.pose.position.x
        self.delta_t1[0] =  self.parameters_matrix[0,0]/self.parameters_matrix[0,1]
        self.delta_t2[0] =  np.abs(pose[0])/self.parameters_matrix[0,0] - self.delta_t1[0]
        self.land_profile.max_vel[0] = np.sign(pose[0]) * self.parameters_matrix[0,0]
        self.land_profile.max_acc[0] = np.sign(pose[0]) * self.parameters_matrix[0,1]
        # y
        pose[1] = rel_pose.pose.position.y
        self.delta_t1[1] =  self.parameters_matrix[1,0]/self.parameters_matrix[1,1]
        self.delta_t2[1] =  np.abs(pose[1])/self.parameters_matrix[1,0] - self.delta_t1[1]
        self.land_profile.max_vel[1] = np.sign(pose[1]) * self.parameters_matrix[1,0]
        self.land_profile.max_acc[1] = np.sign(pose[1]) * self.parameters_matrix[1,1]
        # z
        pose[2] = rel_pose.pose.position.z - SAFETY_ALT
        self.delta_t1[2] =  self.parameters_matrix[2,0]/self.parameters_matrix[2,1]
        self.delta_t2[2] =  np.abs(pose[2])/self.parameters_matrix[2,0] - self.delta_t1[2]
        self.land_profile.max_vel[2] = np.sign(pose[2]) * self.parameters_matrix[2,0]
        self.land_profile.max_acc[2] = np.sign(pose[2]) * self.parameters_matrix[2,1]
        # Yaw
        (_, _, pose[3]) = tf.euler_from_quaternion((rel_pose.pose.orientation.x, rel_pose.pose.orientation.y, rel_pose.pose.orientation.z, rel_pose.pose.orientation.w))
        self.delta_t1[3] =  self.parameters_matrix[3,0]/self.parameters_matrix[3,1]
        self.delta_t2[3] =  np.abs(pose[3])/self.parameters_matrix[3,0] - self.delta_t1[3]
        self.land_profile.max_vel[3] = np.sign(pose[3]) * self.parameters_matrix[3,0]
        self.land_profile.max_acc[3] = np.sign(pose[3]) * self.parameters_matrix[3,1]
        
        self.land_profile.init_time = rel_pose.header.stamp.to_sec()
        self.land_profile.last_time = self.land_profile.init_time
        
        self.land_profile.init_pose = pose
        
        self.land_profile.vel_int = np.zeros((4,))
        self.land_profile.c_error = np.zeros((4,))
        
        self.land_profile.status = True
         
    def uav_rel_pose_callback(self, msg):
        # Get Landing Profile
        if not self.land_profile.status:
            self.set_land_profile(msg)
            return
        
        # IN UGV FRAME
        # Extract Information from message
        delta_t = msg.header.stamp.to_sec() - self.land_profile.init_time
        dt_k_1  = msg.header.stamp.to_sec() - self.land_profile.last_time
        self.land_profile.last_time = msg.header.stamp.to_sec()
        (_, _, pos_Y) = tf.euler_from_quaternion((msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w))
        estimated_pose = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z-SAFETY_ALT, pos_Y])

        if self.control_stage == 0:
            # fix only yaw
            vel = self.land_profile.max_vel[3]
            acc = self.land_profile.max_acc[3]
            
            # Extract current goal velocity
            if delta_t <= self.delta_t1[3]: # trapezoid part 1
                v_k_Y = -acc * delta_t
            elif delta_t <= self.delta_t1[3] + self.delta_t2[3]: # trapezoid part 2
                v_k_Y = -vel
            elif delta_t <= 2*self.delta_t1[3] + self.delta_t2[3]: # trapezoid part 3
                v_k_Y = -(- acc * delta_t + acc * (2*self.delta_t1[3] + self.delta_t2[3]))
            else: # trapezoid should have ended
                v_k_Y = 0
            
            print(f"delta_t: {delta_t}, v_k_Y: {v_k_Y}")  # For yaw stage
        
            v_k = np.array([0, 0, 0, v_k_Y])
            # Expected position at this time:
            self.land_profile.vel_int[3] += v_k_Y*dt_k_1
            # IN UGV FRAME
            expected_Y = self.land_profile.init_pose[3] + self.land_profile.vel_int[3]
            error_Y = expected_Y - estimated_pose[3]
            error_Y = (error_Y + np.pi) % (2 * np.pi) - np.pi

            self.land_profile.c_error[3] += error_Y
            delta_v_d_Y = self.parameters_matrix[3,2]*error_Y + self.parameters_matrix[3,3]*self.land_profile.c_error[3]

            v_desired = [0,0,0, v_k_Y + delta_v_d_Y]
            
            v_desired = np.sign(v_desired) * np.minimum(np.abs(v_desired), self.parameters_matrix[:, 0])
            error_Y = 0 #removeeeeeeeee
            # Confirm Landing only when error is bellow a threshold
            if ((delta_t > 2*self.delta_t1[3] + self.delta_t2[3]) and (np.abs(error_Y) <= 0.15)):
                self.control_stage += 1
                self.land_profile.init_time = msg.header.stamp.to_sec()
                self.land_profile.c_error[3] = 0
            
            self.publish_vel_command(v_desired)
            
        elif self.control_stage == 1:
            # fix only x and y
            vel_x = self.land_profile.max_vel[0]
            acc_x = self.land_profile.max_acc[0]
            vel_y = self.land_profile.max_vel[1]
            acc_y = self.land_profile.max_acc[1]
            d1_max = np.max([self.delta_t1[0], self.delta_t1[1]])
            d2_max = np.max([self.delta_t2[0], self.delta_t2[1]])
            # Extract current goal velocity
            if delta_t <= d1_max: # trapezoid part 1
                v_k_x = -acc_x * delta_t
                v_k_y = -acc_y * delta_t
            elif delta_t <= d1_max + d2_max: # trapezoid part 2
                v_k_x = -vel_x
                v_k_y = -vel_y
            elif delta_t <= 2*d1_max + d2_max: # trapezoid part 3
                v_k_x = -(- acc_x * delta_t + acc_x * (2*self.delta_t1[0] + self.delta_t2[0]))
                v_k_y = -(- acc_y * delta_t + acc_y * (2*self.delta_t1[1] + self.delta_t2[1]))
            else: # trapezoid should have ended
                v_k_x = 0 
                v_k_y = 0  
                
            print(f"delta_t: {delta_t}, v_k_x: {v_k_x}")  # For yaw stage
            print(f"delta_t: {delta_t}, v_k_y: {v_k_y}")  # For yaw stage
            
            v_k = np.array([v_k_x, v_k_y, 0, 0])
            # Expected position at this time:
            self.land_profile.vel_int[0] += v_k_x*dt_k_1
            self.land_profile.vel_int[1] += v_k_y*dt_k_1
            expected_pose = np.array([self.land_profile.init_pose[0] + self.land_profile.vel_int[0], self.land_profile.init_pose[1] + self.land_profile.vel_int[1],self.land_profile.init_pose[2],0])
            
            # CONVERT TO UAV FRAME
            estimated_pose_in_UAV_frame = np.copy(estimated_pose)
            expected_pose_in_UAV_frame  = np.copy(expected_pose)
            v_k_in_UAV_frame            = np.copy(v_k)
            rotation_matrix = tf.euler_matrix(0., 0., -pos_Y)[:3, :3]
            estimated_pose_in_UAV_frame[:3] = np.dot(rotation_matrix, estimated_pose[:3])
            expected_pose_in_UAV_frame[:3]  = np.dot(rotation_matrix, expected_pose[:3])
            v_k_in_UAV_frame[:3]            = np.dot(rotation_matrix, v_k[:3])
                
            # PI error control
            error = expected_pose_in_UAV_frame - estimated_pose_in_UAV_frame
            error[3] = (error[3] + np.pi) % (2 * np.pi) - np.pi
            self.land_profile.c_error += error
            delta_v_d = np.multiply(self.parameters_matrix[:,2],error) + np.multiply(self.parameters_matrix[:,3],self.land_profile.c_error)

            v_desired = v_k_in_UAV_frame + delta_v_d
            v_desired = np.sign(v_desired) * np.minimum(np.abs(v_desired), self.parameters_matrix[:, 0])
            
            # Confirm Landing only when error is bellow a threshold
            if ((delta_t > 2*d1_max + d2_max) and (np.linalg.norm(error[0:2]) <= 0.2)):
                self.control_stage += 1
                self.land_profile.init_time = msg.header.stamp.to_sec()
                self.land_profile.c_error = np.array([0.,0.,0.,0.])
            
            self.publish_vel_command(v_desired)
            
        elif self.control_stage == 2:
            # fix only z
            vel = self.land_profile.max_vel[2]
            acc = self.land_profile.max_acc[2]
            # Extract current goal velocity
            if delta_t <= self.delta_t1[2]: # trapezoid part 1
                v_k_z = -acc * delta_t
            elif delta_t <= self.delta_t1[2] + self.delta_t2[2]: # trapezoid part 2
                v_k_z = -vel
            elif delta_t <= 2*self.delta_t1[2] + self.delta_t2[2]: # trapezoid part 3
                v_k_z = -(- acc * delta_t + acc * (2*self.delta_t1[2] + self.delta_t2[2]))
            else: # trapezoid should have ended
                v_k_z = 0 

            v_k = np.array([0, 0, v_k_z, 0])
            # Expected position at this time:
            self.land_profile.vel_int[2] += v_k_z*dt_k_1
            expected_pose = np.array([0, 0, self.land_profile.init_pose[2] + self.land_profile.vel_int[2], 0])
            
            # CONVERT TO UAV FRAME
            estimated_pose_in_UAV_frame = np.copy(estimated_pose)
            expected_pose_in_UAV_frame  = np.copy(expected_pose)
            v_k_in_UAV_frame            = np.copy(v_k)
            rotation_matrix = tf.euler_matrix(0., 0., -pos_Y)[:3, :3]
            estimated_pose_in_UAV_frame[:3] = np.dot(rotation_matrix, estimated_pose[:3])
            expected_pose_in_UAV_frame[:3]  = np.dot(rotation_matrix, expected_pose[:3])
                
            # PI error control
            error = expected_pose_in_UAV_frame - estimated_pose_in_UAV_frame
            error[3] = (error[3] + np.pi) % (2 * np.pi) - np.pi
            self.land_profile.c_error += error
            delta_v_d = np.multiply(self.parameters_matrix[:,2],error) + np.multiply(self.parameters_matrix[:,3],self.land_profile.c_error)

            v_desired = v_k_in_UAV_frame + delta_v_d
            v_desired = np.sign(v_desired) * np.minimum(np.abs(v_desired), self.parameters_matrix[:, 0])
            
            # Confirm Landing only when error is bellow a threshold
            if ((delta_t > 2*self.delta_t1[2] + self.delta_t2[2]) and (np.linalg.norm(error[0:2]) <= 0.2)):
                self.publish_vel_command(np.zeros_like(v_desired))
                self.finish_landing() 
                return
            
            self.publish_vel_command(v_desired)

    def finish_landing(self):
        rospy.wait_for_service('set_flight_mode')
        try:
            flightModeService = rospy.ServiceProxy('set_flight_mode', SetFlightMode)
            response = flightModeService(2, 0)
            print("landing service response: " + str(response))
        except rospy.ServiceException as e:
            print("Service set_flight_mode call failed: %s." % e)
        
        self.land_profile.delete_profile_instance()
        self.control_stage = 0
        
    def publish_vel_command(self, v_desired):
        # PUBLISH COMMANDS IN BODY FRAME -> IN THE APM_CONFIG yaml FILE ADD :
        # Just update the https://github.com/mavlink/mavros/blob/master/mavros/launch/px4_config.yaml#L111 to BODY_NED. PX4 does not support BODY_OFFSET_NED frame.
        vel_command = Twist()
        vel_command.linear.x = v_desired[0]
        vel_command.linear.y = v_desired[1]
        vel_command.linear.z = v_desired[2]
        
        vel_command.angular.x = 0
        vel_command.angular.y = 0
        vel_command.angular.z = v_desired[3]
        
        self.pub_commands.publish(vel_command)
