#!/usr/bin/env python

import rospy
import numpy as np
import actionlib
from geometry_msgs.msg import PoseStamped, Quaternion, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from actionlib_msgs.msg import GoalStatusArray, GoalStatus
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseActionGoal
from mavros_msgs.msg import State
from command_uav.srv import SetFlightMode

class Path():
    def __init__(self):
        # Initialize Node
        rospy.init_node('goal_points')
        
        self.init_x = rospy.get_param("~init_x", 0.0)
        self.init_y = rospy.get_param("~init_y", 0.0)
        self.ugv_pose_topic = rospy.get_param("~ugv_pose_topic", "rovid/ekf_ugv")
        
        # Parameters
        self.corridors_to_visit = rospy.get_param("~corridors_to_visit", [10])
        self.current_ugv_pose = None
        self.current_uav_pose = None
        self.current_goal_status = None
        self.current_uav_state = State()
        self.current_uav_state.connected = False
        self.current_uav_state.system_status = 0
        
        # Subscribers
        rospy.Subscriber(self.ugv_pose_topic, Odometry, self.ugv_position)
        rospy.Subscriber("/rovid/move_base/status", GoalStatusArray, self.move_base_callback)
        rospy.Subscriber("mavros/vision_pose/pose_cov", PoseWithCovarianceStamped, self.uav_position)
        rospy.Subscriber("mavros/state", State, self.uav_status_callback)
        
        # Publishers
        self.move_base_pub = rospy.Publisher("/rovid/move_base/goal", MoveBaseActionGoal, queue_size=10)
        
        # Initialize Movement
        self.initialize_journey()
    
    def extract_corridor_positions(self):
        '''
            Gets the entry and leaving positions of each wanted corridor
        '''
        self.corridor_points = []
        for corridor in self.corridors_to_visit:
            point_1 = rospy.get_param("corr_" + str(corridor) + "/init", None)
            point_2 = rospy.get_param("corr_" + str(corridor) + "/end", None)
            print("points 1 and 2: " + str(point_1) + " " + str(point_2))
            if ((point_1 is None) or (point_2 is None)):
                rospy.loginfo("Couldn't find corridor %i in parameters.", corridor)
                continue

            self.corridor_points.append(point_1)
            self.corridor_points.append(point_2)
        
    def decide_corridor(self):
        '''
            decides which corridor to visit next based on the closest point to the UGV's current position
        '''
        if len(self.corridor_points) == 0:
            rospy.loginfo("Queen chegou ao fim")
            return None, None
        
        while self.current_ugv_pose is None:
            continue
        
        ugv_pose = np.array([self.current_ugv_pose.x, self.current_ugv_pose.y])
        
        distances = [np.linalg.norm(ugv_pose - np.array(candidate)) for candidate in self.corridor_points]

        # Get the index of the closest 2D array
        closest_index = np.argmin(distances)
        corridor_index = closest_index // 2
        init_point = self.corridor_points[closest_index]
        if((closest_index-1)// 2  == corridor_index):
            c = closest_index
            end_point = self.corridor_points[closest_index - 1]
        elif((closest_index+1)// 2  == corridor_index):
            c = closest_index + 1
            end_point = self.corridor_points[closest_index + 1]
        else:
            rospy.loginfo("Something went wrong when deciding corridor")
            return None, None

        rospy.loginfo("Going to corridor " + str(self.corridors_to_visit[corridor_index]))
        
        self.corridors_to_visit.pop(corridor_index)
        self.corridor_points.pop(c)
        self.corridor_points.pop(c-1)
        
        if init_point[0] > end_point[0] + 0.1: # make ugv stop when it is inside the corridor and not just the corner
            init_point[0] -= 0.5 
            end_point[0]  += 0.5 
        elif init_point[0] < end_point[0] + 0.1:
            init_point[0] += 0.5 
            end_point[0]  -= 0.5 
        elif init_point[1] > end_point[1] + 0.1:
            init_point[1] -= 0.5 
            end_point[1]  += 0.5 
        elif init_point[1] < end_point[1] + 0.1:
            init_point[1] += 0.5 
            end_point[1]  -= 0.5 
        
        print(init_point, end_point)
        return init_point, end_point
    
    def send_goal_command(self, goal_point, goal_id, orientation):
        goal = MoveBaseActionGoal()
        goal.header.frame_id = "map"
        goal.header.stamp = rospy.Time.now()
        
        goal.goal_id.id = str(goal_id)
        
        goal.goal.target_pose.header = goal.header
        goal.goal.target_pose.pose.position.x = goal_point[0]
        goal.goal.target_pose.pose.position.y = goal_point[1]
        
        goal.goal.target_pose.pose.orientation = orientation
        
        self.move_base_pub.publish(goal)
    
    def check_status(self, goal_id):
        while self.current_goal_status is None:
            continue
        
        for arr in self.current_goal_status.status_list:
            if arr.goal_id.id == str(goal_id):
                return arr.status
            
        return GoalStatus.LOST
    
    def uav_take_off(self, height):
        print("changed to take off ")
        rospy.wait_for_service('set_flight_mode') 
        response = False
        rate = rospy.Rate(1)
        
        # Wait for Take-off command to be set correctly
        while not response:
            try:
                flightModeService = rospy.ServiceProxy('set_flight_mode', SetFlightMode)
                response = flightModeService(4, height)
                print("take off service response: " + str(response))
            except rospy.ServiceException as e:
                print("Service set_flight_mode call failed: %s." % e)
            
            rate.sleep()
        
        # wait for Take-off to finish
        old_height = 0
        while not (self.current_uav_pose.z > height - 0.5 and abs(self.current_uav_pose.z - old_height) < 0.1):
            old_height = self.current_uav_pose.z
            rate.sleep()
            
        return True
        
    def uav_follow(self, height):
        print("changed to follow ")
        rospy.wait_for_service('set_flight_mode')
        response = False
        rate = rospy.Rate(1)
        
        # Wait for Take-off command to be set correctly
        while not response:
            try:
                flightModeService = rospy.ServiceProxy('set_flight_mode', SetFlightMode)
                response = flightModeService(1, height)
                print("landing service response: " + str(response))
            except rospy.ServiceException as e:
                print("Service set_flight_mode call failed: %s." % e)
            
            rate.sleep()
        
        return True
    
    def uav_land(self):
        rospy.wait_for_service('set_flight_mode')
        response = False
        rate = rospy.Rate(1)
        
        # Wait for uav to complete following the uav
        while np.linalg.norm(np.array([self.current_uav_pose.x - self.current_ugv_pose.x + self.init_x, self.current_uav_pose.y - self.current_ugv_pose.y + self.init_y])) > 0.3:
            print("waiting to land")
            print(np.linalg.norm(np.array([self.current_uav_pose.x - self.current_ugv_pose.x + self.init_x, self.current_uav_pose.y - self.current_ugv_pose.y + self.init_y])))
            rate.sleep()
        
        print("going to land")
        # Wait for Land command to be set correctly
        while not response:
            try:
                flightModeService = rospy.ServiceProxy('set_flight_mode', SetFlightMode)
                response = flightModeService(3, 0)
                print("landing service response: " + str(response))
            except rospy.ServiceException as e:
                print("Service set_flight_mode call failed: %s." % e)
            
            rate.sleep()
        
        # wait for landing to finish
        while self.current_uav_state.armed:
            rate.sleep()
        
        rospy.loginfo("UAV has landed :)")
        return True
      
    def travel_corridor(self, init_point, end_point):
        num_segments = 4  #MUST BE AN EVEN NUMBER
        if init_point[0] - end_point[0] < -0.1:
            # corridor direction in -x
            direction = 0
            orientation = Quaternion(0,0,0,1)
        elif init_point[1] - end_point[1] < -0.1:
            # corridor direction in -y
            direction = 1
            orientation = Quaternion(0, 0, 0.7071081, 0.7071055)
        elif init_point[0] - end_point[0] > 0.1:
            # corridor direction in +x
            direction = 0
            orientation = Quaternion(0, 0, -1, 0)
        elif init_point[1] - end_point[1] > 0.1:
            # corridor direction in +y
            direction = 1
            orientation = Quaternion(0, 0, -0.7071081, 0.7071055)
        
        goal_id = 0
        segment_len = (end_point[direction] - init_point[direction])/(num_segments-1)
        rate = rospy.Rate(0.1)
        
        current_goal = init_point
        for i in range(int(num_segments/2)):
            # UAV will Take-Off            
            goal_id += 1
            self.send_goal_command(current_goal, goal_id, orientation)
            
            goal_status = self.check_status(goal_id)
            while goal_status != GoalStatus.SUCCEEDED:
                goal_status = self.check_status(goal_id)
                if (goal_status == GoalStatus.PENDING or goal_status == GoalStatus.ACTIVE):
                    continue
                else:
                    goal_id += 1
                    self.send_goal_command(current_goal, goal_id, orientation)
                    rospy.loginfo("GOal status has flopped. it is: %s\n", goal_status)
                rate.sleep()
            
            rospy.loginfo("Reached Goal point!!!\n")
            
            # New goal
            current_goal[direction] += segment_len
            
            take_off = self.uav_take_off(5)
            follow = self.uav_follow(5)

            # UAV will Land
            goal_id += 1
            self.send_goal_command(current_goal, goal_id, orientation)
            
            goal_status = self.check_status(goal_id)
            while goal_status != GoalStatus.SUCCEEDED:
                goal_status = self.check_status(goal_id)
                if (goal_status == GoalStatus.PENDING or goal_status == GoalStatus.ACTIVE):
                    continue
                else:
                    goal_id += 1
                    self.send_goal_command(current_goal, goal_id, orientation)
                    rospy.loginfo("GOal status has flopped. it is: %s\n", goal_status)
                rate.sleep()
                
            rospy.loginfo("Reached Goal point!!!\n")   
            land = self.uav_land()
            
            current_goal[direction] += segment_len
            
            
    def initialize_journey(self):
        self.extract_corridor_positions()
        
        # Only start journey when UAV is connected
        rate = rospy.Rate(0.1)
        while(self.current_uav_state.connected==False or self.current_uav_state.system_status < 3):
            rate.sleep()
            
        while len(self.corridor_points) > 0:
            init_point, end_point = self.decide_corridor()
            if init_point is None:
                break
            self.travel_corridor(init_point, end_point)
            
        rospy.loginfo("Trip ended, thank you for flying with ryanair :)")
               
    # Callbacks 
    def ugv_position(self, msg):
        self.current_ugv_pose = msg.pose.pose.position
    
    def uav_position(self, msg):
        # self.current_uav_pose = msg.pose.position
        self.current_uav_pose = msg.pose.pose.position
           
    def move_base_callback(self, msg):
        self.current_goal_status = msg
        
    def uav_status_callback(self, msg):
        self.current_uav_state = msg
               
if __name__ == '__main__': 
    try:
        l = Path()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
