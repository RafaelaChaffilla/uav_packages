#!/usr/bin/env python

import rospy
from mavros_msgs.srv import SetMode
from command_uav.srv import SetFlightMode
from set_follow_mode import FollowMode
from set_prec_land_mode import PrecLandCtrl
from set_take_off_mode import TakeOffMode
from set_ekf_source import EkfSource
from set_follow_pid_mode import FollowPIDMode

class SetHolyMode():
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('set_holy_mode', anonymous=True)
        
        # Service
        s = rospy.Service('set_flight_mode', SetFlightMode, self.set_flight_mode)
    
        # Init all classes
        self.follow_mode    = FollowMode()
        self.prec_land_mode = PrecLandCtrl()
        self.takeoff_mode   = TakeOffMode()
        self.ekf_src        = EkfSource()
        self.follow_pid_mode    = FollowPIDMode()
        self.current_mode   = None
        
        # Definitions
        self.mode_dict = {
            "GUIDED": 1,
            "LAND":    2
        }

    def set_mavros_mode(self, mode):  
        '''
            Mode: ['GUIDED', 'LAND']
        '''
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', SetMode)
            response = flightModeService(custom_mode=mode)
            if response:
                self.current_mode = self.mode_dict[mode]
            return response.mode_sent
        except rospy.ServiceException as e:
            print("Service mavros/set_mode call failed: %s. %s mode could not be set." % (e, mode))
            return False
        
    def set_flight_mode(self, req):
        if req.mode == req.FOLLOW:
            if self.current_mode != "GUIDED":
                self.success = self.set_mavros_mode("GUIDED")
            
            self.follow_mode.activate(req.altitude)
            self.prec_land_mode.deactivate()
            self.follow_pid_mode.deactivate()
            self.takeoff_mode.deactivate()
            
            rospy.loginfo("Set Flight Mode: Switched to Follow") 
            
            return self.success

        elif req.mode == req.PRECISION_LAND:
            if self.current_mode != "GUIDED":
                self.success = self.set_mavros_mode("GUIDED")
                
            self.prec_land_mode.activate()
            self.follow_mode.deactivate()
            self.follow_pid_mode.deactivate()
            self.takeoff_mode.deactivate()
            
            rospy.loginfo("Set Flight Mode: Switched to Precision Landing") 
            
            return self.success
            
        elif req.mode == req.LAND:
            if self.current_mode != "LAND":
                self.success = self.set_mavros_mode("LAND")
            
            self.prec_land_mode.deactivate()
            self.follow_mode.deactivate()
            self.follow_pid_mode.deactivate()
            self.takeoff_mode.deactivate()
            
            rospy.loginfo("Set Flight Mode: Switched to Land") 
            
            return self.success
        
        elif req.mode == req.TAKE_OFF:
            if self.current_mode != "GUIDED":
                self.success = self.set_mavros_mode("GUIDED")
            
            self.prec_land_mode.deactivate()
            self.follow_mode.deactivate()
            self.follow_pid_mode.deactivate()
            self.takeoff_mode.activate(req.altitude)
            
            rospy.loginfo("Set Flight Mode: Switched to Take-Off") 
            
            return self.success
        
        elif req.mode == req.SET_EKF_SRC:
            self.ekf_src.change_src(req.altitude)
            rospy.loginfo("Set Flight Mode: Changed EKF Source") 
            
            return True
        
        elif req.mode == req.STOP:
            self.prec_land_mode.deactivate()
            self.follow_mode.deactivate()
            self.follow_pid_mode.deactivate()
            self.takeoff_mode.deactivate()
            rospy.loginfo("Set Flight Mode: Changed EKF Source") 
            
            return True
        
        if req.mode == req.FOLLOW_PID:
            if self.current_mode != "GUIDED":
                self.success = self.set_mavros_mode("GUIDED")
            
            self.follow_pid_mode.activate(req.altitude)
            self.follow_mode.deactivate()
            self.prec_land_mode.deactivate()
            self.takeoff_mode.deactivate()
            
            rospy.loginfo("Set Flight Mode: Switched to Follow") 
            
            return self.success

        else:
            rospy.loginfo("Set Flight Mode: Requested mode not recognized")
             
            return False
        
if __name__ == '__main__': 
    try:
        set_mode = SetHolyMode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
