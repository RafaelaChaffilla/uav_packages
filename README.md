
# uav_packages

UAV ROS packages for UAV-UGV collaboration for warehouse inventory missions found in https://github.com/RafaelaChaffilla/aerial_ground_robotic_system_for_warehouse_inventory. These packages were designed for ROS Noetic.
A description of each package is given in [Documentation](#documentation).

### Installation:

Perform the following steps to install these packages in your catkin_workspace:

    cd catkin_ws/src
    
    git clone --recurse-submodules https://github.com/RafaelaChaffilla/uav_packages.git
    
    cd ../
    
    catkin_make

  
The complete system is integrated in the *holybro* package, particularly in the launch files:

 1. **apm_icarus.launch**: Launches the MAVROS node in order to communicate with ArduPilot. To understand the configuration, please refer to [http://wiki.ros.org/mavros#Usage](http://wiki.ros.org/mavros#Usage).
 2. **spawn_model.launch**: Launches the UAV model in Gazebo and sets ArduPilot's EKF origin (needed when no GPS is used). 
	 - **Parameters**:
		 - *robot_name*:  name of robot, used for prefixes;
		 - *init_x, init_y, init_z, init_th:* UAV's initial x, y, z and yaw positions in map frame;
		 - *set_ekf_origin*: Boolean indicating if ArduPilot's EKF origin needs to be set, i.e., if there are no GPS sensors setting the EKF origin;
		 - *uav_model_path*: Path to the UAV's gazebo model to be spawned.
 3. **spawn_uav_icarus.launch**: Launches the ArUco-based localization nodes and UAV's command nodes. Intended for use real-world testing when there is no communication between the UGV/patterns and the UAV. 
	  - **Parameters**:
		 - *robot_name*:  name of robot, used for prefixes;
		 - *init_x, init_y, init_z, init_th:* UAV's initial x, y, z and yaw positions in map frame;
 4. **spawn_uav.launch**: Launches the ArUco-based localization nodes, UAV's command nodes and UAV-UGV collaboration node. Additionally it launches a script, "spawn_uav_model.py", that waits for the UGV to indicate its position in the map frame to then spawn the UAV model through the *spawn_model.launch* file. Intended for simulation or real-world tests when there is communication with the UGV.
	 - **Parameters**:
		 - *robot_name*:  name of robot, used for prefixes;
		 - *init_x, init_y, init_z, init_th:* UAV's initial x, y, z and yaw positions in map frame;
		 - *ugv_pose_topic*: String indicating the topic where the UGV's position in the map frame is published;
		 - *launch_all*: indicates if the UAV model should be spawned to gazebo when the file is launched instead of waiting for the UGV model to be spawned first;
		 - *corridors_to_visit*: Array with the list of the corridor's IDs that should be visited during the warehouse inventory mission.


### Documentation<a name="documentation"></a>:
TODO.
