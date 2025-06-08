# uav_packages

### Description

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
		 - *land_accuracy*: Value, in meters, of the maximum allowed offset from the pattern's center for the UAV landing;
		 - *launch_all*: indicates if the UAV model should be spawned to gazebo when the file is launched instead of waiting for the UGV model to be spawned first;
		 - *corridors_to_visit*: Array with the list of the corridor's IDs that should be visited during the warehouse inventory mission.


### Documentation<a name="documentation"></a>:
 -  **command_uav**: Package that creates a service to send personalized Take-off, follow and landing commands to ArduPilot. The service can be called by  `rosservice call /set_flight_mode $mode $input` where the modes and inputs can be found in the srv/SetFlightMode.srv file. 
For Take-Off, ArduPilot will try to take-off vertically to the altitude set in the *input* variable. In follow mode, the UAV will follow the *x, y* and *yaw* global position published in the *ugv_pose_topic* parameter and the altitude set in the *input* variable. For the land mode, the UAV will land on the center of the UGV/ArUco pattern within the accuracy defined in *land_accuracy*. 
Additionally, the SET_EKF_SRC mode allows the change in ArduPilot's EKF source, where the *input* variable indicates which source to use.
 - **pose_from_aruco**: This package calculates the UAV's position in the map frame based on the ArUco markers detection and UGV's estimated position in the map. The name of the map, UAV, UGV and camera frames and UGV pose and UAV orientation topics can be defined in the launch files found in the *holybro* package. 
A service is also created to account for the offsets between the local position estimate based on GPS and ArUco-based localization to allow for more seamless transitions between EKF sources. 
Finally, the *ugv_substitute_publisher.py* file publishes a fixed transform between the map and the pattern frame and extracts the real pattern orientation based on the UAV's compass and visual information about the orientation. This is designed for the UAV tests where there is no communication with the UGV/pattern frame.
 - **ugv_state_machine**: Package responsible for controlling the UAV-UGV coordination during the warehouse inventory missions. The robotic system will move to the closest desired corridor, the UAV will take-off, the UGV will travel a quarter of the corridor while the UAV follows, the UAV will land and the process is repeated. The coordination depends on the */move_base/status* topic from the UGV's *move_base* package, used for the UGV path planning and control. 
	 - **Parameters**;
		 - *init_x, init_y:* UAV's initial x and y positions;
		 - *corridors_to_visit*:  Array with the list of the corridor's IDs that should be visited during the warehouse inventory mission;
		 - *config_file*: Name of the .yaml file containing the IDs and entry positions of the corridors of the warehouse. The files are expected to be stored in the *config* folder. An example of this file for the used simulated warehouse world is in the same folder. 