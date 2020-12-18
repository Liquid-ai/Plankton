## 0.6.1

### General

- Fixed the issue affecting the CPC sensor and the custom battery plugin when loaded

## 0.6.0

### General

- Migration to ROS 2 Foxy
- Added support for Gazebo 11
- Files parsed by Xacro are now directly output as a string and loaded as a node parameter in the various launch files
- Known issue: teleop shows unexpected behaviour with the trajectory control package's controllers, as well as the rov_mb_sm controller / rov_nl_pid_controller
- Known issue: the CPC sensor and the custom battery plugins crash when loaded
- Known issue: DVL and camera sensor won't show in Rviz
- Known issue: The subprocess based sim time request fails when the plankton sim time node is launched within the same launch file. A simple fix is to start the sim time node first and separately

### uuv_thruster_manager

- [**thruster_manager.py**] The thruster manager now gets and parse the urdf robot description from a topic instead of a file 

## 0.5.3

### General

- Fixed a problem where the simulated time request was not correctly handled while using subprocess and failed
- Fixed many issues occuring in the trajectory control and control utils
- Known issues: teleop shows unexpected behaviour with the trajectory control package's controllers, as well as the rov_mb_sm controller / rov_nl_pid_controller when station is keeping on.
- Known issues: the CPC sensor and the custom battery plugins crash when loaded
- Increased the timeout while waiting for the tf transforms in the thruster_manager
- Reverted yaml format for file-based waypoints to plain old yaml as the values are not read as ROS parameters

### plankton_utils

- Increased the default timeout for sim time request to 10s

### uuv_control_utils

- [**start_circular_trajectory.launch**] Changed incorrect arg name `starting_time`

### uuv_thruster_manager

- Increased the default timeout while waiting for tf transforms to 10s

## 0.5.2

### uuv_sensor_ros_plugins

- Updated deprecated remapping syntax warnings occuring in URDF files

## 0.5.1

### General

- Fixed incorrect dependencies in package.xml

### uuv_assistants

- [**publish_vehicle_footprint**, **unpause_simulation**] Syntax error fixes

### uuv_control_utils

- Typo in some launch files
- Fixed the crashes occuring in the disturbance services files
- [**apply_link_wrench.launch**] Changed file name from *apply_body_wrench* for consistency. A param was added to change the service namespace
- [**apply_link_wrench.py**] Previously, the user was not informed when a service request failed
- [**set_thruster_output_efficiency**] Fixed an incorrect loop condition

### uuv_trajectory_control

- [**dp_pid_controller_base**] Added a missing return instruction

### uuv_gazebo_ros_plugins

- [**test files**] A misleading function was renamed

### uuv_teleop

- [**finned_uuv_teleop**, **vehicle_keyboard_teleop**, **vehicle_teleop**] Sim time was not handled correctly

### uuv_world_ros_plugins

- [**GaussMarkovProcess**] Changed model validity criterion from `min must be < max` to `min must be <= max`

## 0.5.0

## General

- General migration of UUV simulator to ROS 2 Eloquent  
- Gazebo plugins now instantiate 1 ROS node per plugin  
- Parameters are auto declared in each node  
- Sim time: current solution implies that each Plankton node asks at startup for the use or not of use_sim_time to a global node created when initializing gazebo  
- Some occurrences can remain, but tf_prefix is not used anymore
- The provided Gazebo world files were modified to load the gazebo_ros_state plugin. The gazebo_ros_force_system plugin is also loaded in the launch files initializing gazebo  
- The official Gazebo ros plugins live in their own namespace `/gazebo`  

## Missing from ROS 1

- Currently removed the ability for nodes to respawn if they were parameterized in this way in the launch file  
- Currently removed the ability to stop the ROS 2 system if a node tagged as required in the launch file terminates  
- Currently not migrated `UnderwaterCamera` and `sonar_image sensor plugins`  
- The assistant tools to create new robots are not ported yet  
- Model_spawner has not been ported yet and was replaced with the default entity_spawner of gazebo  


## More detailed information about per package changes

### Plankton_utils  

- New utility package (time conversions, sim time handling and helper functions to manipulate parameters more easily)  

### uuv_assistants

- Updated tf to tf2  
- Migrated launch files to Eloquent  
- Files in the *template* directory were not migrated  
- [**message_to_tf**] Removed multicallback ability  
- [**publish_footprints**] is not functional and needs a rework  
- [**publish_world_models**] Changed service name from `get_model_state` to `get_entity_state`  
- [**set_simulation_timer.launch**] has currently no effect as the launch file is not able to kill the simulation  

### uuv_auv_control_allocator

- [**actuator_manager**] Renamed topic `/${namespace}/${topic_prefix}/${id}/${topic_suffix}` to `/${namespace}/${topic_prefix}/id_${id}/${topic_suffix}` according to ROS 2 naming convention.

### uuv_cascaded_pids

- Replaced dynamic reconfigure from ROS 1 with ROS 2 integrated system. Parameters can be changed using the command line interface *ros2 param get / set*
- Changed python file naming convention to be consistent

### uuv_control_msgs

- [**GetMBSMControllerParams**, **SetMBSMControllerParams**] Changed field `lambda` as it is a reserved keyword in Python
- [**GetPIDParams**, **SetPIDParams**, **SetSMControllerParams**, **GetSMControllerParams**] Turned field names to lowercase names as it is now a required convention

### uuv_control_utils

- Changed yaml syntax for the files used in *disturbance_manager.py* and *send_waypoints.py* due to ROS 2 limitations with sequences. 
Each disturbance now lives in a namespace `d${X}`, where `${X}` is the number of the disturbance.
Each waypoint now lives in a namespace `wp${X}` where `${X}` is the number of the waypoint.
Two example files are provided.
- [**apply_link_wrench**] The file was renamed from *apply_body_wrench* to reflect the name change of the Gazebo ros service .
The service name and message type were modified to `apply_link_wrench` and `ApplyLinkWrench` respectively, according to (https://github.com/ros-simulation/gazebo_ros_pkgs/wiki/ROS-2-Migration:-Body-Wrench-and-Joint-Effort)  
- [**disturbance_manager**, **set_truster_output_efficiency**, **set_thruster_state**] Changed the topic prefix from `/${ns}/thruster/${x}/`  (with `${x}` a numeric id) to `/${ns}/thruster/id_${x}` due to ROS 2 naming conventions.

### uuv_thruster_manager

- All of the test files have been migrated to the python way.
- [**thruster_manager**] Changed topic name in the same way as it is in *uuv_control_utils*
- [**thruster_manager**] Replaced tf occurrences with tf2
- [**thruster_manager**] Stripped first “/” in tf2 frames
- [**thruster_manager**, **thruster_allocator**] The classes now initialize asynchronously. `ready` flag can be used to know if the initialization is done.
- [**yaml files**] Migrated the syntax to ROS 2 and turned the TAM matrix to a 1D list which will be further converted in numpy matrix in the thruster_allocator code due to ROS 2 yaml limitations.
- [**test_thruster_manager_proportional_correct**] Changed topic name in the same way as it is in *uuv_control_utils*  

### uuv_trajectory_control
- Replaced tf with tf2
- [**auv_geometric_tracking_controller**] Changed the topic prefix from `/${ns}/thruster/${x}/`  (with `${x}` a numeric id)  to `/${ns}/thruster/id_${x}` and similarly for `fin` instead of `thruster`, due to ROS 2 naming conventions .

### uuv_descriptions

- Due to several limitations with the integration of xacro in the XML frontend, launch files were rewritten in python
- [**xacro files**]: Changed *libunderwater_ros_plugin* plugin name from `uuv_plugin` to `$(arg namespace)_uuv_plugin`  
- [**launch files**]: Replaced *spawn_model.py* with Gazebo provided *spawn_entity.py*
- [**spawn_model**] The file was not migrated

### uuv_gazebo

- The package was left unchanged and is not usable.

### uuv_gazebo_plugins

- [**FinPlugin**] Changed the topic prefix from `/${ns}/fin/${x}/`  (with `${x}` a numeric id)  to `/${ns}/fin/id_${x}` due to ROS 2 naming conventions
- [**FinPlugin**] TopicPrefix variable does no longer have the final “/”
- [**ThrusterPlugin**] Changed the topic prefix from `/${ns}/thruster/${x}/`  (with `${x}` a numeric id)  to `/${ns}/thruster/id_${x}` due to ROS 2 naming conventions
- [**test_thrusters**] Changed the topic prefix from `/${ns}/fin/${x}/`  (with `${x}` a numeric id)  to `/${ns}/fin/id_${x}` due to ROS 2 naming conventions

### uuv_gazebo_ros_plugins

- Removed Boost dependency
- Test files have been migrated to the python way.
- [**battery_snippets**, **misc**] Added `<ros><namespace/></ros>` element to enforce the namespace of the created ROS node under Gazebo
- [**misc**] Added <ros><namespace/></ros> in the `uuv_joint_state_publisher` element to enforce the namespace of the created ROS node under Gazebo
- [**CustomBatteryConsumerROSPlugin**, **FinROSPlugin**, **UnderwaterObjectROSPlugin**, **ThrusterROSPlugin**] The node does not have namespace
- [**JointStatePublisher**, **LinearBatteryROSPlugin**] The node is started with the provided namespace in the corresponding SDF file.


### uuv_gazebo_worlds

- [**launch files**] The Plankton sim time node is started with every launch file
- [**launch files**] Gazebo is started with the namespace `/gazebo`
- [**launch files**] The `gazebo_ros_force_system plugin` is also loaded in the launch files initializing gazebo
- [**world files**] The Gazebo world files were modified to load the *gazebo_ros_state* plugin.

### uuv_sensor_plugins

- Removed Boost dependency
- All nodes are started with the provided namespace in the corresponding SDF file.
- [**xacro files**] Replaced `<robot_namespace>` with `<ros><namespace></namespace></ros>`
- [**camera_snippets**] Migrated SDF format according to (https://github.com/ros-simulation/gazebo_ros_pkgs/wiki/ROS-2-Migration:-Camera)   
- [**gazebo_ros_image_sonar**, **UnderwaterCameraROSPlugin**] These sensors have not been migrated yet

### uuv_world_ros_plugins

- [**UnderwaterCurrentROSPlugins**] The node is started with the namespace given in the plugin’s tag namespace in the SDF.

### uuv_teleop

- [**finned_uuv_teleop.py**] Changed fin topic name from `${fin_topic_prefix}/${x}/${fin_topic_suffix}` to `${fin_topic_prefix}/id_${x}/${fin_topic_suffix}`  
- [**finned_uuv_teleop.launch**] Change variable `thruster_topic` from `thrusters/0/input` to `thrusters/id_0/input`


