# ranger-ws
enable CAN bus each time the robot reboots
```
cd ranger_ws
source install/setup.bash
sudo bash / src/ranger_ros2/ranger_bringup/scripts/bringup_can2usb.bash
```

launch the lidar driver, camera driver, lidar transform publisher, odom to tf publisher, and ranger base wrapper 

```
ros2 launch ranger_ctrl ranger_bringup.launch.py
```


ros2 topic descriptions: 

# /actuator_state
State of the 8 actuators on the robot.

Type: ranger_msgs/msg/ActuatorStateArray

Example message: 

```
- id: 7 
  motor: 
    rpm: 0 
    current: 0.0 
    pulse_count: 0 
  driver: 
    driver_voltage: 49.20000076293945 
    driver_temperature: 28.0 
    motor_temperature: 26.0
    driver_state: 0
```


 where No. 0 is the right front wheel motor, No. 1 is the right rear wheel motor, No. 2 is the left rear wheel motor, No. 3 is the left front wheel motor, No. 4 is the right front steering motor, No. 5 is the right rear steering motor, No. 6 is the left rearsteering motor, and No. 7 is the left front steering motor.

 # /battery_state
 State of the battery 
 
 Type: sensor_msgs/msg/BatteryState

 Example message: 

```
 header:
   stamp:
     sec: 1748899754
     nanosec: 526543712
   frame_id: ''
 voltage: 498.0
 temperature: 25.100000381469727
 current: -0.30000001192092896
 charge: .nan
 capacity: .nan
 design_capacity: .nan
 percentage: 96.0
 power_supply_status: 0
 power_supply_health: 0
 power_supply_technology: 2
 present: false
 cell_voltage: []
 cell_temperature: []
 location: ''
 serial_number: ''
```

# /motion_state
Motion state of the robot.

Type: ranger_msgs/msg/MotionState

Example message:

```
header:
  stamp:
    sec: 1748899958
    nanosec: 958790685
  frame_id: ''
motion_mode: 3
```

where motion_mode = 0 means the robot is in dual ackermann steering mode, motion_mode = 1 means the robot's wheels are parallel, motion_mode = 2 means the robot is in spinning mode, motion_mode = 3 means the robot is parked, motion_mode = 4 means the robot is in side slip mode. 


# /rc_state
States of controller switches and joysticks.

Type: ranger_msgs/msg/RCState

Example message: 
```
swa: 0
swb: 2
swc: 1
swd: 0
stick_right_v: 0
stick_right_h: 0
stick_left_v: 0
stick_left_h: 0
var_a: 0
```



# /system_state
Type: ranger_msgs/msg/SystemState

Example message: 
```
header:
  stamp:
    sec: 1748900279
    nanosec: 918745888
  frame_id: ''
vehicle_state: 0
control_mode: 1
error_code: 0
battery_voltage: 50.900001525878906
motion_mode: 0
```

where
```
uint8 VEHICLE_STATE_NORMAL = 0
uint8 VEHICLE_STATE_ESTOP = 1
uint8 VEHICLE_STATE_EXCEPTION = 2

uint8 CONTROL_MODE_RC = 0
uint8 CONTROL_MODE_CAN = 1
```


### Modifications to the package launch file 
commented out the area file path as it was giving issues with tracking\
commented out velodyne's laserscan node launch, using the ros2 package pointcloud_to_laserscan package instead.

### Note on pointcloud_to_laserscan
The package p2l subscribes to the topic named `/cloud_in` and doesn't take other topic names, currently getting by with 
```
ros2 run topic_tools relay /velodyne_points /cloud_in
```
TODO: add permanent fix in launch file

### Run online_async SLAM using SLAM toolbox and nav2
```
 ros2 launch slam_toolbox online_async_launch.py slam_params_file:=/home/dliujm/ranger_ws/src/ranger_ctrl/config/mapper_params_online_async.yaml
 ```
save the map with 
```
ros2 run nav2_map_server map_saver_cli -f my_map
```

to load a saved map and visualize (publish to /map)
```
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=/yaml/file/path
```
in a second terminal run
```
ros2 lifecycle set /map_server configure 
ros2 lifecycle set /map_server activate
```
to run localization only (this will activate a map server)
```
ros2 launch nav2_bringup localization_launch.py use_sim_time:=False map:=/path/to/map/yaml/file 
```
give initial pose in rviz or foxglove studio \
to run the navigation stack (requires simultaneously running localization node )
```
ros2 launch nav2_bringup bringup_launch.py use_sim_time:=False map:=/path/to/map/yaml/file params_file:=/path/to/params/yaml/file
```
use rviz of foxglove to pushlish a 2d pose to the topic `/goal_pose` and the node will publish twist commands to `/cmd_vel`


### run rtabmap SLAM

2D
```
ros2 launch rtabmap_launch rtabmap.launch.py   rtabmapviz:=false rtabmap_viz:=false rviz:=false use_rviz:=false   rgb_topic:=/zed/zed_node/rgb/image_rect_color   depth_topic:=/zed/zed_node/depth/depth_registered   camera_info_topic:=/zed/zed_node/left/camera_info   rgb_image_transport:=compressed   depth_image_transport:=compressedDepth   frame_id:=base_link   approx_sync:=true   scan_from_depth:=true   rtabmap_args:="--delete_db_on_start" 
  ```

  take odom from robot odom 
  ```
  odom_topic:=/odom
  ```

  use visual odom
  ```
  visual_odometry:=true
  ```

  3D
  ```
  ros2 launch rtabmap_launch rtabmap.launch.py rtabmapviz:=false rtabmap_viz:=false rviz:=false use_rviz:=false rgb_image_transport:=compressed depth_image_transport:=compressedDepth frame_id:=base_link approx_sync:=false   rtabmap_args:="--delete_db_on_start" stereo:=true left_image_topic:=/zed/zed_node/left/image_rect_color right_image_topic:=/zed/zed_node/right/image_rect_color left_camera_info_topic:=/zed/zed_node/left/camera_info right_camera_info_topic:=/zed/zed_node/right/camera_info odom_topic:=/odom visual_odom:=false wait_for_transform:=1.0
  ```

  Launch ZED example (seems to give the best results). Example uses `subscribe_rgbd:=true`, `wait_imu_to_init:=true`, uses imu data and rgbd odom. Also changes ZED's grab resolution, making the image very blurry, but makes SLAM run a lot faster. 
  ```
   ros2 launch rtabmap_examples zed.launch.py camera_model:=zedm
  ```

  To launch rtabmap 3D using lidar, use 
  ```
  ros2 launch ranger_ctrl rtabmap_lidar.launch.py lidar_topc:=/velodyne_points database_path:=/path/to/database frame_id:=base_link rtabmapviz:=false rtabmap_viz:=false rviz:=false use_rviz:=false
  ```
  needed to make our own copy of the launch because the rtabmap example doesn't support custom file path and deletes the map on start up every time.

  Localization 
  ```
  ros2 launch rtabmap_launch rtabmap.launch.py \
  database_path:=~/maps/rtabmap_10_10.db \
  localization:=true \
  rtabmapviz:=false rviz:=false \
  frame_id:=base_link odom_frame_id:=odom map_frame_id:=map \
  rgb_topic:=/zed/zed_node/rgb/image_rect_color   depth_topic:=/zed/zed_node/depth/depth_registered  camera_info_topic:=/zed/zed_node/rgb/camera_info rtabmapviz:=false rtabmap_viz:=false rviz:=false use_rviz:=false 
  ```

  Save the 3d map as a mesh from, `database_path`
   parameter. Example `database_path:=maps/lab_map.db`
   Save the map with 
  ```
  rtabmap-export --cloud path/to/database
  ```

  ### KISS SLAM and KISS-ICP Odometry 
```
kiss_slam_pipeline /path/to/rosbag/folder --topic /pointcloud_topic --config /path/to/config.yaml
```



