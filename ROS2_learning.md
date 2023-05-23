##  一、参数设置

#### 1.前端参数

src/cartographer_core/cartographer/configuration_files

trajectory_builder_2d.lua文件

ros2 run camera_calibration cameracalibrator --size 5x8 --square 0.27 image:=/image_raw camera:=/narrow_stereo


#### 2.后端参数

src/cartographer_core/cartographer/configuration_files

pose_graph.lua文件



#### 3.cartographer_ros参数设置

src/cartographer_core/cartographer_ros/cartographer_ros/configuration_files

backpack_2d.lua文件

## 二、学习ros2

#### 1.日志调试功能

```
ros2 run package-name node-name --ros-args --log-level debug
```

#### 2.It’s good practice to run `rosdep` in the root of your workspace (`ros2_ws`) to check for missing dependencies before building:

```
rosdep install -i --from-path src --rosdistro galactic -y
```

#### 3.显示tf2-tree

```
ros2 run tf2_tools view_frames
```

#### 4.杀死gazebo进程

```
killall gzserver
killall gzclient
```

#### 5.键盘映射

```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

#### 6.nav2地图相关

```
ros2 run nav2_map_server map_saver_cli --help

Usage:
  map_saver_cli [arguments] [--ros-args ROS remapping args]

Arguments:
  -h/--help
  -t <map_topic>
  -f <mapname>
  --occ <threshold_occupied>
  --free <threshold_free>
  --fmt <image_format>
  --mode trinary(default)/scale/raw
 
 保存地图
  ros2 run nav2_map_server map_saver_cli -t map -f fishbot_map
 
 打开保存的地图
  ros2 run nav2_map_server map_server --ros-args --param yaml_filename:=map/fishbot_map.yaml
```

#### 7.静态转换 Publisher
```
发布静态TF变换
ros2 run tf2_ros static_transform_publisher 0.1 0 0.2 0 0 0 base_link base_laser 

获取传感器数据对应的frame_id
ros2 topic echo /scan | grep frame_id

输出两个TF坐标系之间的变换关系
ros2 run tf2_ros tf2_echo base_link base_laser
输出结果如下：
At time 0.0
- Translation: [0.100, 0.000, 0.200]
- Rotation: in Quaternion [0.000, 0.000, 0.000, 1.000]

```

#### 8.相机标定
```
ros2 run camera_calibration cameracalibrator --size 7x9 --square 0.20 image:=/my_camera/image_raw camera:=/my_camera
ros2 run camera_calibration cameracalibrator --size 11x8 --square 0.015 image:=/image_raw camera:=/my_camera

ros2 run camera_calibration cameracalibrator --size 11x8 --square 0.035 image:=/image_raw camera:=/my_camera
```
数据保存到 "/tmp/calibrationdata.tar.gz"   
tar -xvf calibration.tar.gz


```
// 提高代码效率
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

//监视所有tf关系
ros2 run tf2_ros tf2_monitor
```

//
ros2 topic echo -f /processor/target

// foxglove 
ros2 launch rosbridge_server rosbridge_websocket_launch.xml


gsettings set org.gnome.settings-daemon.plugins.media-keys max-screencast-length 300




