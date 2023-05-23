cmake_minimum_required(VERSION 3.8)
project(roborts_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#######################
## Find dependencies ##
#######################
find_package(OpenCV REQUIRED)
# find_package(ament_cmake_auto REQUIRED)
# ament_auto_find_build_dependencies()
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(auto_aim_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)

###########
## Build ##
###########

add_executable(video_test src/video_pub.cpp)
ament_target_dependencies(video_test rclcpp sensor_msgs camera_info_manager image_transport cv_bridge)
add_executable(message_pub src/message_pub.cpp)
ament_target_dependencies(message_pub rclcpp auto_aim_interfaces sensor_msgs) 
add_executable(energy_test src/energy_test.cpp)
ament_target_dependencies(energy_test rclcpp  image_transport cv_bridge tf2_ros tf2) 

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

#############
## Install ##
#############
install(TARGETS
  video_test
  message_pub
  energy_test
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
