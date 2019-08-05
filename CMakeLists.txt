cmake_minimum_required(VERSION 2.8.3)
project(new_quadruped_model_kp)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  gazebo_dev
  roscpp
  std_msgs
  control_toolbox
  controller_manager
  controller_interface
  hardware_interface
  transmission_interface
  pluginlib
  joint_limits_interface
  urdf
  angles
  gazebo_ros_control
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    std_msgs
    controller_manager
    control_toolbox
    pluginlib
    hardware_interface
    transmission_interface
    joint_limits_interface
    urdf
    angles
  INCLUDE_DIRS include
)

find_package(roslaunch)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    )
# add_library(single_joint_control
#     src/single_joint_control.cpp
#   )
# target_link_libraries(single_joint_control ${catkin_LIBRARIES})

add_library(robot_hardware_interface
    lib/robot_hardware_interface.cpp
  )
target_link_libraries(robot_hardware_interface ${catkin_LIBRARIES})

add_library(single_joint_hardware_interface
    src/single_joint_hardware_interface.cpp
  )
# target_link_libraries(single_joint_control ${catkin_LIBRARIES})

add_library(gazebo_sim
    lib/gazebo_sim.cpp
  )
target_link_libraries(gazebo_sim ${catkin_LIBRARIES})

add_library(gazebo_ros_control_plugin_kp
    lib/gazebo_ros_control_plugin_kp.cpp
  )
target_link_libraries(gazebo_ros_control_plugin_kp ${catkin_LIBRARIES})

add_executable(control_single_joint_node src/control_single_joint.cpp)
target_link_libraries(control_single_joint_node
  ${catkin_LIBRARIES}
)

add_library(single_joint_controller src/single_joint_controller.cpp)
target_link_libraries(single_joint_controller
  ${catkin_LIBRARIES}
)

foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
