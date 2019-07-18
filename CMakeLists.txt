cmake_minimum_required(VERSION 2.8.3)
project(new_quadruped_model_kp)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
    roscpp
    rospy)

catkin_package()

find_package(roslaunch)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    )

add_executable(control_single_joint_node src/control_single_joint.cpp)
target_link_libraries(control_single_joint_node
  ${catkin_LIBRARIES}
)

foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
