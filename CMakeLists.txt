cmake_minimum_required(VERSION 2.8.3)
project(kdl_3dof_planar_man)


find_package(catkin REQUIRED COMPONENTS
    cmake_modules
    geometry_msgs
    roscpp
    roslib
    rospy
    sensor_msgs
    std_msgs
)

find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(orocos_kdl REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(3dof_planar_man_node src/3dof_planar_man.cpp)

target_link_libraries(3dof_planar_man_node ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${Eigen3_LIBRARIES} ${orocos_kdl_LIBRARIES})
