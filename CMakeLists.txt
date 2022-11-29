cmake_minimum_required(VERSION 3.0.2)

project(bellows_ik)

find_package(catkin REQUIRED COMPONENTS
  rospy
  joy
  bellows_arm
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS rospy joy bellows_arm
)