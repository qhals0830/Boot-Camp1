cmake_minimum_required(VERSION 2.8.3)
project(tutorial_msgs)


find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  roscpp
  rospy
)


 add_message_files(
   FILES
  Chassis0.msg
  Chassis1.msg
  Chassis2.msg
  gps.msg

 )


 generate_messages(
   DEPENDENCIES
   std_msgs
 )

catkin_package(
  CATKIN_DEPENDS
    message_runtime
    std_msgs
)


