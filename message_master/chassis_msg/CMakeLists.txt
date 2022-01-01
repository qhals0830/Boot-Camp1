cmake_minimum_required(VERSION 2.8.3)
project(chassis_msg)


find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  roscpp
)


 add_message_files(
   FILES
  LOG_BYTE0.msg
  LOG_BYTE1.msg
  LOG_BYTE2.msg
  LOG_BYTE3.msg

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


