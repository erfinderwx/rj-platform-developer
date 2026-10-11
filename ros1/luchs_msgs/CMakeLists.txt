cmake_minimum_required(VERSION 3.10)
project(luchs_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs geometry_msgs)

add_message_files(FILES ChassisState.msg SafetyState.msg LampCommand.msg DriveCommand.msg)
add_service_files(FILES SetMotionModel.srv SetPayloadPower.srv SetSpeedLimit.srv)
generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs geometry_msgs)
