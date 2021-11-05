cmake_minimum_required(VERSION 2.8.3)
project(genesis_msgs)

find_package(catkin REQUIRED COMPONENTS
	message_generation
	std_msgs
)


## Generate messages in the 'msg' folder
add_message_files(
  FILES
  SteeringReport.msg
  WheelSpeedReport.msg
  VehicleImuReport.msg
  LaneReport.msg
  MandoObjectReport.msg
  ESRTrackReport.msg
  ESRValidReport.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  std_msgs
)
