cmake_minimum_required(VERSION 3.8)
project(agv_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LineState.msg"
  "msg/MarkerState.msg"
  "msg/ObstacleState.msg"
  "msg/LiftState.msg"
  "msg/MotorState.msg"
  "msg/Stm32Status.msg"
  "msg/SafetyStatus.msg"
  "msg/MissionStatus.msg"
  "msg/SystemEvent.msg"
  "srv/Lift.srv"
  "srv/MissionCommand.srv"
  "srv/SetWheelPid.srv"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
