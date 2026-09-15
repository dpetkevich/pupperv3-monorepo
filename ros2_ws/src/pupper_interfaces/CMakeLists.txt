cmake_minimum_required(VERSION 3.8)
project(pupper_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/TurnDegrees.action"
  "action/MoveMeters.action"
  "action/GoToPose.action"
  "action/ReturnToStart.action"
  "action/ExecutePlan.action"
  "action/FollowPerson.action"
  "action/FindPerson.action"
  "srv/MarkPose.srv"
  "srv/Look.srv"
  "srv/EnrollPerson.srv"
  "srv/ForgetPerson.srv"
  "srv/ListPeople.srv"
  "msg/MotionStatus.msg"
  "msg/ExecutorState.msg"
  "msg/ExecutorEvent.msg"
  "msg/PersonTrack.msg"
  "msg/PersonTrackArray.msg"
  "msg/FollowStatus.msg"
  DEPENDENCIES action_msgs std_msgs builtin_interfaces
)

# Plan JSON schema (validated by pupper_planner and by the agent's tests)
install(DIRECTORY schema DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)
ament_package()
