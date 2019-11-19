cmake_minimum_required(VERSION 2.8.3)
project(pilz_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  moveit_msgs)

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   MotionSequenceItem.msg
   MotionSequenceRequest.msg
   IsBrakeTestRequiredResult.msg
 )

 #Generate services in the 'srv' folder
 add_service_files(
   FILES
   BrakeTest.srv
   GetMotionSequence.srv
   IsBrakeTestRequired.srv
   GetSpeedOverride.srv
 )

# Generate actions in the 'action' folder
add_action_files(
  FILES
  MoveGroupSequence.action
)

# Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES moveit_msgs)

catkin_package(CATKIN_DEPENDS message_runtime moveit_msgs)



