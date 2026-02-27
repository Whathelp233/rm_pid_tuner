cmake_minimum_required(VERSION 3.0.2)
project(rm_pid_tuner)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslib
  std_msgs
  geometry_msgs
  nav_msgs
  rm_msgs
  message_generation
  dynamic_reconfigure
  realtime_tools
  control_toolbox
)

## System dependencies
find_package(Boost REQUIRED COMPONENTS thread)
find_package(nlohmann_json REQUIRED)
find_package(yaml-cpp REQUIRED)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  PidParams.msg
  TuningStatus.msg
  TuningLog.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  StartTuning.srv
  StopTuning.srv
  SetPidParams.srv
  LLMAnalyze.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

## Generate dynamic reconfigure parameters
generate_dynamic_reconfigure_options(
  cfg/PidTuner.cfg
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    rospy
    std_msgs
    geometry_msgs
    nav_msgs
    rm_msgs
    message_runtime
    dynamic_reconfigure
    realtime_tools
  DEPENDS Boost
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

## PID Tuner Library
add_library(${PROJECT_NAME}
  src/data_buffer.cpp
  src/pid_tuner.cpp
  src/visualizer.cpp
  src/tuning_history.cpp
  src/pid_analyzer.cpp
)

add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  nlohmann_json::nlohmann_json
  yaml-cpp
)

## PID Tuner Node
add_executable(pid_tuner_node
  src/pid_tuner_node.cpp
)

add_dependencies(pid_tuner_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(pid_tuner_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

#############
## Install ##
#############

## Mark executables for installation
install(TARGETS ${PROJECT_NAME} pid_tuner_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

## Mark launch files for installation
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

## Mark config files for installation
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

## Install Python scripts
catkin_install_python(PROGRAMS
  scripts/llm_interface.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install rqt plugin
install(DIRECTORY plugins/rqt_pid_tuner/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/plugins/rqt_pid_tuner
)
