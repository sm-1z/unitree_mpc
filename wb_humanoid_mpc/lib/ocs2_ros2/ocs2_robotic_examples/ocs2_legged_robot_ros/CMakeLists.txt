cmake_minimum_required(VERSION 3.5)

project(ocs2_legged_robot_ros)

# Generate compile_commands.json for clang tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(urdf REQUIRED)
find_package(ocs2_core REQUIRED)
find_package(ocs2_oc REQUIRED)
find_package(ocs2_ddp REQUIRED)
find_package(ocs2_mpc REQUIRED)
find_package(ocs2_sqp REQUIRED)
find_package(ocs2_robotic_tools REQUIRED)
find_package(ocs2_pinocchio_interface REQUIRED)
find_package(ocs2_centroidal_model REQUIRED)
find_package(ocs2_ros2_msgs REQUIRED)
find_package(ocs2_ros2_interfaces REQUIRED)
find_package(ocs2_legged_robot REQUIRED)
find_package(pinocchio REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(Boost REQUIRED COMPONENTS
        system
        filesystem
        log
        log_setup
)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

find_package(PkgConfig REQUIRED)
pkg_check_modules(pinocchio REQUIRED pinocchio)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS
    include
    ${EIGEN3_INCLUDE_DIRS}
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    ${CATKIN_PACKAGE_DEPENDENCIES}
  DEPENDS
    Boost
    pinocchio
)

###########
## Build ##
###########

set(dependencies
        rclcpp
        urdf
        ocs2_core
        ocs2_oc
        ocs2_ddp
        ocs2_mpc
        ocs2_sqp
        ocs2_robotic_tools
        ocs2_pinocchio_interface
        ocs2_centroidal_model
        ocs2_ros2_msgs
        ocs2_ros2_interfaces
        ocs2_legged_robot
        pinocchio
        sensor_msgs
)

include_directories(
  include
  ${pinocchio_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

link_directories(
  ${pinocchio_LIBRARY_DIRS}
)

# Main library
add_library(${PROJECT_NAME}
  src/gait/GaitKeyboardPublisher.cpp
  src/gait/GaitReceiver.cpp
  src/visualization/LeggedRobotVisualizer.cpp
)
ament_target_dependencies(${PROJECT_NAME}
        ${dependencies}
)
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})

# DDP-MPC node for legged robot
add_executable(legged_robot_ddp_mpc
  src/LeggedRobotDdpMpcNode.cpp
)
ament_target_dependencies(legged_robot_ddp_mpc
        ${dependencies}
)
target_link_libraries(legged_robot_ddp_mpc
  ${PROJECT_NAME}
)
target_compile_options(legged_robot_ddp_mpc PRIVATE ${OCS2_CXX_FLAGS})

## SQP-MPC node for legged robot
add_executable(legged_robot_sqp_mpc
  src/LeggedRobotSqpMpcNode.cpp
)
ament_target_dependencies(legged_robot_sqp_mpc
        ${dependencies}
)
target_link_libraries(legged_robot_sqp_mpc
  ${PROJECT_NAME}
)
target_compile_options(legged_robot_sqp_mpc  PRIVATE ${OCS2_CXX_FLAGS})

## IPM-MPC node for legged robot
add_executable(legged_robot_ipm_mpc
  src/LeggedRobotIpmMpcNode.cpp
)
ament_target_dependencies(legged_robot_ddp_mpc
        ${dependencies}
)
target_link_libraries(legged_robot_ddp_mpc
  ${PROJECT_NAME}
)
target_compile_options(legged_robot_ipm_mpc  PRIVATE ${OCS2_CXX_FLAGS})

# Dummy node
add_executable(legged_robot_dummy
  src/LeggedRobotDummyNode.cpp
)
ament_target_dependencies(legged_robot_dummy
        ${dependencies}
)
target_link_libraries(legged_robot_dummy
  ${PROJECT_NAME}
)
target_compile_options(legged_robot_dummy PRIVATE ${OCS2_CXX_FLAGS})

# Keyboard command node for target trajectories
add_executable(legged_robot_target
  src/LeggedRobotPoseCommandNode.cpp
)
ament_target_dependencies(legged_robot_target
        ${dependencies}
)
target_link_libraries(legged_robot_target
  ${PROJECT_NAME}
)
target_compile_options(legged_robot_target PRIVATE ${OCS2_CXX_FLAGS})

# Keyboard command node for gait
add_executable(legged_robot_gait_command
  src/LeggedRobotGaitCommandNode.cpp
)
ament_target_dependencies(legged_robot_gait_command
        ${dependencies}
)
target_link_libraries(legged_robot_gait_command
  ${PROJECT_NAME}
)
target_compile_options(legged_robot_gait_command PRIVATE ${OCS2_CXX_FLAGS})

#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
   message(STATUS "Run clang tooling for target ocs2_legged_robot_ros")
   add_clang_tooling(
     TARGETS
       ${PROJECT_NAME}
       legged_robot_ddp_mpc
       legged_robot_sqp_mpc
       legged_robot_dummy
       legged_robot_target
       legged_robot_gait_command
     SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
     CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
     CF_WERROR
)
endif(cmake_clang_tools_FOUND)

#############
## Install ##
#############
install(DIRECTORY include/
        DESTINATION include/${PROJECT_NAME}
)
install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)
install(
        TARGETS
        legged_robot_ddp_mpc
        legged_robot_sqp_mpc
        legged_robot_dummy
        legged_robot_target
        legged_robot_gait_command
        DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch rviz
        DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})
ament_package()
