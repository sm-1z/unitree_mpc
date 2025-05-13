cmake_minimum_required(VERSION 3.5)
project(ocs2_ballbot_ros)

# Generate compile_commands.json for clang tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(urdf REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ocs2_ros2_msgs REQUIRED)
find_package(ocs2_core REQUIRED)
find_package(ocs2_ddp REQUIRED)
find_package(ocs2_mpc REQUIRED)
find_package(ocs2_sqp REQUIRED)
find_package(ocs2_ros2_interfaces REQUIRED)
find_package(ocs2_robotic_tools REQUIRED)
find_package(ocs2_ballbot REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem log log_setup)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

# main library
add_library(${PROJECT_NAME}
  src/BallbotDummyVisualization.cpp
)
ament_target_dependencies(ballbot_mpc_mrt
  rclcpp ocs2_core ocs2_ddp ocs2_sqp ocs2_mpc ocs2_ros2_interfaces ocs2_ballbot ocs2_robotic_tools ocs2_ros2_msgs sensor_msgs
  )
target_link_libraries(ballbot_mpc_mrt
  ${Boost_LIBRARIES}
  )
target_compile_options(ballbot_ddp PRIVATE ${OCS2_CXX_FLAGS})

# Mpc node
add_executable(ballbot_ddp
  src/BallbotDdpMpcNode.cpp
)
ament_target_dependencies(ballbot_ddp
  rclcpp ocs2_core ocs2_ddp ocs2_sqp ocs2_mpc ocs2_ros2_interfaces ocs2_ballbot ocs2_robotic_tools
)
target_link_libraries(ballbot_ddp
  ${Boost_LIBRARIES}
)
target_compile_options(ballbot_ddp PRIVATE ${OCS2_CXX_FLAGS})

# Dummy node
add_executable(ballbot_dummy_test
  src/DummyBallbotNode.cpp
  src/BallbotDummyVisualization.cpp
)
ament_target_dependencies(ballbot_dummy_test
  rclcpp ocs2_core ocs2_ddp ocs2_sqp ocs2_mpc ocs2_ros2_interfaces ocs2_ballbot ocs2_robotic_tools sensor_msgs
)
target_link_libraries(ballbot_dummy_test
  ${Boost_LIBRARIES}
)
target_compile_options(ballbot_dummy_test PRIVATE ${OCS2_CXX_FLAGS})

# Target node
add_executable(ballbot_target
  src/BallbotTargetPoseCommand.cpp
)
ament_target_dependencies(ballbot_target
  rclcpp ocs2_core ocs2_ddp ocs2_sqp ocs2_mpc ocs2_ros2_interfaces ocs2_ballbot ocs2_robotic_tools tf2_ros
)
target_link_libraries(ballbot_target
  ${Boost_LIBRARIES}
)
target_compile_options(ballbot_target PRIVATE ${OCS2_CXX_FLAGS})

## Combined MPC MRT example
add_executable(ballbot_mpc_mrt
  src/BallbotMpcMrtNode.cpp
  src/BallbotDummyVisualization.cpp
  )
ament_target_dependencies(ballbot_mpc_mrt
  rclcpp ocs2_core ocs2_ddp ocs2_sqp ocs2_mpc ocs2_ros2_interfaces ocs2_ballbot ocs2_robotic_tools ocs2_ros2_msgs sensor_msgs
  )
target_link_libraries(ballbot_mpc_mrt
  ${Boost_LIBRARIES}
  )
target_compile_options(ballbot_mpc_mrt  PRIVATE ${OCS2_CXX_FLAGS})

## SQP node for ballbot
add_executable(ballbot_sqp
  src/BallbotSqpMpcNode.cpp
)
ament_target_dependencies(ballbot_sqp
  rclcpp ocs2_core ocs2_ddp ocs2_sqp ocs2_mpc ocs2_ros2_interfaces ocs2_ballbot ocs2_robotic_tools
)
target_link_libraries(ballbot_sqp
  ${Boost_LIBRARIES}
)
target_compile_options(ballbot_sqp  PRIVATE ${OCS2_CXX_FLAGS})

## SLP node for ballbot
add_executable(ballbot_slp
  src/BallbotSlpMpcNode.cpp
)
ament_target_dependencies(ballbot_mpc_mrt
  rclcpp ocs2_core ocs2_ddp ocs2_sqp ocs2_mpc ocs2_ros2_interfaces ocs2_ballbot ocs2_robotic_tools ocs2_ros2_msgs sensor_msgs
  )
target_link_libraries(ballbot_mpc_mrt
  ${Boost_LIBRARIES}
  )
target_compile_options(ballbot_slp  PRIVATE ${OCS2_CXX_FLAGS})

#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
  message(STATUS "Run clang tooling for target ocs2_ballbot")
  add_clang_tooling(
    TARGETS
        ${PROJECT_NAME}
        ballbot_ddp
        ballbot_sqp
        ballbot_dummy_test
        ballbot_target
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
install(TARGETS ballbot_mpc ballbot_dummy_test ballbot_target ballbot_sqp
        DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch rviz urdf
        DESTINATION share/${PROJECT_NAME}
)


#############
## Testing ##
#############

ament_add_gtest(ballbot_integration_test
  test/ballbotIntegrationTest.cpp
)
target_link_libraries(ballbot_integration_test
  ${Boost_LIBRARIES}
)

ament_package()
