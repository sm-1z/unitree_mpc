cmake_minimum_required(VERSION 3.5)
project(ocs2_double_integrator_ros)

# Generate compile_commands.json for clang tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ocs2_core REQUIRED)
find_package(ocs2_ddp REQUIRED)
find_package(ocs2_sqp REQUIRED)
find_package(ocs2_mpc REQUIRED)
find_package(ocs2_ros2_interfaces REQUIRED)
find_package(ocs2_double_integrator REQUIRED)
find_package(ocs2_robotic_tools REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem log log_setup)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# Include directories
include_directories(
        include
        ${EIGEN3_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
)

# Mpc node
add_executable(double_integrator_mpc
        src/DoubleIntegratorMpcNode.cpp
)
ament_target_dependencies(double_integrator_mpc
        rclcpp ocs2_core ocs2_ddp ocs2_sqp ocs2_mpc ocs2_ros2_interfaces ocs2_double_integrator ocs2_robotic_tools
)
target_link_libraries(double_integrator_mpc
        ${Boost_LIBRARIES}
)

# Dummy node
add_executable(double_integrator_dummy_test
        src/DummyDoubleIntegratorNode.cpp
        src/DoubleIntegratorDummyVisualization.cpp
)
ament_target_dependencies(double_integrator_dummy_test
        rclcpp ocs2_core ocs2_ddp ocs2_sqp ocs2_mpc ocs2_ros2_interfaces ocs2_double_integrator ocs2_robotic_tools sensor_msgs
)
target_link_libraries(double_integrator_dummy_test
        ${Boost_LIBRARIES}
)

# Target node
add_executable(double_integrator_target
        src/DoubleIntegratorTargetPoseCommand.cpp
)
ament_target_dependencies(double_integrator_target
        rclcpp ocs2_core ocs2_ddp ocs2_sqp ocs2_mpc ocs2_ros2_interfaces ocs2_double_integrator ocs2_robotic_tools
)
target_link_libraries(double_integrator_target
        ${Boost_LIBRARIES}
)
target_compile_options(double_integrator_target PRIVATE ${OCS2_CXX_FLAGS})

# Install
install(TARGETS
        double_integrator_mpc
        double_integrator_dummy_test
        double_integrator_target
        DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
        DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY
        launch
        rviz
        urdf
        DESTINATION share/${PROJECT_NAME}
)

# Export dependencies
ament_package()
