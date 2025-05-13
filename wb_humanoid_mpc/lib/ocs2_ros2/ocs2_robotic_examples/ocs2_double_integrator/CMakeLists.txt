cmake_minimum_required(VERSION 3.5)
project(ocs2_double_integrator)

# Generate compile_commands.json for clang tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pybind11 REQUIRED)
find_package(ocs2_core REQUIRED)
find_package(ocs2_mpc REQUIRED)
find_package(ocs2_ddp REQUIRED)
find_package(ocs2_sqp REQUIRED)
find_package(ocs2_robotic_tools REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem log_setup)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# Include directories
include_directories(
        include
        ${EIGEN3_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
)

# double integrator interface library
add_library(${PROJECT_NAME}
        src/DoubleIntegratorInterface.cpp
)
ament_target_dependencies(${PROJECT_NAME}
        rclcpp ocs2_core ocs2_mpc ocs2_ddp ocs2_sqp ocs2_robotic_tools Boost Eigen3
)
target_link_libraries(${PROJECT_NAME}
        ${Boost_LIBRARIES}
        dl
)
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})

# python bindings
#pybind11_add_module(DoubleIntegratorPyBindings SHARED
#        src/pyBindModule.cpp
#)
#target_link_libraries(DoubleIntegratorPyBindings PRIVATE
#        ${PROJECT_NAME}
#)

# Install
install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION lib/
        LIBRARY DESTINATION lib/
        RUNTIME DESTINATION bin
)
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION include/${PROJECT_NAME}/
)
install(DIRECTORY config
        DESTINATION share/${PROJECT_NAME}
)
#install(TARGETS DoubleIntegratorPyBindings
#        ARCHIVE DESTINATION lib/${PROJECT_NAME}
#        LIBRARY DESTINATION lib/${PROJECT_NAME}
#)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Add tests here
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(ocs2_core ocs2_mpc ocs2_ddp ocs2_sqp ocs2_robotic_tools Boost Eigen3)
ament_package()
