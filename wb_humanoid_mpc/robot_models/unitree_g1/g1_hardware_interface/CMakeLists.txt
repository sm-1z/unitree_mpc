cmake_minimum_required(VERSION 3.8)
project(g1_hardware_interface)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# Generate symbols for IDE indexer
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(THIS_PACKAGE_INCLUDE_DEPENDS
        unitree_go
        unitree_hg
        unitree_api
        rclcpp
        std_msgs
        rosbag2_cpp
        ocs2_core
        ocs2_ros2_msgs
        humanoid_common_mpc
)

# find dependencies
find_package(ament_cmake REQUIRED)

foreach (Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach ()



find_package(Boost REQUIRED COMPONENTS
  system
  filesystem
  log
  log_setup
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(pinocchio REQUIRED pinocchio)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

#############
## Library ##
#############

add_library(${PROJECT_NAME}
  src/HardwareUnitreeG1.cpp
  src/motor_crc_hg.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  PRIVATE src
)

target_link_libraries(${PROJECT_NAME} 
        unitree_go 
        unitree_hg
        unitree_go::unitree_go__rosidl_typesupport_cpp
        unitree_hg::unitree_hg__rosidl_typesupport_cpp
)

ament_target_dependencies(
        ${PROJECT_NAME}
        ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})

###########
## Build ##
###########

set(CMAKE_CXX_STANDARD 17)

set(FLAGS ${OCS2_CXX_FLAGS} -Wno-invalid-partial-specialization)


#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
  # EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include/
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_export_libraries(${PROJECT_NAME})


# ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()

