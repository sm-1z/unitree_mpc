cmake_minimum_required(VERSION 3.10.2)
project(ocs2_oc)

## Find libraries
find_package(ament_cmake REQUIRED)
find_package(ocs2_core REQUIRED)
find_package(OpenMP REQUIRED)
find_package(Boost REQUIRED COMPONENTS
  system
  filesystem
  log
  log_setup
)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# Generate compile_commands.json for clang tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)


###########
## Build ##
###########

add_library(${PROJECT_NAME}
  src/approximate_model/ChangeOfInputVariables.cpp
  src/approximate_model/LinearQuadraticApproximator.cpp
  src/multiple_shooting/Helpers.cpp
  src/multiple_shooting/Initialization.cpp
  src/multiple_shooting/LagrangianEvaluation.cpp
  src/multiple_shooting/MetricsComputation.cpp
  src/multiple_shooting/PerformanceIndexComputation.cpp
  src/multiple_shooting/ProjectionMultiplierCoefficients.cpp
  src/multiple_shooting/Transcription.cpp
  src/oc_data/LoopshapingPrimalSolution.cpp
  src/oc_data/PerformanceIndex.cpp
  src/oc_data/TimeDiscretization.cpp
  src/oc_problem/OptimalControlProblem.cpp
  src/oc_problem/LoopshapingOptimalControlProblem.cpp
  src/oc_problem/OptimalControlProblemHelperFunction.cpp
  src/oc_problem/OcpSize.cpp
  src/oc_problem/OcpToKkt.cpp
  src/oc_solver/SolverBase.cpp
  src/precondition/Ruzi.cpp
  src/rollout/PerformanceIndicesRollout.cpp
  src/rollout/RolloutBase.cpp
  src/rollout/RootFinder.cpp
  src/rollout/InitializerRollout.cpp
  src/rollout/StateTriggeredRollout.cpp
  src/rollout/TimeTriggeredRollout.cpp
  src/rollout/RolloutSettings.cpp
  src/synchronized_module/ReferenceManager.cpp
  src/synchronized_module/LoopshapingReferenceManager.cpp
  src/synchronized_module/LoopshapingSynchronizedModule.cpp
  src/synchronized_module/SolverObserver.cpp
  src/search_strategy/FilterLinesearch.cpp
  src/trajectory_adjustment/TrajectorySpreading.cpp
)
ament_target_dependencies(${PROJECT_NAME}
  ocs2_core
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(${PROJECT_NAME}_lintTarget
  src/lintTarget.cpp
)

ament_target_dependencies(${PROJECT_NAME}_lintTarget
  ocs2_core
)

target_include_directories(${PROJECT_NAME}_lintTarget PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
  message(STATUS "Run clang tooling for target " ${PROJECT_NAME}_lintTarget)
  add_clang_tooling(
    TARGETS ${PROJECT_NAME}_lintTarget
    SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/test/include
    CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
    CF_WERROR
)
endif(cmake_clang_tools_FOUND)

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

install(DIRECTORY include/
        DESTINATION include
)

install(DIRECTORY test/include/
        DESTINATION include
)

ament_export_include_directories(include)

ament_export_libraries(${PROJECT_NAME})
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_export_dependencies(ocs2_core)

#############
## Testing ##
#############

if(BUILD_TESTING)

# Include linting tests
find_package(ament_lint_auto REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
ament_lint_auto_find_test_dependencies()

ament_add_gtest(test_${PROJECT_NAME}_multiple_shooting
  test/multiple_shooting/testProjectionMultiplierCoefficients.cpp
  test/multiple_shooting/testTranscriptionMetrics.cpp
  test/multiple_shooting/testTranscriptionPerformanceIndex.cpp
)
target_link_libraries(test_${PROJECT_NAME}_multiple_shooting
  ${PROJECT_NAME}
  ${Boost_LIBRARIES}
  gtest_main
)
target_include_directories(test_${PROJECT_NAME}_multiple_shooting PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/test/include>
  $<INSTALL_INTERFACE:include>
)

ament_add_gtest(test_${PROJECT_NAME}_data
  test/oc_data/testTimeDiscretization.cpp
)
target_link_libraries(test_${PROJECT_NAME}_data
  ${PROJECT_NAME}
  ${Boost_LIBRARIES}
  gtest_main
)
target_include_directories(test_${PROJECT_NAME}_data PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/test/include>
  $<INSTALL_INTERFACE:include>
)

ament_add_gtest(test_${PROJECT_NAME}_rollout
   test/rollout/testTimeTriggeredRollout.cpp
   test/rollout/testStateTriggeredRollout.cpp
)
target_link_libraries(test_${PROJECT_NAME}_rollout
  ${PROJECT_NAME}
  ${Boost_LIBRARIES}
  gtest_main
)
target_include_directories(test_${PROJECT_NAME}_rollout PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/test/include>
  $<INSTALL_INTERFACE:include>
)

ament_add_gtest(test_change_of_variables
  test/testChangeOfInputVariables.cpp
)
target_link_libraries(test_change_of_variables
  ${PROJECT_NAME}
  ${Boost_LIBRARIES}
  gtest_main
)
target_include_directories(test_change_of_variables PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/test/include>
  $<INSTALL_INTERFACE:include>
)

ament_add_gtest(test_trajectory_spreading
  test/trajectory_adjustment/TrajectorySpreadingTest.cpp
)
target_link_libraries(test_trajectory_spreading
  ${PROJECT_NAME}
  ${Boost_LIBRARIES}
  gtest_main
)
target_include_directories(test_trajectory_spreading PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/test/include>
  $<INSTALL_INTERFACE:include>
)

ament_add_gtest(test_ocp_to_kkt
  test/oc_problem/testOcpToKkt.cpp
)
target_link_libraries(test_ocp_to_kkt
  ${PROJECT_NAME}
  ${Boost_LIBRARIES}
  gtest_main
)
target_include_directories(test_ocp_to_kkt PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/test/include>
  $<INSTALL_INTERFACE:include>
)

ament_add_gtest(test_precondition
  test/precondition/testPrecondition.cpp
)
target_link_libraries(test_precondition
  ${PROJECT_NAME}
  ${Boost_LIBRARIES}
  gtest_main
)
target_include_directories(test_precondition PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/test/include>
  $<INSTALL_INTERFACE:include>
)

endif(BUILD_TESTING)

ament_package()
