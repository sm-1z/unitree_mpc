/******************************************************************************
Copyright (c) 2017, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/rclcpp.hpp"

#include <ocs2_legged_robot/LeggedRobotInterface.h>
#include <ocs2_ros2_interfaces/mpc/MPC_ROS_Interface.h>
#include <ocs2_ros2_interfaces/synchronized_module/RosReferenceManager.h>
#include <ocs2_ros_interfaces/synchronized_module/SolverObserverRosCallbacks.h>
#include <ocs2_sqp/SqpMpc.h>

#include "ocs2_legged_robot_ros/gait/GaitReceiver.h"

using namespace ocs2;
using namespace legged_robot;

int main(int argc, char** argv) {
  const std::string robotName = "legged_robot";

  // task file
  std::vector<std::string> programArgs = rclcpp::remove_ros_arguments(argc, argv);
  if (programArgs.size() <= 1) {
    throw std::runtime_error("No task file specified. Aborting.");
  }
  std::string taskFileFolderName = std::string(programArgs[1]);
  std::string referenceFileFolderName = std::string(programArgs[2]);

  // Initialize ros node
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr nodeHandle = rclcpp::Node::make_shared(robotName + "_mpc");

  // Get node parameters
  const std::string taskFile =
      ament_index_cpp::get_package_share_directory("ocs2_legged_robot") + "/config/" + taskFileFolderName + "/task.info";
  const std::string referenceFile =
      ament_index_cpp::get_package_share_directory("ocs2_legged_robot") + "/config/" + referenceFileFolderName + "/reference.info";
  const std::string urdfFile = ament_index_cpp::get_package_share_directory("ocs2_legged_robot_ros") + "/urdf/urdf/anymal.urdf";


  // Robot interface
  LeggedRobotInterface interface(taskFile, urdfFile, referenceFile);

  // Gait receiver
  auto gaitReceiverPtr =
      std::make_shared<GaitReceiver>(nodeHandle, interface.getSwitchedModelReferenceManagerPtr()->getGaitSchedule(), robotName);

  // ROS ReferenceManager
  auto rosReferenceManagerPtr = std::make_shared<RosReferenceManager>(robotName, interface.getReferenceManagerPtr());
  rosReferenceManagerPtr->subscribe(nodeHandle, rclcpp::QoS(1));

  // MPC
  SqpMpc mpc(interface.mpcSettings(), interface.sqpSettings(), interface.getOptimalControlProblem(), interface.getInitializer());
  mpc.getSolverPtr()->setReferenceManager(rosReferenceManagerPtr);
  mpc.getSolverPtr()->addSynchronizedModule(gaitReceiverPtr);

  // observer for zero velocity constraints (only add this for debugging as it slows down the solver)
  if (multiplot) {
    auto createStateInputBoundsObserver = [&](const std::string& termName) {
      const ocs2::scalar_array_t observingTimePoints{0.0};
      const std::vector<std::string> topicNames{"metrics/" + termName + "/0MsLookAhead"};
      auto callback = ocs2::ros::createConstraintCallback(nodeHandle, {0.0}, topicNames,
                                                          ocs2::ros::CallbackInterpolationStrategy::linear_interpolation);
      return ocs2::SolverObserver::ConstraintTermObserver(ocs2::SolverObserver::Type::Intermediate, termName, std::move(callback));
    };
    for (size_t i = 0; i < interface.getCentroidalModelInfo().numThreeDofContacts; i++) {
      const std::string& footName = interface.modelSettings().contactNames3DoF[i];
      mpc.getSolverPtr()->addSolverObserver(createStateInputBoundsObserver(footName + "_zeroVelocity"));
    }
  }

  // Launch MPC ROS node
  MPC_ROS_Interface mpcNode(mpc, robotName);
  mpcNode.launchNodes(nodeHandle, rclcpp::QoS(1));

  // Successful exit
  return 0;
}
