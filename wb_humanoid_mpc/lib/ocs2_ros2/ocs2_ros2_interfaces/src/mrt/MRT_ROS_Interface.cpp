/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

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

#include "ocs2_ros2_interfaces/mrt/MRT_ROS_Interface.h"

#include <ocs2_core/control/FeedforwardController.h>
#include <ocs2_core/control/LinearController.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
MRT_ROS_Interface::MRT_ROS_Interface(std::string topicPrefix) : topicPrefix_(std::move(topicPrefix)) {
// Start thread for publishing
#ifdef PUBLISH_THREAD
  // Close old thread if it is already running
  shutdownPublisher();
  terminateThread_ = false;
  readyToPublish_ = false;
  publisherWorker_ = std::thread(&MRT_ROS_Interface::publisherWorkerThread, this);
#endif
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
MRT_ROS_Interface::~MRT_ROS_Interface() {
  shutdownNodes();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MRT_ROS_Interface::resetMpcNode(const TargetTrajectories& initTargetTrajectories) {
  this->reset();

  auto resetSrv = std::make_shared<ocs2_ros2_msgs::srv::Reset::Request>();
  resetSrv->reset = static_cast<uint8_t>(true);
  resetSrv->target_trajectories = ros_msg_conversions::createTargetTrajectoriesMsg(initTargetTrajectories);

  while (!mpcResetServiceClient_->wait_for_service(std::chrono::seconds(5)) && rclcpp::ok()) {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "Failed to call service to reset MPC, retrying...");
  }

  mpcResetServiceClient_->async_send_request(resetSrv);
  RCLCPP_INFO_STREAM(node_->get_logger(), "MPC node has been reset.");
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MRT_ROS_Interface::setCurrentObservation(const SystemObservation& currentObservation) {
#ifdef PUBLISH_THREAD
  std::unique_lock<std::mutex> lk(publisherMutex_);
#endif

  // create the message
  mpcObservationMsg_ = ros_msg_conversions::createObservationMsg(currentObservation);

  // publish the current observation
#ifdef PUBLISH_THREAD
  readyToPublish_ = true;
  lk.unlock();
  msgReady_.notify_one();
#else
  mpcObservationPublisher_->publish(mpcObservationMsg_);
#endif
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MRT_ROS_Interface::publisherWorkerThread() {
  while (!terminateThread_) {
    std::unique_lock<std::mutex> lk(publisherMutex_);

    msgReady_.wait(lk, [&] { return (readyToPublish_ || terminateThread_); });

    if (terminateThread_) {
      break;
    }

    mpcObservationMsgBuffer_ = std::move(mpcObservationMsg_);

    readyToPublish_ = false;

    lk.unlock();
    msgReady_.notify_one();

    mpcObservationPublisher_->publish(mpcObservationMsgBuffer_);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MRT_ROS_Interface::readPolicyMsg(const ocs2_ros2_msgs::msg::MpcFlattenedController& msg, CommandData& commandData,
                                      PrimalSolution& primalSolution, PerformanceIndex& performanceIndices) {
  commandData.mpcInitObservation_ = ros_msg_conversions::readObservationMsg(msg.init_observation);
  commandData.mpcTargetTrajectories_ = ros_msg_conversions::readTargetTrajectoriesMsg(msg.plan_target_trajectories);
  performanceIndices = ros_msg_conversions::readPerformanceIndicesMsg(msg.performance_indices);

  const size_t N = msg.time_trajectory.size();
  if (N == 0) {
    throw std::runtime_error("[MRT_ROS_Interface::readPolicyMsg] controller message is empty!");
  }
  if (msg.state_trajectory.size() != N && msg.input_trajectory.size() != N) {
    throw std::runtime_error("[MRT_ROS_Interface::readPolicyMsg] state and input trajectories must have same length!");
  }
  if (msg.data.size() != N) {
    throw std::runtime_error("[MRT_ROS_Interface::readPolicyMsg] Data has the wrong length!");
  }

  primalSolution.clear();

  primalSolution.modeSchedule_ = ros_msg_conversions::readModeScheduleMsg(msg.mode_schedule);

  size_array_t stateDim(N);
  size_array_t inputDim(N);
  primalSolution.timeTrajectory_.reserve(N);
  primalSolution.stateTrajectory_.reserve(N);
  primalSolution.inputTrajectory_.reserve(N);
  for (size_t i = 0; i < N; i++) {
    stateDim[i] = msg.state_trajectory[i].value.size();
    inputDim[i] = msg.input_trajectory[i].value.size();
    primalSolution.timeTrajectory_.emplace_back(msg.time_trajectory[i]);
    primalSolution.stateTrajectory_.emplace_back(
        Eigen::Map<const Eigen::VectorXd>(msg.state_trajectory[i].value.data(), stateDim[i]));
    primalSolution.inputTrajectory_.emplace_back(
        Eigen::Map<const Eigen::VectorXd>(msg.input_trajectory[i].value.data(), inputDim[i]));
  }

  primalSolution.postEventIndices_.reserve(msg.post_event_indices.size());
  for (auto ind : msg.post_event_indices) {
    primalSolution.postEventIndices_.emplace_back(static_cast<size_t>(ind));
  }

  std::vector<std::vector<double> const*> controllerDataPtrArray(N, nullptr);
  for (int i = 0; i < N; i++) {
    controllerDataPtrArray[i] = &(msg.data[i].data);
  }

  // instantiate the correct controller
  switch (msg.controller_type) {
    case ocs2_ros2_msgs::msg::MpcFlattenedController::CONTROLLER_FEEDFORWARD: {
      auto controller = FeedforwardController::unFlatten(primalSolution.timeTrajectory_, controllerDataPtrArray);
      primalSolution.controllerPtr_.reset(new FeedforwardController(std::move(controller)));
      break;
    }
    case ocs2_ros2_msgs::msg::MpcFlattenedController::CONTROLLER_LINEAR: {
      auto controller = LinearController::unFlatten(stateDim, inputDim, primalSolution.timeTrajectory_, controllerDataPtrArray);
      primalSolution.controllerPtr_.reset(new LinearController(std::move(controller)));
      break;
    }
    default:
      throw std::runtime_error("[MRT_ROS_Interface::readPolicyMsg] Unknown controllerType!");
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MRT_ROS_Interface::mpcPolicyCallback(const ocs2_ros2_msgs::msg::MpcFlattenedController::SharedPtr msg) {
  // read new policy and command from msg
  std::unique_ptr<CommandData> commandPtr(new CommandData);
  std::unique_ptr<PrimalSolution> primalSolutionPtr(new PrimalSolution);
  std::unique_ptr<PerformanceIndex> performanceIndicesPtr(new PerformanceIndex);
  readPolicyMsg(*msg, *commandPtr, *primalSolutionPtr, *performanceIndicesPtr);

  this->moveToBuffer(std::move(commandPtr), std::move(primalSolutionPtr), std::move(performanceIndicesPtr));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MRT_ROS_Interface::shutdownNodes() {
#ifdef PUBLISH_THREAD
  RCLCPP_INFO_STREAM(node_->get_logger(), "Shutting down workers ...");

  shutdownPublisher();

  RCLCPP_INFO_STREAM(node_->get_logger(), "All workers are shut down.");
#endif

  // clean up callback queue
  mpcPolicySubscriber_.reset();

  // shutdown publishers
  mpcObservationPublisher_.reset();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MRT_ROS_Interface::shutdownPublisher() {
  std::unique_lock<std::mutex> lk(publisherMutex_);
  terminateThread_ = true;
  lk.unlock();

  msgReady_.notify_all();

  if (publisherWorker_.joinable()) {
    publisherWorker_.join();
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MRT_ROS_Interface::spinMRT() {
  rclcpp::spin_some(node_);
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MRT_ROS_Interface::launchNodes(rclcpp::Node::SharedPtr node, const rclcpp::QoS& qos) {
  this->reset();

  node_ = node;

  // display
  RCLCPP_INFO_STREAM(node->get_logger(), "MRT node is setting up ...");

  // observation publisher
  mpcObservationPublisher_ = node->create_publisher<ocs2_ros2_msgs::msg::MpcObservation>(topicPrefix_ + "/mpc_observation", qos);

  // policy subscriber
  mpcPolicySubscriber_ = node->create_subscription<ocs2_ros2_msgs::msg::MpcFlattenedController>(
      topicPrefix_ + "/mpc_policy", qos, std::bind(&MRT_ROS_Interface::mpcPolicyCallback, this, std::placeholders::_1));

  // MPC reset service client
  mpcResetServiceClient_ = node->create_client<ocs2_ros2_msgs::srv::Reset>(topicPrefix_ + "/mpc_reset");
  // display
#ifdef PUBLISH_THREAD
  RCLCPP_INFO_STREAM(node->get_logger(), "Publishing MRT messages on a separate thread.");
#endif

  RCLCPP_INFO_STREAM(node->get_logger(), "MRT node is ready.");

  spinMRT();
}

}  // namespace ocs2
