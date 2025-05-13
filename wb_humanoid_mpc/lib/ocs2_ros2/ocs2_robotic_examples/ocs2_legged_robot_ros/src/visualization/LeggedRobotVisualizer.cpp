/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.

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

// Pinocchio forward declarations must be included first
#include <pinocchio/fwd.hpp>

// Pinocchio
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include "ocs2_legged_robot_ros/visualization/LeggedRobotVisualizer.h"

// OCS2
#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_core/misc/LinearInterpolation.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <ocs2_ros2_interfaces/visualization/VisualizationHelpers.h>
#include "ocs2_legged_robot/gait/MotionPhaseDefinition.h"

// Additional messages not in the helpers file
#include <geometry_msgs/msg/pose_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// URDF related
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>

namespace ocs2 {
namespace legged_robot {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
LeggedRobotVisualizer::LeggedRobotVisualizer(PinocchioInterface pinocchioInterface,
                                             CentroidalModelInfo centroidalModelInfo,
                                             const PinocchioEndEffectorKinematics& endEffectorKinematics,
                                             rclcpp::Node::SharedPtr nodeHandle,
                                             scalar_t maxUpdateFrequency)
    : pinocchioInterface_(std::move(pinocchioInterface)),
      centroidalModelInfo_(std::move(centroidalModelInfo)),
      endEffectorKinematicsPtr_(endEffectorKinematics.clone()),
      lastTime_(std::numeric_limits<scalar_t>::lowest()),
      minPublishTimeDifference_(1.0 / maxUpdateFrequency),
      nodeHandle_(nodeHandle) {
  endEffectorKinematicsPtr_->setPinocchioInterface(pinocchioInterface_);
  launchVisualizerNode(nodeHandle);
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LeggedRobotVisualizer::launchVisualizerNode(rclcpp::Node::SharedPtr nodeHandle) {
  costDesiredBasePositionPublisher_ =
      nodeHandle->create_publisher<visualization_msgs::msg::Marker>("/legged_robot/desiredBaseTrajectory", 1);
  costDesiredFeetPositionPublishers_.resize(centroidalModelInfo_.numThreeDofContacts);
  costDesiredFeetPositionPublishers_[0] =
      nodeHandle->create_publisher<visualization_msgs::msg::Marker>("/legged_robot/desiredFeetTrajectory/LF", 1);
  costDesiredFeetPositionPublishers_[1] =
      nodeHandle->create_publisher<visualization_msgs::msg::Marker>("/legged_robot/desiredFeetTrajectory/RF", 1);
  costDesiredFeetPositionPublishers_[2] =
      nodeHandle->create_publisher<visualization_msgs::msg::Marker>("/legged_robot/desiredFeetTrajectory/LH", 1);
  costDesiredFeetPositionPublishers_[3] =
      nodeHandle->create_publisher<visualization_msgs::msg::Marker>("/legged_robot/desiredFeetTrajectory/RH", 1);
  stateOptimizedPublisher_ =
      nodeHandle->create_publisher<visualization_msgs::msg::MarkerArray>("/legged_robot/optimizedStateTrajectory", 1);
  currentStatePublisher_ = nodeHandle->create_publisher<visualization_msgs::msg::MarkerArray>("/legged_robot/currentState", 1);

  jointStatePublisher_ = nodeHandle->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 1);
  tfBroadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(nodeHandle);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LeggedRobotVisualizer::update(const SystemObservation& observation, const PrimalSolution& primalSolution, const CommandData& command) {
  if (observation.time - lastTime_ > minPublishTimeDifference_) {
    const auto& model = pinocchioInterface_.getModel();
    auto& data = pinocchioInterface_.getData();
    pinocchio::forwardKinematics(model, data, centroidal_model::getGeneralizedCoordinates(observation.state, centroidalModelInfo_));
    pinocchio::updateFramePlacements(model, data);

    const auto timeStamp = nodeHandle_->get_clock()->now();
    publishObservation(timeStamp, observation);
    publishDesiredTrajectory(timeStamp, command.mpcTargetTrajectories_);
    publishOptimizedStateTrajectory(timeStamp, primalSolution.timeTrajectory_, primalSolution.stateTrajectory_,
                                    primalSolution.modeSchedule_);
    lastTime_ = observation.time;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LeggedRobotVisualizer::publishObservation(const builtin_interfaces::msg::Time timeStamp, const SystemObservation& observation) {
  // Extract components from state
  const auto basePose = centroidal_model::getBasePose(observation.state, centroidalModelInfo_);
  const auto qJoints = centroidal_model::getJointAngles(observation.state, centroidalModelInfo_);

  // Compute cartesian state and inputs
  const auto feetPositions = endEffectorKinematicsPtr_->getPosition(observation.state);
  std::vector<vector3_t> feetForces(centroidalModelInfo_.numThreeDofContacts);
  for (size_t i = 0; i < centroidalModelInfo_.numThreeDofContacts; i++) {
    feetForces[i] = centroidal_model::getContactForces(observation.input, i, centroidalModelInfo_);
  }

  // Publish
  publishJointTransforms(timeStamp, qJoints);
  publishBaseTransform(timeStamp, basePose);
  publishCartesianMarkers(timeStamp, modeNumber2StanceLeg(observation.mode), feetPositions, feetForces);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LeggedRobotVisualizer::publishJointTransforms(const builtin_interfaces::msg::Time timeStamp, const vector_t& jointAngles) const {
  if (jointStatePublisher_ != nullptr) {
    std::map<std::string, scalar_t> jointPositions{{"LF_HAA", jointAngles[0]}, {"LF_HFE", jointAngles[1]},  {"LF_KFE", jointAngles[2]},
                                                   {"LH_HAA", jointAngles[3]}, {"LH_HFE", jointAngles[4]},  {"LH_KFE", jointAngles[5]},
                                                   {"RF_HAA", jointAngles[6]}, {"RF_HFE", jointAngles[7]},  {"RF_KFE", jointAngles[8]},
                                                   {"RH_HAA", jointAngles[9]}, {"RH_HFE", jointAngles[10]}, {"RH_KFE", jointAngles[11]}};

    sensor_msgs::msg::JointState jointState;
    jointState.header.stamp = timeStamp;
    for (const auto& [jointName, jointPosition] : jointPositions) {
      jointState.name.push_back(jointName);
      jointState.position.push_back(jointPosition);
    }

    jointStatePublisher_->publish(jointState);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LeggedRobotVisualizer::publishBaseTransform(const builtin_interfaces::msg::Time timeStamp, const vector_t& basePose) {
  if (jointStatePublisher_ != nullptr) {
    geometry_msgs::msg::TransformStamped baseToWorldTransform;
    baseToWorldTransform.header = getHeaderMsg(frameId_, timeStamp);
    baseToWorldTransform.child_frame_id = "base";

    const Eigen::Quaternion<scalar_t> q_world_base = getQuaternionFromEulerAnglesZyx(vector3_t(basePose.tail<3>()));
    baseToWorldTransform.transform.rotation = getOrientationMsg(q_world_base);
    baseToWorldTransform.transform.translation = getVectorMsg(basePose.head<3>());
    tfBroadcaster_->sendTransform(baseToWorldTransform);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LeggedRobotVisualizer::publishTrajectory(const std::vector<SystemObservation>& system_observation_array, scalar_t speed) {
  for (size_t k = 0; k < system_observation_array.size() - 1; k++) {
    scalar_t frameDuration = speed * (system_observation_array[k + 1].time - system_observation_array[k].time);
    scalar_t publishDuration =
        timedExecutionInSeconds([&]() { publishObservation(nodeHandle_->get_clock()->now(), system_observation_array[k]); });
    if (frameDuration > publishDuration) {
      rclcpp::WallRate(frameDuration - publishDuration).sleep();
    }
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LeggedRobotVisualizer::publishCartesianMarkers(const builtin_interfaces::msg::Time timeStamp,
                                                    const contact_flag_t& contactFlags,
                                                    const std::vector<vector3_t>& feetPositions,
                                                    const std::vector<vector3_t>& feetForces) const {
  // Reserve message
  const size_t numberOfCartesianMarkers = 10;
  visualization_msgs::msg::MarkerArray markerArray;
  markerArray.markers.reserve(numberOfCartesianMarkers);

  // Feet positions and Forces
  for (size_t i = 0; i < centroidalModelInfo_.numThreeDofContacts; ++i) {
    markerArray.markers.emplace_back(
        getFootMarker(feetPositions[i], contactFlags[i], feetColorMap_[i], footMarkerDiameter_, footAlphaWhenLifted_));
    markerArray.markers.emplace_back(getForceMarker(feetForces[i], feetPositions[i], contactFlags[i], Color::green, forceScale_));
  }

  // Center of pressure
  markerArray.markers.emplace_back(getCenterOfPressureMarker(feetForces.begin(), feetForces.end(), feetPositions.begin(),
                                                             contactFlags.begin(), Color::green, copMarkerDiameter_));

  // Support polygon
  markerArray.markers.emplace_back(
      getSupportPolygonMarker(feetPositions.begin(), feetPositions.end(), contactFlags.begin(), Color::black, supportPolygonLineWidth_));

  // Give markers an id and a frame
  assignHeader(markerArray.markers.begin(), markerArray.markers.end(), getHeaderMsg(frameId_, timeStamp));
  assignIncreasingId(markerArray.markers.begin(), markerArray.markers.end());

  // Publish cartesian markers (minus the CoM Pose)
  currentStatePublisher_->publish(markerArray);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LeggedRobotVisualizer::publishDesiredTrajectory(const builtin_interfaces::msg::Time timeStamp,
                                                     const TargetTrajectories& targetTrajectories) {
  const auto& stateTrajectory = targetTrajectories.stateTrajectory;
  const auto& inputTrajectory = targetTrajectories.inputTrajectory;

  // Reserve com messages
  std::vector<geometry_msgs::msg::Point> desiredBasePositionMsg;
  desiredBasePositionMsg.reserve(stateTrajectory.size());

  // Reserve feet messages
  feet_array_t<std::vector<geometry_msgs::msg::Point>> desiredFeetPositionMsgs;
  for (size_t i = 0; i < centroidalModelInfo_.numThreeDofContacts; i++) {
    desiredFeetPositionMsgs[i].reserve(stateTrajectory.size());
  }

  for (size_t j = 0; j < stateTrajectory.size(); j++) {
    const auto state = stateTrajectory.at(j);
    vector_t input(centroidalModelInfo_.inputDim);
    if (j < inputTrajectory.size()) {
      input = inputTrajectory.at(j);
    } else {
      input.setZero();
    }

    // Construct base pose msg
    const auto basePose = centroidal_model::getBasePose(state, centroidalModelInfo_);
    geometry_msgs::msg::Pose pose;
    pose.position = getPointMsg(basePose.head<3>());

    // Fill message containers
    desiredBasePositionMsg.push_back(pose.position);

    // Fill feet msgs
    const auto& model = pinocchioInterface_.getModel();
    auto& data = pinocchioInterface_.getData();
    pinocchio::forwardKinematics(model, data, centroidal_model::getGeneralizedCoordinates(state, centroidalModelInfo_));
    pinocchio::updateFramePlacements(model, data);

    const auto feetPositions = endEffectorKinematicsPtr_->getPosition(state);
    for (size_t i = 0; i < centroidalModelInfo_.numThreeDofContacts; i++) {
      geometry_msgs::msg::Pose footPose;
      footPose.position = getPointMsg(feetPositions[i]);
      desiredFeetPositionMsgs[i].push_back(footPose.position);
    }
  }

  // Headers
  auto comLineMsg = getLineMsg(std::move(desiredBasePositionMsg), Color::green, trajectoryLineWidth_);
  comLineMsg.header = getHeaderMsg(frameId_, timeStamp);
  comLineMsg.id = 0;

  // Publish
  costDesiredBasePositionPublisher_->publish(comLineMsg);
  for (size_t i = 0; i < centroidalModelInfo_.numThreeDofContacts; i++) {
    auto footLineMsg = getLineMsg(std::move(desiredFeetPositionMsgs[i]), feetColorMap_[i], trajectoryLineWidth_);
    footLineMsg.header = getHeaderMsg(frameId_, timeStamp);
    footLineMsg.id = 0;
    costDesiredFeetPositionPublishers_[i]->publish(footLineMsg);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LeggedRobotVisualizer::publishOptimizedStateTrajectory(const builtin_interfaces::msg::Time timeStamp,
                                                            const scalar_array_t& mpcTimeTrajectory,
                                                            const vector_array_t& mpcStateTrajectory,
                                                            const ModeSchedule& modeSchedule) {
  if (mpcTimeTrajectory.empty() || mpcStateTrajectory.empty()) {
    return;  // Nothing to publish
  }

  // Reserve Feet msg
  feet_array_t<std::vector<geometry_msgs::msg::Point>> feetMsgs;
  std::for_each(feetMsgs.begin(), feetMsgs.end(), [&](std::vector<geometry_msgs::msg::Point>& v) { v.reserve(mpcStateTrajectory.size()); });

  // Reserve Com Msg
  std::vector<geometry_msgs::msg::Point> mpcComPositionMsgs;
  mpcComPositionMsgs.reserve(mpcStateTrajectory.size());

  // Extract Com and Feet from state
  std::for_each(mpcStateTrajectory.begin(), mpcStateTrajectory.end(), [&](const vector_t& state) {
    const auto basePose = centroidal_model::getBasePose(state, centroidalModelInfo_);

    // Fill com position and pose msgs
    geometry_msgs::msg::Pose pose;
    pose.position = getPointMsg(basePose.head<3>());
    mpcComPositionMsgs.push_back(pose.position);

    // Fill feet msgs
    const auto& model = pinocchioInterface_.getModel();
    auto& data = pinocchioInterface_.getData();
    pinocchio::forwardKinematics(model, data, centroidal_model::getGeneralizedCoordinates(state, centroidalModelInfo_));
    pinocchio::updateFramePlacements(model, data);

    const auto feetPositions = endEffectorKinematicsPtr_->getPosition(state);
    for (size_t i = 0; i < centroidalModelInfo_.numThreeDofContacts; i++) {
      const auto position = getPointMsg(feetPositions[i]);
      feetMsgs[i].push_back(position);
    }
  });

  // Convert feet msgs to Array message
  visualization_msgs::msg::MarkerArray markerArray;
  markerArray.markers.reserve(centroidalModelInfo_.numThreeDofContacts +
                              2);  // 1 trajectory per foot + 1 for the future footholds + 1 for the com trajectory
  for (size_t i = 0; i < centroidalModelInfo_.numThreeDofContacts; i++) {
    markerArray.markers.emplace_back(getLineMsg(std::move(feetMsgs[i]), feetColorMap_[i], trajectoryLineWidth_));
    markerArray.markers.back().ns = "EE Trajectories";
  }
  markerArray.markers.emplace_back(getLineMsg(std::move(mpcComPositionMsgs), Color::red, trajectoryLineWidth_));
  markerArray.markers.back().ns = "CoM Trajectory";

  // Future footholds
  visualization_msgs::msg::Marker sphereList;
  sphereList.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  sphereList.scale.x = footMarkerDiameter_;
  sphereList.scale.y = footMarkerDiameter_;
  sphereList.scale.z = footMarkerDiameter_;
  sphereList.ns = "Future footholds";
  sphereList.pose.orientation = getOrientationMsg({1., 0., 0., 0.});
  const auto& eventTimes = modeSchedule.eventTimes;
  const auto& subsystemSequence = modeSchedule.modeSequence;
  const auto tStart = mpcTimeTrajectory.front();
  const auto tEnd = mpcTimeTrajectory.back();
  for (size_t event = 0; event < eventTimes.size(); ++event) {
    if (tStart < eventTimes[event] && eventTimes[event] < tEnd) {  // Only publish future footholds within the optimized horizon
      const auto preEventContactFlags = modeNumber2StanceLeg(subsystemSequence[event]);
      const auto postEventContactFlags = modeNumber2StanceLeg(subsystemSequence[event + 1]);
      const auto postEventState = LinearInterpolation::interpolate(eventTimes[event], mpcTimeTrajectory, mpcStateTrajectory);

      const auto& model = pinocchioInterface_.getModel();
      auto& data = pinocchioInterface_.getData();
      pinocchio::forwardKinematics(model, data, centroidal_model::getGeneralizedCoordinates(postEventState, centroidalModelInfo_));
      pinocchio::updateFramePlacements(model, data);

      const auto feetPosition = endEffectorKinematicsPtr_->getPosition(postEventState);
      for (size_t i = 0; i < centroidalModelInfo_.numThreeDofContacts; i++) {
        if (!preEventContactFlags[i] && postEventContactFlags[i]) {  // If a foot lands, a marker is added at that location.
          sphereList.points.emplace_back(getPointMsg(feetPosition[i]));
          sphereList.colors.push_back(getColor(feetColorMap_[i]));
        }
      }
    }
  }
  markerArray.markers.push_back(std::move(sphereList));

  // Add headers and Id
  assignHeader(markerArray.markers.begin(), markerArray.markers.end(), getHeaderMsg(frameId_, timeStamp));
  assignIncreasingId(markerArray.markers.begin(), markerArray.markers.end());

  stateOptimizedPublisher_->publish(markerArray);
}

}  // namespace legged_robot
}  // namespace ocs2
