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

#include <pinocchio/fwd.hpp>

#include <pinocchio/algorithm/frames-derivatives.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <ocs2_robotic_tools/common/AngularVelocityMapping.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <ocs2_robotic_tools/common/SkewSymmetricMatrix.h>

#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
PinocchioEndEffectorKinematics::PinocchioEndEffectorKinematics(const PinocchioInterface& pinocchioInterface,
                                                               const PinocchioStateInputMapping<scalar_t>& mapping,
                                                               std::vector<std::string> endEffectorIds)
    : pinocchioInterfacePtr_(nullptr), mappingPtr_(mapping.clone()), endEffectorIds_(std::move(endEffectorIds)) {
  for (const auto& bodyName : endEffectorIds_) {
    endEffectorFrameIds_.push_back(pinocchioInterface.getModel().getBodyId(bodyName));
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
PinocchioEndEffectorKinematics::PinocchioEndEffectorKinematics(const PinocchioEndEffectorKinematics& rhs)
    : EndEffectorKinematics<scalar_t>(rhs),
      pinocchioInterfacePtr_(nullptr),
      mappingPtr_(rhs.mappingPtr_->clone()),
      endEffectorIds_(rhs.endEffectorIds_),
      endEffectorFrameIds_(rhs.endEffectorFrameIds_) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
PinocchioEndEffectorKinematics* PinocchioEndEffectorKinematics::clone() const {
  return new PinocchioEndEffectorKinematics(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
const std::vector<std::string>& PinocchioEndEffectorKinematics::getIds() const {
  return endEffectorIds_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
auto PinocchioEndEffectorKinematics::getPosition(const vector_t& state) const -> std::vector<vector3_t> {
  if (pinocchioInterfacePtr_ == nullptr) {
    throw std::runtime_error("[PinocchioEndEffectorKinematics] pinocchioInterfacePtr_ is not set. Use setPinocchioInterface()");
  }

  const pinocchio::Data& data = pinocchioInterfacePtr_->getData();

  std::vector<vector3_t> positions;
  positions.reserve(endEffectorFrameIds_.size());
  for (const auto& frameId : endEffectorFrameIds_) {
    positions.emplace_back(data.oMf[frameId].translation());
  }
  return positions;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
auto PinocchioEndEffectorKinematics::getVelocity(const vector_t& state, const vector_t& input) const -> std::vector<vector3_t> {
  if (pinocchioInterfacePtr_ == nullptr) {
    throw std::runtime_error("[PinocchioEndEffectorKinematics] pinocchioInterfacePtr_ is not set. Use setPinocchioInterface()");
  }

  const pinocchio::ReferenceFrame rf = pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED;
  const pinocchio::Model& model = pinocchioInterfacePtr_->getModel();
  const pinocchio::Data& data = pinocchioInterfacePtr_->getData();

  std::vector<vector3_t> velocities;
  velocities.reserve(endEffectorFrameIds_.size());
  for (const auto& frameId : endEffectorFrameIds_) {
    velocities.emplace_back(pinocchio::getFrameVelocity(model, data, frameId, rf).linear());
  }
  return velocities;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::vector<VectorFunctionLinearApproximation> PinocchioEndEffectorKinematics::getPositionLinearApproximation(const vector_t& state) const {
  if (pinocchioInterfacePtr_ == nullptr) {
    throw std::runtime_error("[PinocchioEndEffectorKinematics] pinocchioInterfacePtr_ is not set. Use setPinocchioInterface()");
  }

  const pinocchio::ReferenceFrame rf = pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED;
  const pinocchio::Model& model = pinocchioInterfacePtr_->getModel();
  // const pinocchio::Data& data = pinocchioInterfacePtr_->getData();
  // TODO(mspieler): Need to copy here because getFrameJacobian() modifies data. Will be fixed in pinocchio version 3.
  pinocchio::Data data = pinocchio::Data(pinocchioInterfacePtr_->getData());

  std::vector<VectorFunctionLinearApproximation> positions;
  positions.reserve(endEffectorFrameIds_.size());
  for (const auto& frameId : endEffectorFrameIds_) {
    matrix_t J = matrix_t::Zero(6, model.nq);
    pinocchio::getFrameJacobian(model, data, frameId, rf, J);

    VectorFunctionLinearApproximation pos;
    pos.f = data.oMf[frameId].translation();
    std::tie(pos.dfdx, std::ignore) = mappingPtr_->getOcs2Jacobian(state, J.topRows<3>(), matrix_t::Zero(3, model.nv));
    positions.emplace_back(std::move(pos));
  }
  return positions;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::vector<VectorFunctionLinearApproximation> PinocchioEndEffectorKinematics::getVelocityLinearApproximation(const vector_t& state,
                                                                                                              const vector_t& input) const {
  if (pinocchioInterfacePtr_ == nullptr) {
    throw std::runtime_error("[PinocchioEndEffectorKinematics] pinocchioInterfacePtr_ is not set. Use setPinocchioInterface()");
  }

  const pinocchio::ReferenceFrame rf = pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED;
  const pinocchio::Model& model = pinocchioInterfacePtr_->getModel();
  // const pinocchio::Data& data = pinocchioInterfacePtr_->getData();
  // TODO(mspieler): Need to copy here because getFrameJacobian() modifies data. Will be fixed in pinocchio version 3.
  pinocchio::Data data = pinocchio::Data(pinocchioInterfacePtr_->getData());

  std::vector<VectorFunctionLinearApproximation> velocities;
  velocities.reserve(endEffectorFrameIds_.size());
  for (const auto& frameId : endEffectorFrameIds_) {
    matrix_t v_partial_dq = matrix_t::Zero(6, model.nv);
    matrix_t v_partial_dv = matrix_t::Zero(6, model.nv);
    pinocchio::getFrameVelocityDerivatives(model, data, frameId, rf, v_partial_dq, v_partial_dv);
    const auto frameVel = pinocchio::getFrameVelocity(model, data, frameId, rf);
    // For reference frame LOCAL_WORLD_ALIGNED the jacobian needs to be corrected.
    if (rf == pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED) {
      v_partial_dq.topRows<3>() += skewSymmetricMatrix(vector3_t(frameVel.angular())) * v_partial_dv.topRows<3>();
    }
    VectorFunctionLinearApproximation vel;
    vel.f = frameVel.linear();
    std::tie(vel.dfdx, vel.dfdu) = mappingPtr_->getOcs2Jacobian(state, v_partial_dq.topRows<3>(), v_partial_dv.topRows<3>());
    velocities.emplace_back(std::move(vel));
  }
  return velocities;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

auto PinocchioEndEffectorKinematics::getOrientation(const vector_t& state) const -> std::vector<quaternion_t> {
  if (pinocchioInterfacePtr_ == nullptr) {
    throw std::runtime_error("[PinocchioEndEffectorKinematics] pinocchioInterfacePtr_ is not set. Use setPinocchioInterface()");
  }

  const pinocchio::ReferenceFrame rf = pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED;
  const pinocchio::Data& data = pinocchioInterfacePtr_->getData();

  std::vector<quaternion_t> orientations;
  orientations.reserve(endEffectorFrameIds_.size());
  for (int i = 0; i < endEffectorFrameIds_.size(); i++) {
    const size_t frameId = endEffectorFrameIds_[i];
    orientations.emplace_back(matrixToQuaternion(data.oMf[frameId].rotation()));
  }
  return orientations;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
auto PinocchioEndEffectorKinematics::getOrientationError(const vector_t& state,
                                                         const std::vector<quaternion_t>& referenceOrientations) const
    -> std::vector<vector3_t> {
  if (pinocchioInterfacePtr_ == nullptr) {
    throw std::runtime_error("[PinocchioEndEffectorKinematics] pinocchioInterfacePtr_ is not set. Use setPinocchioInterface()");
  }

  const pinocchio::ReferenceFrame rf = pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED;
  const pinocchio::Data& data = pinocchioInterfacePtr_->getData();

  std::vector<vector3_t> errors;
  errors.reserve(endEffectorFrameIds_.size());
  for (int i = 0; i < endEffectorFrameIds_.size(); i++) {
    const size_t frameId = endEffectorFrameIds_[i];
    errors.emplace_back(quaternionDistance(matrixToQuaternion(data.oMf[frameId].rotation()), referenceOrientations[i]));
  }
  return errors;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

auto PinocchioEndEffectorKinematics::getOrientationErrorWrtPlane(const vector_t& state, const std::vector<vector3_t>& planeNormals) const
    -> std::vector<vector3_t> {
  if (pinocchioInterfacePtr_ == nullptr) {
    throw std::runtime_error("[PinocchioEndEffectorKinematics] pinocchioInterfacePtr_ is not set. Use setPinocchioInterface()");
  }

  const pinocchio::Data& data = pinocchioInterfacePtr_->getData();

  std::vector<vector3_t> errors;
  errors.reserve(endEffectorFrameIds_.size());
  for (int i = 0; i < endEffectorFrameIds_.size(); i++) {
    errors.emplace_back(rotationMatrixDistanceToPlane<scalar_t>(data.oMf[endEffectorFrameIds_[i]].rotation(), planeNormals[i]));
  }
  return errors;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

auto PinocchioEndEffectorKinematics::getAngularVelocity(const vector_t& state, const vector_t& input) const -> std::vector<vector3_t> {
  if (pinocchioInterfacePtr_ == nullptr) {
    throw std::runtime_error("[PinocchioEndEffectorKinematics] pinocchioInterfacePtr_ is not set. Use setPinocchioInterface()");
  }

  const pinocchio::ReferenceFrame rf = pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED;
  const pinocchio::Model& model = pinocchioInterfacePtr_->getModel();
  const pinocchio::Data& data = pinocchioInterfacePtr_->getData();

  std::vector<vector3_t> angularVelocities;
  angularVelocities.reserve(endEffectorFrameIds_.size());
  for (const auto& frameId : endEffectorFrameIds_) {
    angularVelocities.emplace_back(pinocchio::getFrameVelocity(model, data, frameId, rf).angular());
  }
  return angularVelocities;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

auto PinocchioEndEffectorKinematics::getTwist(const vector_t& state, const vector_t& input) const -> std::vector<vector6_t> {
  if (pinocchioInterfacePtr_ == nullptr) {
    throw std::runtime_error("[PinocchioEndEffectorKinematics] pinocchioInterfacePtr_ is not set. Use setPinocchioInterface()");
  }

  const pinocchio::ReferenceFrame rf = pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED;
  const pinocchio::Model& model = pinocchioInterfacePtr_->getModel();
  const pinocchio::Data& data = pinocchioInterfacePtr_->getData();

  std::vector<vector6_t> twists;
  twists.reserve(endEffectorFrameIds_.size());
  for (const auto& frameId : endEffectorFrameIds_) {
    auto motion = pinocchio::getFrameVelocity(model, data, frameId, rf);
    vector6_t currTwist;
    currTwist << motion.linear(), motion.angular();
    twists.emplace_back(currTwist);
  }
  return twists;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::vector<VectorFunctionLinearApproximation> PinocchioEndEffectorKinematics::getOrientationErrorLinearApproximation(
    const vector_t& state, const std::vector<quaternion_t>& referenceOrientations) const {
  if (pinocchioInterfacePtr_ == nullptr) {
    throw std::runtime_error("[PinocchioEndEffectorKinematics] pinocchioInterfacePtr_ is not set. Use setPinocchioInterface()");
  }

  const pinocchio::ReferenceFrame rf = pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED;
  const pinocchio::Model& model = pinocchioInterfacePtr_->getModel();
  // const pinocchio::Data& data = pinocchioInterfacePtr_->getData();
  // TODO(mspieler): Need to copy here because getFrameJacobian() modifies data. Will be fixed in pinocchio version 3.
  pinocchio::Data data = pinocchio::Data(pinocchioInterfacePtr_->getData());

  std::vector<VectorFunctionLinearApproximation> errors;
  errors.reserve(endEffectorFrameIds_.size());
  for (int i = 0; i < endEffectorFrameIds_.size(); i++) {
    VectorFunctionLinearApproximation err;
    const size_t frameId = endEffectorFrameIds_[i];
    const quaternion_t q = matrixToQuaternion(data.oMf[frameId].rotation());
    err.f = quaternionDistance(q, referenceOrientations[i]);
    matrix_t J = matrix_t::Zero(6, model.nq);
    pinocchio::getFrameJacobian(model, data, frameId, rf, J);
    const matrix_t Jqdist =
        (quaternionDistanceJacobian(q, referenceOrientations[i]) * angularVelocityToQuaternionTimeDerivative(q)) * J.bottomRows<3>();
    std::tie(err.dfdx, std::ignore) = mappingPtr_->getOcs2Jacobian(state, Jqdist, matrix_t::Zero(3, model.nv));
    errors.emplace_back(std::move(err));
  }
  return errors;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

std::vector<VectorFunctionLinearApproximation> PinocchioEndEffectorKinematics::getOrientationErrorWrtPlaneLinearApproximation(
    const vector_t& state, const std::vector<vector3_t>& planeNormals) const {
  if (pinocchioInterfacePtr_ == nullptr) {
    throw std::runtime_error("[PinocchioEndEffectorKinematics] pinocchioInterfacePtr_ is not set. Use setPinocchioInterface()");
  }

  const pinocchio::ReferenceFrame rf = pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED;
  pinocchio::Data data = pinocchioInterfacePtr_->getData();
  const pinocchio::Model& model = pinocchioInterfacePtr_->getModel();

  std::vector<VectorFunctionLinearApproximation> errors;
  errors.reserve(endEffectorFrameIds_.size());
  for (int i = 0; i < endEffectorFrameIds_.size(); i++) {
    const size_t frameId = endEffectorFrameIds_[i];
    auto R = data.oMf[frameId].rotation();
    Eigen::Quaternion<scalar_t> q_frame = matrixToQuaternion(R);
    // Compute the quaternion that rotates the z-axis of the end-effector to the normal of the plane.
    Eigen::Quaternion<scalar_t> q_correction;
    q_correction.setFromTwoVectors(R * planeNormals[i], planeNormals[i]);
    // Compute corrected end effector orientation where x and y axis lie in the plane.
    Eigen::Quaternion<scalar_t> q_reference = q_frame * q_correction;

    VectorFunctionLinearApproximation err;
    err.f = quaternionDistance(q_frame, q_reference);
    matrix_t J = matrix_t::Zero(6, model.nq);
    pinocchio::getFrameJacobian(model, data, frameId, rf, J);
    const matrix_t Jqdist =
        (quaternionDistanceJacobian(q_frame, q_reference) * angularVelocityToQuaternionTimeDerivative(q_frame)) * J.bottomRows<3>();
    std::tie(err.dfdx, std::ignore) = mappingPtr_->getOcs2Jacobian(state, Jqdist, matrix_t::Zero(0, model.nv));
    errors.emplace_back(std::move(err));
  }
  return errors;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

std::vector<VectorFunctionLinearApproximation> PinocchioEndEffectorKinematics::getAngularVelocityLinearApproximation(
    const vector_t& state, const vector_t& input) const {
  if (pinocchioInterfacePtr_ == nullptr) {
    throw std::runtime_error("[PinocchioEndEffectorKinematics] pinocchioInterfacePtr_ is not set. Use setPinocchioInterface()");
  }

  const pinocchio::ReferenceFrame rf = pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED;
  const pinocchio::Model& model = pinocchioInterfacePtr_->getModel();
  // const pinocchio::Data& data = pinocchioInterfacePtr_->getData();
  // TODO(mspieler): Need to copy here because getFrameJacobian() modifies data. Will be fixed in pinocchio version 3.
  pinocchio::Data data = pinocchio::Data(pinocchioInterfacePtr_->getData());

  std::vector<VectorFunctionLinearApproximation> angularVelocities;
  angularVelocities.reserve(endEffectorFrameIds_.size());
  for (const auto& frameId : endEffectorFrameIds_) {
    matrix_t v_partial_dq = matrix_t::Zero(6, model.nv);
    matrix_t v_partial_dv = matrix_t::Zero(6, model.nv);
    pinocchio::getFrameVelocityDerivatives(model, data, frameId, rf, v_partial_dq, v_partial_dv);
    const auto frameVel = pinocchio::getFrameVelocity(model, data, frameId, rf);
    // For reference frame LOCAL_WORLD_ALIGNED the jacobian needs to be corrected.
    if (rf == pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED) {
      v_partial_dq.topRows<3>() += skewSymmetricMatrix(vector3_t(frameVel.angular())) * v_partial_dv.topRows<3>();
    }
    VectorFunctionLinearApproximation vel;
    vel.f = frameVel.angular();
    std::tie(vel.dfdx, vel.dfdu) = mappingPtr_->getOcs2Jacobian(state, v_partial_dq.bottomRows<3>(), v_partial_dv.bottomRows<3>());
    angularVelocities.emplace_back(std::move(vel));
  }
  return angularVelocities;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::vector<VectorFunctionLinearApproximation> PinocchioEndEffectorKinematics::getTwistLinearApproximation(const vector_t& state,
                                                                                                           const vector_t& input) const {
  if (pinocchioInterfacePtr_ == nullptr) {
    throw std::runtime_error("[PinocchioEndEffectorKinematics] pinocchioInterfacePtr_ is not set. Use setPinocchioInterface()");
  }

  const pinocchio::ReferenceFrame rf = pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED;
  const pinocchio::Model& model = pinocchioInterfacePtr_->getModel();
  // const pinocchio::Data& data = pinocchioInterfacePtr_->getData();
  // TODO(mspieler): Need to copy here because getFrameJacobian() modifies data. Will be fixed in pinocchio version 3.
  pinocchio::Data data = pinocchio::Data(pinocchioInterfacePtr_->getData());

  std::vector<VectorFunctionLinearApproximation> twists;
  twists.reserve(endEffectorFrameIds_.size());
  for (const auto& frameId : endEffectorFrameIds_) {
    matrix_t v_partial_dq = matrix_t::Zero(6, model.nv);
    matrix_t v_partial_dv = matrix_t::Zero(6, model.nv);
    pinocchio::getFrameVelocityDerivatives(model, data, frameId, rf, v_partial_dq, v_partial_dv);
    const auto frameVel = pinocchio::getFrameVelocity(model, data, frameId, rf);
    // For reference frame LOCAL_WORLD_ALIGNED the jacobian needs to be corrected.
    if (rf == pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED) {
      v_partial_dq.topRows<3>() += skewSymmetricMatrix(vector3_t(frameVel.angular())) * v_partial_dv.topRows<3>();
    }
    VectorFunctionLinearApproximation vel;
    vector6_t currFrameVel;
    currFrameVel << frameVel.linear(), frameVel.angular();
    vel.f = currFrameVel;
    std::tie(vel.dfdx, vel.dfdu) = mappingPtr_->getOcs2Jacobian(state, v_partial_dq, v_partial_dv);
    twists.emplace_back(std::move(vel));
  }
  return twists;
}

}  // namespace ocs2
