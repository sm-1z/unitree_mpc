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

#include <ocs2_core/penalties/penalties/PieceWisePolynomialBarrierPenalty.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t PieceWisePolynomialBarrierPenalty::getValue(scalar_t t, scalar_t h) const {
  if (h <= 0) {
    scalar_t penalty = 0.5 * h * h - config_.delta * h / 2 + config_.delta * config_.delta / 6;
    return penalty * config_.mu;
  } else if (h > 0 && h < config_.delta) {
    scalar_t penalty = -h * h * h / (6 * config_.delta) + 0.5 * h * h - config_.delta * h / 2 + config_.delta * config_.delta / 6;
    return penalty * config_.mu;
  } else {
    return 0;
  };
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t PieceWisePolynomialBarrierPenalty::getDerivative(scalar_t t, scalar_t h) const {
  if (h <= 0) {
    scalar_t penalty = h - config_.delta / 2;
    return penalty * config_.mu;
  } else if (h > 0 && h < config_.delta) {
    scalar_t penalty = -h * h / (2 * config_.delta) + h - config_.delta / 2;
    return penalty * config_.mu;
  } else {
    return 0;
  };
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t PieceWisePolynomialBarrierPenalty::getSecondDerivative(scalar_t t, scalar_t h) const {
  if (h <= 0) {
    return config_.mu;
  } else if (h > 0 && h < config_.delta) {
    return config_.mu * (-h / config_.delta + 1);
  } else {
    return 0;
  };
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void PieceWisePolynomialBarrierPenalty::setParameters(const vector_t& parameters) {
  if (parameters.size() != 2) {
    throw std::runtime_error("PieceWisePolynomialBarrierPenalty::setParameters: Invalid number of parameters.");
  }
  config_.mu = parameters[0];
  config_.delta = parameters[1];
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void PieceWisePolynomialBarrierPenalty::getParameters(vector_t& parameters) const {
  parameters.resize(2);
  parameters[0] = config_.mu;
  parameters[1] = config_.delta;
}

}  // namespace ocs2
