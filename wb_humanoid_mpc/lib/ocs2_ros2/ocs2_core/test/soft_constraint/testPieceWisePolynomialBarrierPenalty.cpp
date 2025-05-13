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
#include <random>

#include <gtest/gtest.h>

#include <ocs2_core/penalties/penalties/PieceWisePolynomialBarrierPenalty.h>

TEST(testPieceWisePolynomialBarrierPenalty, testPenalty) {
  // Define the range
  int min_val = 0.0001;
  int max_val = 100;

  // Create a random engine and a distribution
  std::random_device rd;                                    // Obtain a random number from hardware
  std::mt19937 eng(rd());                                   // Seed the generator
  std::uniform_int_distribution<> distr(min_val, max_val);  // Define the range
  for (int i = 0; i < 50; i++) {
    ocs2::scalar_t mu = distr(eng);
    ocs2::scalar_t delta = distr(eng);
    ocs2::PieceWisePolynomialBarrierPenalty::Config config(mu, delta);
    ocs2::PieceWisePolynomialBarrierPenalty penalty(config);
    for (int j = 0; j < 10; j++) {
      ocs2::scalar_t x = distr(eng);
      EXPECT_TRUE(penalty.getValue(0.0, x) >= 0);
      EXPECT_TRUE(penalty.getDerivative(0.0, x) <= 0);
      EXPECT_TRUE(penalty.getSecondDerivative(0.0, x) >= 0);
      EXPECT_NEAR(penalty.getValue(0.0, 0.0), config.mu * config.delta * config.delta / 6, 1e-4);  // Test zero crossing
      ASSERT_DOUBLE_EQ(penalty.getValue(0.0, config.delta), 0.0);                                  // Test delta crossing
      ASSERT_DOUBLE_EQ(penalty.getValue(0.0, config.delta + distr(eng)), 0.0);                     // Test past delta crossing
    }
  }
}