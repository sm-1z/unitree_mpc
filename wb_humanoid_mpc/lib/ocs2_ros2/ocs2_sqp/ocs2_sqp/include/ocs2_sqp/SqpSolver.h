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

#pragma once

#include <ocs2_core/initialization/Initializer.h>
#include <ocs2_core/integration/SensitivityIntegrator.h>
#include <ocs2_core/misc/Benchmark.h>
#include <ocs2_core/thread_support/ThreadPool.h>

#include <ocs2_oc/multiple_shooting/ProjectionMultiplierCoefficients.h>
#include <ocs2_oc/oc_data/TimeDiscretization.h>
#include <ocs2_oc/oc_problem/OptimalControlProblem.h>
#include <ocs2_oc/oc_solver/SolverBase.h>
#include <ocs2_oc/search_strategy/FilterLinesearch.h>

#include <hpipm_catkin/HpipmInterface.h>

#include "ocs2_sqp/SqpLogging.h"
#include "ocs2_sqp/SqpSettings.h"
#include "ocs2_sqp/SqpSolverStatus.h"

namespace ocs2 {

class SqpSolver : public SolverBase {
 public:
  /**
   * Constructor
   *
   * @param settings : settings for the multiple shooting SQP solver.
   * @param [in] optimalControlProblem: The optimal control problem formulation.
   * @param [in] initializer: This class initializes the state-input for the time steps that no controller is available.
   */
  SqpSolver(sqp::Settings settings, const OptimalControlProblem& optimalControlProblem, const Initializer& initializer);

  ~SqpSolver() override;

  void reset() override;

  scalar_t getFinalTime() const override { return primalSolution_.timeTrajectory_.back(); };

  void getPrimalSolution(scalar_t finalTime, PrimalSolution* primalSolutionPtr) const override { *primalSolutionPtr = primalSolution_; }

  const ProblemMetrics& getSolutionMetrics() const override { return problemMetrics_; }

  size_t getNumIterations() const override { return totalNumIterations_; }

  const OptimalControlProblem& getOptimalControlProblem() const override { return ocpDefinitions_.front(); }

  const PerformanceIndex& getPerformanceIndeces() const override { return getIterationsLog().back(); };

  const std::vector<PerformanceIndex>& getIterationsLog() const override;

  ScalarFunctionQuadraticApproximation getValueFunction(scalar_t time, const vector_t& state) const override;

  ScalarFunctionQuadraticApproximation getHamiltonian(scalar_t time, const vector_t& state, const vector_t& input) override {
    throw std::runtime_error("[SqpSolver] getHamiltonian() not available yet.");
  }

  vector_t getStateInputEqualityConstraintLagrangian(scalar_t time, const vector_t& state) const override {
    throw std::runtime_error("[SqpSolver] getStateInputEqualityConstraintLagrangian() not available yet.");
  }

  MultiplierCollection getIntermediateDualSolution(scalar_t time) const override {
    throw std::runtime_error("[SqpSolver] getIntermediateDualSolution() not available yet.");
  }

  std::vector<OptimalControlProblem>& getOcpDefinitions() { return ocpDefinitions_; }

  /** All timings are expressed in milliseconds */
  struct Benchmarks {
    scalar_t linearQuadraticApproximationTime;
    scalar_t solveQpTime;
    scalar_t linesearchTime;
    scalar_t computeControllerTime;
  };
  Benchmarks getBenchmarks() const;

 private:
  void runImpl(scalar_t initTime, const vector_t& initState, scalar_t finalTime) override;

  void runImpl(scalar_t initTime, const vector_t& initState, scalar_t finalTime, const ControllerBase* externalControllerPtr) override {
    if (externalControllerPtr == nullptr) {
      runImpl(initTime, initState, finalTime);
    } else {
      throw std::runtime_error("[SqpSolver::run] This solver does not support external controller!");
    }
  }

  void runImpl(scalar_t initTime, const vector_t& initState, scalar_t finalTime, const PrimalSolution& primalSolution) override {
    // Copy all except the controller
    primalSolution_.timeTrajectory_ = primalSolution.timeTrajectory_;
    primalSolution_.stateTrajectory_ = primalSolution.stateTrajectory_;
    primalSolution_.inputTrajectory_ = primalSolution.inputTrajectory_;
    primalSolution_.postEventIndices_ = primalSolution.postEventIndices_;
    primalSolution_.modeSchedule_ = primalSolution.modeSchedule_;
    runImpl(initTime, initState, finalTime);
  }

  /** Run a task in parallel with settings.nThreads */
  void runParallel(std::function<void(int)> taskFunction);

  /** Get profiling information as a string */
  std::string getBenchmarkingInformation() const;

  /** Creates QP around t, x(t), u(t). Returns performance metrics at the current {t, x(t), u(t)} */
  PerformanceIndex setupQuadraticSubproblem(const std::vector<AnnotatedTime>& time, const vector_t& initState, const vector_array_t& x,
                                            const vector_array_t& u, std::vector<Metrics>& metrics);

  /** Computes only the performance metrics at the current {t, x(t), u(t)} */
  PerformanceIndex computePerformance(const std::vector<AnnotatedTime>& time, const vector_t& initState, const vector_array_t& x,
                                      const vector_array_t& u, std::vector<Metrics>& metrics);

  /** Returns solution of the QP subproblem in delta coordinates: */
  struct OcpSubproblemSolution {
    vector_array_t deltaXSol;      // delta_x(t)
    vector_array_t deltaUSol;      // delta_u(t)
    scalar_t armijoDescentMetric;  // inner product of the cost gradient and decision variable step
  };
  OcpSubproblemSolution getOCPSolution(const vector_t& delta_x0);

  /** Extract the value function based on the last solved QP */
  void extractValueFunction(const std::vector<AnnotatedTime>& time, const vector_array_t& x);

  /** Constructs the primal solution based on the optimized state and input trajectories */
  PrimalSolution toPrimalSolution(const std::vector<AnnotatedTime>& time, vector_array_t&& x, vector_array_t&& u);

  /** Decides on the step to take and overrides given trajectories {x(t), u(t)} <- {x(t) + a*dx(t), u(t) + a*du(t)} */
  sqp::StepInfo takeStep(const PerformanceIndex& baseline, const std::vector<AnnotatedTime>& timeDiscretization, const vector_t& initState,
                         const OcpSubproblemSolution& subproblemSolution, vector_array_t& x, vector_array_t& u,
                         std::vector<Metrics>& metrics);

  /** Determine convergence after a step */
  sqp::Convergence checkConvergence(int iteration, const PerformanceIndex& baseline, const sqp::StepInfo& stepInfo) const;

  // Problem definition
  const sqp::Settings settings_;
  DynamicsDiscretizer discretizer_;
  DynamicsSensitivityDiscretizer sensitivityDiscretizer_;
  std::vector<OptimalControlProblem> ocpDefinitions_;
  std::unique_ptr<Initializer> initializerPtr_;
  FilterLinesearch filterLinesearch_;

  // Solver interface
  HpipmInterface hpipmInterface_;

  // Threading
  ThreadPool threadPool_;

  // Solution
  PrimalSolution primalSolution_;

  // Value function in absolute state coordinates (without the constant value)
  std::vector<ScalarFunctionQuadraticApproximation> valueFunction_;

  // LQ approximation
  std::vector<ScalarFunctionQuadraticApproximation> cost_;
  std::vector<VectorFunctionLinearApproximation> dynamics_;
  std::vector<VectorFunctionLinearApproximation> stateInputEqConstraints_;
  std::vector<VectorFunctionLinearApproximation> stateIneqConstraints_;
  std::vector<VectorFunctionLinearApproximation> stateInputIneqConstraints_;
  std::vector<VectorFunctionLinearApproximation> constraintsProjection_;

  // Lagrange multipliers
  std::vector<multiple_shooting::ProjectionMultiplierCoefficients> projectionMultiplierCoefficients_;

  // Iteration performance log
  std::vector<PerformanceIndex> performanceIndeces_;

  // The ProblemMetrics associated to primalSolution_
  ProblemMetrics problemMetrics_;

  // Benchmarking
  size_t numProblems_{0};
  size_t totalNumIterations_{0};
  sqp::Logger<sqp::LogEntry> logger_;
  benchmark::RepeatedTimer initializationTimer_;
  benchmark::RepeatedTimer linearQuadraticApproximationTimer_;
  benchmark::RepeatedTimer solveQpTimer_;
  benchmark::RepeatedTimer linesearchTimer_;
  benchmark::RepeatedTimer computeControllerTimer_;
};

}  // namespace ocs2
