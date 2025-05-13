// Haolin Ma 2025-04-10 21:57:39

#pragma once

#include <fstream>
#include <iomanip>  // for std::setprecision

#include "g1_hardware_interface/HardwareUnitreeG1.h"
#include "humanoid_wb_mpc/common/WBAccelMpcRobotModel.h"
#include "ocs2_pinocchio_interface/PinocchioInterface.h"
#include "ocs2_ros2_interfaces/mrt/DummyObserver.h"
#include "ocs2_ros2_interfaces/mrt/MRT_ROS_Interface.h"
#include "ocs2_core/Types.h"
#include "humanoid_common_mpc/reference_manager/BreakFrequencyAlphaFilter.h"
#include "humanoid_common_mpc/reference_manager/MedianFilter.h"

#define ENABLE_SIMULATE true

namespace ocs2::humanoid {

/**
 * This class implements a loop to test MPC-MRT communication interface using ROS.
 */
class MRT_ROS_Real_Loop {
 public:
  /**
   * Constructor.
   *
   * @param [in] mrt: The underlying MRT class to be used. If MRT contains a rollout object, the dummy will roll out
   * the received controller using the MRT::rolloutPolicy() method instead of just sending back a planned state.
   * @param [in] mrtDesiredFrequency: MRT loop frequency in Hz. This should always set to a positive number.
   * @param [in] mpcDesiredFrequency: MPC loop frequency in Hz. If set to a positive number, MPC loop
   * will be simulated to run by this frequency. Note that this might not be the MPC's real-time frequency.
   */
  MRT_ROS_Real_Loop(MRT_ROS_Interface& mrt,
                    HardwareUnitreeG1& hardware_interface,
                    const PinocchioInterface& pinocchioInterface,
                    const WBAccelMpcRobotModel<scalar_t>& mpcRobotModel,
                    scalar_t mrtDesiredFrequency,
                    scalar_t mpcDesiredFrequency = -1);

  /**
   * Destructor.
   */
  virtual ~MRT_ROS_Real_Loop() = default;

  /**
   * Runs the real MRT loop.
   *
   * @param [in] initObservation: The initial observation.
   * @param [in] initTargetTrajectories: The initial TargetTrajectories.
   */
  void run(const SystemObservation& initObservation, const TargetTrajectories& initTargetTrajectories);

  /**
   * Subscribe a set of observers to the dummy loop. Observers are updated in the provided order at the end of each timestep.
   * The previous list of observers is overwritten.
   *
   * @param observers : vector of observers.
   */
  void subscribeObservers(const std::vector<std::shared_ptr<DummyObserver>>& observers) { observers_ = observers; }

 protected:
  /**
   * A user-defined function which modifies the observation before publishing.
   *
   * @param [in] observation: The current observation.
   */
  virtual void modifyObservation(SystemObservation& observation) {}

 private:
  /**
   * Runs a loop where mpc optimizations are synchronized with the forward simulation of the system
   */
  void synchronizedRealLoop(const SystemObservation& initObservation, const TargetTrajectories& initTargetTrajectories, const scalar_t& start_bias_time);

  /**
   * Runs a loop where mpc optimizations and simulation of the system are asynchronous.
   * The simulation runs as the specified mrtFrequency, and the MPC runs as fast as possible.
   */
  void realtimeRealLoop(const SystemObservation& initObservation, const TargetTrajectories& initTargetTrajectories, const scalar_t& start_bias_time);

  /** Forward tests the system from current observation*/
  SystemObservation forwardReal(const SystemObservation& currentObservation, const scalar_t& start_bias_time);

  MRT_ROS_Interface& mrt_;
  std::vector<std::shared_ptr<DummyObserver>> observers_;

  HardwareUnitreeG1& hardware_interface_;

  scalar_t mrtDesiredFrequency_;
  scalar_t mpcDesiredFrequency_;

  PinocchioInterface pinocchioInterface_;
  WBAccelMpcRobotModel<scalar_t>* mpcRobotModelPtr_;

  std::ofstream logFile_;

  MedianFilter optimalInputFilter_;
  MedianFilter optimalStateFilter_;

  scalar_t mpc_start_time_ = 15.0;
  size_t justUpdatedCounter_ = 0;
  const size_t justUpdatedThreshold_ = 30;
};

}  // namespace ocs2
