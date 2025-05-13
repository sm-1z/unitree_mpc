// Haolin Ma 2025-04-10 21:57:48

#include "humanoid_wb_mpc/mrt/MRT_ROS_Real_Loop.h"

#include "humanoid_wb_mpc/dynamics/DynamicsHelperFunctions.h"

#include "g1_hardware_interface/motor_crc_hg.h"



namespace ocs2::humanoid {

static auto LOGGER = rclcpp::get_logger("MRT_ROS_Real_Loop");

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
MRT_ROS_Real_Loop::MRT_ROS_Real_Loop(
    MRT_ROS_Interface& mrt, HardwareUnitreeG1& hardware_interface,
    const PinocchioInterface& pinocchioInterface,
    const WBAccelMpcRobotModel<scalar_t>& mpcRobotModel,
    scalar_t mrtDesiredFrequency, scalar_t mpcDesiredFrequency)
    : mrt_(mrt),
      hardware_interface_(hardware_interface),
      pinocchioInterface_(pinocchioInterface),
      mpcRobotModelPtr_(mpcRobotModel.clone()),
      mrtDesiredFrequency_(mrtDesiredFrequency),
      mpcDesiredFrequency_(mpcDesiredFrequency),
      optimalInputFilter_(10.0, 1, vector_t::Zero(mpcRobotModelPtr_->getInputDim())),
      optimalStateFilter_ (5.0, 5, vector_t::Zero(mpcRobotModelPtr_->getStateDim())){
    if (mrtDesiredFrequency_ < 0) {
        throw std::runtime_error(
            "MRT loop frequency should be a positive number.");
    }

    if (mpcDesiredFrequency_ > 0) {
        RCLCPP_WARN_STREAM(LOGGER,
                           "MPC loop is not realtime! For realtime setting, "
                           "set mpcDesiredFrequency to any negative number.");
    }

    // 在构造函数或 run() 函数的开头打开文件
    logFile_.open("./log/mpc_debug_log.csv");
    logFile_ << "time,state,input\n";  // CSV 表头
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MRT_ROS_Real_Loop::run(const SystemObservation& initObservation, const TargetTrajectories& initTargetTrajectories) {
  RCLCPP_INFO_STREAM(LOGGER, "Waiting for the initial policy ...");

  const double timeStep = (1.0 / mrtDesiredFrequency_);
  // Reset MPC node
  mrt_.resetMpcNode(initTargetTrajectories);

  // Wait for the initial policy
  while (!mrt_.initialPolicyReceived() && rclcpp::ok()) {
    mrt_.spinMRT();
    mrt_.setCurrentObservation(initObservation);

    rclcpp::sleep_for(std::chrono::nanoseconds(int(timeStep * 1e9)));
  }

  RCLCPP_INFO_STREAM(LOGGER, "Initial policy has been received.");

  scalar_t start_bias_time = hardware_interface_.getTime();

  // Pick simulation loop mode
  if (mpcDesiredFrequency_ > 0.0) {
    synchronizedRealLoop(initObservation, initTargetTrajectories,
    start_bias_time);
  } else {
    realtimeRealLoop(initObservation, initTargetTrajectories,
    start_bias_time);
  }

}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MRT_ROS_Real_Loop::synchronizedRealLoop(const SystemObservation& initObservation, const TargetTrajectories& initTargetTrajectories, const scalar_t& start_bias_time) {
  // Determine the ratio between MPC updates and simulation steps.
  const auto mpcUpdateRatio = std::max(static_cast<size_t>(mrtDesiredFrequency_ / mpcDesiredFrequency_), size_t(1));

  // Loop variables
  size_t loopCounter = 0;
  SystemObservation currentObservation = initObservation;

  // Helper function to check if policy is updated and starts at the given time.
  // Due to ROS message conversion delay and very fast MPC loop, we might get an old policy instead of the latest one.
  const auto policyUpdatedForTime = [this](scalar_t time) {
    constexpr scalar_t tol = 0.1;  // policy must start within this fraction of dt
    return mrt_.updatePolicy() && std::abs(mrt_.getPolicy().timeTrajectory_.front() - time) < (tol / mpcDesiredFrequency_);
  };

  rclcpp::Rate simRate(mrtDesiredFrequency_);
  while (rclcpp::ok()) {
    std::cout << "### Current time " << currentObservation.time << "\n";

    // Trigger MRT callbacks
    mrt_.spinMRT();

    // Update the MPC policy if it is time to do so
    if (loopCounter % mpcUpdateRatio == 0) {
      // Wait for the policy to be updated
      while (!policyUpdatedForTime(currentObservation.time) && rclcpp::ok()) {
        mrt_.spinMRT();
      }
      std::cout << "<<< New MPC policy starting at " << mrt_.getPolicy().timeTrajectory_.front() << "\n";
    }

    // Forward simulation
    currentObservation = forwardReal(currentObservation, start_bias_time);

    // User-defined modifications before publishing
    modifyObservation(currentObservation);

    // Publish observation if at the next step we want a new policy
    if ((loopCounter + 1) % mpcUpdateRatio == 0) {
      mrt_.setCurrentObservation(currentObservation);
      std::cout << ">>> Observation is published at " << currentObservation.time << "\n";
    }

    // mrt_.setCurrentObservation(currentObservation);

    // Update observers
    for (auto& observer : observers_) {
      observer->update(currentObservation, mrt_.getPolicy(), mrt_.getCommand());
    }

    ++loopCounter;
    simRate.sleep();
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MRT_ROS_Real_Loop::realtimeRealLoop(const SystemObservation& initObservation, const TargetTrajectories& initTargetTrajectories, const scalar_t& start_bias_time) {
  // Loop variables
  SystemObservation currentObservation = initObservation;

  const scalar_t maxPolicyTimeLag = 0.02;  // 最大允许的策略时间滞后（单位：秒）

  rclcpp::Rate simRate(mrtDesiredFrequency_);
  while (rclcpp::ok()) {
    std::cout << "### Current time " << currentObservation.time << "\n";

    // Trigger MRT callbacks
    mrt_.spinMRT();

    // // Update the policy if a new on was received
    // if (mrt_.updatePolicy()) {
    //   std::cout << "<<< New MPC policy starting at " << mrt_.getPolicy().timeTrajectory_.front() << "\n";
    // }

    // RCLCPP_INFO_STREAM(LOGGER,
    // "Current obs time: " << currentObservation.time <<
    // ", Policy start time: " << mrt_.getPolicy().timeTrajectory_.front() <<
    // ", Time diff: " << (currentObservation.time - mrt_.getPolicy().timeTrajectory_.front()));

    // Update the policy if a new one was received and time is valid
    bool policyUpdated = mrt_.updatePolicy();
    if (policyUpdated) {
        scalar_t policyTime = mrt_.getPolicy().timeTrajectory_.front();
        scalar_t timeDiff = currentObservation.time - policyTime;

        RCLCPP_INFO_STREAM(LOGGER, "Current obs time: "
                                       << currentObservation.time
                                       << ", Policy start time: " << policyTime
                                       << ", Time diff: " << timeDiff);

        if (std::abs(timeDiff) > maxPolicyTimeLag) {
            RCLCPP_WARN_STREAM(
                LOGGER, "Policy time mismatch! Skipping this policy update.");
            policyUpdated = false;  // 忽略本次策略
        } else {
            std::cout << "<<< New MPC policy accepted, starting at "
                      << policyTime << "\n";
        }
    }

    // Forward simulation
    currentObservation = forwardReal(currentObservation, start_bias_time);

    // User-defined modifications before publishing
    modifyObservation(currentObservation);

    // Publish observation
    mrt_.setCurrentObservation(currentObservation);

    // Update observers
    for (auto& observer : observers_) {
      observer->update(currentObservation, mrt_.getPolicy(), mrt_.getCommand());
    }

    simRate.sleep();
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
SystemObservation MRT_ROS_Real_Loop::forwardReal(const SystemObservation& currentObservation, const scalar_t& start_bias_time) {
  const scalar_t dt = 1.0 / mrtDesiredFrequency_;
  const scalar_t hardware_time = hardware_interface_.getTime() - start_bias_time;

  SystemObservation nextObservation;

  scalar_t internal_query_time = currentObservation.time + dt; // 计算下一步应该查询的时间
  const scalar_t max_allowable_time_lag = 0.5 * dt;  // 半个dt的容忍，比如dt=2ms，容忍1ms
  scalar_t time_lag = hardware_time - internal_query_time;
  if (std::abs(time_lag) > max_allowable_time_lag) {
      // RCLCPP_WARN_STREAM(
      //     LOGGER, "[TIME SYNC] Detected MPC time drift: "
      //                 << "hardware_time = " << hardware_time
      //                 << ", internal_query_time = " << internal_query_time
      //                 << ", lag = " << time_lag << ". Forcing resync!");
      internal_query_time = hardware_time;  // 强制对齐
  }

  // nextObservation.time = currentObservation.time + dt;
  
  
  // if (mrt_.isRolloutSet()) {  // If available, use the provided rollout as to integrate the dynamics.
  //   mrt_.rolloutPolicy(currentObservation.time, currentObservation.state, dt, nextObservation.state, nextObservation.input,
  //                      nextObservation.mode);
  // } else {  // Otherwise, we fake integration by interpolating the current MPC policy at t+dt
  //   mrt_.evaluatePolicy(currentObservation.time + dt, currentObservation.state, nextObservation.state, nextObservation.input,
  //                       nextObservation.mode);
  // }

  mrt_.evaluatePolicy(internal_query_time, //hardware_interface_.getTime() - start_bias_time,
                      hardware_interface_.getState(), nextObservation.state,
                      nextObservation.input, nextObservation.mode);
                      
  // == 1. 定义静态变量以记录状态 ==
  static bool contact_last[2] = {false, false};
  static double contact_t0[2] = {0.0, 0.0};
  double transitionDuration = 0.02;  // [s]

  auto hermite = [](double t, double t0, double duration) {
      double s = std::clamp((t - t0) / duration, 0.2, 1.0);
      return 3 * s * s - 2 * s * s * s;
  };
  // == 2. 获取当前离散接触状态 ==
  std::vector<bool> rawContact = hardware_interface_.getContactState();
  std::vector<double> smoothedContact(2, 0.0);

  // == 3. 逐脚进行平滑插值 ==
  for (int leg = 0; leg < 2; ++leg) {
      if (rawContact[leg] && !contact_last[leg]) {
          contact_t0[leg] = hardware_time;  // 记录接触开始时间
      }
      contact_last[leg] = rawContact[leg];

      smoothedContact[leg] = rawContact[leg] ? hermite(hardware_time, contact_t0[leg], transitionDuration) : 0.2;
  }

  scalar_t time_duration = 65.0;
  if (ENABLE_SIMULATE) {
      time_duration = 15.0;
  } else {
      time_duration = 65.0;
  }

  // if (hardware_interface_.getMPCMode() && hardware_time > time_duration) {
  //     for (int i = 0; i < 6; ++i) {  // left foot
  //       if (i == 2 || i == 4) continue;  // 不乘以平滑因子
  //       nextObservation.input(i) *= smoothedContact[0];
  //     }
  //     for (int i = 6; i < 12; ++i) {  // right foot
  //       if (i == 8|| i == 10) continue;       // 不乘以平滑因子
  //       nextObservation.input(i) *= smoothedContact[1];
  //     }
  // }

  // RCLCPP_INFO_STREAM(LOGGER, "[INFO]Current Time: "
  //                                << hardware_time
  //                                << ", Smooth param: " << smoothedContact[0]
  //                                << ", " << smoothedContact[1]);

  nextObservation.input = optimalInputFilter_.getFilteredVector(nextObservation.input);

  nextObservation.state = optimalStateFilter_.getFilteredVector(nextObservation.state);


  // 计算关节力矩
  vector_t jointTorques = computeJointTorques(nextObservation.state, nextObservation.input, pinocchioInterface_, *mpcRobotModelPtr_);

  // 写成逐项写入，每个数字都写入一个 CSV 列
  logFile_ << std::fixed << std::setprecision(6)
           << (hardware_interface_.getTime());

  for (int i = 0; i < nextObservation.state.size(); ++i) {
      logFile_ << "," << nextObservation.state(i);
  }

  const auto& stateTraj = mrt_.getPolicy().stateTrajectory_;
  vector3_t com_start = hardware_interface_.getState().segment<3>(0);  // 取第一个状态的前三维为CoM位置
  vector3_t com_end   = stateTraj.back().segment<3>(0);   // 取最后一个状态的前三维为CoM位置
  scalar_t com_distance = (com_end - com_start).norm();
  bool stepJustUpdated = com_distance < 0.03;
  // 连续计数器更新逻辑
  if (stepJustUpdated) {
      justUpdatedCounter_++;
      if (justUpdatedCounter_ > justUpdatedThreshold_) {
          justUpdatedCounter_ = justUpdatedThreshold_;  // 限制最大值
      }
  } else {
      justUpdatedCounter_ = 0;
  }

  // 满足连续50次的判断条件
  bool justUpdated = (justUpdatedCounter_ >= justUpdatedThreshold_);

  // RCLCPP_INFO_STREAM(
  //     LOGGER, "[INFO]Current State: " << " CoM position: " << hardware_interface_.getState().segment<3>(0) << ", CoM RPY: " << hardware_interface_.getState().segment<3>(3));

  // RCLCPP_INFO_STREAM(
  //     LOGGER, "[INFO]Current Time: " << hardware_time
  //                                    << ", Contact state: " << hardware_interface_.getContactState()[0] << ", " << hardware_interface_.getContactState()[1]);

  // 将平滑后的力矩转换为低级命令

  
  vector_t test_state = hardware_interface_.getState();
  test_state.head<6>() << 0.0, 0.0, 0.79, 0.0, 0.0, 0.0;
  test_state.segment<6>(28).setZero();
  vector_t test_input = vector_t::Zero(mpcRobotModelPtr_->getInputDim());
  
  /**************************************************************/
  // // 仅测试动力学，真实运行时记得注释掉
  // scalar_t f = 1.0;
  // scalar_t amp = 0.3;
  // scalar_t t = hardware_interface_.getTime() - start_bias_time;
  // scalar_t q = amp * std::sin(2 * M_PI * f * t);
  // scalar_t dq = amp * 2 * M_PI * f * std::cos(2 * M_PI * f * t);
  // scalar_t ddq = -amp * 4 * M_PI * M_PI * f * std::sin(2 * M_PI * f * t);
  // size_t joint_index = 1;

  // test_input(12 + joint_index) = ddq;
  // test_state(6 + joint_index) = q;
  // test_state(35 + joint_index) = dq;

  // test_input(12 + 6 + joint_index) = ddq;
  // test_state(6 + 6 + joint_index) = q;
  // test_state(35 + 6 + joint_index) = dq;
  
  /**************************************************************/

  
  
  if((hardware_time < time_duration) && !hardware_interface_.getMPCMode()){
    if (hardware_time > 30.0) {
      test_input(2) = 171.0;
      test_input(8) = 171.0;
      test_input(4) = -1.4;
      test_input(10) = -1.4;
    }
    vector_t test_jointTorques = computeJointTorques(test_state, test_input, pinocchioInterface_, *mpcRobotModelPtr_);
    hardware_interface_.convertMPCInput2LowCmd(test_jointTorques, test_state, test_input, justUpdated);
  }
  else{
    // test_input(2) = 171.0;
    // test_input(8) = 171.0;
    // test_input(4) = -1.4;
    // test_input(10) = -1.4;
    // vector_t test_jointTorques = computeJointTorques(nextObservation.state, test_input, pinocchioInterface_, *mpcRobotModelPtr_);
    // jointTorques(1) = test_jointTorques(1);
    // jointTorques(7) = test_jointTorques(7);
    // jointTorques(2) = test_jointTorques(2);
    // jointTorques(8) = test_jointTorques(8);
    hardware_interface_.convertMPCInput2LowCmd(jointTorques, nextObservation.state, nextObservation.input, justUpdated);
  }

  /**************************************************************/
  // // 仅测试动力学，真实运行时记得注释掉
  // vector_t test_jointTorques = computeJointTorques(
  //     test_state, test_input, pinocchioInterface_, *mpcRobotModelPtr_);
  // hardware_interface_.convertMPCInput2LowCmd(test_jointTorques, test_state, test_input, justUpdated);
  /**************************************************************/

  // RCLCPP_INFO_STREAM(LOGGER, "[Standing Input]: " << nextObservation.input.head<12>());

  unitree_hg::msg::LowCmd mylowcmd = hardware_interface_.getLowCmd();
  get_crc(mylowcmd);
  hardware_interface_.lowcmd_pub_->publish(mylowcmd);

  nextObservation.state = hardware_interface_.getState();
  nextObservation.time = internal_query_time;  // 按同步后的internal时间推进

  for (int i = 0; i < nextObservation.state.size(); ++i) {
        logFile_ << "," << nextObservation.state(i);
  }
  for (int i = 0; i < jointTorques.size(); ++i) {
      logFile_ << "," << jointTorques(i);
  }
  logFile_ << "\n";

  return nextObservation;
  }

}  // namespace ocs2
