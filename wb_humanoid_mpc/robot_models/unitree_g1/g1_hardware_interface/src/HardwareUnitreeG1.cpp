// Haolin Ma 2025-04-11 00:00:06


#include "g1_hardware_interface/HardwareUnitreeG1.h"
#include "g1_hardware_interface/motor_crc_hg.h"





enum PRorAB { PR = 0, AB = 1 };

using std::placeholders::_1;

enum G1JointIndex {
    LeftHipPitch = 0,
    LeftHipRoll = 1,
    LeftHipYaw = 2,
    LeftKnee = 3,
    LeftAnklePitch = 4,
    LeftAnkleB = 4,
    LeftAnkleRoll = 5,
    LeftAnkleA = 5,
    RightHipPitch = 6,
    RightHipRoll = 7,
    RightHipYaw = 8,
    RightKnee = 9,
    RightAnklePitch = 10,
    RightAnkleB = 10,
    RightAnkleRoll = 11,
    RightAnkleA = 11,
    WaistYaw = 12,
    WaistRoll = 13,   // NOTE INVALID for g1 23dof/29dof with waist locked
    WaistA = 13,      // NOTE INVALID for g1 23dof/29dof with waist locked
    WaistPitch = 14,  // NOTE INVALID for g1 23dof/29dof with waist locked
    WaistB = 14,      // NOTE INVALID for g1 23dof/29dof with waist locked
    LeftShoulderPitch = 15,
    LeftShoulderRoll = 16,
    LeftShoulderYaw = 17,
    LeftElbow = 18,
    LeftWristRoll = 19,
    LeftWristPitch = 20,  // NOTE INVALID for g1 23dof
    LeftWristYaw = 21,    // NOTE INVALID for g1 23dof
    RightShoulderPitch = 22,
    RightShoulderRoll = 23,
    RightShoulderYaw = 24,
    RightElbow = 25,
    RightWristRoll = 26,
    RightWristPitch = 27,  // NOTE INVALID for g1 23dof
    RightWristYaw = 28     // NOTE INVALID for g1 23dof
};

#define IS_FIXED_G1_JOINT(IDX)    ((IDX) == 19 || (IDX) == 20 || (IDX) == 21 || (IDX) == 26 || (IDX) == 27 || (IDX) == 28)

#define IS_DISABLE_G1_JOINT(IDX)  ((IDX) == 13 || (IDX) == 14 || (IDX) == 16 || (IDX) == 17 || (IDX) == 19 || (IDX) == 20 || (IDX) == 21 || (IDX) == 23 || (IDX) == 24 || (IDX) == 26 || (IDX) == 27 || (IDX) == 28)




namespace ocs2::humanoid {

const vector_t init_state = []() {
    vector_t v = vector_t::Zero(58);
    v[2] = 0.79;
    return v;
}();

HardwareUnitreeG1::HardwareUnitreeG1(const rclcpp::Node::SharedPtr& node, scalar_t mrtDesiredFrequency,
                               scalar_t mpcDesiredFrequency)
    : node_(node),
    // mpcStateFilter_(10.0, init_state),
    base_position_initialized_(false),
    base_rpy_initialized_(false){
    // 创建订阅器和发布器时使用 node_
    lowstate_sub_ = node_->create_subscription<unitree_hg::msg::LowState>(TOPIC_LOWSTATE, mrtDesiredFrequency, std::bind(&HardwareUnitreeG1::lowstateCallback, this, _1));

    sportstate_sub_ = node_->create_subscription<unitree_go::msg::SportModeState>(TOPIC_SPORT_STATE, mrtDesiredFrequency, std::bind(&HardwareUnitreeG1::sportstateCallback, this, _1));

    lowcmd_pub_ = node_->create_publisher<unitree_hg::msg::LowCmd>(TOPIC_LOWCMD, 10);

    last_mpcstate_.state.value = std::vector<double>(58, 0.0);

    // 在构造函数或 run() 函数的开头打开文件
    // 获取当前时间字符串
    auto now = std::chrono::system_clock::now();
    std::time_t t_now = std::chrono::system_clock::to_time_t(now);
    std::tm tm_now = *std::localtime(&t_now);

    std::ostringstream timestamp_stream;
    timestamp_stream << std::put_time(&tm_now, "%Y%m%d_%H%M");
    std::string timestamp = timestamp_stream.str();

    // 拼接带时间的文件名
    std::string filename;
    if (ENABLE_SIMULATE) {
        filename = "./log/param_debug_" + timestamp + ".csv";
    } else {
        filename = "./log/real_param_debug_" + timestamp + ".csv";
    }
    logFile_.open(filename);
    logFile_ << "time,state,input\n";  // CSV 表头

    initial_q_.resize(G1_NUM_MOTOR, 0.0);

    // if (TEST_MODE) {
    //     testLowstateLogFile_.open("./log/lowstate_test_log.csv");
    //     testLowstateLogFile_ << "time";
    //     for (int i = 0; i < G1_NUM_MOTOR; ++i) {
    //         testLowstateLogFile_ << ",q" << i << ",dq" << i << ",tau_est" << i;
    //     }
    //     testLowstateLogFile_ << "\n";
    // }

    // if (TEST_MODE) {
    //     recordedLowstateFile_.open("./log/lowstate_test_log.csv");
    //     std::string line;
    //     std::getline(recordedLowstateFile_, line);  // 跳过表头

    //     while (std::getline(recordedLowstateFile_, line)) {
    //         std::stringstream ss(line);
    //         std::string value;
    //         std::vector<double> row;

    //         while (std::getline(ss, value, ',')) {
    //             row.push_back(std::stod(value));
    //         }

    //         recordedLowstateData_.push_back(row);
    //     }

    //     std::cout << "[TEST_MODE] Loaded " << recordedLowstateData_.size()
    //               << " steps of lowstate data." << std::endl;
    // }

}

void HardwareUnitreeG1::lowstateCallback(
    const unitree_hg::msg::LowState::SharedPtr msg) {
    last_lowstate_ = *msg;

    if(ENABLE_SIMULATE){
        if (!base_rpy_initialized_){
            for (size_t i = 0; i < 3; ++i) {
                base_rpy_offset_[i] = static_cast<scalar_t>(msg->imu_state.rpy[i]) - base_rpy_offset_[i];
            }
            base_rpy_initialized_ = true;
        }
        last_mpcstate_.state.value[3] = msg->imu_state.rpy[2] - base_rpy_offset_[2];
        last_mpcstate_.state.value[4] = msg->imu_state.rpy[1] - base_rpy_offset_[1];
        last_mpcstate_.state.value[5] = msg->imu_state.rpy[0] - base_rpy_offset_[0];
    }

    if (ENABLE_SIMULATE){
        // 可能有问题，用的是机器人坐标系的角速度
        last_mpcstate_.state.value[32] = msg->imu_state.gyroscope[2];
        last_mpcstate_.state.value[33] = msg->imu_state.gyroscope[1];
        last_mpcstate_.state.value[34] = msg->imu_state.gyroscope[0];
    }
    else{
        // last_mpcstate_.state.value[32] = 0.0;
        // last_mpcstate_.state.value[33] = 0.0;
        // last_mpcstate_.state.value[34] = 0.0;
        last_mpcstate_.state.value[32] = msg->imu_state.gyroscope[2];
        last_mpcstate_.state.value[33] = msg->imu_state.gyroscope[1];
        last_mpcstate_.state.value[34] = msg->imu_state.gyroscope[0];
    }
        

    robot_time_ = 0.001 * msg->tick;

    size_t offset = 0;
    for (size_t i = 0; i < G1_NUM_MOTOR; i++) {
        if (IS_FIXED_G1_JOINT(i)) {
            offset++;  // 固定关节跳过，后续取 joint_state 的 index 需要偏移
            continue;
        }

        last_mpcstate_.state.value[i + 6 - offset] = msg->motor_state[i].q;
        last_mpcstate_.state.value[i + 35 - offset] = msg->motor_state[i].dq;
    }

    // 只在第一次设置完成后，将标志位设为 true
    if (!initial_q_recorded_) {
        initial_q_recorded_ = true;
        std::cout << "[LowState] Initial joint positions recorded." << std::endl;
    }

    // if (TEST_MODE) {
    //     testLowstateLogFile_ << std::fixed << std::setprecision(6)
    //                          << robot_time_;
    //     for (int i = 0; i < G1_NUM_MOTOR; ++i) {
    //         if (i < msg->motor_state.size()) {
    //             testLowstateLogFile_ << "," << msg->motor_state[i].q << ","
    //                                  << msg->motor_state[i].dq << ","
    //                                  << msg->motor_state[i].tau_est;
    //         } else {
    //             testLowstateLogFile_ << ",0.0,0.0,0.0";
    //         }
    //     }
    //     testLowstateLogFile_ << "\n";
    // }

    // if (TEST_MODE) {
    //     if (init_time_ < 0) {
    //         init_time_ = robot_time_;  // 第一次进入时设定初始时间
    //     }

    //     double relative_time = robot_time_ - init_time_;

    //     if (replay_index_ < recordedLowstateData_.size()) {
    //         const auto& data = recordedLowstateData_[replay_index_];
    //         replay_index_++;

    //         // 第一列是时间，后面是 q, dq, tau_est 依次
    //         for (size_t i = 0; i < G1_NUM_MOTOR; ++i) {
    //             size_t base_idx = 1 + i * 3;
    //             if (base_idx + 2 < data.size()) {
    //                 last_lowcmd_.motor_cmd[i].mode = 1;
    //                 last_lowcmd_.motor_cmd[i].q    = data[base_idx];
    //                 last_lowcmd_.motor_cmd[i].dq   = data[base_idx + 1];
    //                 if (relative_time > 10.0) {
    //                     last_lowcmd_.motor_cmd[i].tau = data[base_idx + 2];
    //                 } else {
    //                     last_lowcmd_.motor_cmd[i].tau = 0.0;
    //                 }

    //                 if (ENABLE_SIMULATE){
    //                     last_lowcmd_.motor_cmd[i].kp = (i < 15) ? 100.0 : 50.0;
    //                     last_lowcmd_.motor_cmd[i].kd = (i < 15) ? 5.0 : 3.0;
    //                 }
    //                 else{
    //                     last_lowcmd_.motor_cmd[i].kp = (i < 15) ? 150.0 : 50.0;
    //                     last_lowcmd_.motor_cmd[i].kd = (i < 15) ? 4.0 : 2.0;
    //                     last_lowcmd_.motor_cmd[i].kp = (i == 4 || i == 5 || i == 10 || i == 11) ? 80.0 : last_lowcmd_.motor_cmd[i].kp;
    //                     last_lowcmd_.motor_cmd[i].kd = (i == 4 || i == 5 || i == 10 || i == 11) ? 2.0 : last_lowcmd_.motor_cmd[i].kd;
    //                 }
    //             }
    //         }

    //         // 实际发布
    //         lowcmd_pub_->publish(last_lowcmd_);
    //     } else {
    //         std::cout << "[TEST_MODE] All recorded data has been replayed." << std::endl;
    // }
// }
}

void HardwareUnitreeG1::sportstateCallback(
    const unitree_go::msg::SportModeState::SharedPtr msg) {
    last_sportstate_ = *msg;

    if (!base_position_initialized_) {
        for (size_t i = 0; i < 3; ++i) {
            base_position_offset_[i] = static_cast<scalar_t>(msg->position[i]) - base_position_offset_[i];
        }
        base_position_initialized_ = true;
    }

    last_mpcstate_.state.value[0] = msg->position[0] - base_position_offset_[0];
    last_mpcstate_.state.value[1] = msg->position[1] - base_position_offset_[1];
    last_mpcstate_.state.value[2] = msg->position[2] - base_position_offset_[2];

    last_mpcstate_.state.value[29] = msg->velocity[0];
    last_mpcstate_.state.value[30] = msg->velocity[1];
    last_mpcstate_.state.value[31] = msg->velocity[2];

    if(!ENABLE_SIMULATE){
        if (!base_rpy_initialized_){
            for (size_t i = 0; i < 3; ++i) {
                base_rpy_offset_[i] = static_cast<scalar_t>(msg->imu_state.rpy[i]) - base_rpy_offset_[i];
            }
            base_rpy_initialized_ = true;
        }
        last_mpcstate_.state.value[3] = msg->imu_state.rpy[2] - base_rpy_offset_[2];
        last_mpcstate_.state.value[4] = msg->imu_state.rpy[1] - base_rpy_offset_[1];
        last_mpcstate_.state.value[5] = msg->imu_state.rpy[0] - base_rpy_offset_[0];
    }
}

void HardwareUnitreeG1::convertMPCInput2LowCmd(const vector_t& jointTorques,
                                               const vector_t& state,
                                               const vector_t& input,
                                               const bool justUpdated) {
    PRorAB mode_ = PRorAB::PR;  // Enable PR mode
    last_lowcmd_.mode_pr = mode_;
    last_lowcmd_.mode_machine = (int)last_lowstate_.mode_machine;

    size_t offset = 0;

    // 写成逐项写入，每个数字都写入一个 CSV 列
    logFile_ << std::fixed << std::setprecision(6) << (robot_time_);

    for (size_t i = 0; i < G1_NUM_MOTOR; ++i) {

        // if(TEST_MODE){
        //     last_lowcmd_.motor_cmd[i].mode = 0;
        //     last_lowcmd_.motor_cmd[i].tau = 0.0;
        //     last_lowcmd_.motor_cmd[i].q = 0.0;
        //     last_lowcmd_.motor_cmd[i].dq = 0.0;
        //     last_lowcmd_.motor_cmd[i].kp = 0.0;
        //     last_lowcmd_.motor_cmd[i].kd = 0.0;
        // }
        if (TEST_MODE) {
            last_lowcmd_.motor_cmd[i].mode = 1;
            last_lowcmd_.motor_cmd[i].tau = 0.0;
            last_lowcmd_.motor_cmd[i].dq = 0.0;

            static double init_time = robot_time_;  // 只记录第一次调用的时间
            double time = robot_time_ - init_time;
            double duration = 5.0;  // 阶段一持续时间（秒）

            if (time < duration) {
                // 阶段一：从当前姿态逐渐过渡到零位
                double ratio = std::clamp(time / duration, 0.0, 1.0);
                double target_q = 0.0;
                // if (i == G1JointIndex::LeftHipPitch || i == G1JointIndex::RightHipPitch) {
                //     target_q = 0.5;  // 3号和9号关节目标是0.5
                // }
                last_lowcmd_.motor_cmd[i].q = (1.0 - ratio) * initial_q_[i] + ratio * target_q;
                if(ENABLE_SIMULATE){
                    last_lowcmd_.motor_cmd[i].kp = (i < 12) ? 100.0 : 50.0; //100.0 : 50.0; 
                    last_lowcmd_.motor_cmd[i].kd =(i < 12) ? 5.0 : 3.0; // ? 5.0 : 3.0;
                    last_lowcmd_.motor_cmd[i].kp = (i == 4 || i == 5 || i == 10 || i == 11) ? 40.0 : last_lowcmd_.motor_cmd[i].kp; // 40.0 : last_lowcmd_.motor_cmd[i].kp; 
                    last_lowcmd_.motor_cmd[i].kd = (i == 4 || i == 5 || i == 10 || i == 11) ? 2.0 : last_lowcmd_.motor_cmd[i].kd;
                }
                else{
                    last_lowcmd_.motor_cmd[i].kp = (i < 12) ? 150.0 : 50.0; //100.0 : 50.0; 
                    last_lowcmd_.motor_cmd[i].kd =(i < 12) ? 4.0 : 2.0; // ? 5.0 : 3.0;
                    last_lowcmd_.motor_cmd[i].kp = (i == 4 || i == 5 || i == 10 || i == 11) ? 80.0 : last_lowcmd_.motor_cmd[i].kp;
                    last_lowcmd_.motor_cmd[i].kd = (i == 4 || i == 5 || i == 10 || i == 11) ? 2.0 : last_lowcmd_.motor_cmd[i].kd;
                }
            } else {
                // 阶段二：正弦轨迹控制脚踝
                double t = time - duration;
                if (IS_DISABLE_G1_JOINT(i)) {
                    last_lowcmd_.motor_cmd[i].tau = 0.0;
                    last_lowcmd_.motor_cmd[i].q = 0.0;
                    last_lowcmd_.motor_cmd[i].dq = 0.0;
                    last_lowcmd_.motor_cmd[i].kp = (i < 12) ? 150.0 : 50.0; //100.0 : 50.0; 
                    last_lowcmd_.motor_cmd[i].kd =(i < 12) ? 4.0 : 2.0; // ? 5.0 : 3.0;
                    if (IS_FIXED_G1_JOINT(i)) {
                        offset++;  // 固定关节跳过，后续取 joint_state 的 index
                                // 需要偏移
                    }
                }
                else{
                    if(ENABLE_SIMULATE){
                        last_lowcmd_.motor_cmd[i].kp = (i < 12) ? 100.0 : 50.0; //100.0 : 50.0; 
                        last_lowcmd_.motor_cmd[i].kd =(i < 12) ? 5.0 : 3.0; // ? 5.0 : 3.0;
                        last_lowcmd_.motor_cmd[i].kp = (i == 4 || i == 5 || i == 10 || i == 11) ? 40.0 : last_lowcmd_.motor_cmd[i].kp; // 40.0 : last_lowcmd_.motor_cmd[i].kp; 
                        last_lowcmd_.motor_cmd[i].kd = (i == 4 || i == 5 || i == 10 || i == 11) ? 2.0 : last_lowcmd_.motor_cmd[i].kd;
                    }
                    else{
                        last_lowcmd_.motor_cmd[i].kp = (i < 12) ? 150.0 : 50.0; //100.0 : 50.0; 
                        last_lowcmd_.motor_cmd[i].kd =(i < 12) ? 4.0 : 2.0; // ? 5.0 : 3.0;
                        last_lowcmd_.motor_cmd[i].kp = (i == 4 || i == 5 || i == 10 || i == 11) ? 80.0 : last_lowcmd_.motor_cmd[i].kp;
                        last_lowcmd_.motor_cmd[i].kd = (i == 4 || i == 5 || i == 10 || i == 11) ? 2.0 : last_lowcmd_.motor_cmd[i].kd;
                    }
                    last_lowcmd_.motor_cmd[i].q = 0.0;
                    last_lowcmd_.motor_cmd[i].tau = 0.0;
                    last_lowcmd_.motor_cmd[i].dq = 0.0;

                    if (i == G1JointIndex::LeftHipRoll || i == G1JointIndex::RightHipRoll) {
                        last_lowcmd_.motor_cmd[i].q = state(i + 6 - offset);
                        last_lowcmd_.motor_cmd[i].dq = state(i + 35 - offset);
                        last_lowcmd_.motor_cmd[i].tau = jointTorques(i - offset);
                        // last_lowcmd_.motor_cmd[i].kp = 0.0;
                        // last_lowcmd_.motor_cmd[i].kd = 0.0;
                    }
                }
            }
            
        } 
        else {
            if (IS_DISABLE_G1_JOINT(i)) {
                last_lowcmd_.motor_cmd[i].mode = 1;  //  1:Enable, 0:Disable
            } else {
                last_lowcmd_.motor_cmd[i].mode = 1;  // 1:Enable, 0:Disable
            }

            if (IS_DISABLE_G1_JOINT(i)) {
                last_lowcmd_.motor_cmd[i].tau = 0.0;
                last_lowcmd_.motor_cmd[i].q = 0.0;
                last_lowcmd_.motor_cmd[i].dq = 0.0;
                last_lowcmd_.motor_cmd[i].kp =
                    (i < 15) ? 80.0 : 25.0;  // ? 80.0 : 25.0;
                last_lowcmd_.motor_cmd[i].kd = (i < 15) ? 8.0 : 3.0;
                if (IS_FIXED_G1_JOINT(i)) {
                    offset++;  // 固定关节跳过，后续取 joint_state 的 index
                               // 需要偏移
                }
            } else {
                static bool initialized = false;
                static double init_time = robot_time_;
                double time_since_start = robot_time_ - init_time;
                double transition_duration = 3.0;
                

                if (time_since_start < transition_duration) {
                    // S曲线插补
                    double t = time_since_start;
                    double T = transition_duration;
                    double ratio = 0.0;

                    if (t < T / 2.0) {
                        ratio = 2.0 * std::pow(t / T, 2);
                    } else {
                        ratio = 1.0 - 2.0 * std::pow(1.0 - t / T, 2);
                    }

                    double q_init = last_lowstate_.motor_state[i].q;
                    double q_target = q_target_map.at(i);

                    // 计算插补后的目标位置
                    double q_desired = (1.0 - ratio) * q_init + ratio * q_target;

                    last_lowcmd_.motor_cmd[i].mode = 1;
                    last_lowcmd_.motor_cmd[i].q = q_desired;
                    last_lowcmd_.motor_cmd[i].dq = 0.0;
                    
                    if (ENABLE_SIMULATE){
                        last_lowcmd_.motor_cmd[i].kp = (i < 15) ? 100.0 : 50.0;
                        last_lowcmd_.motor_cmd[i].kd = (i < 15) ? 5.0 : 3.0;
                    }
                    else{
                        last_lowcmd_.motor_cmd[i].kp = (i < 15) ? 150.0 : 50.0;
                        last_lowcmd_.motor_cmd[i].kd = (i < 15) ? 4.0 : 2.0;
                        last_lowcmd_.motor_cmd[i].kp = (i == 4 || i == 5 || i == 10 || i == 11) ? 80.0 : last_lowcmd_.motor_cmd[i].kp;
                        last_lowcmd_.motor_cmd[i].kd = (i == 4 || i == 5 || i == 10 || i == 11) ? 2.0 : last_lowcmd_.motor_cmd[i].kd;
                    }
                    last_lowcmd_.motor_cmd[i].tau = jointTorques(i - offset);  // 可选设为0防止干扰

                    continue;  // 跳过后续MPC控制逻辑
                } else if (!entered_mpc_mode_ && robot_time_ < 15.0 && ENABLE_SIMULATE) {
                    last_lowcmd_.motor_cmd[i].mode = 1;
                    last_lowcmd_.motor_cmd[i].q = q_target_map.at(i);
                    last_lowcmd_.motor_cmd[i].dq = 0.0;
                    last_lowcmd_.motor_cmd[i].kp = (i < 15) ? 100.0 : 50.0;
                    last_lowcmd_.motor_cmd[i].kd = (i < 15) ? 5.0 : 3.0;
                    last_lowcmd_.motor_cmd[i].tau = 0.0;  // 可选设为0防止干扰

                } else if (!entered_mpc_mode_ && robot_time_ - init_time < 60.0 && !ENABLE_SIMULATE) {
                    last_lowcmd_.motor_cmd[i].mode = 1;
                    last_lowcmd_.motor_cmd[i].q = q_target_map.at(i);
                    last_lowcmd_.motor_cmd[i].dq = 0.0;
                    last_lowcmd_.motor_cmd[i].kp = (i < 15) ? 150.0 : 50.0;
                    last_lowcmd_.motor_cmd[i].kd = (i < 15) ? 4.0 : 2.0;
                    last_lowcmd_.motor_cmd[i].kp = (i == 4 || i == 5 || i == 10 || i == 11) ? 80.0 : last_lowcmd_.motor_cmd[i].kp;
                    last_lowcmd_.motor_cmd[i].kd = (i == 4 || i == 5 || i == 10 || i == 11) ? 2.0 : last_lowcmd_.motor_cmd[i].kd;
                    last_lowcmd_.motor_cmd[i].tau = jointTorques(i - offset);  // 可选设为0防止干扰

                    if (!offsets_updated_ && robot_time_ - init_time > 50.0 &&robot_time_ - init_time < 50.1 ){
                        base_position_offset_[0] += last_mpcstate_.state.value[0];
                        base_position_offset_[1] += last_mpcstate_.state.value[1];
                        // base_rpy_offset_[0] += last_mpcstate_.state.value[5];
                        base_rpy_offset_[1] += last_mpcstate_.state.value[4] * 0.75;
                        base_rpy_offset_[2] += last_mpcstate_.state.value[3];

                        offsets_updated_ = true;
                    }
                } else if (justUpdated || entered_mpc_mode_) {
                    entered_mpc_mode_ = true;

                    scalar_t rawTau = jointTorques(i - offset);
                    scalar_t clampedTau = clampTorque(i, rawTau);
                    scalar_t torqueRatio = 1.0;
                    scalar_t qRatio = 1.0;

                    // if ((i == 2 || i == 8)) {
                    //     torqueRatio = 0.0;
                    //     qRatio = 0.0;
                    // }
                    // if ((i == 1 || i == 7)) {
                    //     qRatio = 0.0;
                    // }
                    last_lowcmd_.motor_cmd[i].tau = clampedTau * torqueRatio; // * warmup_ratio;

                    last_lowcmd_.motor_cmd[i].q = state(i + 6 - offset) * qRatio;
                    if(ENABLE_SIMULATE){
                        last_lowcmd_.motor_cmd[i].dq = state(i + 35 - offset);
                        // last_lowcmd_.motor_cmd[i].dq = 0.0;
                        last_lowcmd_.motor_cmd[i].kp = (i < 12) ? 100.0 : 50.0; //100.0 : 50.0; 
                        last_lowcmd_.motor_cmd[i].kp = (i == 4 || i == 5 || i == 10 || i == 11) ? 40.0 : last_lowcmd_.motor_cmd[i].kp; // 40.0 : last_lowcmd_.motor_cmd[i].kp; 
                        last_lowcmd_.motor_cmd[i].kd =(i < 12) ? 5.0 : 3.0; // ? 5.0 : 3.0;
                        last_lowcmd_.motor_cmd[i].kd = (i == 4 || i == 5 || i == 10 || i == 11) ? 2.0 : last_lowcmd_.motor_cmd[i].kd; // 2.0 : last_lowcmd_.motor_cmd[i].kd;
                    }
                    else{
                        last_lowcmd_.motor_cmd[i].dq = state(i + 35 - offset);
                        // last_lowcmd_.motor_cmd[i].dq = 0.0;
                        last_lowcmd_.motor_cmd[i].kp = (i < 12) ? 150.0 : 50.0; //100.0 : 50.0; 
                        last_lowcmd_.motor_cmd[i].kd =(i < 12) ? 4.0 : 2.0; // ? 5.0 : 3.0;
                        last_lowcmd_.motor_cmd[i].kp = (i == 4 || i == 5 || i == 10 || i == 11) ? 80.0 : last_lowcmd_.motor_cmd[i].kp;
                        last_lowcmd_.motor_cmd[i].kd = (i == 4 || i == 5 || i == 10 || i == 11) ? 2.0 : last_lowcmd_.motor_cmd[i].kd;
                    }
                    
                    if ((i == 4 || i == 5 || i == 10 || i == 11)) {
                        // 提取 pitch 和 roll 误差（单位 rad），假设期望为 0
                        scalar_t pitch_err = last_mpcstate_.state.value[4];  // theta_base_y
                        scalar_t roll_err  = last_mpcstate_.state.value[5];  // theta_base_x
                        scalar_t vel_pitch_err = last_mpcstate_.state.value[33];  // theta_base_y
                        scalar_t vel_roll_err  = last_mpcstate_.state.value[34];  // theta_base_x

                        // 根据关节编号选择使用哪个误差分量
                        scalar_t angle_err = (i == 4 || i == 10) ? pitch_err : roll_err;
                        scalar_t vel_angle_err = (i == 4 || i == 10) ? vel_pitch_err : vel_roll_err;

                        // 简单 PD 补偿力矩，系数你可以调节
                        scalar_t kp_comp = 10.0;
                        scalar_t kd_comp = 0.5;

                        scalar_t compensating_tau = -kp_comp * angle_err - kd_comp * vel_angle_err;

                        // 叠加补偿项
                        last_lowcmd_.motor_cmd[i].tau += compensating_tau;
                    }
                }
            }
        }
        if (i < last_lowcmd_.motor_cmd.size()) {
            logFile_ << "," << last_lowcmd_.motor_cmd[i].q;
        } else {
            logFile_ << ",0.0";
        }

        // 写入 last_lowstate_.motor_state[i].q，如果不存在则写入 0
        if (i < last_lowstate_.motor_state.size()) {
            logFile_ << "," << last_lowstate_.motor_state[i].q;
        } else {
            logFile_ << ",0.0";
        }

        
        if (i < last_lowcmd_.motor_cmd.size()) {
            logFile_ << "," << last_lowcmd_.motor_cmd[i].tau;
        } else {
            logFile_ << ",0.0";
        }

        if (i < last_lowstate_.motor_state.size()) {
            logFile_ << "," << last_lowstate_.motor_state[i].tau_est;
        } else {
            logFile_ << ",0.0";
        }
    }
    // 记录质心位置、角度、速度和角速度的反馈 logFile_
    logFile_ << "," << last_mpcstate_.state.value[0]    // 质心位置 x
             << "," << last_mpcstate_.state.value[1]    // 质心位置 y
             << "," << last_mpcstate_.state.value[2]    // 质心位置 z
             << "," << last_mpcstate_.state.value[3]    // 质心角度 yaw
             << "," << last_mpcstate_.state.value[4]    // 质心角度 pitch
             << "," << last_mpcstate_.state.value[5]    // 质心角度 roll
             << "," << last_mpcstate_.state.value[29]   // 质心速度 x
             << "," << last_mpcstate_.state.value[30]   // 质心速度 y
             << "," << last_mpcstate_.state.value[31]   // 质心速度 z
             << "," << last_mpcstate_.state.value[32]   // 质心角速度 yaw
             << "," << last_mpcstate_.state.value[33]   // 质心角速度 pitch
             << "," << last_mpcstate_.state.value[34];  // 质心角速度 roll

    // 记录质心位置、角度、速度和角速度的指令
    logFile_ << "," << state(0)    // 指令质心位置 x
             << "," << state(1)    // 指令质心位置 y
             << "," << state(2)    // 指令质心位置 z
             << "," << state(3)    // 指令质心角度 yaw
             << "," << state(4)    // 指令质心角度 pitch
             << "," << state(5)    // 指令质心角度 roll
             << "," << state(29)   // 指令质心速度 x
             << "," << state(30)   // 指令质心速度 y
             << "," << state(31)   // 指令质心速度 z
             << "," << state(32)   // 指令质心角速度 yaw
             << "," << state(33)   // 指令质心角速度 pitch
             << "," << state(34);  // 指令质心角速度 roll

    // 记录双脚支撑力输入
    logFile_ << "," << input(0)    // 左腿支撑力 x  
             << "," << input(1)    // 左腿支撑力 y  
             << "," << input(2)    // 左腿支撑力 z  
             << "," << input(3)    // 左腿支撑力矩 x
             << "," << input(4)    // 左腿支撑力矩 y
             << "," << input(5)    // 左腿支撑力矩 z
             << "," << input(6)    // 右腿支撑力 x
             << "," << input(7)    // 右腿支撑力 y
             << "," << input(8)    // 右腿支撑力 z
             << "," << input(9)    // 右腿支撑力矩 x
             << "," << input(10)   // 右腿支撑力矩 y
             << "," << input(11);  // 右腿支撑力矩 z

    logFile_ << "\n";

    checkJointProtection();
    if (emergency_triggered) emergencyProtect("Emergency", 999);
    // get_crc(last_lowcmd_);
}

void HardwareUnitreeG1::checkJointProtection() {
    scalar_t now = robot_time_;
    for (size_t i = 0; i < G1_NUM_MOTOR; ++i) {
        if (IS_DISABLE_G1_JOINT(i)) continue;

        // 力矩保护（Nm）
        auto tau = last_lowstate_.motor_state[i].tau_est;
        auto tau_lim = G1_TORQUE_LIMITS.at(i);
        if (tau < tau_lim.first + 5.0 || tau > tau_lim.second -5.0) {
            if (torqueViolationStartTime[i] < 0) torqueViolationStartTime[i] = now;
            if ((now - torqueViolationStartTime[i]) > PROTECTION_TRIGGER_DURATION) {
                RCLCPP_ERROR_STREAM(node_->get_logger(), "[PROTECT] Joint " << i << " over torque limit for too long: " << tau);
                emergencyProtect("TorqueLimit", i);
                return;
            }
        } else {
            torqueViolationStartTime[i] = -1.0;  // Reset the violation time
        }

        // 位置保护（rad）
        auto q = last_lowstate_.motor_state[i].q;
        auto q_lim = G1_POSITION_LIMITS.at(i);
        if (q < q_lim.first || q > q_lim.second) {
            if (positionViolationStartTime[i] < 0) positionViolationStartTime[i] = now;
            if ((now - positionViolationStartTime[i]) > PROTECTION_TRIGGER_DURATION) {
                RCLCPP_ERROR_STREAM(node_->get_logger(), "[PROTECT] Joint " << i << " over position limit for too long: " << q);
                emergencyProtect("PositionLimit", i);
                return;
            }
        } else {
            positionViolationStartTime[i] = -1.0;  // Reset the violation time
        }
    }
}

void HardwareUnitreeG1::emergencyProtect(const std::string& reason,
                                         size_t jointIdx) {
    emergency_triggered = true;
    for (size_t i = 0; i < G1_NUM_MOTOR; ++i) {
        last_lowcmd_.motor_cmd[i].mode = 1;
        last_lowcmd_.motor_cmd[i].kp = 0.0;
        last_lowcmd_.motor_cmd[i].kd = 2.0;
        last_lowcmd_.motor_cmd[i].q = 0.0;
        last_lowcmd_.motor_cmd[i].dq = 0.0;
        last_lowcmd_.motor_cmd[i].tau = 0.0;
    }
}

}  // namespace ocs2::humanoid