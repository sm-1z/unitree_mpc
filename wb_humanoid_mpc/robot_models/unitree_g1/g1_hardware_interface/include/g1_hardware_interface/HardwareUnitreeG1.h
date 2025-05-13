// Haolin Ma 2025-04-10 17:01:04

// HardwareUnitreeG1.h
#pragma once

#include <ocs2_ros2_msgs/msg/mpc_input.hpp>
#include <ocs2_ros2_msgs/msg/mpc_observation.hpp>
#include <rclcpp/rclcpp.hpp>
#include <unitree_go/msg/sport_mode_state.hpp>
#include <unitree_hg/msg/low_cmd.hpp>
#include <unitree_hg/msg/low_state.hpp>
#include <ocs2_core/Types.h>
#include <map>
#include <algorithm>
#include <fstream>
#include <iomanip>  // for std::setprecision

#define ENABLE_SIMULATE true
#define TEST_MODE false

#define TOPIC_MPCOBSERVATION "/g1/mpc_observation"
// #define TOPIC_MPCPOLICY "/g1/mpc_policy"
#define TOPIC_LOWSTATE "/lowstate"
#if ENABLE_SIMULATE
    #define TOPIC_SPORT_STATE "/sportmodestate"  // 仿真
    #define TOPIC_LOWCMD "/lowcmd"
#else
    #define TOPIC_SPORT_STATE "/odommodestate"  // 实验
    #define TOPIC_LOWCMD "/mylowcmd"  // 中转
#endif


constexpr size_t G1_NUM_MOTOR = 29;

namespace ocs2::humanoid {

class HardwareUnitreeG1{
   public:
    explicit HardwareUnitreeG1(const rclcpp::Node::SharedPtr& node,
                               scalar_t mrtDesiredFrequency,
                               scalar_t mpcDesiredFrequency = -1);

    // 发布器
    rclcpp::Publisher<unitree_hg::msg::LowCmd>::SharedPtr lowcmd_pub_;

    void convertMPCInput2LowCmd(const vector_t& jointTorques,
                                const vector_t& state, const vector_t& input, const bool justUpdated);

    scalar_t getTime() { return robot_time_; }

    vector_t getState(){
        // return mpcStateFilter_.getFilteredVector(Eigen::Map<ocs2::vector_t>(last_mpcstate_.state.value.data(), last_mpcstate_.state.value.size()));
        return Eigen::Map<ocs2::vector_t>(last_mpcstate_.state.value.data(), last_mpcstate_.state.value.size());

    }

    std::vector<bool> getContactState() {
        return std::vector<bool> {
            isFootInContact(last_lowstate_.motor_state[4].tau_est,
                            last_lowstate_.motor_state[4].dq,
                            last_lowstate_.motor_state[3].tau_est,
                            last_lowstate_.motor_state[3].dq),
            isFootInContact(last_lowstate_.motor_state[10].tau_est,
                            last_lowstate_.motor_state[10].dq,
                            last_lowstate_.motor_state[9].tau_est,
                            last_lowstate_.motor_state[9].dq)
        };
    }

    unitree_hg::msg::LowCmd getLowCmd() { return last_lowcmd_; }
    
    bool getMPCMode() { return entered_mpc_mode_; }
    private :
    // 回调函数
    rclcpp::Node::SharedPtr node_;
    void lowstateCallback(const unitree_hg::msg::LowState::SharedPtr msg);
    void sportstateCallback(
        const unitree_go::msg::SportModeState::SharedPtr msg);
    

    // 订阅器
    rclcpp::Subscription<unitree_hg::msg::LowState>::SharedPtr lowstate_sub_;
    rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr sportstate_sub_;

    // 缓存消息
    unitree_hg::msg::LowState last_lowstate_;
    unitree_go::msg::SportModeState last_sportstate_;
    unitree_hg::msg::LowCmd last_lowcmd_;
    ocs2_ros2_msgs::msg::MpcInput last_mpcpolicy_;
    ocs2_ros2_msgs::msg::MpcObservation last_mpcstate_;
    scalar_t robot_time_ = 0.0;
    scalar_t init_time_ = 0.0;

    // 力矩限制表（单位 Nm），与 XML <ctrlrange> 对应
    const std::map<int, std::pair<double, double>> G1_TORQUE_LIMITS = {
        {0, {-88.0, 88.0}},   {1, {-88.0, 88.0}},  {2, {-88.0, 88.0}},
        {3, {-139.0, 139.0}}, {4, {-50.0, 50.0}},  {5, {-50.0, 50.0}},
        {6, {-88.0, 88.0}},   {7, {-88.0, 88.0}},  {8, {-88.0, 88.0}},
        {9, {-139.0, 139.0}}, {10, {-50.0, 50.0}}, {11, {-50.0, 50.0}},
        {12, {-88.0, 88.0}},  {13, {-50.0, 50.0}}, {14, {-50.0, 50.0}},
        {15, {-25.0, 25.0}},  {16, {-25.0, 25.0}}, {17, {-25.0, 25.0}},
        {18, {-25.0, 25.0}},  {19, {-25.0, 25.0}}, {20, {-5.0, 5.0}},
        {21, {-5.0, 5.0}},    {22, {-25.0, 25.0}}, {23, {-25.0, 25.0}},
        {24, {-25.0, 25.0}},  {25, {-25.0, 25.0}}, {26, {-25.0, 25.0}},
        {27, {-5.0, 5.0}},    {28, {-5.0, 5.0}}};

    // 位置限制表（单位 rad），与 XML <range> 对应
    const std::map<int, std::pair<double, double>> G1_POSITION_LIMITS = {
        {0, {-1.745, 2.094}},  // LHipPitch
        {1, {-0.523, 2.094}},  // LHipRoll
        {2, {-1.571, 1.571}},  // LHipYaw
        {3, {-0.1, 2.444}},   // LKnee
        {4, {-0.785, 0.436}},  // LAnklePitch
        {5, {-0.3, 0.3}},  // LAnkleRoll

        {6, {-1.745, 2.094}},   // RHipPitch
        {7, {-2.094, 0.523}},   // RHipRoll
        {8, {-1.571, 1.571}},   // RHipYaw
        {9, {-0.1, 2.444}},    // RKnee
        {10, {-0.785, 0.436}},  // RAnklePitch
        {11, {-0.3, 0.3}},  // RAnkleRoll

        {12, {-1.571, 1.571}},  // WaistYaw
        {13, {-0.436, 0.436}},  // WaistRoll
        {14, {-0.436, 0.436}},  // WaistPitch

        {15, {-2.618, 1.745}},  // LShoulderPitch
        {16, {-1.047, 1.571}},  // LShoulderRoll
        {17, {-1.571, 1.571}},  // LShoulderYaw
        {18, {-0.524, 1.571}},  // LElbow

        {19, {-1.571, 1.571}},  // LWristRoll
        {20, {-0.785, 0.785}},  // LWristPitch
        {21, {-0.785, 0.785}},  // LWristYaw

        {22, {-2.618, 1.745}},  // RShoulderPitch
        {23, {-1.571, 1.047}},  // RShoulderRoll
        {24, {-1.571, 1.571}},  // RShoulderYaw
        {25, {-0.524, 1.571}},  // RElbow

        {26, {-1.571, 1.571}},  // RWristRoll
        {27, {-0.785, 0.785}},  // RWristPitch
        {28, {-0.785, 0.785}}   // RWristYaw
    };

    inline double clampTorque(size_t jointIndex, scalar_t tau) {
        auto it = G1_TORQUE_LIMITS.find(jointIndex);
        if (it != G1_TORQUE_LIMITS.end()) {
            return std::clamp(tau, it->second.first, it->second.second);
        }
        return tau;
    }

    static constexpr scalar_t PROTECTION_TRIGGER_DURATION = 0.3;  // seconds

    vector_t torqueViolationStartTime = vector_t::Constant(G1_NUM_MOTOR, -1.0);
    vector_t positionViolationStartTime = vector_t::Constant(G1_NUM_MOTOR, -1.0);

    void checkJointProtection();
    void emergencyProtect(const std::string& reason, size_t jointIdx);
    bool emergency_triggered = false;

    bool base_position_initialized_ = false;
    bool base_rpy_initialized_ = false;

    bool entered_mpc_mode_ = false;

    bool offsets_updated_ = false;

    std::ofstream logFile_;

    std::vector<double> initial_q_;
    bool initial_q_recorded_ = false;

    // std::ofstream testLowstateLogFile_;

    std::ifstream recordedLowstateFile_;
    std::vector<std::vector<double>> recordedLowstateData_;
    size_t replay_index_ = 0;

    bool isFootInContact(double ankle_torque, double ankle_dq,
                         double knee_torque, double knee_dq,
                         double torque_threshold = 3.0,
                         double dq_threshold = 0.4) {
        bool ankle_contact = std::abs(ankle_torque) > torque_threshold &&
                             std::abs(ankle_dq) < dq_threshold;

        bool knee_contact = std::abs(knee_torque) > torque_threshold &&
                            std::abs(knee_dq) < dq_threshold;

        // RCLCPP_INFO_STREAM(
        //     rclcpp::get_logger("ContactDetection"),
        //     "Ankle torque: " << ankle_torque << ", ankle dq: " << ankle_dq
        //                      << " | Knee torque: " << knee_torque
        //                      << ", knee dq: " << knee_dq);

        // 两者之一接触即可判为触地（或都满足更稳健）
        return ankle_contact || knee_contact;
    }

    std::vector<bool> foot_contact_state_ = {false, false};  // 左右脚接触状态

#if ENABLE_SIMULATE
        const std::map<int, double> q_target_map = {
            {0, -0.53},   // left_hip_pitch_joint
            {1, 0.0},    // left_hip_roll_joint
            {2, 0.0},    // left_hip_yaw_joint
            {3, 1.0},    // left_knee_joint
            {4, -0.5},   // left_ankle_pitch_joint
            {5, 0.0},    // left_ankle_roll_joint
            {6, -0.53},   // right_hip_pitch_joint
            {7, 0.0},    // right_hip_roll_joint
            {8, 0.0},    // right_hip_yaw_joint
            {9, 1.0},    // right_knee_joint
            {10, -0.5},  // right_ankle_pitch_joint
            {11, 0.0},   // right_ankle_roll_joint
            {12, 0.0},   // waist_yaw_joint
            {13, 0.0},   // waist_roll_joint
            {14, 0.0},   // waist_pitch_joint
            {15, 0.0},   // left_shoulder_pitch_joint
            {16, 0.0},   // left_shoulder_roll_joint
            {17, 0.0},   // left_shoulder_yaw_joint
            {18, 0.0},   // left_elbow_joint
            {19, 0.0},   // left_wrist_roll_joint
            {20, 0.0},   // left_wrist_pitch_joint
            {21, 0.0},   // left_wrist_yaw_joint
            {22, 0.0},   // right_shoulder_pitch_joint
            {23, 0.0},   // right_shoulder_roll_joint
            {24, 0.0},   // right_shoulder_yaw_joint
            {25, 0.0},   // right_elbow_joint
            {26, 0.0},   // right_wrist_roll_joint
            {27, 0.0},   // right_wrist_pitch_joint
            {28, 0.0}    // right_wrist_yaw_joint
        };
    #else
        const std::map<int, double> q_target_map = {
            {0, -0.5},   // left_hip_pitch_joint
            {1, 0.0},    // left_hip_roll_joint
            {2, 0.0},    // left_hip_yaw_joint
            {3, 1.0},    // left_knee_joint
            {4, -0.5},   // left_ankle_pitch_joint
            {5, 0.0},    // left_ankle_roll_joint
            {6, -0.5},   // right_hip_pitch_joint
            {7, 0.0},    // right_hip_roll_joint
            {8, 0.0},    // right_hip_yaw_joint
            {9, 1.0},    // right_knee_joint
            {10, -0.5},  // right_ankle_pitch_joint
            {11, 0.0},   // right_ankle_roll_joint
            {12, 0.0},   // waist_yaw_joint
            {13, 0.0},   // waist_roll_joint
            {14, 0.0},   // waist_pitch_joint
            {15, 0.0},   // left_shoulder_pitch_joint
            {16, 0.0},   // left_shoulder_roll_joint
            {17, 0.0},   // left_shoulder_yaw_joint
            {18, 0.0},   // left_elbow_joint
            {19, 0.0},   // left_wrist_roll_joint
            {20, 0.0},   // left_wrist_pitch_joint
            {21, 0.0},   // left_wrist_yaw_joint
            {22, 0.0},   // right_shoulder_pitch_joint
            {23, 0.0},   // right_shoulder_roll_joint
            {24, 0.0},   // right_shoulder_yaw_joint
            {25, 0.0},   // right_elbow_joint
            {26, 0.0},   // right_wrist_roll_joint
            {27, 0.0},   // right_wrist_pitch_joint
            {28, 0.0}    // right_wrist_yaw_joint
        };
    #endif
    #if ENABLE_SIMULATE
        std::array<double, 3> base_position_offset_ = {0.0000, 0.0000, 0.7900};
        std::array<double, 3> base_rpy_offset_ = {0.0000, 0.0000, 0.0000};
#else
        std::array<double, 3> base_position_offset_ = {0.00, 0.00, 0.7894};
        std::array<double, 3> base_rpy_offset_ = {0.00, 0.00, 0.0};
    #endif
};

}  // namespace ocs2::humanoid
