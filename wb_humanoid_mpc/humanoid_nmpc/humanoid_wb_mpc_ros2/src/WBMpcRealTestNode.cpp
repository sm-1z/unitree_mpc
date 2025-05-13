// Haolin Ma 2025-04-10 21:41:05

#include <humanoid_wb_mpc/WBMpcInterface.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
// #include <ocs2_ros2_interfaces/mrt/MRT_ROS_Dummy_Loop.h>
#include<humanoid_wb_mpc/mrt/MRT_ROS_Real_Loop.h>
#include <ocs2_ros2_interfaces/mrt/MRT_ROS_Interface.h>

#include "g1_hardware_interface/HardwareUnitreeG1.h"

#include "humanoid_common_mpc_ros2/visualization/HumanoidVisualizer.h"
#include "humanoid_wb_mpc/common/WBAccelPinocchioStateInputMapping.h"

using namespace ocs2;
using namespace ocs2::humanoid;

int main(int argc, char** argv) {
    std::vector<std::string> programArgs;
    programArgs = rclcpp::remove_ros_arguments(argc, argv);
    if (programArgs.size() < 5) {
        throw std::runtime_error(
            "No robot name, config folder, target command file, or description "
            "name specified. Aborting.");
    }

    const std::string robotName(argv[1]);
    const std::string taskFile(argv[2]);
    const std::string referenceFile(argv[3]);
    const std::string urdfFile(argv[4]);

    // Initialize ros2 node
    rclcpp::init(argc, argv);
    // auto nodeHandle = std::make_shared<rclcpp::Node>(robotName +
    // "_dummy_mrt");
    auto qos = rclcpp::QoS(1);
    qos.best_effort();

    // Robot interface
    WBMpcInterface interface(taskFile, urdfFile, referenceFile);

    // MRT
    rclcpp::Node::SharedPtr nodeHandle =
        std::make_shared<rclcpp::Node>(robotName + "_mrt");

    MRT_ROS_Interface mrt(robotName);
    mrt.initRollout(&interface.getRollout());
    mrt.launchNodes(nodeHandle, qos);

    std::shared_ptr<HumanoidVisualizer> humanoidVisualizer(
        new HumanoidVisualizer(taskFile, interface.getPinocchioInterface(),
                               interface.getMpcRobotModel(), nodeHandle));
    

    // // Dummy legged robot
    // MRT_ROS_Dummy_Loop dummySimulator(
    //     mrt, 80, interface.mpcSettings().mpcDesiredFrequency_);
    // dummySimulator.subscribeObservers({humanoidVisualizer});

    // Real legged robot
    HardwareUnitreeG1 hardwareInterface(
        nodeHandle, interface.mpcSettings().mrtDesiredFrequency_,
        interface.mpcSettings().mpcDesiredFrequency_);

    MRT_ROS_Real_Loop realTestRunner(
        mrt, hardwareInterface, interface.getPinocchioInterface(),
        interface.getMpcRobotModel(),
        interface.mpcSettings().mrtDesiredFrequency_,
        interface.mpcSettings().mpcDesiredFrequency_);

    realTestRunner.subscribeObservers({humanoidVisualizer});

    

    // Initial state

    SystemObservation initObservation;
    // initObservation.state = interface.getInitialState();
    // scalar_t start_bias_time = hardwareInterface.getTime();
    initObservation.state = hardwareInterface.getState();
    std::cout << "initial state:" << initObservation.state << std::endl;
    initObservation.input =
        vector_t::Zero(interface.getMpcRobotModel().getInputDim());
    initObservation.mode = ModeNumber::STANCE;

    // Initial command
    TargetTrajectories initTargetTrajectories({0.0},
                                              {initObservation.state},
                                              {initObservation.input});

    // // run dummy
    // dummySimulator.run(initObservation, initTargetTrajectories);

    // run real
    realTestRunner.run(initObservation, initTargetTrajectories);

    // Successful exit
    return 0;
}
