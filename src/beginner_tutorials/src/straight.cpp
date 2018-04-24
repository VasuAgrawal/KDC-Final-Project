#include <iostream>
#include <memory>
#include <vector>

#include <ros/ros.h>
#include "youbot_driver/youbot/YouBotBase.hpp"
#include "youbot_driver/youbot/YouBotManipulator.hpp"
#include "youbot_driver/youbot/YouBotJointParameter.hpp"

inline float deg2rad(float rad) {
    return rad * 3.14 / 180;
}

void fold(youbot::YouBotManipulator* youbot_arm) {
    const std::vector<youbot::JointAngleSetpoint> fold_angles = {
        0.11 * radian,
        0.11 * radian,
        -0.11 * radian,
        0.11 * radian,
        0.12 * radian
    };

    youbot_arm->setJointData(fold_angles);
    SLEEP_MILLISEC(4000);
    youbot_arm->getArmGripper().open();
    SLEEP_MILLISEC(4000);
    youbot_arm->getArmGripper().close();
    SLEEP_MILLISEC(4000);
}

int main() {

    std::cout << "Loading configuration from:"
              << YOUBOT_CONFIGURATIONS_DIR << std::endl;


    // Let's hope this doesn't throw.
    auto youbot_base = new youbot::YouBotBase(
            "youbot-base", YOUBOT_CONFIGURATIONS_DIR);
    auto youbot_arm = new youbot::YouBotManipulator(
            "youbot-manipulator", YOUBOT_CONFIGURATIONS_DIR);

    if (youbot_base == nullptr || youbot_arm == nullptr) {
        std::cout << "Failed to initialize the arm" << std::endl;
        return 1;
    }

    youbot_base->doJointCommutation();
    youbot_arm->doJointCommutation();
    youbot_arm->calibrateManipulator();
    youbot_arm->calibrateGripper();
    std::cout << "Calibrated Arm" << std::endl;

    fold(youbot_arm);
    std::cout << "Folded Arm" << std::endl;

    // Experimentally determined joint limits from the arm:
    std::vector<quantity<plane_angle>> lower_angle_limits = {
    // quantity<plane_angle> lower_angle_limits[] = {
        .011f * radian,
        .011f * radian,
        -5.02f * radian,
        0.023f * radian,
        .12f * radian};
    // quantity<plane_angle> upper_angle_limits[] = {
    std::vector<quantity<plane_angle>> upper_angle_limits = {
        5.84f * radian,
        2.61f * radian,
        -0.02f * radian,
        3.42f * radian,
        5.64f * radian};


    for (int joint = 1; joint <= 5; ++joint) {
        auto upper = upper_angle_limits[joint - 1];
        auto lower = lower_angle_limits[joint - 1];

        youbot_arm->getArmJoint(joint).setData(lower);
        for (int step = 0; step <= 10; ++step) {
            auto target = ((upper.value() - lower.value()) * step / 10 + lower.value()) * radian;
            std::cout << target.value() << std::endl;
            youbot_arm->getArmJoint(joint).setData(target);
            SLEEP_MILLISEC(500);
        }

        SLEEP_MILLISEC(5000);
    }
}
