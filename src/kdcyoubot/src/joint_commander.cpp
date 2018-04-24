#include <ros/ros.h>

#include "kdcyoubot/joint_vector.h"

#include <youbot_driver/youbot/YouBotBase.hpp>
#include <youbot_driver/youbot/YouBotManipulator.hpp>
#include <youbot_driver/youbot/YouBotJointParameter.hpp>

void angle_callback(const kdcyoubot::joint_vector& msg) {
    return;
}

void velocity_callback(const kdcyoubot::joint_vector& msg) {
    return;
}

void torque_callback(const kdcyoubot::joint_vector& msg) {
    return;
}

void current_callback(const kdcyoubot::joint_vector& msg) {
    return;
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

/**
 * Main routine that manages the youbot arm
 *
 * All commands to the arm will happen in the callbacks. Queued callbacks are 
 * called serially at the end of the main loop, when spinOnce() is called.
 *
 * Each loop, feedback will be published regardless of commands being called.
 *
 * There should be no commands sent to the arm outside of the callbacks.
 *
 * To avoid weirdness, also try not to send commands to more than one channel
 * at a time.
 */
int main(int argc, char **argv) {
    ros::init(argc, argv, "joint-commander");
    ros::NodeHandle n;

    // Set up subscribers for command messages
    ros::Subscriber angle_sub = n.subscribe("angle_setpoints", 1, angle_callback);
    ros::Subscriber velocity_sub = n.subscribe("velocity_setpoints", 1, velocity_callback);
    ros::Subscriber torque_sub = n.subscribe("torque_setpoints", 1, torque_callback);
    ros::Subscriber current_sub = n.subscribe("current_setpoints", 1, current_callback);

    // Set up publisher for feedback messages
    ros::Publisher angle_pub = n.advertise<kdcyoubot::joint_vector>("angle_feedback", 1000);
    ros::Publisher velocity_pub = n.advertise<kdcyoubot::joint_vector>("velocity_feedback", 1000);
    ros::Publisher torque_pub = n.advertise<kdcyoubot::joint_vector>("torque_feedback", 1000);
    ros::Publisher current_pub = n.advertise<kdcyoubot::joint_vector>("current_feedback", 1000);

    // Set up youbot arm
    ROS_INFO("Loading configuration from: %s", YOUBOT_CONFIGURATIONS_DIR);

    auto youbot_base = new youbot::YouBotBase("youbot-base", YOUBOT_CONFIGURATIONS_DIR);
    auto youbot_arm = new youbot::YouBotManipulator("youbot-manipulator", YOUBOT_CONFIGURATIONS_DIR);

    if (youbot_base == nullptr || youbot_arm == nullptr) {
        ROS_ERROR("Failed to initialize the arm");
        return 1;
    }

    youbot_base->doJointCommutation();
    youbot_arm->doJointCommutation();
    youbot_arm->calibrateManipulator();
    youbot_arm->calibrateGripper();
    ROS_INFO("Calibrated arm");

    fold(youbot_arm);
    ROS_INFO("Folded arm");

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

    ros::Rate loop_rate(50);

    // Main loop
    while(ros::ok()) {
        //@TODO: Fill these in with actual feedback
        kdcyoubot::joint_vector angle_feedback;
        kdcyoubot::joint_vector velocity_feedback;
        kdcyoubot::joint_vector torque_feedback;
        kdcyoubot::joint_vector current_feedback;



        angle_pub.publish(angle_feedback);
        velocity_pub.publish(velocity_feedback);
        torque_pub.publish(torque_feedback);
        current_pub.publish(current_feedback);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
