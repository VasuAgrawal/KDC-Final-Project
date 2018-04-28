#include <limits>
#include <ros/ros.h>

#include "std_msgs/Float32MultiArray.h"
#include "kdcyoubot/joint_vector.h"

#include <youbot_driver/youbot/YouBotBase.hpp>
#include <youbot_driver/youbot/YouBotManipulator.hpp>
#include <youbot_driver/youbot/YouBotJointParameter.hpp>

// Yay globals
youbot::YouBotBase* youbot_base = nullptr;
youbot::YouBotManipulator* youbot_arm = nullptr;

template<typename T>
void try_set_joint_data(youbot::YouBotJoint& joint, const T& data) {
    try {
        joint.setData(data);
    } catch (const std::out_of_range& e) {
        ROS_WARN("Error while commanding joint: %s", e.what());
    }
}

// void angle_callback(const kdcyoubot::joint_vector& msg) {
//     ROS_INFO("Called angle callback");
//     for (int i = 0; i < msg.data.size(); ++i) {
//         if (msg.data[i] != std::numeric_limits<float>::infinity()) {
//             int joint = i + 1; // Joints are 1 indexed ...
//             ROS_INFO("Sending %f radians to joint %d", msg.data[i], joint);
//             youbot::JointAngleSetpoint setpoint;
//             setpoint.angle = msg.data[i] * radians;
// 
//             try_set_joint_data(youbot_arm->getArmJoint(joint), setpoint);
//         }
//     }
//     return;
// }

void angle_callback(const std_msgs::Float32MultiArray& msg) {
    ROS_INFO("Called angle callback");
}

void velocity_callback(const kdcyoubot::joint_vector& msg) {
    ROS_INFO("Called velocity callback");
    for (int i = 0; i < msg.data.size(); ++i) {
        if (msg.data[i] != std::numeric_limits<float>::infinity()) {
            int joint = i + 1; // Joints are 1 indexed ...
            ROS_INFO("Sending %f radians/sec to joint %d", msg.data[i], joint);
            youbot::JointVelocitySetpoint setpoint;
            setpoint.angularVelocity = msg.data[i] * radians_per_second;

            try_set_joint_data(youbot_arm->getArmJoint(joint), setpoint);
        }
    }
    return;
}

void torque_callback(const kdcyoubot::joint_vector& msg) {
    ROS_INFO("Called torque callback");
    for (int i = 0; i < msg.data.size(); ++i) {
        if (msg.data[i] != std::numeric_limits<float>::infinity()) {
            int joint = i + 1; // Joints are 1 indexed ...
            ROS_INFO("Sending %f Nm to joint %d", msg.data[i], joint);
            youbot::JointTorqueSetpoint setpoint;
            setpoint.torque = msg.data[i] * newton_meters;

            try_set_joint_data(youbot_arm->getArmJoint(joint), setpoint);
        }
    }
    return;
}

void current_callback(const kdcyoubot::joint_vector& msg) {
    ROS_INFO("Called current callback");
    for (int i = 0; i < msg.data.size(); ++i) {
        if (msg.data[i] != std::numeric_limits<float>::infinity()) {
            int joint = i + 1; // Joints are 1 indexed ...
            ROS_INFO("Sending %f A to joint %d", msg.data[i], joint);
            youbot::JointCurrentSetpoint setpoint;
            setpoint.current = msg.data[i] * ampere;

            try_set_joint_data(youbot_arm->getArmJoint(joint), setpoint);
        }
    }
    return;
}

void fold() {
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

void send_feedback(ros::Publisher& angle_pub, ros::Publisher& velocity_pub,
        ros::Publisher& torque_pub, ros::Publisher& current_pub) {

    // kdcyoubot::joint_vector angle_feedback;
    kdcyoubot::joint_vector velocity_feedback;
    kdcyoubot::joint_vector torque_feedback;
    kdcyoubot::joint_vector current_feedback;

    for (int i = 0; i < 5; ++i) {
        auto& joint = youbot_arm->getArmJoint(i + 1); // Stupid 1 indexing ...

        // youbot::JointSensedAngle angle;
        youbot::JointSensedVelocity velocity;
        youbot::JointSensedTorque torque;
        youbot::JointSensedCurrent current;

        // joint.getData(angle);
        joint.getData(velocity);
        joint.getData(torque);
        joint.getData(current);

        // angle_feedback.data[i] = angle.angle.value();
        velocity_feedback.data[i] = velocity.angularVelocity.value();
        torque_feedback.data[i] = torque.torque.value();
        current_feedback.data[i] = current.current.value();
    }

    // angle_pub.publish(angle_feedback);
    velocity_pub.publish(velocity_feedback);
    torque_pub.publish(torque_feedback);
    current_pub.publish(current_feedback);

    std_msgs::Float32MultiArray angle_feedback;
    angle_feedback.data[0] = 1.0;
    angle_feedback.data[1] = 2.0;
    angle_feedback.data[2] = 3.0;

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
    ros::init(argc, argv, "joint_commander");
    ros::NodeHandle n;

    // Set up subscribers for command messages
    ros::Subscriber angle_sub = n.subscribe("angle_setpoints", 1, angle_callback);
    ros::Subscriber velocity_sub = n.subscribe("velocity_setpoints", 1, velocity_callback);
    ros::Subscriber torque_sub = n.subscribe("torque_setpoints", 1, torque_callback);
    ros::Subscriber current_sub = n.subscribe("current_setpoints", 1, current_callback);

    // Set up publisher for feedback messages
    // ros::Publisher angle_pub = n.advertise<kdcyoubot::joint_vector>("angle_feedback", 1000);
    ros::Publisher angle_pub = n.advertise<std_msgs::Float32MultiArray>("angle_feedback", 1000);
    ros::Publisher velocity_pub = n.advertise<kdcyoubot::joint_vector>("velocity_feedback", 1000);
    ros::Publisher torque_pub = n.advertise<kdcyoubot::joint_vector>("torque_feedback", 1000);
    ros::Publisher current_pub = n.advertise<kdcyoubot::joint_vector>("current_feedback", 1000);

    // Set up youbot arm
    ROS_INFO("Loading configuration from: %s", YOUBOT_CONFIGURATIONS_DIR);

    youbot_base = new youbot::YouBotBase("youbot-base", YOUBOT_CONFIGURATIONS_DIR);
    youbot_arm = new youbot::YouBotManipulator("youbot-manipulator", YOUBOT_CONFIGURATIONS_DIR);

    if (youbot_base == nullptr || youbot_arm == nullptr) {
        ROS_ERROR("Failed to initialize the arm");
        return 1;
    }

    youbot_base->doJointCommutation();
    youbot_arm->doJointCommutation();
    youbot_arm->calibrateManipulator();
    youbot_arm->calibrateGripper();
    ROS_INFO("Calibrated arm");

    fold();
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
        // Apply the control first (if any), and then send all feedback.
        ros::spinOnce();

        send_feedback(angle_pub, velocity_pub, torque_pub, current_pub);

        loop_rate.sleep();
    }

    return 0;
}
