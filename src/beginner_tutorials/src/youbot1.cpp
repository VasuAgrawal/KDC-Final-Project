#include "ros/ros.h"
#include "youbot_driver/youbot/YouBotBase.hpp"
#include "youbot_driver/youbot/YouBotManipulator.hpp"

using namespace youbot;

int main(int argc, char** argv) {
    ros::init(argc, argv, "YouBot");
    ros::NodeHandle n;

    ROS_INFO(YOUBOT_CONFIGURATIONS_DIR);

    YouBotManipulator myYouBot("YouBot", YOUBOT_CONFIGURATIONS_DIR);

    myYouBot.doJointCommutation();
    myYouBot.calibrateManipulator();

    std::vector<JointSensedAngle> feedback;
    myYouBot.getJointData(feedback);

    ROS_INFO("Joint feedback data:");
    for (int i = 0; i < feedback.size(); i++) {
        ROS_INFO("Joint %d angle: %f", i, feedback[i].angle.value());
    }
}
