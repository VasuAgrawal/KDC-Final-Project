#include "ros/ros.h"
#include "youbot_driver/youbot/YouBotBase.hpp"
// #include "youbot/YouBotManipulator.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "youbot1");
    ros::NodeHandle n;

    youbot::YouBotBase ybb("dicks");
}
