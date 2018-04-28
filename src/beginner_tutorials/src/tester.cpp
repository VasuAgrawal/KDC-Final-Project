#include <ros/ros.h>
#include "std_msgs/Float32MultiArray.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "tester");
    ros::NodeHandle n;
    ros::Publisher test_pub = n.advertise<std_msgs::Float32MultiArray>("test", 1);
    ros::Rate loop_rate(10);

    while (ros::ok()) {

        std_msgs::Float32MultiArray arr;
        arr.data.push_back(1);
        test_pub.publish(arr);

        loop_rate.sleep();
    }
}
