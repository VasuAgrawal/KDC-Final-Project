
#include "ros/ros.h"
#include <beginner_tutorials/angle_setpoint.h>

void angle_callback(const beginner_tutorials::angle_setpoint& msg) {
    for (int i = 0; i < 5; ++i) {
        std::cout << "Angle " << i + 1 << ": " << msg.angle[i] << std::endl;
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "joint-slave");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("angle_setpoints", 1, angle_callback);

}
