#include "Scene.h"

using namespace std;

int main(int argc, char **argv)
{
    ROS_INFO("Static obstacle avoidance starts.");
    ros::init(argc, argv, "scenario");
    ros::NodeHandle nh;
    Scene scene("Static obstacle avoidance", nh);
}