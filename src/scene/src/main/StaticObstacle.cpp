#include "Scene.h"

using namespace std;

int main(int argc, char **argv)
{
    ROS_INFO("Static obstacle avoidance starts.");
    ros::init(argc, argv, "scenario");
    ros::NodeHandle nh;
    string csv_path="/home/saxijing/cilqr/data/static_avoid_centerline.csv";
    Scene scene("Static obstacle avoidance", csv_path, nh);
}