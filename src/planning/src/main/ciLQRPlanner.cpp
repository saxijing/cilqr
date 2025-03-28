#include "ciLQR.h"

using namespace std;

int main(int argc, char **argv)
{
    ROS_INFO("ciLQR Planner  starts.");
    ros::init(argc, argv, "planner");
    ros::NodeHandle nh;
    ciLQR cilqr_planner(nh);
}