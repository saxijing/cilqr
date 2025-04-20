#include "Scene.h"

using namespace std;

int main(int argc, char **argv)
{
    ROS_INFO("Static obstacle avoidance starts.");
    ros::init(argc, argv, "static_obstacle_scene");
    ros::NodeHandle nh;
    Scene scene("Static obstacle avoidance", nh);
    scene.reposeEgoVehicle(10, 100, 0.746, 5.0, 0.01);
    Object static_vehicle(1, "static_vehicle", 50.388, 137.33, 0.746, 0.0, 0.0, 0.01);
    scene.addObject(static_vehicle);
    scene.update();
}