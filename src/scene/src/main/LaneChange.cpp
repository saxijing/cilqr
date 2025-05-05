#include "Scene.h"

using namespace std;

int main(int argc, char **argv)
{
    ROS_INFO("Lane Change planner start!");
    ros::init(argc, argv, "lane_change_scene");
    ros::NodeHandle nh;
    Scene scene("Lance change", nh);
    scene.reposeEgoVehicle(7.624,102.57, 0.746, 5.0, 0.01);
    Object npc_vehicle(1, "static_vehicle",15.14, 104.752, 0.746, 0.18, 0.0, 0.01);
    scene.addObject(npc_vehicle);
    scene.update();
}