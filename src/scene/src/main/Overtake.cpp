#include "Scene.h"

using namespace std;

int main(int argc, char **argv)
{
    ROS_INFO("Overtake starts.");
    ros::init(argc, argv, "overtake_scene");
    ros::NodeHandle nh;
    Scene scene("Overtake", nh);
    scene.reposeEgoVehicle(10, 100, 0.746, 5.0, 0.01);
    Object npc_vehicle(1, "npc_vehicle",33.50, 121.72, 0.746, 0.15, 0.0, 0.01);
    scene.addObject(npc_vehicle);
    scene.update();
}