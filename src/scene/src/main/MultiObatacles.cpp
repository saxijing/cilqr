#include "Scene.h"

using namespace std;

int main(int argc, char **argv)
{
    ROS_INFO("Starting Multiple Obstacles Scene!");
    ros::init(argc, argv, "multiple_obstacle_scene");
    ros::NodeHandle nh;
    Scene scene("Multi obstacles avoidance", nh);
    scene.reposeEgoVehicle(10, 100, 0.746, 5.0, 0.01);
    Object static_vehicle1(1, "static_vehicle1",33.50, 121.72, 0.746, 0.0, 0.0, 0.01);
    scene.addObject(static_vehicle1);
    Object static_vehicle2(2, "static_vehicle2", 61.2303, 152.122, 0.746, 0.0, 0.0, 0.01);
    scene.addObject(static_vehicle2);
    Object static_vehicle3(3, "static_vehicle", 113.01, 194.811, 0.746, 0.0, 0.0, 0.01);
    scene.addObject(static_vehicle3);
    Object static_vehicle4(4, "static_vehicle4", 141.272, 226.11, 0.746, 0.0, 0.0, 0.01);
    scene.addObject(static_vehicle4);
    scene.update();
}