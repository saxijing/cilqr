#include "Scene.h"

using namespace std;

int main(int argc, char **argv)
{
    ROS_INFO("Follow leading car starts.");
    ros::init(argc, argv, "following_scene");
    ros::NodeHandle nh;
    Scene scene("Following", nh);
    scene.reposeEgoVehicle(10, 100, 0.746, 5.0, 0.01);
    Object leading_vehicle(1, "leading_vehicle", 40.11, 127.83, 0.746, 0.3, 0.0, 0.01);
    scene.addObject(leading_vehicle);
    scene.update();
}