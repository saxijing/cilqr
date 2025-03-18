#ifndef SCENE_H_
#define SCENE_H_

#include<ros/ros.h>
#include<visualization_msgs/Marker.h>
#include<visualization_msgs/MarkerArray.h>
#include<nav_msgs/Path.h>
#include<saturn_msgs/State.h>
#include<saturn_msgs/StateArray.h>
#include<fstream>
#include "VehicleModel.h"
#include "Objects.h"
//#include "cilqr.h"

using namespace std;

class Scene
{
    public:
        Scene(const string name, ros::NodeHandle &nh_);
        ~Scene();
        void readGlobalWaypoints(const string filepath);
        void recvCilqrPlannerWaypoints(const nav_msgs::Path &msg);
        void recvCilqrPlannerControl(const geometry_msgs::Vector3 &msg);
        void update();

    protected:
        string scene_name;
        ros::NodeHandle nh;
        ros::Publisher ego_vehicle_pub;
        ros::Publisher obstacles_pub;
        ros::Subscriber cilqr_planner_waypoints_sub;
        ros::Subscriber cilqr_planner_control_sub;
        //for display in rviz,
        ros::Publisher  ego_rviz_pub;
        ros::Publisher  obstacles_rviz_pub;
        ros::Publisher  centerline_pub;
        ros::Publisher  rightedge_pub;
        ros::Publisher  leftedge_pub;
        VehicleModel ego_vehicle;
        Objects obstacles;
        ObjState waypoint;
        vector<ObjState>  global_waypoints;
        double dt;
        double lane_width;
        int lane_num;
        int global_horizon;
};
#endif