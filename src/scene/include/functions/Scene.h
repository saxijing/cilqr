#ifndef SCENE_H_
#define SCENE_H_

#include<ros/ros.h>
#include<visualization_msgs/Marker.h>
#include<visualization_msgs/MarkerArray.h>
#include<nav_msgs/Path.h>
#include<saturn_msgs/State.h>
#include<saturn_msgs/StateLite.h>
#include<saturn_msgs/Size.h>
#include<saturn_msgs/ObstacleState.h>
#include<saturn_msgs/ObstacleStateArray.h>
#include<saturn_msgs/Control.h>
#include<saturn_msgs/ControlArray.h>
#include<saturn_msgs/Path.h>
#include<common/VehicleModel.h>
#include<fstream>
#include<mutex>
#include "Objects.h"

//#include "cilqr.h"

using namespace std;

class Scene
{
    public:
        Scene(const string name, ros::NodeHandle &nh_);
        ~Scene();
        void readCenterlineAndCalRoadEdge();
        void findClosestIndex(const vector<ObjState>& point_lst, const ObjState& point, int& closest_i, bool isFromStart);
        void recvCilqrPlannedPath(const saturn_msgs::Path& msg);
        void reposeEgoVehicle(const double x, const double y, const double theta, const double v0, const double dT);
        void reposeEgoVehicle(const ObjState& target_state);
        void resizeEgoVehicle(const double l, const double w, const double h);
        void reconfigEgoVehVDpara(const double m, const double L, const double B);
        void addObject(const Object &obj);
        void removeObjectByID(const int id);
        void removeObjectByIndex(const int obj_index);
        void update();

    protected:
        string scene_name;
        string centerline_filepath;
        ros::NodeHandle nh;
        ros::Publisher ego_vehicle_pub;
        ros::Publisher obstacles_pub;
        ros::Publisher rviz_planned_path_pub;
        ros::Subscriber cilqr_planned_path_sub;
        //for display in rviz,
        ros::Publisher  ego_rviz_pub;
        ros::Publisher  obstacles_rviz_pub;
        ros::Publisher  centerline_pub;
        ros::Publisher  rightedge_pub;
        ros::Publisher  leftedge_pub;

        VehicleModel ego_vehicle;
        Objects obstacles;
        ObjState waypoint;
        ObjState right_point;
        ObjState left_point;
        ObjState path_point;
        CtrlInput control_signal;
        ObjState ego_vehicle_state;
        ObjState ego_predict_state;
        vector<ObjState>  centerline_points;
        vector<ObjState> rightedge_points;
        vector<ObjState> leftedge_points;
        vector<ObjState> planned_path_lst;
        vector<CtrlInput> local_control_lst;
        bool isFirstFrameFlag;
        double ego_max_speed;
        double dt, planning_dt;
        double lane_width;
        int lane_num;
        int global_horizon;
        int prediction_horizon;
        int control_index;
        int planning_start_index, delta_start_index;
        int current_index;
        double path_joint_error;
        double curr_error;
        double dist, closest_dist;
        // int closest_index;
        int planning_start_index_multiple;
};
#endif