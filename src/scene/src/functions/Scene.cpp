#include "Scene.h"

using namespace std;

Scene::Scene(const string name, ros::NodeHandle &nh_)
{
    scene_name=name;
    nh=nh_;
    ego_vehicle_pub=nh.advertise<saturn_msgs::State>("ego_vehicle/state", 10, true);
    obstacles_pub=nh.advertise<saturn_msgs::StateArray>("obstacles/state", 10, true);
    cilqr_planner_waypoints_sub=nh.subscribe("/cilqr_planner/waypoints", 10, &Scene::recvCilqrPlannerWaypoints, this);
    cilqr_planner_control_sub=nh.subscribe("/cilqr_planner/control", 10, &Scene::recvCilqrPlannerControl, this);
    ego_rviz_pub=nh.advertise<visualization_msgs::Marker>("ego_vehicle/rviz/state", 10, true);
    obstacles_rviz_pub=nh.advertise<visualization_msgs::MarkerArray>("obstacles/rviz/state", 10, true);
    centerline_pub=nh.advertise<nav_msgs::Path>("centerline", 10, true);
    rightedge_pub=nh.advertise<nav_msgs::Path>("rightedge", 10, true);
    leftedge_pub=nh.advertise<nav_msgs::Path>("leftedge", 10, true);

    ROS_INFO("Scene parameters loading...");
    nh.getParam("/timestep/control", dt);
    nh.getParam("/road_info/lane_width", lane_width);
    nh.getParam("/road_info/lane_num", lane_num);
    nh.getParam("/planner/global_horizon", global_horizon);
}

Scene::~Scene()
{
    ego_vehicle.~VehicleModel();
    obstacles.~Objects();
    global_waypoints.clear();
    ROS_INFO("Scene destructed!");
}

void Scene::readGlobalWaypoints(const string filepath)
{
    ifstream filew(filepath);
    if(!filew.is_open())
    {
        cerr<<"无法打开文件"<<endl;
        return;
    }
    string linew;
    //read the first line of title
    getline(filew, linew);
    while(getline(filew, linew))
    {
        stringstream sss(linew);
        string itemw;
        for(int col=0; getline(sss, itemw, ','); col++)
        {
            try
            {
                switch(col)
                {
                    case 0:
                        waypoint.x=stod(itemw);
                        break;
                    case 1:
                        waypoint.y=stod(itemw);
                        break;
                    case 2:
                        waypoint.theta=stod(itemw);
                        break; 
                    case 3:
                        waypoint.v=stod(itemw);
                        break;
                    case 4:
                        waypoint.accel=stod(itemw);
                        break;
                    case 5:
                        waypoint.yaw_rate=stod(itemw);
                        break;
                }
            }
            catch(const invalid_argument &e)
            {    cerr<<"转换错误："<<e.what()<<endl;}
            catch(const out_of_range &e)
            {    cerr<<"值超出范围："<<e.what()<<endl;}
        }
        global_waypoints.push_back(waypoint);
    }
    filew.close();
    cout<<"read "<<global_waypoints.size()<<"  row, global waypoints is loaded!"<<endl;
}

void Scene::recvCilqrPlannerWaypoints(const nav_msgs::Path &msg)
{
    
}

void Scene::recvCilqrPlannerControl(const saturn_msgs::ControlArray  &msg)
{

}

void Scene::update()
{
    //readGlobalWaypoints("/home/saxijing/catkin_ws/src/motion_planner/data/global_waypoints.txt");
}