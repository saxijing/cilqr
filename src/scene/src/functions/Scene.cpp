#include "Scene.h"

using namespace std;

Scene::Scene(const string name, ros::NodeHandle &nh_)
{
    scene_name=name;
    nh=nh_;
    ego_vehicle_pub=nh.advertise<saturn_msgs::State>("/ego_vehicle/state", 10, true);
    obstacles_pub=nh.advertise<saturn_msgs::ObstacleStateArray>("/obstacles/state", 10, true);
    cilqr_planner_control_sub=nh.subscribe("/cilqr_planner/control", 10, &Scene::recvCilqrPlannerControl, this);
    ego_rviz_pub=nh.advertise<visualization_msgs::Marker>("/ego_vehicle/rviz/state", 10, true);
    obstacles_rviz_pub=nh.advertise<visualization_msgs::MarkerArray>("/obstacles/rviz/state", 10, true);
    centerline_pub=nh.advertise<nav_msgs::Path>("/centerline", 10, true);
    rightedge_pub=nh.advertise<nav_msgs::Path>("/rightedge", 10, true);
    leftedge_pub=nh.advertise<nav_msgs::Path>("/leftedge", 10, true);

    ROS_INFO("Scene parameters loading...");
    nh.getParam("/planner/global_file_path", centerline_filepath);
    nh.getParam("/timestep/control", dt);
    nh.getParam("/road_info/lane_width", lane_width);
    nh.getParam("/road_info/lane_num", lane_num);
    nh.getParam("/planner/global_horizon", global_horizon);
    nh.getParam("/planner/prediction_horizon", prediction_horizon);
    
    control_index=0;

    readCenterlineAndCalRoadEdge();
    update();
}

Scene::~Scene()
{
    ego_vehicle.~VehicleModel();
    obstacles.~Objects();
    centerline_points.clear();
    rightedge_points.clear();
    leftedge_points.clear();
    local_control_lst.clear();
    ROS_INFO("Scene destructed!");
}

void Scene::readCenterlineAndCalRoadEdge()
{
    centerline_points.clear();
    rightedge_points.clear();
    leftedge_points.clear();
    ifstream filew(centerline_filepath);
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
                        waypoint.v=stod(itemw);
                        break; 
                    case 3:
                        waypoint.theta=stod(itemw);
                        break;
                }
            }
            catch(const invalid_argument &e)
            {    cerr<<"转换错误："<<e.what()<<endl;}
            catch(const out_of_range &e)
            {    cerr<<"值超出范围："<<e.what()<<endl;}
        }
        waypoint.accel=0;
        waypoint.yaw_rate=0;
        centerline_points.push_back(waypoint);

        right_point.x=waypoint.x+0.5*lane_num*lane_width*sin(waypoint.theta);
        right_point.y=waypoint.y-0.5*lane_num*lane_width*cos(waypoint.theta);
        right_point.theta=waypoint.theta;
        right_point.v=waypoint.v;
        right_point.accel=waypoint.accel;
        right_point.yaw_rate=waypoint.yaw_rate;
        rightedge_points.push_back(right_point);

        left_point.x=waypoint.x-0.5*lane_num*lane_width*sin(waypoint.theta);
        left_point.y=waypoint.y+0.5*lane_num*lane_width*cos(waypoint.theta);
        left_point.theta=waypoint.theta;
        left_point.v=waypoint.v;
        left_point.accel=waypoint.accel;
        left_point.yaw_rate=waypoint.yaw_rate;
        leftedge_points.push_back(left_point);
    }
    filew.close();
    cout<<"read "<<centerline_points.size()<<"  row, road points is loaded!"<<endl;
}

void Scene::recvCilqrPlannerControl(const saturn_msgs::ControlArray  &msg)
{
    lock_guard<mutex> lock(data_mutex);
    local_control_lst.clear();
    for(int i=0; i<msg.control_lst.size(); i++)
    {
        control_signal.accel=msg.control_lst[i].u_accel;
        control_signal.yaw_rate=msg.control_lst[i].u_yawrate;
        local_control_lst.push_back(control_signal);
    }
    control_index=0;
}

void Scene::update()
{
    //readCenterlineAndCalRoadEdge("/home/saxijing/catkin_ws/src/motion_planner/data/centerline.csv");
    int control_rate=1/dt;
    ros::Rate rate(control_rate);
    saturn_msgs::State ego_state_msg;
    saturn_msgs::StateLite obj_statelite_msg;
    saturn_msgs::ObstacleState obs_state_msg;
    saturn_msgs::ObstacleStateArray obs_statearray_msg;
    visualization_msgs::Marker ego_rviz_msg;
    visualization_msgs::Marker obj_rviz_msg;
    visualization_msgs::MarkerArray obstacles_rviz_msg;
    nav_msgs::Path centerline_msg;
    nav_msgs::Path rightedge_msg;
    nav_msgs::Path leftedge_msg; 

    geometry_msgs::PoseStamped road_pose;

    //send road line to rviz
    centerline_msg.header.frame_id="map";
    centerline_msg.header.stamp=ros::Time::now();
    rightedge_msg.header.frame_id="map";
    rightedge_msg.header.stamp=ros::Time::now();
    leftedge_msg.header.frame_id="map";
    leftedge_msg.header.stamp=ros::Time::now();
    for(int i=0; i<centerline_points.size(); i++)
    {
        road_pose.pose.position.x=centerline_points[i].x;
        road_pose.pose.position.y=centerline_points[i].y;
        road_pose.pose.position.z=0;
        road_pose.pose.orientation.x=0;
        road_pose.pose.orientation.y=0;
        road_pose.pose.orientation.z=sin(centerline_points[i].theta/2);
        road_pose.pose.orientation.w=cos(centerline_points[i].theta/2);
        centerline_msg.poses.push_back(road_pose);

        road_pose.pose.position.x=rightedge_points[i].x;
        road_pose.pose.position.y=rightedge_points[i].y;
        road_pose.pose.position.z=0;
        road_pose.pose.orientation.x=0;
        road_pose.pose.orientation.y=0;
        road_pose.pose.orientation.z=sin(rightedge_points[i].theta/2);
        road_pose.pose.orientation.w=cos(rightedge_points[i].theta/2);
        rightedge_msg.poses.push_back(road_pose);

        road_pose.pose.position.x=leftedge_points[i].x;
        road_pose.pose.position.y=leftedge_points[i].y;
        road_pose.pose.position.z=0;
        road_pose.pose.orientation.x=0;
        road_pose.pose.orientation.y=0;
        road_pose.pose.orientation.z=sin(leftedge_points[i].theta/2);
        road_pose.pose.orientation.w=cos(leftedge_points[i].theta/2);
        leftedge_msg.poses.push_back(road_pose);
    }
    centerline_pub.publish(centerline_msg);
    rightedge_pub.publish(rightedge_msg);
    leftedge_pub.publish(leftedge_msg);

    //set ego_vehicle and obstacles rviz geometry
    ego_rviz_msg.type=visualization_msgs::Marker::CUBE;
    ego_rviz_msg.scale.x=ego_vehicle.getLength();
    ego_rviz_msg.scale.y=ego_vehicle.getWidth();
    ego_rviz_msg.scale.z=ego_vehicle.getHeight();
    ego_rviz_msg.color.g=1.0;
    ego_rviz_msg.color.a=0.5;

    obj_rviz_msg.type=visualization_msgs::Marker::CUBE;
    obj_rviz_msg.color.a=0.5;


    while(ros::ok())
    {
        //fulfill ego_vehicle_state_msg and publish
        ego_state_msg.header.frame_id="ego_state";
        ego_state_msg.header.stamp=ros::Time::now();
        ego_state_msg.id=0;
        ego_state_msg.name="ego_vehicle";
        ego_state_msg.x=ego_vehicle.getPoseX();
        ego_state_msg.y=ego_vehicle.getPoseY();
        ego_state_msg.theta=ego_vehicle.getPoseTheta();
        ego_state_msg.v=ego_vehicle.getVelocity();
        ego_state_msg.accel=ego_vehicle.getAccelerate();
        ego_state_msg.yawrate=ego_vehicle.getYawRate();
        ego_vehicle_pub.publish(ego_state_msg);

        //fulfill ego_state_rviz_msg and publish
        ego_rviz_msg.header.frame_id="ego_rviz";
        ego_rviz_msg.header.stamp=ros::Time::now();
        ego_rviz_msg.id=1;
        ego_rviz_msg.pose.position.x=ego_vehicle.getPoseX();
        ego_rviz_msg.pose.position.y=ego_vehicle.getPoseY();
        ego_rviz_msg.pose.position.z=0;
        ego_rviz_msg.pose.orientation.x=0;
        ego_rviz_msg.pose.orientation.y=0;
        ego_rviz_msg.pose.orientation.z=sin(ego_vehicle.getPoseTheta()/2);
        ego_rviz_msg.pose.orientation.w=cos(ego_vehicle.getPoseTheta()/2);
        ego_rviz_pub.publish(ego_rviz_msg);

        //fulfill obstacles_state_msg and publish
        obs_statearray_msg.header.frame_id="obstacles_state";
        obs_statearray_msg.header.stamp=ros::Time::now();
        obs_statearray_msg.obstacles.clear();
        for(int i=0; i<obstacles.getObjectsNum(); i++)
        {
            obstacles.getObjectByIndexForModify(i).resetPredictHorizon(prediction_horizon);
            obstacles.getObjectByIndexForModify(i).ObjectPredict();

            obs_state_msg.id=obstacles.getObjectByIndex(i).getID();
            obs_state_msg.name=obstacles.getObjectByIndex(i).getName();
            obs_state_msg.predicted_states.clear();
            for(int j=0; j<obstacles.getObjectByIndex(i).getPredictedTraj().size(); j++)
            {
                obj_statelite_msg.x=obstacles.getObjectByIndex(i).getPredictedTraj()[j].x;
                obj_statelite_msg.y=obstacles.getObjectByIndex(i).getPredictedTraj()[j].y;
                obj_statelite_msg.theta=obstacles.getObjectByIndex(i).getPredictedTraj()[j].theta;
                obj_statelite_msg.v=obstacles.getObjectByIndex(i).getPredictedTraj()[j].v;
                obj_statelite_msg.accel=obstacles.getObjectByIndex(i).getPredictedTraj()[j].accel;
                obj_statelite_msg.yawrate=obstacles.getObjectByIndex(i).getPredictedTraj()[j].yaw_rate;
                obs_state_msg.predicted_states.push_back(obj_statelite_msg);
            }
            obs_state_msg.size.length=obstacles.getObjectByIndex(i).getLength();
            obs_state_msg.size.width=obstacles.getObjectByIndex(i).getWidth();
            obs_state_msg.size.height=obstacles.getObjectByIndex(i).getHeight();
            obs_state_msg.size.wheel_base=0;
            obs_state_msg.size.wheel_track=0;

            obs_statearray_msg.obstacles.push_back(obs_state_msg);
        }
        obstacles_pub.publish(obs_statearray_msg);

        //fulfill obstacles_state_rviz_msg and publish
        obstacles_rviz_msg.markers.clear();
        for(int i=0; i<obstacles.getObjectsNum(); i++)
        {
            obj_rviz_msg.scale.x=obstacles.getObjectByIndex(i).getLength();
            obj_rviz_msg.scale.y=obstacles.getObjectByIndex(i).getWidth();
            obj_rviz_msg.scale.z=obstacles.getObjectByIndex(i).getHeight();
            obj_rviz_msg.pose.position.x=obstacles.getObjectByIndex(i).getPoseX();
            obj_rviz_msg.pose.position.y=obstacles.getObjectByIndex(i).getPoseY();
            obj_rviz_msg.pose.position.z=0;
            obj_rviz_msg.pose.orientation.x=0;
            obj_rviz_msg.pose.orientation.y=0;
            obj_rviz_msg.pose.orientation.z=sin(obstacles.getObjectByIndex(i).getPoseTheta()/2);
            obj_rviz_msg.pose.orientation.w=cos(obstacles.getObjectByIndex(i).getPoseTheta()/2);
            obstacles_rviz_msg.markers.push_back(obj_rviz_msg);
        }
        obstacles_rviz_pub.publish(obstacles_rviz_msg);

        //update ego vehicle state
        {
            lock_guard<mutex> lock(data_mutex);
            ego_vehicle.applyU(local_control_lst[control_index].accel, local_control_lst[control_index].yaw_rate);
            ego_vehicle.update();
            control_index++;
            if(control_index>=local_control_lst.size())
            {
                control_index=0;
            }
        }

        //update obstacles state
        for(int i=0; i<obstacles.getObjectsNum(); i++)
        {
            obstacles.getObjectByIndexForModify(i).update();
        }

        rate.sleep();
        ros::spinOnce();
    }
}