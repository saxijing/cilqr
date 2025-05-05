#include "Scene.h"

using namespace std;

Scene::Scene(const string name, ros::NodeHandle &nh_)
{
    scene_name=name;
    nh=nh_;
    ego_vehicle_pub=nh.advertise<saturn_msgs::State>("/scene/ego_vehicle/state", 10, true);
    //ego_predict_pub=nh.advertise<saturn_msgs::State>("/scene/ego_vehicle/predict", 10, true);
    obstacles_pub=nh.advertise<saturn_msgs::ObstacleStateArray>("/scene/obstacles/state", 10, true);
    rviz_planned_path_pub=nh.advertise<nav_msgs::Path>("/rviz/scene/planned_path", 10, true);
    //cilqr_planner_control_sub=nh.subscribe("/cilqr_planner/control", 10, &Scene::recvCilqrPlannerControl, this);
    cilqr_planned_path_sub=nh.subscribe("/cilqr_planner/planned_path", 10, &Scene::recvCilqrPlannedPath, this);
    ego_rviz_pub=nh.advertise<visualization_msgs::Marker>("/scene/ego_vehicle/rviz/state", 1, true);
    obstacles_rviz_pub=nh.advertise<visualization_msgs::MarkerArray>("/scene/obstacles/rviz/state", 1, true);
    centerline_pub=nh.advertise<nav_msgs::Path>("/scene/centerline", 10, true);
    rightedge_pub=nh.advertise<nav_msgs::Path>("/scene/rightedge", 10, true);
    leftedge_pub=nh.advertise<nav_msgs::Path>("/scene/leftedge", 10, true);

    ROS_INFO("Scene parameters loading...");
    nh.getParam("/planner/global_file_path", centerline_filepath);
    nh.getParam("/timestep/control", dt);
    nh.getParam("/timestep/planning", planning_dt);
    nh.getParam("/road_info/lane_width", lane_width);
    nh.getParam("/road_info/lane_num", lane_num);
    nh.getParam("/planner/global_horizon", global_horizon);
    nh.getParam("/planner/prediction_horizon", prediction_horizon);
    nh.getParam("/ego_vehicle/max_speed", ego_max_speed);
    nh.getParam("/planner/path_joint/max_error", path_joint_error);
    nh.getParam("/timestep/multiple/planner_start_index", planning_start_index_multiple);
    planning_start_index=0;
    delta_start_index=planning_start_index_multiple*planning_dt/dt;
    isFirstFrameFlag=true;
    planned_path_lst.clear();
    current_index=0;

    // {
    //     lock_guard<mutex> lock(control_lst_mutex);
    //     lock_control_signal.accel=0;
    //     lock_control_signal.yaw_rate=0;
    // }

    ego_vehicle.resetTimestamp(dt);
    ego_vehicle.setMaxSpeed(ego_max_speed);
    readCenterlineAndCalRoadEdge();
}

Scene::~Scene()
{
    ego_vehicle.~VehicleModel();
    obstacles.~Objects();
    centerline_points.clear();
    rightedge_points.clear();
    leftedge_points.clear();
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
                        waypoint.theta=stod(itemw);
                        break; 
                    case 4:
                        waypoint.v=stod(itemw);
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

void Scene::findClosestIndex(const vector<ObjState>& point_lst, const ObjState& point, int& closest_i, bool isFromStart)
{
    if(point_lst.size()==0)
        return;
    if(isFromStart)
        closest_i=0;
    closest_dist=numeric_limits<double>::max();
    for(int i=closest_i; i<point_lst.size(); i++)
    {
        dist=pow(point_lst[i].x-point.x, 2)+pow(point_lst[i].y-point.y, 2);
        if(dist<closest_dist)
        {
            closest_i=i;
            closest_dist=dist;
        }
    }
}

// void Scene::findPathJointIndex(const vector<ObjState>& point_lst, const ObjState& joint, int& joint_i)
// {
//     joint_i=-1;
//     for(int i=0; i<point_lst.size(); i++)
//     {
//         if(abs(point_lst[i].x-joint.x)<1e-9 && abs(point_lst[i].y-joint.y)<1e-9)
//         {
//             joint_i=i;
//             break;
//         }
//     }
//     if(joint_i==-1)
//         cout<<"search joint index failed!"<<endl;
// }

// void Scene::recvCilqrPlannerControl(const saturn_msgs::ControlArray& msg)
// {
//     ROS_INFO("receive cilqr planner control!");
//     local_control_lst.clear();
//     //cout<<"msg.control_lst.size()="<<msg.control_lst.size()<<endl;
//     for(int i=0; i<msg.control_lst.size(); i++)
//     {
//         control_signal.accel=msg.control_lst[i].u_accel;
//         control_signal.yaw_rate=msg.control_lst[i].u_yawrate;
//         local_control_lst.push_back(control_signal);
//     }
//     //cout<<"local_control_lst.size()="<<local_control_lst.size()<<endl;
//     {
//         lock_guard<mutex> lock(control_lst_mutex);
//         lock_local_control_lst=move(local_control_lst);
//         control_index=0;
//         for(int i=0;i<lock_local_control_lst.size();i++)
//         {
//             cout<<"recv control: "<<lock_local_control_lst[i].accel<<","<<lock_local_control_lst[i].yaw_rate<<endl;
//         }
//     }
// }

void Scene::recvCilqrPlannedPath(const saturn_msgs::Path& msg)
{
    cout<<"receive cilqr planned path!"<<endl;
    if(msg.path.size()==0)
        return;
    if(planned_path_lst.size()>0)
    {
        planning_start_index+=delta_start_index;
        cout<<"planning_start_index="<<planning_start_index<<endl;
        planned_path_lst.erase(planned_path_lst.begin()+planning_start_index, planned_path_lst.end());
        cout<<"after cut 1 path_length="<<planned_path_lst.size()<<endl;
        findClosestIndex(planned_path_lst, ego_vehicle_state, current_index, false);
        cout<<"current_index="<<current_index<<endl;
        planned_path_lst.erase(planned_path_lst.begin(), planned_path_lst.begin()+current_index);
        cout<<"after cut 2 path_length="<<planned_path_lst.size()<<endl;
        planning_start_index-=current_index;
        current_index-=current_index;
        //current_index++;
    }
    for(int i=0; i<msg.path.size(); i++)
    {
        path_point.x=msg.path[i].x;
        path_point.y=msg.path[i].y;
        path_point.v=msg.path[i].v;
        path_point.theta=msg.path[i].theta;
        path_point.accel=msg.path[i].accel;
        path_point.yaw_rate=msg.path[i].yawrate;
        planned_path_lst.push_back(path_point);
    }
    cout<<"after joint path_length="<<planned_path_lst.size()<<endl;
}

void Scene::reposeEgoVehicle(const double x, const double y, const double theta, const double v0, const double dT)
{
    ego_vehicle.repose(x, y, theta, v0, dT);
}

void Scene::reposeEgoVehicle(const ObjState& target_state)
{
    ego_vehicle.repose(target_state);
}

void Scene::resizeEgoVehicle(const double l, const double w, const double h)
{
    ego_vehicle.resize(l, w, h);
}

void Scene::reconfigEgoVehVDpara(const double m, const double L, const double B)
{
    ego_vehicle.reconfigVDpara(m, L, B);
}

void Scene::addObject(const Object &obj)
{
    obstacles.addObject(obj);
}

void Scene::removeObjectByID(const int id)
{
    obstacles.removeObjectByID(id);
}

void Scene::removeObjectByIndex(const int obj_index)
{
    obstacles.removeObjectByIndex(obj_index);
}



void Scene::update()
{
    cout<<"come into update!"<<endl;
    int control_rate=1/dt;
    double ego_initial_speed=ego_vehicle.getVelocity();
    ros::Rate controller_rate(control_rate);
    saturn_msgs::State ego_state_msg;
    saturn_msgs::State ego_predict_msg;
    saturn_msgs::StateLite obj_statelite_msg;
    saturn_msgs::ObstacleState obs_state_msg;
    saturn_msgs::ObstacleStateArray obs_statearray_msg;
    visualization_msgs::Marker ego_rviz_msg;
    visualization_msgs::Marker obj_rviz_msg;
    visualization_msgs::MarkerArray obstacles_rviz_msg;
    nav_msgs::Path planned_path_rviz_msg;
    nav_msgs::Path centerline_msg;
    nav_msgs::Path rightedge_msg;
    nav_msgs::Path leftedge_msg; 

    geometry_msgs::PoseStamped road_pose;
    geometry_msgs::PoseStamped path_pose;

    //send road line to rviz
    centerline_msg.header.frame_id="map";
    centerline_msg.header.stamp=ros::Time::now();
    rightedge_msg.header.frame_id="map";
    rightedge_msg.header.stamp=ros::Time::now();
    leftedge_msg.header.frame_id="map";
    leftedge_msg.header.stamp=ros::Time::now();
    centerline_msg.poses.clear();
    rightedge_msg.poses.clear();
    leftedge_msg.poses.clear();
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
    ego_rviz_msg.action=visualization_msgs::Marker::ADD;
    ego_rviz_msg.pose.position.z=ego_vehicle.getHeight()/2;
    ego_rviz_msg.scale.x=ego_vehicle.getLength();
    ego_rviz_msg.scale.y=ego_vehicle.getWidth();
    ego_rviz_msg.scale.z=ego_vehicle.getHeight();
    ego_rviz_msg.color.g=1.0;
    ego_rviz_msg.color.a=0.5;

    obj_rviz_msg.type=visualization_msgs::Marker::CUBE;
    obj_rviz_msg.color.r=205;
    obj_rviz_msg.color.g=92;
    obj_rviz_msg.color.b=92;
    obj_rviz_msg.color.a=0.5;



    while(ros::ok())
    {
        cout<<"come into ros loop!"<<endl;
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

        //fulfill ego_predict_msg and publish
        // ego_predict_msg.header.frame_id="ego_state";
        // ego_predict_msg.header.stamp.ros::Time::now();
        // ego_predict_msg.id=0;
        // ego_predict_msg.name="ego_vehicle";
        // if(planned_path_lst.size()<=current_index+delta_start_index)
        // {
        //     ego_vehicle.getVehicleState(ego_vehicle_state);
        //     ego_vehicle.getPredictedPose(ego_vehicle_state, ego_vehicle_state.accel, planning_start_index_multiple*planning_dt, ego_predict_state);
        //     //cout<<"ego_pose: "<<ego_vehicle_state.x<<", "<<ego_vehicle_state.y<<", "<<ego_vehicle_state.theta<<", "<<ego_vehicle_state.v<<endl;
        //     //cout<<"ego_predict: "<<ego_predict_state.x<<", "<<ego_predict_state.y<<", "<<ego_predict_state.theta<<", "<<ego_predict_state.v<<endl;
        // }
        // else
        // {
        //     cout<<"path_size="<<planned_path_lst.size()<<",  index="<<delta_start_index<<endl;
        //     ego_predict_state.x=planned_path_lst[current_index+delta_start_index].x;
        //     ego_predict_state.y=planned_path_lst[current_index+delta_start_index].y;
        //     ego_predict_state.theta=planned_path_lst[current_index+delta_start_index].theta;
        //     ego_predict_state.v=planned_path_lst[current_index+delta_start_index].v;
        //     ego_predict_state.accel=planned_path_lst[current_index+delta_start_index].accel;
        //     ego_predict_state.yaw_rate=planned_path_lst[current_index+delta_start_index].yaw_rate;
        //     ego_vehicle.getVehicleState(ego_vehicle_state);
        //     cout<<"ego pose: "<<ego_vehicle_state.x<<", "<<ego_vehicle_state.y<<", "<<ego_vehicle_state.theta<<", "<<ego_vehicle_state.v<<endl;
        //     cout<<"ego_predict from last path:"<<ego_predict_state.x<<", "<<ego_predict_state.y<<", "<<ego_predict_state.theta<<", "<<ego_predict_state.v<<endl;
        // }
        // ego_predict_msg.x=ego_predict_state.x;
        // ego_predict_msg.y=ego_predict_state.y;
        // ego_predict_msg.theta=ego_predict_state.theta;
        // ego_predict_msg.v=ego_predict_state.v;
        // ego_predict_msg.accel=ego_predict_state.accel;
        // ego_predict_msg.yawrate=ego_predict_state.yaw_rate;
        // ego_predict_pub.publish(ego_predict_msg);
        
        //fulfill ego_state_rviz_msg and publish
        ego_rviz_msg.header.frame_id="map";
        ego_rviz_msg.header.stamp=ros::Time::now();
        ego_rviz_msg.ns="ego_rviz";
        ego_rviz_msg.id=1;
        ego_rviz_msg.pose.position.x=ego_vehicle.getPoseX();
        ego_rviz_msg.pose.position.y=ego_vehicle.getPoseY();
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
            obj_rviz_msg.header.frame_id="map";
            obj_rviz_msg.header.stamp=ros::Time::now();
            obj_rviz_msg.ns="obstacle_rviz";
            obj_rviz_msg.id=i+2;
            obj_rviz_msg.type=visualization_msgs::Marker::CUBE;
            obj_rviz_msg.action=visualization_msgs::Marker::ADD;
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

        for(int i=0; i<obstacles.getObjectsNum(); i++)
        {
            obstacles.getObjectByIndexForModify(i).update();
        }

        //fulfill planned_path_msg and publish
        planned_path_rviz_msg.header.frame_id="map";
        planned_path_rviz_msg.header.stamp=ros::Time::now();
        planned_path_rviz_msg.poses.clear();
        for(int i=0; i<planned_path_lst.size(); i++)
        {
            path_pose.pose.position.x=planned_path_lst[i].x;
            path_pose.pose.position.y=planned_path_lst[i].y;
            path_pose.pose.position.z=0;
            path_pose.pose.orientation.x=0;
            path_pose.pose.orientation.y=0;
            path_pose.pose.orientation.z=sin(planned_path_lst[i].theta/2);
            path_pose.pose.orientation.w=cos(planned_path_lst[i].theta/2);
            planned_path_rviz_msg.poses.push_back(path_pose);
        }
        rviz_planned_path_pub.publish(planned_path_rviz_msg);

        //set ego vehicle with initial speed
        cout<<"set speed!"<<endl;
        if(isFirstFrameFlag&&ego_initial_speed>0)
        {
            ego_vehicle.initializeVelocity(ego_initial_speed);
            ego_vehicle.getVehicleState(ego_vehicle_state);
            isFirstFrameFlag=false;
            cout<<"set speed finished!"<<endl;
            ros::spinOnce();
            controller_rate.sleep();
            continue;
        }

        //update ego vehicle state
        //cout<<"here1"<<endl;
        //cout<<"local_control_lst.size()="<<local_control_lst.size()<<endl;
        //cout<<"control_index="<<control_index<<endl;
        //cout<<"before find closest index."<<endl;
        //findClosestIndex(planned_path_lst, ego_vehicle_state, closest_index);
        //cout<<"after find closest index."<<endl;
        // {
        //     //cout<<"come into data lock!"<<endl;
        //     lock_guard<mutex> lock(control_lst_mutex);
        //     if(control_index>=lock_local_control_lst.size()||control_index<0)
        //     {
        //         control_index=0;
        //     }
        //     if(lock_local_control_lst.size()>0)
        //     {
        //         lock_control_signal.accel=lock_local_control_lst[control_index].accel;
        //         lock_control_signal.yaw_rate=lock_local_control_lst[control_index].yaw_rate;
        //         control_index++;
        //         cout<<"control_index="<<control_index<<endl;
        //         cout<<"main loop recved control: "<<lock_control_signal.accel<<","<<lock_control_signal.yaw_rate<<endl;
        //         cout<<"control_signal=["<<lock_control_signal.accel<<", "<<lock_control_signal.yaw_rate<<"];"<<endl;
        //     }
        // }

        // ego_vehicle.applyU(lock_control_signal.accel, lock_control_signal.yaw_rate);
        // ego_vehicle.update();
        
        if(planned_path_lst.size()>0&&current_index+1<planning_start_index+delta_start_index)
        {
            current_index++;
            cout<<"planned path length="<<planned_path_lst.size()<<endl;
            cout<<"current_index="<<current_index<<endl;
            cout<<"next position: "<<planned_path_lst[current_index].x<<", "<<planned_path_lst[current_index].y<<", "<<planned_path_lst[current_index].theta<<endl;
            ego_vehicle.repose(planned_path_lst[current_index]);

            // if(current_index>=0&&current_index<planned_path_lst.size()-1)
            // {
            //     current_index++;
            //     ego_vehicle.repose(planned_path_lst[current_index]);
            //     ego_vehicle.getVehicleState(ego_vehicle_state);
            //     cout<<"update ego vehicle state!"<<endl;
            // }
        }
        ego_vehicle.getVehicleState(ego_vehicle_state);
        ros::spinOnce();
        controller_rate.sleep();
    }
}