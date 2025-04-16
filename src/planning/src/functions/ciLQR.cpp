#include "ciLQR.h"

using namespace std;

ciLQR::ciLQR(ros::NodeHandle &nh_): nh(nh_)
{
    ego_state_sub=nh.subscribe("/scene/ego_vehicle/state", 10, &ciLQR::recvEgoState, this);
    obstacles_state_sub=nh.subscribe("/scene/obstacles/state", 10, &ciLQR::recvObstaclesState, this);
    cilqr_control_pub=nh.advertise<saturn_msgs::ControlArray>("/cilqr_planner/control", 10, true);
    rviz_local_refer_points_pub=nh.advertise<visualization_msgs::Marker>("/cilqr_planner/local_referline/points", 10, true);
    rviz_local_refer_lines_pub=nh.advertise<visualization_msgs::Marker>("/cilqr_planner/local_referline/lines", 10, true);
    rviz_local_planned_path_pub=nh.advertise<nav_msgs::Path>("/cilqr_planner/local_planned_path", 10, true);
    
    ROS_INFO("ciLQR parameters loading...");
    nh.getParam("/planner/global_file_path", global_waypoints_filepath);
    nh.getParam("/variable_num/state", state_num);
    nh.getParam("/variable_num/control", control_num);
    nh.getParam("/planner/global_horizon", global_horizon);
    nh.getParam("/planner/local_horizon", local_horizon);
    nh.getParam("planner/prediction_horizon", prediction_horizon);
    nh.getParam("/planner/poly_order", poly_order);
    nh.getParam("/timestep/planning", dt);
    nh.getParam("/timestep/control", control_dt);
    nh.getParam("/timestep/prediction", predict_dt);
    nh.getParam("/planner/safe_dist/a", safe_a);
    nh.getParam("/planner/safe_dist/b", safe_b);
    nh.getParam("/planner/safe_time", safe_time);
    nh.getParam("/planner/ego_radius", ego_radius);
    nh.getParam("planner/weight/iLQR/w_pos_x", w_posX);
    nh.getParam("planner/weight/iLQR/w_pos_y", w_posY);
    nh.getParam("planner/weight/iLQR/w_vel", w_vel);
    nh.getParam("planner/weight/iLQR/w_theta", w_theta);
    nh.getParam("planner/weight/iLQR/w_accel", w_accel);
    nh.getParam("planner/weight/iLQR/w_yawrate", w_yawrate);
    nh.getParam("/planner/weight/obstacle/q1_front", q1_front);
    nh.getParam("/planner/weight/obstacle/q2_front", q2_front);
    nh.getParam("/planner/weight/obstacle/q1_rear", q1_rear);
    nh.getParam("/planner/weight/obstacle/q2_rear", q2_rear);
    nh.getParam("/planner/weight/control/q1_acc", q1_acc);
    nh.getParam("/planner/weight/control/q2_acc", q2_acc);
    nh.getParam("/planner/weight/control/q1_yawrate", q1_yr);
    nh.getParam("/planner/weight/control/q2_yawrate", q2_yr);
    nh.getParam("/planner/constraints/control/min_accel", a_low);
    nh.getParam("/planner/constraints/control/max_accel", a_high);
    nh.getParam("/planner/constraints/control/min_wheel_angle", steer_low);
    nh.getParam("/planner/constraints/control/max_wheel_angle", steer_high);
    nh.getParam("/ego_vehicle/wheel_base", egoL);
    nh.getParam("/ego_vehicle/height", egoHeight);
    nh.getParam("/planner/ego_lf", ego_lf);
    nh.getParam("/planner/ego_lr", ego_lr);
    nh.getParam("/planner/forward/line_search/beta_min", beta1);
    nh.getParam("/planner/forward/line_search/beta_max", beta2);
    nh.getParam("/planner/forward/line_search/gama", gama);
    nh.getParam("/planner/forward/max_iterate", max_forward_iterate);
    nh.getParam("/planner/optimal/max_iterate", max_optimal_iterate);
    nh.getParam("/obstacle/max_perception_dist", max_percep_dist);
    nh.getParam("/ego_vehicle/max_speed", max_speed);
    nh.getParam("/planner/optimal/lamb/init", lamb_init);
    nh.getParam("/planner/optimal/lamb/decay", lamb_decay);
    nh.getParam("/planner/optimal/lamb/ambify", lamb_ambify);
    nh.getParam("/planner/optimal/lamb/max", lamb_max);
    nh.getParam("/planner/optimal/tol", optimal_tol);
    closest_global_index=0;
    closest_local_index=0;
    isFirstFrame=true;
    planning_start_index=dt/control_dt;
    max_dist=numeric_limits<double>::max();
    dist=0.0;
    alfa=1.0;
    isRecEgoVeh=false;
    
    vd_model.setMaxSpeed(max_speed);
    //Matrix resize
    Q.resize(state_num, state_num);
    Q.setZero();
    R.resize(control_num, control_num);
    R.setZero();
    X.resize(state_num, 1);
    X.setZero();
    X_bar.resize(state_num, 1);
    X_bar.setZero();
    X_nominal.resize(state_num, 1);
    X_nominal.setZero();
    X_front.resize(state_num, 1);
    X_front.setZero();
    X_rear.resize(state_num, 1);
    X_rear.setZero();
    X_obs.resize(state_num, 1);
    X_obs.setZero();
    X_front_obs.resize(state_num, 1);
    X_front_obs.setZero();
    X_rear_obs.resize(state_num, 1);
    X_rear_obs.setZero();
    deltaX.resize(state_num, 1);
    deltaX.setZero();
    U.resize(control_num, 1);
    U.setZero();
    T.resize(state_num, state_num);
    T.setZero();
    dX_front.resize(state_num, state_num);
    dX_front.setZero();
    dX_rear.resize(state_num, state_num);
    dX_rear.setZero();
    dCf.resize(1, state_num);
    dCf.setZero();
    dCr.resize(1, state_num);
    dCr.setZero();
    f_cf.resize(1, 1);
    f_cf.setZero();
    f_cr.resize(1, 1);
    f_cr.setZero();
    dBf.resize(1, state_num);
    dBf.setZero();
    dBr.resize(1, state_num);
    dBr.setZero();
    ddBf.resize(state_num, state_num);
    ddBf.setZero();
    ddBr.resize(state_num, state_num);
    ddBr.setZero();
    dVk.resize(1, state_num);
    dVk.setZero();
    ddVk.resize(state_num, state_num);
    ddVk.setZero();
    dX.resize(1, state_num);
    dX.setZero();
    ddX.resize(state_num, state_num);
    ddX.setZero();
    dU.resize(1, control_num);
    dU.setZero();
    ddU.resize(control_num, control_num);
    ddU.setZero();
    dBu.resize(1,control_num);
    dBu.setZero();
    ddBu.resize(control_num, control_num);
    ddBu.setZero();
    A.resize(state_num, state_num);
    A.setZero();
    B.resize(state_num, control_num);
    B.setZero();
    P.resize(state_num, state_num);
    P.setZero();
    dCu.resize(1, control_num);
    dCu.setZero();
    Qx.resize(1, state_num);
    Qx.setZero();
    Qu.resize(1, control_num);
    Qu.setZero();
    Qxx.resize(state_num, state_num);
    Qxx.setZero();
    Quu.resize(control_num, control_num);
    Quu.setZero();
    Qxu.resize(control_num, state_num);
    Qxu.setZero();
    Qux.resize(state_num, control_num);
    Qux.setZero();
    B_reg.resize(state_num, control_num);
    B_reg.setZero();
    Quu_reg.resize(control_num, control_num);
    Quu_reg.setZero();
    Qxu_reg.resize(control_num, state_num);
    Qxu_reg.setZero();
    Qux_reg.resize(state_num, control_num);
    Qux_reg.setZero();
    K.resize(control_num, state_num);
    K.setZero();
    d.resize(control_num, 1);
    d.setZero();
    deltaU_star.resize(control_num, 1);
    deltaU_star.setZero();
    Regular.resize(state_num, state_num);
    Regular.setZero();
    I.resize(state_num, state_num);
    I.setIdentity();
    M_scalar.resize(1, 1);
    M_scalar.setZero();

    Q(0,0)=w_posX;
    Q(1,1)=w_posY;
    Q(2,2)=w_vel;
    Q(3,3)=w_theta;
    R(0,0)=w_accel;
    R(1,1)=w_yawrate;

    costJ=0;
    costJ_nominal=0;
    forward_counter=0;
    ROS_INFO("ciLQR constructed!");

    readGlobalWaypoints();
}

ciLQR::~ciLQR()
{
    ROS_INFO("ciLQR destructed!");
}

void ciLQR::recvEgoState(const saturn_msgs::State &msg)
{
    isRecEgoVeh=true;
    ego_state.x=msg.x;
    ego_state.y=msg.y;
    ego_state.theta=msg.theta;
    ego_state.v=msg.v;
    ego_state.accel=msg.accel;
    ego_state.yaw_rate=msg.yawrate;
}

void ciLQR::recvObstaclesState(const saturn_msgs::ObstacleStateArray &msg)
{
    if(msg.obstacles.size()==0)
    {
        return;
    }
    obstacles_info.clear();
    for(int i=0; i<msg.obstacles.size();i++)
    {
        obs_info_single.id=msg.obstacles[i].id;
        obs_info_single.name=msg.obstacles[i].name;
        obs_info_single.predicted_states.clear();
        for(int j=0; j<msg.obstacles[i].predicted_states.size(); j++)
        {
            obs_state_single.x=msg.obstacles[i].predicted_states[j].x;
            obs_state_single.y=msg.obstacles[i].predicted_states[j].y;
            obs_state_single.theta=msg.obstacles[i].predicted_states[j].theta;
            obs_state_single.v=msg.obstacles[i].predicted_states[j].v;
            obs_state_single.accel=msg.obstacles[i].predicted_states[j].accel;
            obs_state_single.yaw_rate=msg.obstacles[i].predicted_states[j].yawrate;
            obs_info_single.predicted_states.push_back(obs_state_single);
        }
        obs_info_single.size.length=msg.obstacles[i].size.length;
        obs_info_single.size.width=msg.obstacles[i].size.width;
        obs_info_single.size.height=msg.obstacles[i].size.height;
        obs_info_single.size.wheel_base=msg.obstacles[i].size.wheel_base;
        obs_info_single.size.wheel_track=msg.obstacles[i].size.wheel_track;
        obstacles_info.push_back(obs_info_single);
    }
}

void ciLQR::readGlobalWaypoints()
{
    global_waypoints.clear();
    ifstream filew(global_waypoints_filepath);
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
                    case 5:
                        waypoint.x=stod(itemw);
                        break;
                    case 6:
                        waypoint.y=stod(itemw);
                        break;
                    case 7:
                        waypoint.theta=stod(itemw);
                        break;
                    case 9:
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
        global_waypoints.push_back(waypoint);
    }

    filew.close();
    cout<<"read "<<global_waypoints.size()<<"  row, global waypoints is loaded!"<<endl;
}

void ciLQR::findClosestWaypointIndex(const vector<ObjState> &waypoints, const ObjState &state, int &closest_index, bool isFromStart)
{
    max_dist=numeric_limits<double>::max();
    dist=0.0;
    if(isFromStart)
        closest_index=0;

    for(int i=closest_index; i<waypoints.size(); i++)
    {
        dist=sqrt(pow(waypoints[i].x-state.x, 2)+pow(waypoints[i].y-state.y, 2));
        if(dist<max_dist)
        {
            max_dist=dist;
            closest_index=i;
        }
    }
}

void ciLQR::polynominalFitting()
{
    polyCoeff.clear();
    polyX.resize(local_waypoints.size(), poly_order+1);
    polyY.resize(local_waypoints.size(), 1);
    for(int i=0;i<local_waypoints.size();i++)
    {
        for(int j=0;j<=poly_order;j++)
        {
            polyX(i,j)=pow(local_waypoints[i].x, j);
        }
        polyY(i,0)=local_waypoints[i].y;
    }

    //poly_a=polyX.transpose()*polyX.ldlt().solve(polyX.transpose()*polyY);
    //poly_a = polyX.colPivHouseholderQr().solve(polyY);
    poly_a = polyX.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(polyY);
    for(int i=0;i<=poly_order;i++)
    {
        //order is a0, a1, a2,...an
        polyCoeff.push_back(poly_a(i,0));
    }
}

void ciLQR::getLocalReferPoints(const vector<ObjState>& local_waypoints, const vector<ObjState>& trajectory, vector<ObjState>& local_refer_points)
{
    local_refer_points.clear();
    for(int i=0; i<trajectory.size(); i++)
    {
        findClosestWaypointIndex(local_waypoints, trajectory[i], refer_closest_index, true);
        xm=local_waypoints[refer_closest_index].x;
        ym=local_waypoints[refer_closest_index].y;
        thetam=local_waypoints[refer_closest_index].theta;
        x_ego=trajectory[i].x;
        y_ego=trajectory[i].y;
        theta_ego=trajectory[i].theta;

        xr=xm+(x_ego-xm)*cos(thetam)*cos(thetam)+(y_ego-ym)*sin(thetam)*cos(thetam);
        yr=ym+(x_ego-xm)*sin(thetam)*cos(thetam)+(y_ego-ym)*sin(thetam)*sin(thetam);
        thetar=thetam;

        project_point.x=xr;
        project_point.y=yr;
        project_point.theta=thetar;
        project_point.v=trajectory[i].v;
        local_refer_points.push_back(project_point);
    }
}

double ciLQR::BackwardPassAndGetCostJ(const vector<ObjState>&X_cal_lst, double lambda, bool isCompleteCal)
{
    //X_cal_lst is X_vd_lst, order: 0~N
    double costJ_temp=0;
    double J_temp=0;

    //K, d list order: N-1~0
    if(isCompleteCal)
    {
        K_lst.clear();
        d_lst.clear();

        deltaV1d=0;
        deltaV2d=0;
        cout<<"process: backward pass"<<endl;
    }
    else
        cout<<"process: forward pass"<<endl;
    ROS_INFO("backward_-1");
    cout<<"matrix X size:"<<X.rows()<<"*"<<X.cols()<<endl;
    cout<<"X_cal_lst size="<<X_cal_lst.size()<<endl;
    cout<<X_cal_lst[X_cal_lst.size()-1].x<<", "<<X_cal_lst[X_cal_lst.size()-1].y<<", "<<X_cal_lst[X_cal_lst.size()-1].theta<<", "<<X_cal_lst[X_cal_lst.size()-1].v<<endl;
    // 1. whe k=N, calc relevant variables and ternimal costJ
    X(0,0)=X_cal_lst[X_cal_lst.size()-1].x;
    X(1,0)=X_cal_lst[X_cal_lst.size()-1].y;
    X(2,0)=X_cal_lst[X_cal_lst.size()-1].theta;
    X(3,0)=X_cal_lst[X_cal_lst.size()-1].v;

    X_bar(0,0)=X_bar_lst[X_bar_lst.size()-1].x;
    X_bar(1,0)=X_bar_lst[X_bar_lst.size()-1].y;
    X_bar(2,0)=X_bar_lst[X_bar_lst.size()-1].theta;
    X_bar(3,0)=X_bar_lst[X_bar_lst.size()-1].v;
    ROS_INFO("backward_0");
    dVk=(X_bar-X).transpose()*Q;
    dVk=(dVk.array().abs()<EPS).select(0, dVk);
    ddVk=Q;
    M_scalar=0.5*(X_bar-X).transpose()*Q*(X_bar-X);
    M_scalar=(M_scalar.array().abs()<EPS).select(0, M_scalar);
    costJ_temp+=M_scalar(0,0);
    J_temp+=M_scalar(0,0);
    cout<<"cost_terminal="<<costJ_temp<<endl;
    cout<<"terminalX="<<X(0,0)<<","<<X(1,0)<<","<<X(2,0)<<","<<X(3,0)<<endl;
    cout<<"terminalXbar="<<X_bar(0,0)<<","<<X_bar(1,0)<<","<<X_bar(2,0)<<","<<X_bar(3,0)<<endl;
    for(int m=0; m<X_cal_lst.size(); m++)
    {
        cout<<"X_bar: ("<<X_bar_lst[m].x<<","<<X_bar_lst[m].y<<","<<X_bar_lst[m].theta<<","<<X_bar_lst[m].v<<")    ";
        cout<<"X: ("<<X_cal_lst[m].x<<","<<X_cal_lst[m].y<<","<<X_cal_lst[m].theta<<","<<X_cal_lst[m].v<<")"<<endl;
    }

    cout<<"J_temp_terminal="<<J_temp<<endl;
    ROS_INFO("backward_1");
    // 2. when k=N-1~k=0, i represent timestep
    cout<<"local_horizon="<<local_horizon<<endl;
    for(int i=local_horizon-2; i>=0; i--)
    {
        ROS_INFO("come into backward loop!");
        cout<<"local_i="<<i<<endl;
        //2.1 fulfill X and X_bar matrix
        X(0,0)=X_cal_lst[i].x;
        X(1,0)=X_cal_lst[i].y;
        X(2,0)=X_cal_lst[i].theta;
        X(3,0)=X_cal_lst[i].v;

        X_bar(0,0)=X_bar_lst[i].x;
        X_bar(1,0)=X_bar_lst[i].y;
        X_bar(2,0)=X_bar_lst[i].theta;
        X_bar(3,0)=X_bar_lst[i].v;
        ROS_INFO("backward_2");
        //2.2 obstacles avoidance constraints, j represent different obstacle
        if(obstacles_info.size()!=0)
        {
            cout<<"come into obstacle loop"<<endl;
            for(int j=0; j<obstacles_info.size(); j++)
            {
                if(pow(obstacles_info[j].predicted_states[0].x-ego_state.x,2)+pow(obstacles_info[j].predicted_states[0].y-ego_state.y,2)<max_percep_dist*max_percep_dist)
                {
                    continue;
                }
                ellipse_a=0.5*obstacles_info[j].size.length+obstacles_info[j].predicted_states[i].v*safe_time*cos(obstacles_info[j].predicted_states[i].theta)+safe_a+ego_radius;
                ellipse_b=0.5*obstacles_info[j].size.length+obstacles_info[j].predicted_states[i].v*safe_time*sin(obstacles_info[j].predicted_states[i].theta)+safe_b+ego_radius;
                P(0,0)=1/(ellipse_a*ellipse_a);
                P(1,1)=1/(ellipse_b*ellipse_b);

                X_front(0,0)=X_cal_lst[i].x+ego_lf*cos(X_cal_lst[i].theta);
                X_front(1,0)=X_cal_lst[i].y+ego_lf*sin(X_cal_lst[i].theta);
                X_front(2,0)=X_cal_lst[i].v;
                X_front(3,0)=X_cal_lst[i].theta;

                X_rear(0,0)=X_cal_lst[i].x-ego_lr*cos(X_cal_lst[i].theta);
                X_rear(1,0)=X_cal_lst[i].y-ego_lr*sin(X_cal_lst[i].theta);
                X_rear(2,0)=X_cal_lst[i].v;
                X_rear(3,0)=X_cal_lst[i].theta;

                X_obs(0,0)=obstacles_info[j].predicted_states[i].x;
                X_obs(1,0)=obstacles_info[j].predicted_states[i].y;
                X_obs(2,0)=obstacles_info[j].predicted_states[i].v;
                X_obs(3,0)=obstacles_info[j].predicted_states[i].theta;
                T(0,0)=cos(X_obs(3,0));
                T(0,1)=sin(X_obs(3,0));
                T(1,0)=-sin(X_obs(3,0));
                T(1,1)=cos(X_obs(3,0));
                T=(T.array().abs()<EPS).select(0, T);
                X_front_obs=T*(X_front-X_obs);
                X_front_obs=(X_front_obs.array().abs()<EPS).select(0, X_front_obs);
                cout<<"T="<<T(0,0)<<","<<T(0,1)<<","<<T(0,2)<<","<<T(0,3)<<endl<<T(1,0)<<","<<T(1,1)<<","<<T(1,2)<<","<<T(1,3)<<endl;
                cout<<T(2,0)<<","<<T(2,1)<<","<<T(2,2)<<","<<T(2,3)<<endl<<T(3,0)<<","<<T(3,1)<<","<<T(3,2)<<","<<T(3,3)<<endl;
                cout<<"X_front="<<X_front(0,0)<<","<<X_front(1,0)<<","<<X_front(2,0)<<","<<X_front(3,0)<<endl;
                cout<<"X_obs="<<X_obs(0,0)<<","<<X_obs(1,0)<<","<<X_obs(2,0)<<","<<X_obs(3,0)<<endl;
                X_rear_obs=T*(X_rear-X_obs);
                X_rear_obs=(X_rear_obs.array().abs()<EPS).select(0, X_rear_obs);
                dX_front(0,0)=1;
                dX_front(1,1)=1;
                dX_front(2,2)=1;
                dX_front(3,3)=1;
                dX_front(0,3)=-ego_lf*sin(X_cal_lst[i].theta);
                dX_front(1,3)=ego_lf*sin(X_cal_lst[i].theta);
                dX_front=(dX_front.array().abs()<EPS).select(0, dX_front);
                dX_rear(0,0)=1;
                dX_rear(1,1)=1;
                dX_rear(2,2)=1;
                dX_rear(3,3)=1;
                dX_rear(0,3)=-ego_lr*sin(X_cal_lst[i].theta);
                dX_rear(1,3)=ego_lr*sin(X_cal_lst[i].theta);
                dX_rear=(dX_rear.array().abs()<EPS).select(0, dX_rear);

                dCf=-2*X_front_obs.transpose()*P*T*dX_front;
                dCf=(dCf.array().abs()<EPS).select(0, dCf);
                dCr=-2*X_rear_obs.transpose()*P*T*dX_rear;
                dCr=(dCr.array().abs()<EPS).select(0, dCr);
                f_cf=dCf*X;
                f_cf=(f_cf.array().abs()<EPS).select(0, f_cf);
                f_cr=dCr*X;
                f_cr=(f_cr.array().abs()<EPS).select(0, f_cr);
                //calculate dBf, ddBf, cost_front, dBr, ddBr, cost_rear
                //(1)front:
                M_scalar=X_front_obs.transpose()*P*X_front_obs;
                M_scalar=(M_scalar.array().abs()<EPS).select(0, M_scalar);
                dBf=q1_front*q2_front*exp(q2_front*(1-M_scalar(0,0)))*dCf;
                dBf=(dBf.array().abs()<EPS).select(0, dBf);
                cout<<"obs dBf="<<dBf(0,0)<<","<<dBf(0,1)<<","<<dBf(0,2)<<","<<dBf(0,3)<<endl;
                ddBf=q1_front*q2_front*q2_front*exp(q2_front*(1-M_scalar(0,0)))*dCf.transpose()*dCf;
                ddBf=(ddBf.array().abs()<EPS).select(0, ddBf);
                cout<<"obs ddBf="<<ddBf(0,0)<<","<<ddBf(0,1)<<","<<ddBf(0,2)<<","<<ddBf(0,3)<<endl;
                cout<<ddBf(1,0)<<ddBf(1,1)<<","<<ddBf(1,2)<<","<<ddBf(1,3)<<endl;
                cout<<ddBf(2,0)<<","<<ddBf(2,1)<<","<<ddBf(2,2)<<","<<ddBf(2,3)<<endl;
                cout<<ddBf(3,0)<<","<<ddBf(3,1)<<","<<ddBf(3,2)<<","<<ddBf(3,3)<<endl;
                cost_single=q1_front*exp(q2_front*(1-M_scalar(0,0)));
                cost_single=fabs(cost_single)<EPS?0:cost_single;
                costJ_temp+=cost_single;
                cout<<"X_front_obs=["<<X_front_obs(0,0)<<","<<X_front_obs(1,0)<<X_front_obs(2,0)<<X_front_obs(3,0)<<"]"<<endl;
                cout<<"orgin_front_obs_cost="<<cost_single<<endl;

                //(2)rear:
                M_scalar=X_rear_obs.transpose()*P*X_rear_obs;
                M_scalar =(M_scalar.array().abs()<EPS).select(0, M_scalar);
                dBr=q1_rear*q2_rear*exp(q2_rear*(1-M_scalar(0,0)))*dCr;
                dBr=(dBr.array().abs()<EPS).select(0, dBr);
                cout<<"obs dBr="<<dBr(0,0)<<","<<dBr(0,1)<<","<<dBr(0,2)<<","<<dBr(0,3)<<endl;
                ddBr=q1_rear*q2_rear*q2_rear*exp(q2_rear*(1-M_scalar(0,0)))*dCr.transpose()*dCr;
                ddBr=(ddBr.array().abs()<EPS).select(0, ddBr);
                cout<<"obs ddBr="<<ddBr(0,0)<<","<<ddBr(0,1)<<","<<ddBr(0,2)<<","<<ddBr(0,3)<<endl;
                cout<<ddBr(1,0)<<","<<ddBr(1,1)<<","<<ddBr(1,2)<<","<<ddBr(1,3)<<endl;
                cout<<ddBr(2,0)<<","<<ddBr(2,1)<<","<<ddBr(2,2)<<","<<ddBr(2,3)<<endl;
                cout<<ddBr(3,0)<<","<<ddBr(3,1)<<","<<ddBr(3,2)<<","<<ddBr(3,3)<<endl;
                cost_single=q1_rear*exp(q2_rear*(1-M_scalar(0,0)));
                cost_single=fabs(cost_single)<EPS?0:cost_single;
                costJ_temp+=cost_single;
                cout<<"X_rear_obs=["<<X_rear_obs(0,0)<<","<<X_rear_obs(1,0)<<X_rear_obs(2,0)<<X_rear_obs(3,0)<<"]"<<endl;
                cout<<"orgin_rear_obs_cost="<<cost_single<<endl;
                
                //calculate dX and ddX
                dX=dX+dBf+dBr;
                dX=(dX.array().abs()<EPS).select(0, dX);
                ddX=ddX+ddBf+ddBr;
                ddX=(ddX.array().abs()<EPS).select(0, ddX);
                // cout<<"f_cf="<<f_cf(0,0)<<", f_cr="<<f_cr(0,0)<<endl;
                // cout<<"f_cf_cost="<<q1_front*exp(q2_front*f_cf(0,0))<<endl;
                // cout<<"f_cr_cost="<<q1_rear*exp(q2_rear*f_cr(0,0))<<endl;          
            }
        }
        ROS_INFO("backward_3");
        // 2.3 distance cost
        dX=dX+(X_bar-X).transpose()*Q;
        dX=(dX.array().abs()<EPS).select(0, dX);
        cout<<"distance dX total="<<dX(0,0)<<","<<dX(0,1)<<","<<dX(0,2)<<","<<dX(0,3)<<endl;
        ddX=ddX+Q;
        ddX=(ddX.array().abs()<EPS).select(0, ddX);
        cout<<"distance ddX total="<<ddX(0,0)<<","<<ddX(0,1)<<","<<ddX(0,2)<<","<<ddX(0,3)<<endl;
        cout<<ddX(1,0)<<","<<ddX(1,1)<<","<<ddX(1,2)<<","<<ddX(1,3)<<endl;
        cout<<ddX(2,0)<<","<<ddX(2,1)<<","<<ddX(2,2)<<","<<ddX(2,3)<<endl;
        cout<<ddX(3,0)<<","<<ddX(3,1)<<","<<ddX(3,2)<<","<<ddX(3,3)<<endl;
        M_scalar=0.5*(X_bar-X).transpose()*Q*(X_bar-X);
        M_scalar=(M_scalar.array().abs()<EPS).select(0, M_scalar);
        cout<<"process cost="<<M_scalar(0,0)<<endl;
        costJ_temp+=M_scalar(0,0);
        J_temp+=M_scalar(0,0);
        cout<<"J_temp_process="<<M_scalar(0,0)<<endl;

        // 2.4 control cost vector<vector<double>>K_lst;
        U(0,0)=initial_controls[i].accel+delta_controls[i].accel;
        U(1,0)=initial_controls[i].yaw_rate+delta_controls[i].yaw_rate;
        U(0,0)=U(0,0)>a_high?a_high:U(0,0);
        U(0,0)=U(0,0)<a_low?a_low:U(0,0);
        U(1,0)=U(1,0)>X_cal_lst[i].v*tan(steer_high)/egoL?X_cal_lst[i].v*tan(steer_high)/egoL:U(1,0);
        U(1,0)=U(1,0)<X_cal_lst[i].v*tan(steer_low)/egoL?X_cal_lst[i].v*tan(steer_low)/egoL:U(1,0);
        U=(U.array().abs()<EPS).select(0, U);
        cout<<"U=["<<U(0,0)<<","<<U(1,0)<<"]"<<endl;
        dU=dU+U.transpose()*R;
        dU=(dU.array().abs()<EPS).select(0, dU);
        ddU=ddU+R;
        ddU=(ddU.array().abs()<EPS).select(0, ddU);
        cout<<"control cost dU="<<dU(0,0)<<","<<dU(0,1)<<endl;
        cout<<"control cost ddU="<<ddU(0,0)<<","<<ddU(0,1)<<endl;
        cout<<ddU(1,0)<<","<<ddU(1,1)<<endl;
        M_scalar=0.5*U.transpose()*R*U;
        M_scalar=(M_scalar.array().abs()<EPS).select(0, M_scalar);
        cout<<"control cost="<<M_scalar(0,0)<<endl;
        costJ_temp+=M_scalar(0,0);
        J_temp+=M_scalar(0,0);
        cout<<"J_temp_control="<<M_scalar(0,0)<<endl;

        // 2.5 control constraint cost
        dCu<<1,0;
        dBu=q1_acc*q2_acc*exp(q2_acc*(U(0,0)-a_high))*dCu;
        dBu=(dBu.array().abs()<EPS).select(0, dBu);
        cout<<"dBu="<<dBu(0,0)<<","<<dBu(0,1)<<endl;
        dU=dU+dBu;
        dU=(dU.array().abs()<EPS).select(0, dU);
        ddBu=q1_acc*q2_acc*q2_acc*exp(q2_acc*(U(0,0)-a_high))*dCu.transpose()*dCu;
        ddBu=(ddBu.array().abs()<EPS).select(0, ddBu);
        cout<<"ddBu="<<ddBu(0,0)<<","<<ddBu(0,1)<<endl;
        cout<<ddBu(1,0)<<","<<ddBu(1,1)<<endl;
        ddU=ddU+ddBu;
        ddU=(ddU.array().abs()<EPS).select(0, ddU);
        cost_single=q1_acc*exp(q2_acc*(U(0,0)-a_high));
        cost_single=fabs(cost_single)<EPS?0:cost_single;
        costJ_temp+=cost_single;
        cout<<"control_constraint_cost1="<<cost_single<<endl;

        dCu<<-1,0;
        dBu=q1_acc*q2_acc*exp(q2_acc*(a_low-U(0,0)))*dCu;
        dBu=(dBu.array().abs()<EPS).select(0, dBu);
        cout<<"dBu="<<dBu(0,0)<<","<<dBu(0,1)<<endl;
        dU=dU+dBu;
        dU=(dU.array().abs()<EPS).select(0, dU);
        ddBu=q1_acc*q2_acc*q2_acc*exp(q2_acc*(a_low-U(0,0)))*dCu.transpose()*dCu;
        ddBu=(ddBu.array().abs()<EPS).select(0, ddBu);
        cout<<"ddBu="<<ddBu(0,0)<<","<<ddBu(0,1)<<endl;
        cout<<ddBu(1,0)<<","<<ddBu(1,1)<<endl;
        ddU=ddU+ddBu;
        ddU=(ddU.array().abs()<EPS).select(0, ddU);
        cost_single=q1_acc*exp(q2_acc*(a_low-U(0,0)));
        cost_single=fabs(cost_single)<EPS?0:cost_single;
        costJ_temp+=cost_single;
        cout<<"control_constraint_cost2="<<cost_single<<endl;

        dCu<<0,1;
        dBu=q1_yr*q2_yr*exp(q2_yr*(U(1,0)-X_cal_lst[i].v*tan(steer_high)/egoL))*dCu;
        dBu=(dBu.array().abs()<EPS).select(0, dBu);
        cout<<"dBu="<<dBu(0,0)<<","<<dBu(0,1)<<endl;
        dU=dU+dBu;
        dU=(dU.array().abs()<EPS).select(0, dU);
        ddBu=q1_yr*q2_yr*q2_yr*exp(q2_yr*(U(1,0)-X_cal_lst[i].v*tan(steer_high)/egoL))*dCu.transpose()*dCu;
        ddBu=(ddBu.array().abs()<EPS).select(0, ddBu);
        cout<<"ddBu="<<ddBu(0,0)<<","<<ddBu(0,1)<<endl;
        cout<<ddBu(1,0)<<","<<ddBu(1,1)<<endl;
        ddU=ddU+ddBu;
        ddU=(ddU.array().abs()<EPS).select(0, ddU);
        cost_single=q1_yr*exp(q2_yr*(U(1,0)-X_cal_lst[i].v*tan(steer_high)/egoL));
        cost_single=fabs(cost_single)<EPS?0:cost_single;
        costJ_temp+=cost_single;
        cout<<"control_constraint_cost3="<<cost_single<<endl;

        dCu<<0,-1;
        dBu=q1_yr*q2_yr*exp(q2_yr*(X_cal_lst[i].v*tan(steer_low)/egoL-U(1,0)))*dCu;
        dBu=(dBu.array().abs()<EPS).select(0, dBu);
        cout<<"dBu="<<dBu(0,0)<<","<<dBu(0,1)<<endl;
        dU=dU+dBu;
        dU=(dU.array().abs()<EPS).select(0, dU);
        ddBu=q1_yr*q2_yr*q2_yr*exp(q2_yr*(X_cal_lst[i].v*tan(steer_low)/egoL-U(1,0)))*dCu.transpose()*dCu;
        ddBu=(ddBu.array().abs()<EPS).select(0, ddBu);
        cout<<"ddBu="<<ddBu(0,0)<<","<<ddBu(0,1)<<endl;
        cout<<ddBu(1,0)<<","<<ddBu(1,1)<<endl;
        ddU=ddU+ddBu;
        ddU=(ddU.array().abs()<EPS).select(0, ddU);
        cost_single=q1_yr*exp(q2_yr*(X_cal_lst[i].v*tan(steer_low)/egoL-U(1,0)));
        cost_single=fabs(cost_single)<EPS?0:cost_single;
        costJ_temp+=cost_single;
        cout<<"control_constraint_cost4="<<cost_single<<endl;
        ROS_INFO("backward_4");
        // 2.6 calculate Qx, Qu, Qxx, Quu, Qxu, Qux, deltaV
        vd_model.getVehicleModelAandB(X_cal_lst[i].v, X_cal_lst[i].theta, U(0,0), control_dt, A, B);
        Qx=dX+dVk*A;
        Qx=(Qx.array().abs()<EPS).select(0, Qx);
        Qu=dU+dVk*B;
        Qu=(Qu.array().abs()<EPS).select(0, Qu);
        Qxx=ddX+A.transpose()*ddVk*A;
        Qxx=(Qxx.array().abs()<EPS).select(0, Qxx);
        Quu=ddU+B.transpose()*ddVk*B;
        Quu=(Quu.array().abs()<EPS).select(0, Quu);
        Qxu=B.transpose()*ddVk*A;
        Qxu=(Qxu.array().abs()<EPS).select(0, Qxu);
        Qux=Qxu.transpose();
        Regular=lamb*I;
        B_reg=Regular*B;
        Qux_reg=Qux+A*B_reg;
        Qxu_reg=Qux_reg.transpose();
        Quu_reg=Quu+B_reg.transpose()*B_reg;
        cout<<"Qx="<<Qx(0,0)<<","<<Qx(0,1)<<","<<Qx(0,2)<<","<<Qx(0,3)<<endl;
        cout<<"Qu="<<Qu(0,0)<<","<<Qu(0,1)<<endl;
        cout<<"Qxx="<<Qxx(0,0)<<","<<Qxx(0,1)<<","<<Qxx(0,2)<<","<<Qxx(0,3)<<endl;
        cout<<Qxx(1,0)<<","<<Qxx(1,1)<<","<<Qxx(1,2)<<","<<Qxx(1,3)<<endl;
        cout<<Qxx(2,0)<<","<<Qxx(2,1)<<","<<Qxx(2,2)<<","<<Qxx(2,3)<<endl;
        cout<<Qxx(3,0)<<","<<Qxx(3,1)<<","<<Qxx(3,2)<<","<<Qxx(3,3)<<endl;
        cout<<"Quu="<<Quu(0,0)<<","<<Quu(0,1)<<","<<Quu(1,0)<<","<<Quu(1,1)<<endl;
        cout<<"Qxu="<<Qxu(0,0)<<","<<Qxu(0,1)<<","<<Qxu(0,2)<<","<<Qxu(0,3)<<endl;
        cout<<Qxu(1,0)<<","<<Qxu(1,1)<<","<<Qxu(1,2)<<","<<Qxu(1,3)<<endl;
        if(isCompleteCal)
        {
            K=-(Quu_reg.inverse()).transpose()*Qxu_reg;
            K=(K.array().abs()<EPS).select(0, K);
            cout<<"back K="<<K(0,0)<<","<<K(0,1)<<","<<K(0,2)<<","<<K(0,3)<<endl<<K(1,0)<<","<<K(1,1)<<","<<K(1,2)<<","<<K(1,3)<<endl;
            d=-(Quu_reg.inverse()).transpose()*Qu.transpose();
            d=(d.array().abs()<EPS).select(0, d);
            cout<<"back d="<<d(0,0)<<","<<d(1,0)<<endl;
            K_lst.push_back({K(0,0), K(0,1), K(0,2), K(0,3), K(1,0), K(1,1), K(1,2), K(1,3)});
            d_lst.push_back({d(0,0), d(1,0)});
            M_scalar=Qu*d;
            M_scalar=(M_scalar.array().abs()<EPS).select(0, M_scalar);
            deltaV1d+=M_scalar(0,0);
            M_scalar=0.5*d.transpose()*Quu*d;
            M_scalar=(M_scalar.array().abs()<EPS).select(0, M_scalar);
            deltaV2d=deltaV2d+M_scalar(0,0);
        }
        // 2.7 calculate dVk, ddVk, for next loop
        dVk=Qx+Qu*K+d.transpose()*Quu*K+d.transpose()*Qxu;
        dVk=(dVk.array().abs()<EPS).select(0, dVk);
        ddVk=Qxx+K.transpose()*Quu*K+Qux*K+K.transpose()*Qxu;
        ddVk=(ddVk.array().abs()<EPS).select(0, ddVk);
    }
    ROS_INFO("backward_5");
    return costJ_temp;
}

void ciLQR::ForwardPass()
{
    forward_counter=0;
    cout<<"forward start..."<<endl;
    while(forward_counter<max_forward_iterate)
    {
        // initialize X0 and X_nominal0
        X(0,0)=X_vd_lst[0].x;
        X(1,0)=X_vd_lst[0].y;
        X(2,0)=X_vd_lst[0].v;
        X(3,0)=X_vd_lst[0].theta;
        X_nominal(0,0)=X_vd_lst[0].x;
        X_nominal(1,0)=X_vd_lst[0].y;
        X_nominal(2,0)=X_vd_lst[0].v;
        X_nominal(3,0)=X_vd_lst[0].theta;
        
        //xk as the first point of X_nominal_lst
        xk.x=X_vd_lst[0].x;
        xk.y=X_vd_lst[0].y;
        xk.v=X_vd_lst[0].v;
        xk.theta=X_vd_lst[0].theta;
        xk.accel=0;
        xk.yaw_rate=0;
        X_nominal_lst.clear();
        X_nominal_lst.push_back(xk);
        delta_controls.clear();
        planned_controls.clear();
        //cout<<"planner_here2.5"<<endl;
        for(int i=0; i<local_horizon-1; i++)
        {
            deltaX=(X_nominal-X);
            K(0,0)=K_lst[K_lst.size()-1-i][0];
            K(0,1)=K_lst[K_lst.size()-1-i][1];
            K(0,2)=K_lst[K_lst.size()-1-i][2];
            K(0,3)=K_lst[K_lst.size()-1-i][3];
            K(1,0)=K_lst[K_lst.size()-1-i][4];
            K(1,1)=K_lst[K_lst.size()-1-i][5];
            K(1,2)=K_lst[K_lst.size()-1-i][6];
            K(1,3)=K_lst[K_lst.size()-1-i][7];
            d(0,0)=d_lst[d_lst.size()-1-i][0];
            d(1,0)=d_lst[d_lst.size()-1-i][1];
            deltaU_star=K*deltaX+alfa*d;
            deltaU_star=(deltaU_star.array().abs()<EPS).select(0, deltaU_star);
            delta_control_signal.accel=deltaU_star(0,0);
            delta_control_signal.yaw_rate=deltaU_star(1,0);
            delta_controls.push_back(delta_control_signal);
            control_signal.accel=initial_controls[i].accel+U(0,0);
            control_signal.accel=control_signal.accel>a_high?a_high:control_signal.accel;
            control_signal.accel=control_signal.accel<a_low?a_low:control_signal.accel;
            control_signal.yaw_rate=initial_controls[i].yaw_rate+U(1,0);
            control_signal.yaw_rate=control_signal.yaw_rate>X_nominal_lst[i].v*tan(steer_high)/egoL?X_nominal_lst[i].v*tan(steer_high)/egoL:control_signal.yaw_rate;
            planned_controls.push_back(control_signal);
            cout<<"X_nominal=["<<X_nominal(0,0)<<","<<X_nominal(1,0)<<","<<X_nominal(2,0)<<","<<X_nominal(3,0)<<"]"<<endl;
            cout<<"X=["<<X(0,0)<<","<<X(1,0)<<","<<X(2,0)<<","<<X(3,0)<<"]"<<endl;
            cout<<"deltaX=["<<deltaX(0,0)<<","<<deltaX(1,0)<<","<<deltaX(2,0)<<","<<deltaX(3,0)<<"]"<<endl;
            cout<<"K_matrix:"<<K(0,0)<<","<<K(0,1)<<","<<K(0,2)<<","<<K(0,3)<<endl;
            cout<<K(1,0)<<","<<K(1,1)<<","<<K(1,2)<<","<<K(1,3)<<endl;
            cout<<"d_matrix:"<<d(0,0)<<","<<d(1,0)<<endl;
            cout<<"deltaU_star=["<<deltaU_star(0,0)<<","<<deltaU_star(1,0)<<"]"<<endl;
            cout<<"--------------------------------------"<<endl;
        
            vd_model.updateOneStep(xk, xk1, control_signal, dt);
            X_nominal_lst.push_back(xk1);
            
            // fulfill (i+1) frame variables
            X_nominal(0,0)=xk1.x;
            X_nominal(1,0)=xk1.y;
            X_nominal(2,0)=xk1.v;
            X_nominal(3,0)=xk1.theta;
            X(0,0)=X_vd_lst[i+1].x;
            X(1,0)=X_vd_lst[i+1].y;
            X(2,0)=X_vd_lst[i+1].v;
            X(3,0)=X_vd_lst[i+1].theta;
            xk.x=xk1.x;
            xk.y=xk1.y;
            xk.v=xk1.v;
            xk.theta=xk1.theta;
            xk.accel=xk1.accel;
            xk.yaw_rate=xk1.yaw_rate;
        }
        getLocalReferPoints(local_waypoints_dense, X_nominal_lst, X_bar_lst);

        cout<<"delta_controls:"<<endl;
        for(int i=0; i<delta_controls.size(); i++)
        {
            cout<<delta_controls[i].accel<<","<<delta_controls[i].yaw_rate<<endl;
        }
        //cout<<"planner_here2.6"<<endl;
        //calculate nominal trajectory cost
        costJ_nominal=BackwardPassAndGetCostJ(X_nominal_lst, lamb, false);
        deltaV=alfa*deltaV1d+alfa*alfa*deltaV2d;
        z=(costJ_nominal-costJ)/deltaV;
        cout<<"costJ_nominal="<<costJ_nominal<<endl;
        cout<<"costJ="<<costJ<<endl;
        cout<<"deltaV1d="<<deltaV1d<<endl;
        cout<<"deltaV2d="<<deltaV2d<<endl;
        cout<<"deltaV="<<deltaV<<endl;
        cout<<"alfa="<<alfa<<endl;
        cout<<"z="<<z<<endl;
        if(costJ_nominal<costJ||fabs(costJ_nominal-costJ)<1e-8)//(costJ_nominal<=costJ)
        {
            forward_iter_flag=true;
            break;
            //X_nominal, planned_controls, costJ_nominal are the optimal results
        }
        else
        {
            alfa=gama*alfa;
        }
        forward_counter++;
    }
    cout<<"forward loop: "<<forward_counter<<endl;
}


void ciLQR::iLQRSolver()
{
    //initialize controls list
    control_signal.accel=0.5;
    control_signal.yaw_rate=0.0;
    delta_control_signal.accel=0;
    delta_control_signal.yaw_rate=0;
    initial_controls.clear();
    for(int i=0;  i<local_horizon; i++)
    {
        initial_controls.push_back(control_signal);
        delta_controls.push_back(delta_control_signal);
    }

    // 1. find the closest index in global waypoints and intercept local waypoints
    if(isFirstFrame)
    {
        findClosestWaypointIndex(global_waypoints, ego_state, closest_global_index, true);
        start_state.x=ego_state.x;
        start_state.y=ego_state.y;
        start_state.theta=ego_state.theta;
        start_state.v=ego_state.v;
        start_state.accel=ego_state.accel;
        start_state.yaw_rate=ego_state.yaw_rate;
        isFirstFrame=false;
    }
    else
    {
        ROS_INFO("come to the 2nd frame!");
        if(pow(planned_path[planning_start_index].x-ego_state.x,2)+pow(planned_path[planning_start_index].y-ego_state.y,2)<start_dist*start_dist)
        {
            findClosestWaypointIndex(global_waypoints, planned_path[planning_start_index], closest_global_index, false);
            start_state.x=planned_path[planning_start_index].x;
            start_state.y=planned_path[planning_start_index].y;
            start_state.theta=planned_path[planning_start_index].theta;
            start_state.v=planned_path[planning_start_index].v;
            start_state.accel=planned_path[planning_start_index].accel;
            start_state.yaw_rate=planned_path[planning_start_index].yaw_rate;
            
        }
        else
        {
            findClosestWaypointIndex(global_waypoints, ego_state, closest_global_index, true);
            start_state.x=ego_state.x;
            start_state.y=ego_state.y;
            start_state.theta=ego_state.theta;
            start_state.v=ego_state.v;
            start_state.accel=ego_state.accel;
            start_state.yaw_rate=ego_state.yaw_rate;
        }
    }
    ROS_INFO("planner_here2.1");
    local_waypoints.clear();
    local_end_index=min<int>(global_waypoints.size(), closest_global_index+global_horizon);
    for(int i=closest_global_index; i<local_end_index; i++)
    {
        waypoint.x=global_waypoints[i].x;
        waypoint.y=global_waypoints[i].y;
        waypoint.theta=global_waypoints[i].theta;
        waypoint.v=global_waypoints[i].v;
        waypoint.accel=global_waypoints[i].accel;
        waypoint.yaw_rate=global_waypoints[i].yaw_rate;
        local_waypoints.push_back(waypoint);
    }
    ROS_INFO("planner_here2.2");
    //2. polynominal fitting of local waypoints, make local waypoints dense, and calculate x_bar_lst
    polynominalFitting();
    local_waypoints_dense.clear();
    interplot_increment=(local_waypoints[local_waypoints.size()-1].x-local_waypoints[0].x)/(local_horizon-1);
    cout<<"local_waypoints_dense:"<<endl;
    for(int i=0; i<local_horizon; i++)
    {
        waypoint.x=local_waypoints[0].x+i*interplot_increment;
        waypoint.y=0;
        for(int j=0; j<=poly_order; j++)
        {
            waypoint.y+=polyCoeff[j]*pow(waypoint.x, j);
        }
        local_waypoints_dense.push_back(waypoint);
    }

    X_vd_lst.clear();
    vd_model.CalVDTrajectory(start_state, initial_controls, X_vd_lst, local_horizon, control_dt);

    getLocalReferPoints(local_waypoints_dense, X_vd_lst, X_bar_lst);
    ROS_INFO("planner_here2.3");
    lamb=lamb_init;
    optimization_counter=0;
    while(optimization_counter<max_optimal_iterate)
    {
        //3. backward pass and get the costJ_nominal
        alfa=1.0;
        costJ=BackwardPassAndGetCostJ(X_vd_lst, lamb, true);
        ROS_INFO("planner_here2.4");
        //4. forward pass and get the final cost
        forward_iter_flag=false;
        ForwardPass();
        if(forward_iter_flag)
        {
            X_vd_lst.swap(X_nominal_lst);
            initial_controls.swap(planned_controls);
            costJ_cache=costJ;
            costJ=costJ_nominal;
            costJ_nominal=costJ_cache;
            cout<<"swap!"<<endl;
            if(fabs(costJ-costJ_nominal)<optimal_tol)
            {
                cout<<"arrive optimal tol!"<<endl;
                break;
            }
            else
                lamb*=lamb_decay;
        }
        else
        {
            cout<<"linear search failed!"<<endl;
            lamb*=lamb_ambify;
            if(lamb>lamb_max)
            {
                cout<<"lamb out of range!"<<endl;
                break;
            }
        }
        optimization_counter++;
        cout<<"optimal_counter="<<optimization_counter<<endl;
    }
    cout<<"optimal loop"<<optimization_counter<<endl;

    ROS_INFO("planner_here2.7");
}

void ciLQR::update()
{
    
    cout<<"come into cilqr update!"<<endl;
    int planning_rate=1/dt;
    cout<<"planning_rate="<<planning_rate<<endl;
    ros::Rate planner_rate(planning_rate);

    saturn_msgs::Control control_msg;
    saturn_msgs::ControlArray control_lst_msg;
    
    geometry_msgs::PoseStamped waypoint_msg;
    nav_msgs::Path planned_path_msg;

    geometry_msgs::Point point_msg;
    visualization_msgs::Marker local_points_msg;
    visualization_msgs::Marker local_lines_msg;
    
    cout<<"planner_here1"<<endl;
    while(ros::ok())
    {
        cout<<"come into main loop";
        ros::spinOnce();
        ROS_INFO("planner recv finished!");
        if(!isRecEgoVeh)
        {
            ROS_INFO("ego vehicle state is not received!");
            planner_rate.sleep();
            continue;
        }
        isRecEgoVeh=false;
        ROS_INFO("planner_here2");
        iLQRSolver();
        ROS_INFO("planner_here3");
        //fulfill related message, publish planned_path and planned_controls
        //1. publish controls
        control_lst_msg.header.frame_id="cilqr_planner";
        control_lst_msg.header.stamp=ros::Time::now();
        control_lst_msg.control_lst.clear();
        cout<<"planned_controls:"<<endl;
        for(int i=0; i<planned_controls.size();i++)
        {
            control_msg.u_accel=planned_controls[i].accel;
            control_msg.u_yawrate=planned_controls[i].yaw_rate;
            control_lst_msg.control_lst.push_back(control_msg);
            cout<<planned_controls[i].accel<<","<<planned_controls[i].yaw_rate<<endl;
        }
        cilqr_control_pub.publish(control_lst_msg);
        ROS_INFO("planner_here4");
        //2. planned path for rviz and store
        planned_path_msg.header.frame_id="map";
        planned_path_msg.header.stamp=ros::Time::now();
        planned_path_msg.poses.clear();
        planned_path.clear();
        cout<<"planned_path:"<<endl;
        for(int i=0; i<X_nominal_lst.size(); i++)
        {
            waypoint_msg.pose.position.x=X_nominal_lst[i].x;
            waypoint_msg.pose.position.y=X_nominal_lst[i].y;
            waypoint_msg.pose.position.z=egoHeight/2;
            waypoint_msg.pose.orientation.x=0;
            waypoint_msg.pose.orientation.y=0;
            waypoint_msg.pose.orientation.z=sin(X_nominal_lst[i].theta/2);
            waypoint_msg.pose.orientation.w=cos(X_nominal_lst[i].theta/2);

            planned_path_msg.poses.push_back(waypoint_msg);

            planned_path.push_back(X_nominal_lst[i]);
            cout<<X_nominal_lst[i].x<<","<<X_nominal_lst[i].y<<","<<X_nominal_lst[i].theta<<","<<X_nominal_lst[i].v<<endl;
        }
        cout<<"planned_path fulfill size:"<<planned_path.size()<<endl;
        rviz_local_planned_path_pub.publish(planned_path_msg);
        ROS_INFO("planner_here5");
        //3. local reference waypoint for rviz
        local_points_msg.header.frame_id="map";
        local_points_msg.header.stamp=ros::Time::now();
        local_lines_msg.header.frame_id="map";
        local_lines_msg.header.stamp=ros::Time::now();
        local_points_msg.action=visualization_msgs::Marker::ADD;
        local_lines_msg.action=visualization_msgs::Marker::ADD;
        local_points_msg.ns="local_points_and_lines";
        local_lines_msg.ns="local_points_and_lines";
        local_points_msg.id=1;
        local_lines_msg.id=2;
        local_points_msg.type=visualization_msgs::Marker::POINTS;
        local_lines_msg.type=visualization_msgs::Marker::LINE_STRIP;
        local_points_msg.pose.orientation.w=1.0;
        local_lines_msg.pose.orientation.w=1.0;
        //set scale and color
        local_points_msg.scale.x=0.2;
        local_points_msg.scale.y=0.2;
        local_lines_msg.scale.x=0.1;
        local_lines_msg.scale.y=0.1;
        local_points_msg.color.g=1.0;
        local_points_msg.color.a=1.0;
        local_lines_msg.color.b=1.0;
        local_lines_msg.color.a=1.0;
        local_points_msg.points.clear();
        local_lines_msg.points.clear();
        //fuifill points and lines
        for(int i=0; i<X_bar_lst.size(); i++)
        {
            point_msg.x=X_bar_lst[i].x;
            point_msg.y=X_bar_lst[i].y;
            point_msg.z=0;
            local_points_msg.points.push_back(point_msg);
            local_lines_msg.points.push_back(point_msg);
        }
        rviz_local_refer_points_pub.publish(local_points_msg);
        rviz_local_refer_lines_pub.publish(local_lines_msg);
        ROS_INFO("planner_here6");
        planner_rate.sleep();
    }
}