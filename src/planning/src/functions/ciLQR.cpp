#include "ciLQR.h"

using namespace std;

ciLQR::ciLQR(ros::NodeHandle &nh_): nh(nh_)
{
    ego_state_sub=nh.subscribe("/ego_vehicle/state", 10, &ciLQR::recvEgoState, this);
    obstacles_state_sub=nh.subscribe("/obstacles/state", 10, &ciLQR::recvObstaclesState, this);
    cilqr_control_pub=nh.advertise<saturn_msgs::ControlArray>("/cilqr_planner/control", 10, true);
    rviz_local_refer_points_pub=nh.advertise<visualization_msgs::Marker>("/local_referline/points", 1, true);
    rviz_local_refer_lines_pub=nh.advertise<visualization_msgs::Marker>("/local_referline/lines", 1, true);
    rviz_local_planned_path_pub=nh.advertise<nav_msgs::Path>("/local_planned_path", 10, true);
    
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
    closest_global_index=0;
    closest_local_index=0;
    isFirstFrame=true;
    planning_start_index=dt/control_dt;
    max_dist=numeric_limits<double>::max();
    dist=0.0;
    alfa=1.0;
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
    update();
}

ciLQR::~ciLQR()
{
    ROS_INFO("ciLQR destructed!");
}

void ciLQR::recvEgoState(const saturn_msgs::State &msg)
{
    ego_state.x=msg.x;
    ego_state.y=msg.y;
    ego_state.theta=msg.theta;
    ego_state.v=msg.v;
    ego_state.accel=msg.accel;
    ego_state.yaw_rate=msg.yawrate;
}

void ciLQR::recvObstaclesState(const saturn_msgs::ObstacleStateArray &msg)
{
    obstacles_info.clear();
    for(int i=0; i<msg.obstacles.size();i++)
    {
        obstacles_info[i].id=msg.obstacles[i].id;
        obstacles_info[i].name=msg.obstacles[i].name;
        obstacles_info[i].predicted_states.clear();
        for(int j=0; j<msg.obstacles[i].predicted_states.size(); j++)
        {
            obs_state_single.x=msg.obstacles[i].predicted_states[j].x;
            obs_state_single.y=msg.obstacles[i].predicted_states[j].y;
            obs_state_single.theta=msg.obstacles[i].predicted_states[j].theta;
            obs_state_single.v=msg.obstacles[i].predicted_states[j].v;
            obs_state_single.accel=msg.obstacles[i].predicted_states[j].accel;
            obs_state_single.yaw_rate=msg.obstacles[i].predicted_states[j].yawrate;
            obstacles_info[i].predicted_states.push_back(obs_state_single);
        }
        obstacles_info[i].size.length=msg.obstacles[i].size.length;
        obstacles_info[i].size.width=msg.obstacles[i].size.width;
        obstacles_info[i].size.height=msg.obstacles[i].size.height;
        obstacles_info[i].size.wheel_base=msg.obstacles[i].size.wheel_base;
        obstacles_info[i].size.wheel_track=msg.obstacles[i].size.wheel_track;
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
                    case 4:
                        waypoint.x=stod(itemw);
                        break;
                    case 5:
                        waypoint.y=stod(itemw);
                        break;
                    case 6:
                        waypoint.theta=stod(itemw);
                        break;
                    case 7:
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
    poly_a=polyX.transpose()*polyX.ldlt().solve(polyX.transpose()*polyY);
    for(int i=0;i<=poly_order;i++)
    {
        //order is a0, a1, a2,...an
        polyCoeff.push_back(poly_a(i,0));
    }
}

double ciLQR::BackwardPassAndGetCostJ(const vector<ObjState>&X_cal_lst, bool isCompleteCal)
{
    //X_cal_lst is X_vd_lst, order: 0~N
    double costJ_temp=0;
    cout<<"X_cal_lst length="<<X_cal_lst.size()<<";"<<endl;
    cout<<"X_bar_lst length="<<X_bar_lst.size()<<";"<<endl;

    //K, d list order: N-1~0
    if(isCompleteCal)
    {
        K_lst.clear();
        d_lst.clear();

        deltaV1d=0;
        deltaV2d=0;
    }
    
    // 1. whe k=N, calc relevant variables and ternimal costJ
    X(0,0)=X_cal_lst[X_cal_lst.size()-1].x;
    X(1,0)=X_cal_lst[X_cal_lst.size()-1].y;
    X(2,0)=X_cal_lst[X_cal_lst.size()-1].theta;
    X(3,0)=X_cal_lst[X_cal_lst.size()-1].v;

    X_bar(0,0)=X_bar_lst[X_bar_lst.size()-1].x;
    X_bar(1,0)=X_bar_lst[X_bar_lst.size()-1].y;
    X_bar(2,0)=X_bar_lst[X_bar_lst.size()-1].theta;
    X_bar(3,0)=X_bar_lst[X_bar_lst.size()-1].v;

    dVk=(X_bar-X).transpose()*Q;
    ddVk=Q;
    M_scalar=0.5*(X_bar-X).transpose()*Q*(X_bar-X);
    costJ_temp+=M_scalar(0,0);

    // 2. when k=N-1~k=0, i represent timestep
    for(int i=local_horizon-2; i>=0; i--)
    {
        //2.1 fulfill X and X_bar matrix
        X(0,0)=X_cal_lst[i].x;
        X(1,0)=X_cal_lst[i].y;
        X(2,0)=X_cal_lst[i].theta;
        X(3,0)=X_cal_lst[i].v;

        X_bar(0,0)=X_bar_lst[i].x;
        X_bar(1,0)=X_bar_lst[i].y;
        X_bar(2,0)=X_bar_lst[i].theta;
        X_bar(3,0)=X_bar_lst[i].v;
        
        //2.2 obstacles avoidance constraints, j represent different obstacle
        if(obstacles_info.size()!=0)
        {
            for(int j=0; j<obstacles_info.size(); j++)
            {
                ellipse_a=obstacles_info[j].size.length+obstacles_info[j].predicted_states[i].v*safe_time*cos(obstacles_info[j].predicted_states[i].theta)+safe_a+ego_radius;
                ellipse_b=obstacles_info[j].size.length+obstacles_info[j].predicted_states[i].v*safe_time*sin(obstacles_info[j].predicted_states[i].theta)+safe_b+ego_radius;
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

                T(0,0)=cos(obstacles_info[j].predicted_states[i].theta);
                T(0,1)=sin(obstacles_info[j].predicted_states[i].theta);
                T(1,0)=-sin(obstacles_info[j].predicted_states[i].theta);
                T(1,1)=cos(obstacles_info[j].predicted_states[i].theta);

                X_obs(0,0)=obstacles_info[j].predicted_states[i].x;
                X_obs(1,0)=obstacles_info[j].predicted_states[i].y;
                X_obs(2,0)=obstacles_info[j].predicted_states[i].v;
                X_obs(3,0)=obstacles_info[j].predicted_states[i].theta;
                X_front_obs=T*(X_front-X_obs);
                X_rear_obs=T*(X_rear-X_obs);
                dX_front(0,0)=1;
                dX_front(1,1)=1;
                dX_front(2,2)=1;
                dX_front(3,3)=1;
                dX_front(0,3)=-ego_lf*sin(X_cal_lst[i].theta);
                dX_front(1,3)=ego_lf*sin(X_cal_lst[i].theta);
                dX_rear(0,0)=1;
                dX_rear(1,1)=1;
                dX_rear(2,2)=1;
                dX_rear(3,3)=1;
                dX_rear(0,3)=-ego_lr*sin(X_cal_lst[i].theta);
                dX_rear(1,3)=ego_lr*sin(X_cal_lst[i].theta);

                dCf=-2*X_front_obs.transpose()*P*T*dX_front;
                dCr=-2*X_rear_obs.transpose()*P*T*dX_rear;
                f_cf=dCf*X;
                f_cr=dCr*X;
                dBf=q1_front*q2_front*exp(q2_front*f_cf(0,0))*dCf;
                dBr=q1_rear*q2_rear*exp(q2_rear*f_cr(0,0))*dCr;
                ddBf=q1_front*q2_front*q2_front*exp(q2_front*f_cf(0,0))*dCf.transpose()*dCf;
                ddBr=q1_rear*q2_rear*q2_rear*exp(q2_rear*f_cr(0,0))*dCr.transpose()*dCr;

                //calculate dX and ddX
                dX=dX+dBf+dBr;
                ddX=ddX+ddBf+ddBr;

                //calculate cost
                M_scalar=f_cf*X;
                costJ_temp+=q1_front*exp(q2_front*M_scalar(0,0));
                M_scalar=f_cr*X;
                costJ_temp+=q1_rear*exp(q2_rear*M_scalar(0,0));
            }
        }

        // 2.3 distance cost
        dX=dX+(X_bar-X).transpose()*Q;
        ddX=ddX+Q;
        M_scalar=0.5*(X_bar-X).transpose()*Q*(X_bar-X);
        costJ_temp+=M_scalar(0,0);

        // 2.4 control costvector<vector<double>>K_lst;
        U(0,0)=initial_controls[i].accel+delta_controls[i].accel;
        U(1,0)=initial_controls[i].yaw_rate+delta_controls[i].yaw_rate;
        dU=dU+U.transpose()*R;
        ddU=ddU+R;
        M_scalar=0.5*U.transpose()*R*U;
        costJ_temp+=M_scalar(0,0);

        // 2.5 control constraint cost
        dCu<<1,0;
        dU=dU+q1_acc*q2_acc*exp(q2_acc*(U(0,0)-a_high))*dCu;
        ddU=ddU+q1_acc*q2_acc*q2_acc*exp(q2_acc*(U(0,0)-a_high))*dCu.transpose()*dCu;
        costJ_temp+=q1_acc*exp(q2_acc*(U(0,0)-a_high));

        dCu<<-1,0;
        dU=dU+q1_acc*q2_acc*exp(q2_acc*(a_low-U(0,0)))*dCu;
        ddU=ddU+q1_acc*q2_acc*q2_acc*exp(q2_acc*(a_low-U(0,0)))*dCu.transpose()*dCu;
        costJ_temp+=q1_acc*exp(q2_acc*(a_low-U(0,0)));

        dCu<<0,1;
        dU=dU+q1_yr*q2_yr*exp(q2_yr*(U(1,0)-X_cal_lst[i].v*tan(steer_high)/egoL))*dCu;
        ddU=ddU+q1_yr*q2_yr*q2_yr*exp(q2_yr*(U(1,0)-X_cal_lst[i].v*tan(steer_high)/egoL))*dCu.transpose()*dCu;
        costJ_temp+=q1_yr*exp(q2_yr*(U(1,0)-X_cal_lst[i].v*tan(steer_high)/egoL));

        dCu<<0,-1;
        dU=dU+q1_yr*q2_yr*exp(q2_yr*(X_cal_lst[i].v*tan(steer_low)/egoL-U(1,0)))*dCu;
        ddU=ddU+q1_yr*q2_yr*q2_yr*exp(q2_yr*(X_cal_lst[i].v*tan(steer_low)/egoL-U(1,0)))*dCu.transpose()*dCu;
        costJ_temp+=q1_yr*exp(q2_yr*(X_cal_lst[i].v*tan(steer_low)/egoL-U(1,0)));
        // 2.6 calculate Qx, Qu, Qxx, Quu, Qxu, Qux, deltaV
        vd_model.getVehicleModelAandB(X_cal_lst[i].v, X_cal_lst[i].theta, U(0,0), control_dt, A, B);
        Qx=dX+dVk*A;
        Qu=dU+dVk*B;
        Qxx=ddX+A.transpose()*ddVk*A;
        Quu=ddU+B.transpose()*ddVk*B;
        Qxu=B.transpose()*ddVk*A;
        Qux=Qxu.transpose();
        if(isCompleteCal)
        {
            K=-(Quu.inverse()).transpose()*Qxu;
            d=-(Quu.inverse()).transpose()*Qu.transpose();
            K_lst.push_back({K(0,0), K(0,1), K(0,2), K(0,3), K(1,0), K(1,1), K(1,2), K(1,3)});
            d_lst.push_back({d(0,0), d(1,0)});
            M_scalar=Qu*d;
            deltaV1d+=M_scalar(0,0);
            M_scalar=0.5*d.transpose()*Quu*d;
            deltaV2d=deltaV2d+M_scalar(0,0);
        }
        // 2.7 calculate dVk, ddVk, for next loop
        dVk=Qx+Qu*K+d.transpose()*Quu*K+d.transpose()*Qxu;
        ddVk=Qxx+K.transpose()*Quu*K+Qux*K+K.transpose()*Quu;
    }

    return costJ_temp;
}


void ciLQR::iLQRSolver()
{
    // 1. find the closest index in global waypoints and intercept local waypoints
    alfa=1.0;
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
        findClosestWaypointIndex(global_waypoints, planned_path[planning_start_index], closest_global_index, false);
        start_state.x=planned_path[planning_start_index].x;
        start_state.y=planned_path[planning_start_index].y;
        start_state.theta=planned_path[planning_start_index].theta;
        start_state.v=planned_path[planning_start_index].v;
        start_state.accel=planned_path[planning_start_index].accel;
        start_state.yaw_rate=planned_path[planning_start_index].yaw_rate;
    }
    local_waypoints.clear();
    for(int i=closest_global_index; i<min<int>(global_waypoints.size(), closest_global_index+global_horizon); i++)
    {
        waypoint.x=global_waypoints[i].x;
        waypoint.y=global_waypoints[i].y;
        waypoint.theta=global_waypoints[i].theta;
        waypoint.v=global_waypoints[i].v;
        waypoint.accel=global_waypoints[i].accel;
        waypoint.yaw_rate=global_waypoints[i].yaw_rate;
        local_waypoints.push_back(waypoint);
    }

    //2. polynominal fitting of local waypoints, make local waypoints dense, and calculate x_bar_lst
    polynominalFitting();
    local_waypoints_dense.clear();
    interplot_increment=(local_waypoints[local_waypoints.size()-1].x-local_waypoints[0].x)/(local_horizon-1);
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
    vd_model.CalVDTrajectory(&start_state, &initial_controls, &X_vd_lst, &local_horizon, &control_dt);

    X_bar_lst.clear();
    for(int i=0; i<X_vd_lst.size(); i++)
    {
        findClosestWaypointIndex(local_waypoints_dense, X_vd_lst[i], closest_local_index, true);
        X_bar_lst.push_back(local_waypoints_dense[closest_local_index]);
    }

    //3. backward pass and get the costJ_nominal
    costJ=BackwardPassAndGetCostJ(X_vd_lst, true);

    //4. forward pass and get the final cost
    forward_counter=0;
    while(forward_counter<1000)
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

        for(int i=0; i<local_horizon-1; i++)
        {
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
            deltaU_star=K*(X_nominal-X)+alfa*d;
            delta_control_signal.accel=deltaU_star(0,0);
            delta_control_signal.yaw_rate=deltaU_star(1,0);
            delta_controls.push_back(delta_control_signal);
            control_signal.accel=initial_controls[i].accel+U(0,0);
            control_signal.yaw_rate=initial_controls[i].yaw_rate+U(1,0);
            planned_controls.push_back(control_signal);
        
            vd_model.updateOneStep(&xk, &xk1, &control_signal, dt);
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

        //calculate nominal trajectory cost
        costJ_nominal=BackwardPassAndGetCostJ(X_nominal_lst, false);
        deltaV=alfa*deltaV1d+alfa*alfa*deltaV2d;
        z=(costJ_nominal-costJ)/deltaV;
        if(z>=beta1 && z<=beta2)
            break;
        else
        {
            alfa=gama*alfa;
        }
    }
}

void ciLQR::update()
{
    int planning_rate=1/dt;
    ros::Rate rate(planning_rate);

    saturn_msgs::Control control_msg;
    saturn_msgs::ControlArray control_lst_msg;
    
    geometry_msgs::PoseStamped waypoint_msg;
    nav_msgs::Path planned_path_msg;

    geometry_msgs::Point point_msg;
    visualization_msgs::Marker local_points_msg;
    visualization_msgs::Marker local_lines_msg;

    while(ros::ok())
    {
        ros::spinOnce();
        iLQRSolver();

        //fulfill related message, publish planned_path and planned_controls
        //1. publish controls
        control_lst_msg.header.frame_id="cilqr_planner";
        control_lst_msg.header.stamp=ros::Time::now();
        control_lst_msg.control_lst.clear();
        for(int i=0; i<planned_controls.size();i++)
        {
            control_msg.u_accel=planned_controls[i].accel;
            control_msg.u_yawrate=planned_controls[i].yaw_rate;
            control_lst_msg.control_lst.push_back(control_msg);
        }
        cilqr_control_pub.publish(control_lst_msg);

        //2. planned path for rviz
        planned_path_msg.header.frame_id="cilqr_planner";
        planned_path_msg.header.stamp=ros::Time::now();
        planned_path_msg.poses.clear();
        for(int i=0; i<planned_path.size(); i++)
        {
            waypoint_msg.pose.position.x=planned_path[i].x;
            waypoint_msg.pose.position.y=planned_path[i].y;
            waypoint_msg.pose.position.z=egoHeight/2;
            waypoint_msg.pose.orientation.x=0;
            waypoint_msg.pose.orientation.y=0;
            waypoint_msg.pose.orientation.z=sin(planned_path[i].theta/2);
            waypoint_msg.pose.orientation.w=cos(planned_path[i].theta/2);

            planned_path_msg.poses.push_back(waypoint_msg);
        }
        rviz_local_planned_path_pub.publish(planned_path_msg);

        //3. local reference waypoint for rviz
        local_points_msg.header.frame_id="cilqr_planner/local_referline";
        local_points_msg.header.stamp=ros::Time::now();
        local_lines_msg.header.frame_id="cilqr_planner/local_referline";
        local_lines_msg.header.stamp=ros::Time::now();
        local_points_msg.action=visualization_msgs::Marker::ADD;
        local_lines_msg.action=visualization_msgs::Marker::ADD;
        local_points_msg.ns="local_points_and_lines";
        local_lines_msg.ns="local_points_and_lines";
        local_points_msg.id=1;
        local_lines_msg.id=2;
        local_points_msg.type=visualization_msgs::Marker::POINTS;
        local_lines_msg.type=visualization_msgs::Marker::LINE_STRIP;
        //set scale and color
        local_points_msg.scale.x=0.2;
        local_points_msg.scale.y=0.2;
        local_lines_msg.scale.x=0.1;
        local_lines_msg.scale.y=0.1;
        local_points_msg.color.g=1.0;
        local_points_msg.color.a=1.0;
        local_lines_msg.color.b=1.0;
        local_lines_msg.color.a=1.0;
        //fuifill points and lines
        for(int i=0; i<planned_path.size(); i++)
        {
            point_msg.x=planned_path[i].x;
            point_msg.y=planned_path[i].y;
            point_msg.z=0;
            local_points_msg.points.push_back(point_msg);
            local_lines_msg.points.push_back(point_msg);
        }
        rviz_local_refer_points_pub.publish(local_points_msg);
        rviz_local_refer_lines_pub.publish(local_lines_msg);
        
        rate.sleep();
    }
}