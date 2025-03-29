#ifndef CILQR_H
#define CILQR_H

#include "Common.h"
#include<ros/ros.h>
#include<nav_msgs/Path.h>
#include<visualization_msgs/Marker.h>
#include<saturn_msgs/State.h>
#include<saturn_msgs/StateLite.h>
#include<saturn_msgs/Size.h>
#include<saturn_msgs/ObstacleState.h>
#include<saturn_msgs/ObstacleStateArray.h>
#include<saturn_msgs/Control.h>
#include<saturn_msgs/ControlArray.h>
#include<common/VehicleModel.h>
#include <eigen3/Eigen/Dense> 
#include<fstream>
#include<limits>
using namespace std;

class ciLQR
{
    public:
        ciLQR(ros::NodeHandle &nh_);
        ~ciLQR();
        void recvEgoState(const saturn_msgs::State &msg);
        void recvObstaclesState(const saturn_msgs::ObstacleStateArray &msg);
        void readGlobalWaypoints();
        void findClosestWaypointIndex(const vector<ObjState> &waypoints, const ObjState &state, int &closest_index, bool isFromStart);
        void polynominalFitting();
        double BackwardPassAndGetCostJ(const vector<ObjState>&X_cal_lst, bool isCompleteCal);
        void iLQRSolver();
        void update();
    protected:
        ros::NodeHandle nh;
        ros::Subscriber ego_state_sub;
        ros::Subscriber obstacles_state_sub;
        ros::Publisher cilqr_control_pub;
        ros::Publisher rviz_local_referline_pub;
        ros::Publisher rviz_local_planned_path_pub;
        string global_waypoints_filepath;
        int state_num;
        int control_num;
        int closest_global_index;
        int closest_local_index;
        bool isFirstFrame;
        int planning_start_index;
        VehicleModel vd_model;
        ObjState ego_state;
        ObjState start_state;
        ObjState waypoint;
        CtrlInput control_signal; 
        CtrlInput delta_control_signal;
        //obstacles msg parse
        ObjState obs_state_single;                                 
        vector<ObsInfo> obstacles_info;
        //end
        vector<ObjState> global_waypoints;
        vector<ObjState> local_waypoints;
        vector<ObjState> local_waypoints_dense;
        vector<ObjState> planned_path;
        vector<CtrlInput> initial_controls;
        vector<CtrlInput> delta_controls;
        vector<CtrlInput> planned_controls;
        vector<double> polyCoeff;
        vector<ObjState> X_bar_lst;
        vector<ObjState> X_vd_lst;
        vector<ObjState> X_nominal_lst;
        int global_horizon;
        int local_horizon;
        int prediction_horizon;
        int poly_order;
        double dt;
        double control_dt;
        double predict_dt;
        double max_dist;
        double dist;

        Eigen::MatrixXd polyX;
        Eigen::MatrixXd polyY;
        Eigen::MatrixXd poly_a;
        
        double safe_a;
        double safe_b;
        double safe_time;
        double ego_radius;
        double interplot_increment;
        double alfa;
        double w_posX, w_posY, w_vel, w_theta;
        double w_accel, w_yawrate;
        double q1_front, q2_front;
        double q1_rear, q2_rear;
        double q1_acc, q2_acc;
        double q1_yr, q2_yr;
        double a_high, a_low;
        double steer_high, steer_low;
        double ellipse_a;
        double ellipse_b;
        double egoL;
        double ego_lf, ego_lr;
        Eigen::MatrixXd Q=Eigen::MatrixXd::Zero(state_num, state_num);
        Eigen::MatrixXd R=Eigen::MatrixXd::Zero(control_num, control_num);

        Eigen::MatrixXd X=Eigen::MatrixXd(state_num, 1);
        Eigen::MatrixXd X_bar=Eigen::MatrixXd(state_num, 1);
        Eigen::MatrixXd X_nominal=Eigen::MatrixXd(state_num, 1);
        Eigen::MatrixXd X_k=Eigen::MatrixXd(state_num, 1);
        Eigen::MatrixXd X_front=Eigen::MatrixXd(state_num, 1);
        Eigen::MatrixXd X_rear=Eigen::MatrixXd(state_num, 1);
        Eigen::MatrixXd X_obs=Eigen::MatrixXd(state_num, 1);
        Eigen::MatrixXd X_front_obs=Eigen::MatrixXd(state_num, 1);
        Eigen::MatrixXd X_rear_obs=Eigen::MatrixXd(state_num, 1);
        Eigen::MatrixXd U=Eigen::MatrixXd::Zero(control_num, 1);
        Eigen::MatrixXd T=Eigen::MatrixXd::Zero(state_num, state_num);
        Eigen::MatrixXd dX_front=Eigen::MatrixXd(state_num, state_num); //vector X_front to vector X
        Eigen::MatrixXd dX_rear=Eigen::MatrixXd(state_num, state_num); //vector X_rear to vector X
        Eigen::MatrixXd dCf=Eigen::MatrixXd(1, state_num);
        Eigen::MatrixXd dCr=Eigen::MatrixXd(1, state_num);
        Eigen::MatrixXd f_cf=Eigen::MatrixXd(1, state_num);
        Eigen::MatrixXd f_cr=Eigen::MatrixXd(1, state_num);
        Eigen::MatrixXd dBf=Eigen::MatrixXd(1, state_num);
        Eigen::MatrixXd dBr=Eigen::MatrixXd(1, state_num);
        Eigen::MatrixXd ddBf=Eigen::MatrixXd(state_num, state_num);
        Eigen::MatrixXd ddBr=Eigen::MatrixXd(state_num, state_num);

        Eigen::MatrixXd dVk=Eigen::MatrixXd(1, state_num);
        Eigen::MatrixXd ddVk=Eigen::MatrixXd(state_num, state_num);
        Eigen::MatrixXd dX=Eigen::MatrixXd::Zero(1, state_num);
        Eigen::MatrixXd ddX=Eigen::MatrixXd::Zero(state_num, state_num);
        Eigen::MatrixXd dU=Eigen::MatrixXd::Zero(1, control_num);
        Eigen::MatrixXd ddU=Eigen::MatrixXd::Zero(control_num, control_num);
        Eigen::MatrixXd A=Eigen::MatrixXd::Zero(state_num, state_num);
        Eigen::MatrixXd B=Eigen::MatrixXd::Zero(state_num, control_num);
        Eigen::MatrixXd P=Eigen::MatrixXd::Zero(state_num, state_num);
        Eigen::MatrixXd dCu=Eigen::MatrixXd::Zero(1, control_num);
        Eigen::MatrixXd Qx=Eigen::MatrixXd(1, state_num);
        Eigen::MatrixXd Qu=Eigen::MatrixXd(1, control_num);
        Eigen::MatrixXd Qxx=Eigen::MatrixXd(state_num, state_num);
        Eigen::MatrixXd Quu=Eigen::MatrixXd(control_num, control_num);
        Eigen::MatrixXd Qxu=Eigen::MatrixXd(control_num, state_num);
        Eigen::MatrixXd Qux=Eigen::MatrixXd(state_num, control_num);
        Eigen::MatrixXd K=Eigen::MatrixXd(control_num, state_num);
        Eigen::MatrixXd d=Eigen::MatrixXd(control_num, 1);
        Eigen::MatrixXd deltaU_star=Eigen::MatrixXd(control_num, 1);
        Eigen::MatrixXd M_scalar;  //store 1*1 matrix calculation result
        ObjState xk;
        ObjState xk1;
        vector<vector<double>>K_lst;
        vector<vector<double>>d_lst;
        double deltaV, deltaV1d, deltaV2d;
        // //store temp K, d in forward pass process, additional result, no sense in forward pass
        // vector<vector<double>>K_temp_lst;
        // vector<vector<double>>d_temp_lst;
        // double deltaV1d_temp, deltaV2d_temp;
        double costJ;
        double costJ_nominal;
        int forward_counter;
        double z;
        double beta1, beta2;
        double gama;
};

#endif