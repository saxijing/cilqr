#ifndef CILQR_H
#define CILQR_H

#include "Common.h"
#include<ros/ros.h>
#include<geometry_msgs/PoseStamped.h>
#include<nav_msgs/Path.h>
#include<visualization_msgs/Marker.h>
#include<saturn_msgs/State.h>
#include<saturn_msgs/StateLite.h>
#include<saturn_msgs/Size.h>
#include<saturn_msgs/ObstacleState.h>
#include<saturn_msgs/ObstacleStateArray.h>
#include<saturn_msgs/Control.h>
#include<saturn_msgs/ControlArray.h>
#include<saturn_msgs/Path.h>
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
        void getLocalReferPoints(const vector<ObjState>& local_waypoints, const vector<ObjState>& trajcetory, vector<ObjState>& local_refer_points);
        double getStateConstraintCostAndDeriva(const double& q1, const double& q2, const double& state_value, const double& limit, const string& type, const Eigen::MatrixXd& dcr, Eigen::MatrixXd& dx, Eigen::MatrixXd& ddx);
        double getControlConstraintCostAndDeriva(const double& q1, const double& q2, const double& control_input, const double& limit, const string& type, const Eigen::MatrixXd& dcu, Eigen::MatrixXd& du, Eigen::MatrixXd& ddu);
        double BackwardPassAndGetCostJ(const vector<ObjState>&X_cal_lst, const vector<CtrlInput>& U_cal_lst, double lambda, bool isCompleteCal);
        void ForwardPass();
        void iLQRSolver();
        void update();
    protected:
        ros::NodeHandle nh;
        ros::Subscriber ego_state_sub;
        ros::Subscriber obstacles_state_sub;
        ros::Publisher rviz_local_refer_points_pub;
        ros::Publisher rviz_local_refer_lines_pub;
        ros::Publisher rviz_cilqr_planned_path_pub;
        ros::Publisher local_planned_path_pub;
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
        ObjState center_point;
        ObjState left_point;
        ObjState right_point;
        CtrlInput control_signal; 
        CtrlInput delta_control_signal;
        //obstacles msg parse
        ObjState obs_state_single;
        ObsInfo obs_info_single;                                 
        vector<ObsInfo> obstacles_info;
        //end
        vector<ObjState> global_waypoints;
        vector<ObjState> left_roadedge;
        vector<ObjState> right_roadedge;
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
        int local_end_index;
        int prediction_horizon;
        int poly_order;
        double dt;
        double control_dt;
        double predict_dt;
        double max_dist;
        double dist;
        double lane_width;
        double lane_num;

        Eigen::MatrixXd polyX;
        Eigen::MatrixXd polyY;
        Eigen::MatrixXd poly_a;
        
        double safe_a;
        double safe_b;
        double safe_time;
        double ego_radius;
        double interplot_increment;
        double alfa;
        double w_pos, w_vel, w_theta;
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
        double egoHeight;
        double egoWidth;
        double ego_lf, ego_lr;
        double roadLR_dot;
        double roadLR_norm;
        Eigen::MatrixXd Q;
        Eigen::MatrixXd R;

        Eigen::MatrixXd X;
        Eigen::MatrixXd X_bar;
        Eigen::MatrixXd X_nominal;
        Eigen::MatrixXd X_front;
        Eigen::MatrixXd X_rear;
        Eigen::MatrixXd X_obs;
        Eigen::MatrixXd X_front_obs;
        Eigen::MatrixXd X_rear_obs;
        Eigen::MatrixXd deltaX;
        Eigen::MatrixXd U;
        Eigen::MatrixXd T;
        Eigen::MatrixXd dX_front; //vector X_front to vector X
        Eigen::MatrixXd dX_rear; //vector X_rear to vector X
        Eigen::MatrixXd dCf;
        Eigen::MatrixXd dCr;
        Eigen::MatrixXd f_cf;
        Eigen::MatrixXd f_cr;
        Eigen::MatrixXd dBf;
        Eigen::MatrixXd dBr;
        Eigen::MatrixXd ddBf;
        Eigen::MatrixXd ddBr;

        Eigen::MatrixXd dVk;
        Eigen::MatrixXd ddVk;
        Eigen::MatrixXd dX;
        Eigen::MatrixXd ddX;
        Eigen::MatrixXd dU;
        Eigen::MatrixXd ddU;
        Eigen::MatrixXd dBu;
        Eigen::MatrixXd ddBu;
        Eigen::MatrixXd A;
        Eigen::MatrixXd B;
        Eigen::MatrixXd P;
        Eigen::MatrixXd dCu;
        Eigen::MatrixXd Qx;
        Eigen::MatrixXd Qu;
        Eigen::MatrixXd Qxx;
        Eigen::MatrixXd Quu;
        Eigen::MatrixXd Qxu;
        Eigen::MatrixXd Qux;
        Eigen::MatrixXd K;
        Eigen::MatrixXd d;
        Eigen::MatrixXd deltaU_star;
        Eigen::MatrixXd M_scalar;  //store 1*1 matrix calculation result
        //for regulization
        Eigen::MatrixXd I;
        Eigen::MatrixXd Reg;
        Eigen::MatrixXd B_reg;
        Eigen::MatrixXd Qux_reg;
        Eigen::MatrixXd Quu_reg;

        ObjState xk;
        ObjState xk1;
        vector<vector<double>>K_lst;
        vector<vector<double>>d_lst;
        double deltaV, deltaV1d, deltaV2d;
        double costJ;
        double costJ_nominal;
        double costJ_cache;
        double cost_single; //store indivitual cost value
        int forward_counter;
        int optimization_counter;
        double z;
        double beta1, beta2;
        double gama;
        bool isRecEgoVeh;
        const double EPS=1e-5;
        double start_dist;
        int max_forward_iterate;
        int max_optimal_iterate;
        int refer_closest_index;
        double xm,ym,thetam;
        double xr,yr,thetar;
        double x_ego, y_ego, theta_ego;
        ObjState project_point;
        double max_percep_dist;
        double max_speed;
        double lamb_init;
        double lamb;
        double lamb_decay;
        double lamb_ambify;
        double lamb_max;
        bool forward_iter_flag;
        double optimal_tol;
        double J, J_nominal;
        
        //partial function temp variables
        double state_constraint_cost;
        double ctrl_constraint_cost;
        double constraint_value;
        double obs_constrain_limit;
        string limit_type;
        double yaw_rate_limit;
        double edge_limit;
        
        int start_index_multiple;
};

#endif