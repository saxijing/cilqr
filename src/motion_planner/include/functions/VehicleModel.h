#ifndef VEHICLEMODEL_H_
#define VEHICLEMODEL_H_

#include<ros/ros.h>
#include<iostream>
#include<Common.h>

using namespace std;

class VehicleModel
{
    public:
        VehicleModel();
        VehicleModel(const double x, const double y, const double theta, const double v0, const double dT);
        ~VehicleModel();
        void repose(const double x, const double y, const double theta, const double v0, const double dT);
        void reconfig_vd_para(const double m, const double wheel_base, const double wheel_track);
        void reconfig_vd_para(const double m, const double wheel_base, const double wheel_track);
        void apply_u(const double accel, const double yaw_rate);
        void update();
        void updateOneStep(const ObjState* xk, ObjState* xk1, const CtrlInput* uk, const double dt);
        void CalVDTrajectory(const ObjState* X0, const vector<CtrlInput>* U, vector<ObjState>* traj, const int* traj_len, const double* dt);
        void getVehicleModelAandB(const double v, const double theta, const double accel, const double dt, &MatrixXd MA, &MatrixXd MB);
        double get_posX();
        double get_posY();
        double get_posTheta();
        double get_velocity();
        double get_accelerate();
        double get_yaw_rate();
        double get_length();
        double get_width();
        double get_height();

    protected:
        ObjState veh_state;
        ObjSize veh_size;
        CtrlInput veh_ctrl;
        ObjState X_temp;
        double dt; //timestep, s
        double mass;
        double wheel_base;
        double wheel_track;
}
#end if