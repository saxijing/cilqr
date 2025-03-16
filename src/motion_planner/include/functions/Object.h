#ifndef OBJECT_H_
#define OBJECT_H_

#include<ros/ros.h>
#include<iostream>
#include<vector>
#include<string>

using namespace std;

class Object
{
    friend class Objects;
    public:
        Object();
        Object(double x, double y, double z, double theta, double v0, double dT);
        ~Object();
        void repose(double x, double y, double z, double theta);
        void resize(double l, double w, double h);
        void apply_u(double accel, double yaw_rate);
        void update();
        double get_posX();
        double get_posY();
        double get_posZ();
        double get_posTheta();
        double get_velocity();
        double get_accelerate();
        double get_yaw_rate();
        double get_length();
        double get_width();
        double get_height();

    protected:
        int id;
        string obj_name;
        ObjState obj_state;
        ObjSize obj_size;
        double dt; //timestep, s
        int predict_horizon;
        vector<ObjState> predicted_traj;
};

#endif