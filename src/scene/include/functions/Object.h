#ifndef OBJECT_H_
#define OBJECT_H_

#include "Common.h"
#include<string>

using namespace std;

class Object
{
    friend class Objects;
    public:
        Object();
        Object(const int ID, const string name, const double x, const double y, const double theta, const double v0, const double dT);
        ~Object();
        void repose(const double x, const double y, const double z, const double theta);
        void resize(const double l, const double w, const double h);
        void update(const double accel, const double yaw_rate);
        const vector<ObjState>& ObjectPredict(const int predict_horizon, const double accel, const double dt);
        int getID();
        string getName();
        double getPoseX();
        double getPoseY();
        double getPoseTheta();
        double getVelocity();
        double getAccelerate();
        double getYawRate();
        double getLength();
        double getWidth();
        double getHeight();
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