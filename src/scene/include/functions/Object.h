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
        Object(const int ID, const string name, const double x, const double y, const double theta, const double v0, const double a0, const double dT);
        ~Object();
        void repose(const double x, const double y, const double theta);
        void resize(const double l, const double w, const double h);
        void resetAccel(const double target_accel);
        void resetYawRate(const double target_yaw_rate);
        void resetPredictHorizon(const int horizon);
        void update(); 
        void ObjectPredict();
        int getID() const;
        string getName() const;
        double getPoseX() const;
        double getPoseY() const;
        double getPoseTheta() const;
        double getVelocity() const;
        double getAccelerate() const;
        double getYawRate() const;
        double getLength() const;
        double getWidth() const;
        double getHeight() const;
        const vector<ObjState>& getPredictedTraj() const;
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