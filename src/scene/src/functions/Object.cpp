#include "Object.h"

using namespace std;

Object::Object()
{
    id=0;
    obj_name="";
    obj_state.x=0.0;
    obj_state.y=0.0;
    obj_state.theta=0.0;
    obj_state.v=0.0;
    obj_state.accel=0.0;
    obj_state.yaw_rate=0.0;
     //tesla model3:
    obj_size.length=4.72;
    obj_size.width=1.85;
    obj_size.height=1.44;
    dt=0.1;
    predict_horizon=10;
    predicted_traj.clear();
    cout<<"Object default constructed!"<<endl;
}

Object::Object(const int ID, const string name, const double x, const double y, const double theta, const double v0, const double a0, const double dT)
{
    id=ID;
    obj_name=name;
    obj_state.x=x;
    obj_state.y=y;
    obj_state.theta=theta;
    obj_state.v=v0;
    obj_state.accel=a0;
    obj_state.yaw_rate=0.0;
    //tesla model3:
    obj_size.length=4.72;
    obj_size.width=1.85;
    obj_size.height=1.44;
    dt=dT;
    predict_horizon=10;
    predicted_traj.clear();
    cout<<"Object constructed!"<<endl;
    
}

Object::~Object(){cout<<"Object destructed!"<<endl;}

void Object::repose(const double x, const double y, const double theta)
{
    obj_state.x=x;
    obj_state.y=y;
    obj_state.theta=theta;
}

void Object::resize(const double l, const double w, const double h)
{
    obj_size.length=l;
    obj_size.width=w;
    obj_size.height=h;
}

void Object::resetAccel(const double target_accel)
{
    obj_state.accel=target_accel;
}

void Object::resetYawRate(const double target_yaw_rate)
{
    obj_state.yaw_rate=target_yaw_rate;
}

void Object::resetPredictHorizon(const int horizon)
{
    predict_horizon=horizon;
}

void Object::update()
{
    obj_state.x=obj_state.x+obj_state.v*dt*cos(obj_state.theta)+0.5*dt*dt*cos(obj_state.theta)*obj_state.accel;
    obj_state.y=obj_state.y+obj_state.v*dt*sin(obj_state.theta)+0.5*dt*dt*sin(obj_state.theta)*obj_state.accel;
    obj_state.v=obj_state.v+obj_state.accel*dt;
    obj_state.theta=obj_state.theta+obj_state.yaw_rate*dt;
}

void Object::ObjectPredict()
{
    predicted_traj.clear();
    predicted_traj.push_back(obj_state);
    for(int i=1; i<predict_horizon; i++)
    {
        obj_state.x=predicted_traj[i-1].x+(predicted_traj[i-1].v*dt+0.5*predicted_traj[i-1].accel*dt*dt)*cos(predicted_traj[i-1].theta);
        obj_state.y=predicted_traj[i-1].y+(predicted_traj[i-1].v*dt+0.5*predicted_traj[i-1].accel*dt*dt)*sin(predicted_traj[i-1].theta);
        obj_state.v=predicted_traj[i-1].v+predicted_traj[i-1].accel*dt;
        obj_state.theta=predicted_traj[i-1].theta+predicted_traj[i-1].yaw_rate*dt;
        obj_state.accel=predicted_traj[i-1].accel;
        obj_state.yaw_rate=predicted_traj[i-1].yaw_rate;
        predicted_traj.push_back(obj_state);
    }
}

int Object::getID() const
{
    return id;
}

string Object::getName() const
{
    return obj_name;
}

double Object::getPoseX() const
{
    return obj_state.x;
}

double Object::getPoseY() const
{
    return obj_state.y;
}

double Object::getPoseTheta() const
{
    return obj_state.theta;
}

double Object::getVelocity() const
{
    return obj_state.v;
}

double Object::getAccelerate() const
{
    return obj_state.accel;
}

double Object::getYawRate() const
{
    return obj_state.yaw_rate;
}   

double Object::getLength() const
{
    return obj_size.length;
}

double Object::getWidth() const
{
    return obj_size.width;
}

double Object::getHeight() const
{
    return obj_size.height;
}

const vector<ObjState>& Object::getPredictedTraj() const
{
    return predicted_traj;
}