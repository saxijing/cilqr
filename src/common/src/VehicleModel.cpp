#include "VehicleModel.h"

using namespace std;

VehicleModel::VehicleModel()
{
    veh_state.x=0.0;
    veh_state.y=0.0;
    veh_state.theta=0.0;
    veh_state.v=0.0;
    veh_state.accel=0.0;
    veh_state.yaw_rate=0.0;
    veh_ctrl.accel=0.0;
    veh_ctrl.yaw_rate=0.0;
    X_temp.x=0.0;
    X_temp.y=0.0;
    X_temp.theta=0.0;
    X_temp.v=0.0;
    X_temp.accel=0.0;
    X_temp.yaw_rate=0.0;
    dt=0.1;
    //tesla model3 paras:
    mass=1823;
    wheel_base=2.875;
    wheel_track=1.584; 
    veh_size.length=4.72;
    veh_size.width=1.85;
    veh_size.height=1.44;
    cout<<"VehicleModel default constructed!"<<endl;
}

VehicleModel::VehicleModel(const double x, const double y, const double theta, const double v0, const double dT)
{
    veh_state.x=x;
    veh_state.y=y;
    veh_state.theta=theta;
    veh_state.v=v0;
    veh_state.accel=0.0;
    veh_state.yaw_rate=0.0;
    veh_ctrl.accel=0.0;
    veh_ctrl.yaw_rate=0.0;
    X_temp.x=0.0;
    X_temp.y=0.0;
    X_temp.theta=0.0;
    X_temp.v=0.0;
    X_temp.accel=0.0;
    X_temp.yaw_rate=0.0;
    dt=dT;
    //tesla model3 paras:
    mass=1823;
    wheel_base=2.875;
    wheel_track=1.584;
    veh_size.length=4.72;
    veh_size.width=1.85;
    veh_size.height=1.44;
    cout<<"VehicleModel constructed!"<<endl;
}

VehicleModel::~VehicleModel(){cout<<"VehicleModel destructed!"<<endl;}

void VehicleModel::repose(const double x, const double y, const double theta, const double v0, const double dT)
{
    veh_state.x=x;
    veh_state.y=y;
    veh_state.theta=theta;
    veh_state.v=v0;
    dt=dT;
}

void VehicleModel::resize(const double l, const double w, const double h)
{
    veh_size.length=l;
    veh_size.width=w;
    veh_size.height=h;
}

void VehicleModel::setMaxSpeed(const double& m_speed)
{
    max_speed=m_speed;
}

void VehicleModel::reconfigVDpara(const double m, const double L, const double B)
{
    mass=m;
    wheel_base=L;
    wheel_track=B;
}

void VehicleModel::applyU(const double accel, const double yaw_rate)
{
    cout<<"applied recved U!"<<endl;
    veh_ctrl.accel=accel;
    veh_ctrl.yaw_rate=yaw_rate;
}

void VehicleModel::update()
{
    veh_state.x=veh_state.x+veh_state.v*dt*cos(veh_state.theta)+0.5*dt*dt*cos(veh_state.theta)*veh_ctrl.accel;
    veh_state.y=veh_state.y+veh_state.v*dt*sin(veh_state.theta)+0.5*dt*dt*sin(veh_state.theta)*veh_ctrl.accel;
    calc_speed=veh_state.v+dt*veh_ctrl.accel;
    if(calc_speed>max_speed) { calc_speed=max_speed;}
    else if(calc_speed<0) {calc_speed=0;}
    veh_state.v=calc_speed;
    veh_state.theta=veh_state.theta+dt*veh_ctrl.yaw_rate;
    veh_state.accel=veh_ctrl.accel;
    veh_state.yaw_rate=veh_ctrl.yaw_rate;
}

void VehicleModel::updateOneStep(const ObjState& xk, ObjState& xk1, const CtrlInput& uk, const double& dt)
{
    xk1.x=xk.x+xk.v*dt*cos(xk.theta)+0.5*dt*dt*cos(xk.theta)*uk.accel;
    xk1.y=xk.y+xk.v*dt*sin(xk.theta)+0.5*dt*dt*sin(xk.theta)*uk.accel;
    calc_speed=xk.v+dt*uk.accel;
    if(calc_speed>max_speed) { calc_speed=max_speed;}
    else if(calc_speed<0) {calc_speed=0;}
    xk1.v=calc_speed;
    xk1.theta=xk.theta+dt*uk.yaw_rate;
    xk1.accel=uk.accel;
    xk1.yaw_rate=uk.yaw_rate;
}

void VehicleModel::CalVDTrajectory(const ObjState& X0, const vector<CtrlInput>& U, vector<ObjState>& traj, const int& traj_len, const double& dt)
{
    cout<<"come into CalVDTrajectory!"<<endl;
    traj.clear();
    traj.push_back(X0);
    for(int i=1; i<traj_len; i++)
    {
        X_temp.x=traj[i-1].x+traj[i-1].v*dt*cos(traj[i-1].theta)+0.5*dt*dt*cos(traj[i-1].theta)*U[i-1].accel;
        X_temp.y=traj[i-1].y+traj[i-1].v*dt*sin(traj[i-1].theta)+0.5*dt*dt*sin(traj[i-1].theta)*U[i-1].accel;
        calc_speed=traj[i-1].v+dt*U[i-1].accel;
        if(calc_speed>max_speed) { calc_speed=max_speed;}
        else if(calc_speed<0) {calc_speed=0;}
        X_temp.v=calc_speed;
        X_temp.theta=traj[i-1].theta+dt*U[i-1].yaw_rate;
        X_temp.accel=U[i-1].accel;
        X_temp.yaw_rate=U[i-1].yaw_rate;
        traj.push_back(X_temp);
    }
}

void VehicleModel::getVehicleModelAandB(const double v, const double theta, const double accel, const double dt, Eigen::MatrixXd& MA, Eigen::MatrixXd& MB)
{
    MA(0,0)=1.0;
    MA(0,1)=0.0;
    MA(0,2)=dt*cos(theta);
    MA(0,3)=-sin(theta)*(v*dt+0.5*dt*dt*accel);
    MA(1,0)=0.0;
    MA(1,1)=1.0;
    MA(1,2)=dt*sin(theta);
    MA(1,3)=cos(theta)*(v*dt+0.5*dt*dt*accel);
    MA(2,0)=0.0;
    MA(2,1)=0.0;
    MA(2,2)=1.0;
    MA(2,3)=0.0;
    MA(3,0)=0.0;
    MA(3,1)=0.0;
    MA(3,2)=0.0;
    MA(3,3)=1.0;

    MB(0,0)=0.5*dt*dt*cos(theta);
    MB(0,1)=0.0;
    MB(1,0)=0.5*dt*dt*sin(theta);
    MB(1,1)=0.0;
    MB(2,0)=dt;
    MB(2,1)=0.0;
    MB(3,0)=0.0;
    MB(3,1)=dt;
}   

double VehicleModel::getPoseX() const
{
    return veh_state.x;
}

double VehicleModel::getPoseY() const
{
    return veh_state.y;
}

double VehicleModel::getPoseTheta() const
{
    return veh_state.theta;
}

double VehicleModel::getVelocity() const
{
    return veh_state.v;
}

double VehicleModel::getAccelerate() const
{
    return veh_state.accel;
}

double VehicleModel::getYawRate() const
{
    return veh_state.yaw_rate;
}

double VehicleModel::getLength() const
{
    return veh_size.length;
}

double VehicleModel::getWidth() const
{
    return veh_size.width;
}

double VehicleModel::getHeight() const
{
    return veh_size.height;
}