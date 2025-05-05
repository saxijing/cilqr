#ifndef VEHICLEMODEL_H_
#define VEHICLEMODEL_H_

#include "Common.h"
#include <eigen3/Eigen/Dense>

using namespace std;

class VehicleModel
{
    public:
        VehicleModel();
        VehicleModel(const double x, const double y, const double theta, const double v0, const double dT);
        ~VehicleModel();
        void initializeVelocity(const double& target_vel);
        void normalizeAngle(double& angle_radius);
        void repose(const double x, const double y, const double theta, const double v0, const double dT);
        void repose(const ObjState& state);
        void resize(const double l, const double w, const double h);
        void resetTimestamp(const double& deltaT);
        void setMaxSpeed(const double& m_speed);
        void reconfigVDpara(const double& m, const double& L, const double& B);
        void applyU(const double& accel, const double& yaw_rate);
        void update();
        void updateOneStep(ObjState& xk, ObjState& xk1, const CtrlInput& uk, const double& dt);
        void CalVDTrajectory(ObjState& X0, const vector<CtrlInput>& U, vector<ObjState>& traj, const int& traj_len, const double& dt);
        void getPredictedPose(const ObjState& current_state, const double& accel, const double& timestep, ObjState& predict_state);
        void getVehicleModelAandB(const double v, const double theta, const double accel, const double dt, Eigen::MatrixXd& MA, Eigen::MatrixXd& MB);
        void getVehicleState(ObjState& state);
        double getPoseX() const;
        double getPoseY() const;
        double getPoseTheta();
        double getVelocity() const;
        double getAccelerate() const;
        double getYawRate() const;
        double getLength() const;
        double getWidth() const;
        double getHeight() const;
        
    protected:
        ObjState veh_state;
        ObjSize veh_size;
        CtrlInput veh_ctrl;
        ObjState X_temp;
        double dt; //timestep, s
        double mass;
        double wheel_base;
        double wheel_track;
        double max_speed;
        double calc_speed;
        double predict_s;
};
#endif