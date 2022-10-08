#include <iostream>
#include <uav_state/UAVState.h>

namespace Control
{

Quadrotor::Quadrotor(const double& mass_,const double& g_,const Eigen::Matrix3d& J_)
{
    state_.mass = mass_;
    state_.g = g_;
    state_.pos = Eigen::Vector3d::Zero();
    state_.vel = Eigen::Vector3d::Zero();
    state_.acc = Eigen::Vector3d::Zero();
    state_.jerk = Eigen::Vector3d::Zero();
    state_.eulerAngle = Eigen::Vector3d::Zero();
    state_.omega = Eigen::Vector3d::Zero();
    state_.R = Eigen::Matrix3d::Identity();
    state_.J = J_;
}

Quadrotor::Quadrotor()
{
    state_.mass = 0;
    state_.g = 9.8091;
    state_.pos = Eigen::Vector3d::Zero();
    state_.vel = Eigen::Vector3d::Zero();
    state_.acc = Eigen::Vector3d::Zero();
    state_.jerk = Eigen::Vector3d::Zero();
    state_.eulerAngle = Eigen::Vector3d::Zero();
    state_.omega = Eigen::Vector3d::Zero();
    state_.R = Eigen::Matrix3d::Identity();
    state_.J = Eigen::Matrix3d::Identity();;
}

Quadrotor::~Quadrotor()
{
}


void 
Quadrotor::UpdateState()
{

}

void 
Quadrotor::SetMass(const double& val)
{
    state_.mass=val;
}

const double&
Quadrotor::GetMass(void) const
{
    return state_.mass;
}

void 
Quadrotor::SetGravity(const double& val)
{
    state_.g=val;
}

const double&
Quadrotor::GetGravity(void) const
{
    return state_.g;
}

void 
Quadrotor::SetInertialMatrix(const Eigen::Matrix3d& val)
{
    state_.J=val;
}

const Eigen::Matrix3d& 
Quadrotor::GetInertialMatrix(void)const
{
    return state_.J;
}

void 
Quadrotor::SetPos(const Eigen::Vector3d& val){
    state_.pos = val;
}

const Eigen::Vector3d& 
Quadrotor::GetPos(void)const
{
    return state_.pos;
}


}
