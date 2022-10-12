#ifndef __SO_CONTROL__
#define __SO_CONTROL__

#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Dense>


#include <uav_state/UAVState.h>
namespace Control
{

class SO3Control
{
public:
struct ControlGain
{
    /* data */
    Eigen::Matrix3d KR;
    Eigen::Matrix3d KOmega;
};

private:
    /* data */
    ros::NodeHandle nh_;
    ros::NodeHandle nhParam_;
    double controlRate_;
    ControlGain controlGain_;
    Quadrotor curUavState_;

    Eigen::Vector3d omegaDesired_;
    Eigen::Matrix3d RDesired_;
    Eigen::Vector3d torque_;
    Eigen::Vector3d eR_,eOmega_;



public:
    SO3Control(const ros::NodeHandle& nh,const ros::NodeHandle& nhParam);
    ~SO3Control();

    void SetState(const Quadrotor& val);
    const double& GetControlRate() const;
    void SetRDesired(const Eigen::Matrix3d& val);
    const Eigen::Matrix3d& GetRDesired() const;
    void SetOmegaDesired(const Eigen::Vector3d& val);
    const Eigen::Vector3d& GetOmegaDesired() const;

    const Eigen::Vector3d UpdateER();
    const Eigen::Vector3d UpdateEOmega();
    const Eigen::Vector3d UpdateTorque();
    void operator() (const Eigen::Matrix3d& RDesired,const Eigen::Vector3d& omegaDesired,const Control::Quadrotor& curUavState);
    
};

} // namespace CONTROL

#endif