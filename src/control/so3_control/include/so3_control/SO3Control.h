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

};

private:
    /* data */
    ros::NodeHandle nh_;
    ros::NodeHandle nhParam_;
    double controlRate_;
    ControlGain controlgain_;
    Quadrotor curUavState_;
public:
    SO3Control(const ros::NodeHandle& nh,const ros::NodeHandle& nhParam);
    ~SO3Control();

    void SetState(const Quadrotor& val);
    const double& GetControlRate() const;

    
};

} // namespace CONTROL

#endif