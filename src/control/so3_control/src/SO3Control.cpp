#include <so3_control/SO3Control.h>


namespace Control
{
SO3Control::SO3Control(const ros::NodeHandle& nh,const ros::NodeHandle& nhParam):nhParam_(nhParam)
{
    nhParam_.param("Control_rate",controlRate_,60.0);
}

SO3Control::~SO3Control()
{
}


void 
SO3Control::SetState(const Quadrotor& val)
{
    curUavState_ = val;
}

const double& 
SO3Control::GetControlRate() const
{
    return controlRate_;
}
} // namespace Control
