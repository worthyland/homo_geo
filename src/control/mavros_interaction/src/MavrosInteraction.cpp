#include <mavros_interaction/MavrosInteraction.h>
namespace Control
{

MavrosInteraction::MavrosInteraction()
{
    nh_ =  new ros::NodeHandle();
    posSub = nh_->subscribe("mavros/local_position/pose",1,&MavrosInteraction::PoseCallback,this);
    imuSub = nh_->subscribe("mavros/imu/data",1,&MavrosInteraction::IMUCallback,this);
    velocityBodySub = nh_->subscribe("mavros/local_position/velocity_body",1,&MavrosInteraction::VelocityBodyCallback,this);
    velocityLocalSub = nh_->subscribe("mavros/local_position/velocity_local",1,&MavrosInteraction::VelocityLocalCallback,this);
    posCallbackState = false;
    imuCallbackState = false;
    velocityBodyCallbackState = false;
    velocityLocalCallbackState = false;
} 

MavrosInteraction::MavrosInteraction(const std::string& nh)
{
    
    nh_ =  new ros::NodeHandle(nh);

}

MavrosInteraction::~MavrosInteraction()
{
    delete nh_;
}

void 
MavrosInteraction::PoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    posCallbackState = true;
    Eigen::Vector3d posTemp(msg->pose.position.x,msg->pose.position.y,
                            msg->pose.position.z);
    uav_.SetPos(posTemp);
}

void
MavrosInteraction::IMUCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    imuCallbackState = true;
    //角速度获取 相对于机体坐标系 z轴向上
    Eigen::Vector3d omegaTemp(msg->angular_velocity.x,msg->angular_velocity.y,
                            msg->angular_velocity.z);
    uav_.SetOmega(omegaTemp);
    //加速度获取 相对于机体坐标系 z轴向上
    Eigen::Vector3d accTemp(msg->linear_acceleration.x,msg->linear_acceleration.y,
                            msg->linear_acceleration.z);
    uav_.SetAcc(accTemp);  
    //四元数获取 相对于世界坐标系
    Eigen::Quaterniond  orientationTmp(msg->orientation.w,msg->orientation.x,
                                        msg->orientation.y,msg->orientation.z); 
    uav_.SetOrientation(orientationTmp);
}

void
MavrosInteraction::VelocityBodyCallback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    velocityBodyCallbackState = true;

}

void
MavrosInteraction::VelocityLocalCallback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    velocityLocalCallbackState = true;

}

}