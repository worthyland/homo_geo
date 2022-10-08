#include <mavros_interaction/MavrosInteraction.h>
namespace Control
{

MavrosInteraction::MavrosInteraction()
{
    nh_ =  new ros::NodeHandle();
    pos_sub = nh_->subscribe("mavros/local_position/pose",1,&MavrosInteraction::PoseCallback,this);
    imu_sub = nh_->subscribe("mavros/imu/data",1,&MavrosInteraction::IMUCallback,this);
    velocity_body_sub = nh_->subscribe("mavros/local_position/velocity_body",1,&MavrosInteraction::VelocityBodyCallback,this);
    velocity_local_sub = nh_->subscribe("mavros/local_position/velocity_local",1,&MavrosInteraction::VelocityLocalCallback,this);
}

MavrosInteraction::MavrosInteraction(const std::string& nh)
{
    
    nh_ =  new ros::NodeHandle(nh);
    //pos_sub = nh_->subscribe("mavros/local_position/pose",1,&MavrosInteraction::PoseCallback,this);
    imu_sub = nh_->subscribe("mavros/imu/data",1,&MavrosInteraction::IMUCallback,this);
}

MavrosInteraction::~MavrosInteraction()
{
    delete nh_;
}

void 
MavrosInteraction::PoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    
    Eigen::Vector3d pos_temp(msg->pose.position.x,msg->pose.position.y,msg->pose.position.z);
    SetPos(pos_temp);
    std::cout << pos_temp(0) << ","<<pos_temp(1) << ","<<pos_temp(2) <<std::endl;
}

void
MavrosInteraction::IMUCallback(const sensor_msgs::Imu::ConstPtr& msg)
{

}

void
MavrosInteraction::VelocityBodyCallback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{

}

void
MavrosInteraction::VelocityLocalCallback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{

}


}