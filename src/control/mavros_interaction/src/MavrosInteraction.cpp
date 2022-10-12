#include <mavros_interaction/MavrosInteraction.h>

namespace Control
{

MavrosInteraction::MavrosInteraction()
{

} 

MavrosInteraction::MavrosInteraction(const ros::NodeHandle& nh,const ros::NodeHandle& nhParam):nh_(nh),nhParam_(nhParam)
{
    double mass;
    nhParam_.param("Mass",mass,1.0);
    uav_.SetMass(mass);
    double g;
    nhParam_.param("Gravity",g,9.80665);
    uav_.SetGravity(g);
    posSub = nh_.subscribe("mavros/local_position/pose",1,&MavrosInteraction::PoseCallback,this);
    imuSub = nh_.subscribe("mavros/imu/data",1,&MavrosInteraction::IMUCallback,this);
    velocityBodySub = nh_.subscribe("mavros/local_position/velocity_body",1,&MavrosInteraction::VelocityBodyCallback,this);
    velocityLocalSub = nh_.subscribe("mavros/local_position/velocity_local",1,&MavrosInteraction::VelocityLocalCallback,this);
    px4ControlStateSub = nh_.subscribe("mavros/state",1,&MavrosInteraction::PX4ControlStateCallback,this);
    
    posCallbackState = false;
    imuCallbackState = false;
    velocityBodyCallbackState = false;
    velocityLocalCallbackState = false;


} 


MavrosInteraction::~MavrosInteraction()
{

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
    // uav_.ShowVal("sdaa",orientationTmp,5);
    //更新旋转矩阵
    Eigen::Matrix3d RTmp(orientationTmp.toRotationMatrix());
    uav_.SetR(RTmp);

    //根新欧拉角，以RPY（ZYX）
    Eigen::Vector3d eulerAngle = Eigen::Vector3d::Zero();
    eulerAngle = uav_.QuaternionToEulerAngles(orientationTmp);
    uav_.SetEulerAngle(eulerAngle);
    //  uav_.ShowVal("sdasda",eulerAngle,4);
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
    Eigen::Vector3d vallocalTemp(msg->twist.linear.x,msg->twist.linear.y,
                            msg->twist.linear.z);
    uav_.SetVel(vallocalTemp);
}

void
MavrosInteraction::PX4ControlStateCallback(const mavros_msgs::State::ConstPtr& msg)
{
    currentControlMode_ = *msg;
}




void 
MavrosInteraction::ShowUavState(int num) const
{
    uav_.ShowState(num);
}

const Quadrotor::state& 
MavrosInteraction::GetState() const
{

    return uav_.GetState();
}

const Quadrotor& 
MavrosInteraction::GetQuadrotor()const
{
    return uav_;
}


}