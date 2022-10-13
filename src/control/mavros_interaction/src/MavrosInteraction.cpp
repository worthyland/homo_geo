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
    //mavros 消息订阅
    posSub_ = nh_.subscribe("mavros/local_position/pose",10,&MavrosInteraction::PoseCallback,this);
    imuSub_ = nh_.subscribe("mavros/imu/data",1,&MavrosInteraction::IMUCallback,this);
    velocityBodySub_ = nh_.subscribe("mavros/local_position/velocity_body",10,&MavrosInteraction::VelocityBodyCallback,this);
    velocityLocalSub_ = nh_.subscribe("mavros/local_position/velocity_local",10,&MavrosInteraction::VelocityLocalCallback,this);
    px4ControlStateSub_ = nh_.subscribe("mavros/state",10,&MavrosInteraction::PX4ControlStateCallback,this);
    
    //mavros 消息发布
    mixPub_ = nh_.advertise<mavros_msgs::ActuatorControl>("mavros/actuator_control",10);



    nhParam_.param("MinTorque",minTorque_,-1.0);
    nhParam_.param("MaxTorque",maxTorque_,1.0);
    nhParam_.param("MinThrust",minThrust_,0.0);
    nhParam_.param("MaxThrust",maxThrust_,1.0);

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

    //四元数获取 相对于世界坐标系
    Eigen::Quaterniond  orientationTmp(msg->orientation.w,msg->orientation.x,
                                        msg->orientation.y,msg->orientation.z); 
    uav_.SetOrientation(orientationTmp);

    //更新旋转矩阵
    Eigen::Matrix3d RTmp(orientationTmp.toRotationMatrix());
    uav_.SetR(RTmp);

    //根新欧拉角，以RPY（ZYX）
    Eigen::Vector3d eulerAngle = Eigen::Vector3d::Zero();
    eulerAngle = uav_.QuaternionToEulerAngles(orientationTmp);
    uav_.SetEulerAngle(eulerAngle);

    //加速度获取 相对于机体坐标系 z轴向上
    Eigen::Vector3d accTemp(msg->linear_acceleration.x,msg->linear_acceleration.y,
                            msg->linear_acceleration.z);
                             
    Eigen::Vector3d axisZ(0,0,1);
    uav_.SetAcc(RTmp *accTemp - uav_.GetGravity()*axisZ);  
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
    currentControlState_ = *msg;
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

const mavros_msgs::State& 
MavrosInteraction::GetCurrentControlState()const
{
    return currentControlState_;
}
void 
MavrosInteraction::ActuatorPub(const Eigen::Vector3d& torque,const double& thrust) const
{
    mavros_msgs::ActuatorControl mixTmp;
    mixTmp.group_mix = 0;
    mixTmp.controls[0] = std::max(minTorque_, std::min(maxTorque_,torque(0)));
    mixTmp.controls[1] = std::max(minTorque_, std::min(maxTorque_,torque(1)));
    mixTmp.controls[2] = std::max(minTorque_, std::min(maxTorque_,torque(2)));
    mixTmp.controls[3] = std::max(minThrust_, std::min(maxThrust_, thrust));
    mixTmp.controls[4] = 0.0;
    mixTmp.controls[5] = 0.0;
    mixTmp.controls[6] = 0.0;
    mixTmp.controls[7] = 0.0;
    mixPub_.publish(mixTmp);
}

}