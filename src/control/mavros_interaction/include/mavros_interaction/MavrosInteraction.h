#ifndef __MAVROS_INTERACTION__
#define __MAVROS_INTERACTION__
//系统的头文件
#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>

//需要的头文件
#include <uav_state/UAVState.h>
namespace Control
{

class MavrosInteraction
{
private:
    /* data */
    ros::NodeHandle *nh_;
    /* 话题订阅 */
    ros::Subscriber posSub;//位置信息 世界框架下z轴向上为正 
    ros::Subscriber imuSub;//获取IMU的信息 base_link 话题名称：mavros/imu/data 更新角速度/加速度/    四元数 相对于世界坐标系
    ros::Subscriber velocityBodySub;//机体坐标下速度 机体框架z轴向上为正 base_link 话题名称：mavros/local_position/velocity_body
    ros::Subscriber velocityLocalSub;//世界坐标下速度 世界框架z轴向上为正 base_link 话题名称：mavros/local_position/velocity_local
    ros::Subscriber stateSub;
    /* 话题发布 */
    ros::Publisher mix_pub;
    /* 服务端*/
    ros::ServiceClient armingClient;
    ros::ServiceClient setModeClient;

    bool posCallbackState,imuCallbackState,velocityBodyCallbackState,velocityLocalCallbackState,offboardState; 
public:
    Quadrotor uav_;
public:
    MavrosInteraction();
    MavrosInteraction(const std::string& nh);
    ~MavrosInteraction();
    virtual void PoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    virtual void IMUCallback(const sensor_msgs::Imu::ConstPtr& msg);
    virtual void VelocityBodyCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);
    virtual void VelocityLocalCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);
};




}
#endif