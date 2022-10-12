/*

*/

#ifndef __MAVROS_INTERACTION__
#define __MAVROS_INTERACTION__
//系统的头文件
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>

//需要的头文件
#include <uav_state/UAVState.h>
namespace Control
{

class MavrosInteraction
{
private:
    /* data */
    ros::NodeHandle nh_;
    ros::NodeHandle nhParam_;
    /* 话题订阅 */
    ros::Subscriber posSub;//位置信息 世界框架下z轴向上为正 
    ros::Subscriber imuSub;//获取IMU的信息 base_link 话题名称：mavros/imu/data 更新角速度（相对于机体坐标系）/加速度（相对于机体坐标系）（Z轴为9.8，加速度的值）/    四元数 相对于世界坐标系
    ros::Subscriber velocityBodySub;//相对于机体坐标下速度,角速度 机体框架z轴向上为正 base_link 话题名称：mavros/local_position/velocity_body 
    ros::Subscriber velocityLocalSub;//相对与世界坐标下速度，角速度 世界框架z轴向上为正 base_link 话题名称：mavros/local_position/velocity_local
    ros::Subscriber px4ControlStateSub;
    /* 话题发布 */
    ros::Publisher mixPub;
    /* 服务端*/
    ros::ServiceClient armingClient;
    ros::ServiceClient setModeClient;

    bool posCallbackState,imuCallbackState,velocityBodyCallbackState,velocityLocalCallbackState,offboardState; 

    Quadrotor uav_;
    mavros_msgs::State currentControlMode_;


private:
    virtual void PoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    virtual void IMUCallback(const sensor_msgs::Imu::ConstPtr& msg);
    virtual void VelocityBodyCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);
    virtual void VelocityLocalCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);
    virtual void PX4ControlStateCallback(const mavros_msgs::State::ConstPtr& msg);

public:
    MavrosInteraction();
    MavrosInteraction(const ros::NodeHandle& nh,const ros::NodeHandle& nhParam);
    ~MavrosInteraction();

    void ShowUavState(int num) const;

    const Quadrotor::state& GetState()const;
    const Quadrotor& GetQuadrotor()const;
    
};




}
#endif