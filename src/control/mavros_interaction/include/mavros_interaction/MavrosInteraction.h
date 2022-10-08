#ifndef __MAVROS_INTERACTION__
#define __MAVROS_INTERACTION__
#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>


#include <ros/ros.h>

#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <uav_state/UAVState.h>
namespace Control
{

class MavrosInteraction: public Quadrotor
{
private:
    /* data */
    ros::NodeHandle *nh_;
    /* 话题订阅 */
    ros::Subscriber pos_sub;//位置信息 世界框架下z轴向上为正 
    ros::Subscriber imu_sub;//获取IMU的信息 base_link 话题名称：mavros/imu/data
    ros::Subscriber velocity_body_sub;//机体坐标下速度 机体框架z轴向上为正 base_link 话题名称：mavros/local_position/velocity_body
    ros::Subscriber velocity_local_sub;//世界坐标下速度 世界框架z轴向上为正 base_link 话题名称：mavros/local_position/velocity_local
    ros::Subscriber state_sub;
    /* 话题发布 */
    ros::Publisher mix_pub;
    /* 服务端*/
    ros::ServiceClient arming_client;
    ros::ServiceClient set_mode_client;

public:
    bool IMUUpdateState,VelocityUpdateState,OffboardState; 

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