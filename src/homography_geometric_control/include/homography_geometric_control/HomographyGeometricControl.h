#ifndef __HOMOGRAQPHY_GEOMETRIC_CONTROL__
#define __HOMOGRAQPHY_GEOMETRIC_CONTROL__

#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Dense>


#include <uav_state/UAVState.h>
#include <homo_msgs/HomographyResult.h>


namespace HomographyGeometricControl
{

class HomographyGeometric
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
    ros::Subscriber homographySub;
    ControlGain controlgain_;
    Eigen::Matrix3d homography_;
    Control::Quadrotor curUavState_;//当前的无人机状态
    /*    curUavState_ 内部变量
    *Eigen::Vector3d pos;//位置
    *Eigen::Vector3d vel;//速度 ->需要相对于机体nedzuo坐标系下的速度
    *Eigen::Vector3d acc;//加速度 
    *Eigen::Vector3d jerk;//加加速度    
    *Eigen::Vector3d eulerAngle;//角度  欧拉角
    *Eigen::Vector3d omega;//角速度 默认为相对于机体nedzuo坐标系下的速度
    *Eigen::Matrix3d R;//旋转矩阵->需要相对于世界nedzuo坐标系下的旋转矩阵
    *Eigen::Quaterniond orientation;//四元数 
    *double mass;//质量
    *double g;//重力加速度
    *Eigen::Matrix3d J;//惯性矩阵；
     */
    bool homographyCallbackState;

    virtual void HomographyCallback(const homo_msgs::HomographyResult::ConstPtr& msg);
    void UpdateCurUavState(const Control::Quadrotor& val);
    const Eigen::Vector3d& GetError1(const Eigen::Matrix3d& homography);
    

public:
    HomographyGeometric(const ros::NodeHandle& nh,const ros::NodeHandle& nhParam);
    ~HomographyGeometric();

    void operator() (const Control::Quadrotor& curUavState);

    Control::Quadrotor FrameTransform(const Control::Quadrotor& curUavState);//将世界框架的state状态量进行变换，变换到ned坐标系下
};









}
#endif