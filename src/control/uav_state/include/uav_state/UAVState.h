#ifndef __UAV_STATE__
#define __UAV_STATE__

#include <Eigen/Core>

namespace Control
{


class Quadrotor
{
public:
struct state
{
    /* data */
    Eigen::Vector3d pos;//位置
    Eigen::Vector3d vel;//速度
    Eigen::Vector3d acc;//加速度
    Eigen::Vector3d jerk;//加加速度
    
    Eigen::Vector3d eulerAngle;//角度  欧拉角
    Eigen::Vector3d omega;//角速度
    Eigen::Matrix3d R;//旋转矩阵
    
    double mass;//质量
    double g;//重力加速度
    Eigen::Matrix3d J;//惯性矩阵；
};

    Quadrotor();
    Quadrotor(const double& mass_,const double& g_,const Eigen::Matrix3d& J_);
    ~Quadrotor();
    void UpdateState(void);
    void SetMass(const double& val);
    const double& GetMass(void) const;
    void SetGravity(const double& val);
    const double& GetGravity(void) const;
    void SetInertialMatrix(const Eigen::Matrix3d& val);
    const Eigen::Matrix3d& GetInertialMatrix(void)const;
    void SetPos(const Eigen::Vector3d& val);
    const Eigen::Vector3d& GetPos(void)const;
private:
    state state_;
};



}

#endif