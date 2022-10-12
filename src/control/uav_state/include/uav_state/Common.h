#ifndef __COMMON__
#define __COMMON__

/**
 * @brief Common library 
 * 
 */
#include <Eigen/Core>
#include <Eigen/Dense>
namespace Common
{

    
static Eigen::Matrix3d 
VectorToMatrix(Eigen::Vector3d& v1,Eigen::Vector3d& v2,Eigen::Vector3d& v3)
{   
    Eigen::Matrix3d res;
    res <<  v1(0),v2(0),v3(0),
            v1(1),v2(1),v3(1),
            v1(2),v2(2),v3(2);
    return res;
} 


} // namespace name




#endif