#include <homography_geometric_control/HomographyGeometricControl.h>

namespace HomographyGeometricControl
{

HomographyGeometric::HomographyGeometric(const ros::NodeHandle& nh,const ros::NodeHandle& nhParam):nh_(nh),nhParam_(nhParam)
{
    homographySub = nh_.subscribe("homography_pose",1,&HomographyGeometric::HomographyCallback,this);

    //参数初始化
    homography_ = Eigen::Matrix3d::Identity();
    homographyCallbackState = false;
    
}

HomographyGeometric::~HomographyGeometric()
{
}


void 
HomographyGeometric::HomographyCallback(const homo_msgs::HomographyResult::ConstPtr& msg)
{
    homographyCallbackState = true;
    for(int i=0;i<3;++i){
        for(int j=0;j<3;++j){
            homography_(i,j) = msg->homography[i*3+j];
        }
    }
    // std::cout << "homography_:" << std::endl;
    // std::cout << homography_ << std::endl;
}

void 
HomographyGeometric::UpdateCurUavState(const Control::Quadrotor& val)
{
    curUavState_ = val;
}

const Eigen::Vector3d& 
HomographyGeometric::GetError1(const Eigen::Matrix3d& homography)
{
    Eigen::Vector3d res = Eigen::Vector3d::Zero();
    //res = Eigen::Matrix3d::Identity() - homography

}

void 
HomographyGeometric::operator() (const Control::Quadrotor& curUavState)
{
    curUavState_ = FrameTransform(curUavState);//外部传的state为相对与世界坐标系 转化为论文所定义坐标系下
    curUavState_.ShowState();
    



}

Control::Quadrotor 
HomographyGeometric::FrameTransform(const Control::Quadrotor& curUavState)
{
    Control::Quadrotor  curUavStateTmp = curUavState;

    return curUavStateTmp;
}

}