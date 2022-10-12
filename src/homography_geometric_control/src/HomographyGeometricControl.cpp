#include <homography_geometric_control/HomographyGeometricControl.h>

namespace HomographyGeometricControl
{

HomographyGeometric::HomographyGeometric(const ros::NodeHandle& nh,const ros::NodeHandle& nhParam):nh_(nh),nhParam_(nhParam)
{
    homographySub = nh_.subscribe("homography_pose",1,&HomographyGeometric::HomographyCallback,this);

    //参数初始化
    nhParam_.param("ControlGain/c",controlGain_.c,1.0);
    nhParam_.param("YawDesired",yawDesired_,0.0);
    b1c_ << cos(yawDesired_),sin(yawDesired_),0;
    controlGain_.Kv = Eigen::Matrix3d::Identity();
    nhParam_.param("ControlGain/Kvx",controlGain_.Kv(0,0),1.0);
    nhParam_.param("ControlGain/Kvy",controlGain_.Kv(1,1),1.0);
    nhParam_.param("ControlGain/Kvz",controlGain_.Kv(2,2),1.0);


    homography_ = Eigen::Matrix3d::Identity();
    homographyVirtual_ = Eigen::Matrix3d::Identity();
    RZ_ = Eigen::Matrix3d::Identity();
    RY_ = Eigen::Matrix3d::Identity();
    RX_ = Eigen::Matrix3d::Identity();
    e1_ = Eigen::Vector3d::Zero();
    e2_ = Eigen::Vector3d::Zero();
    FVitual_ = Eigen::Vector3d::Zero();
    RDesired_ = Eigen::Matrix3d::Identity();
    dot_RDesired_ = Eigen::Matrix3d::Zero();
    omegaDesired_ = Eigen::Vector3d::Zero();
    mStar_ << 0,0,1;
    axisZ_ << 0,0,1;
    thrust_ = curUavState_.GetMass() * curUavState_.GetGravity();

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
    homographyVirtual_ = RY_ * RX_ * homography_;
    // std::cout << "homography_:" << std::endl;
    //Common::ShowVal("homographyVirtual_:",homographyVirtual_);
}

void 
HomographyGeometric::SetCurUavState(const Control::Quadrotor& val)
{
    curUavState_ = val;
}

const Eigen::Vector3d
HomographyGeometric::UpdateError1()
{
    Eigen::Vector3d res;
    res = (Eigen::Matrix3d::Identity() - homographyVirtual_) * mStar_;
    return res;
}

const Eigen::Vector3d
HomographyGeometric::UpdateError2()
{
    Eigen::Vector3d res;
    res = e1_ + (RY_*RX_*curUavState_.GetVel())/controlGain_.c;
    
    return res;
}

const Eigen::Vector3d 
HomographyGeometric::UpdateFVitual()
{
    Eigen::Vector3d res;
    res = - controlGain_.Kv * e2_;
    return res;
}

const double 
HomographyGeometric::UpdateThrust()
{
    double res;
    res = - (FVitual_ - curUavState_.GetMass() * curUavState_.GetGravity() * axisZ_).transpose()*(RY_*RX_*axisZ_);
    return res;
}

const Eigen::Matrix3d 
HomographyGeometric::UpdateRotationDesired()
{
    Eigen::Matrix3d res;
    Eigen::Vector3d b1d,b2d,b3d;
    Eigen::Vector3d dot_b1d,dot_b2d,dot_b3d;
    Eigen::Vector3d dot_b1c;
    Eigen::Vector3d tmp,dot_tmp;//tmp代表临时变量 局部变量
    dot_b1c << -sin(yawDesired_),cos(yawDesired_),0;
    b3d = RZ_ * (- FVitual_/curUavState_.GetMass() + curUavState_.GetGravity()*axisZ_ );
    tmp = b3d;
    //限幅处理
    //
    b3d.normalize();//归一化
    b2d = b3d.cross(b1c_);//叉乘
    b2d.normalize();
    b1d = b2d.cross(b3d);
    b1d.normalize();

    // Eigen::Vector3d debugVal1,debugVal2,debugVal3;//调试变量  仅调试用
 

    dot_tmp = RZ_*Common::MatrixHat(curUavState_.GetOmega()(2)*axisZ_)*(- FVitual_/curUavState_.GetMass() + curUavState_.GetGravity()*axisZ_ )
                + RZ_*(controlGain_.Kv/(curUavState_.GetMass()*controlGain_.c) * 
                 ( -Common::MatrixHat(curUavState_.GetOmega()(2)*axisZ_)*e1_ + FVitual_/(curUavState_.GetMass()*controlGain_.c) + controlGain_.c *(e2_-e1_)));
    
    dot_tmp = dot_tmp/tmp.norm();
    dot_b3d = b3d.cross(dot_tmp).cross(b3d);
    dot_b2d = b2d.cross((dot_b3d.cross(b1c_) + dot_b1c.cross(b3d))/b3d.cross(b1c_).norm()).cross(b2d);
    dot_b1d = dot_b2d.cross(b3d) + dot_b3d.cross(b2d);

    res = Common::VectorToMatrix(b1d,b2d,b3d);
    dot_RDesired_ = Common::VectorToMatrix(dot_b1d,dot_b2d,dot_b3d);
    Common::ShowVal("dot_tmp",dot_tmp);
    Common::ShowVal("dot_b3d",dot_b3d);
    Common::ShowVal("dot_b2d",dot_b2d);
    Common::ShowVal("dot_b1d",dot_b1d);
    return res;
}


const Eigen::Vector3d 
HomographyGeometric::UpdateOmegaDesired()
{
    Eigen::Vector3d res;
    res = Common::MatrixHatInv(RDesired_.transpose()*dot_RDesired_);
    return res;
}

const Eigen::Vector3d& 
HomographyGeometric::GetOmegaDesired()const
{
    return omegaDesired_;
}
void 
HomographyGeometric::operator() (const Control::Quadrotor& curUavState)
{
    //curUavState.ShowState();
    Eigen::Matrix3d enu_R_ned;
    enu_R_ned << 1.0, 0.0, 0.0,
                 0.0, -1.0, 0.0, 
                 0.0, 0.0, -1.0;
    SetCurUavState(curUavState);//外部传的state为相对与世界坐标系 
    //相对坐标系转换
    curUavState_.RotationFrameTransform(enu_R_ned.transpose(),enu_R_ned);
    curUavState_.Vector3dFrameTransform(enu_R_ned.transpose());

    //更新内部变量
    Eigen::Vector3d angle(curUavState_.GetEulerAngle());
    RZ_ << cos(angle(0)),-sin(angle(0)),0,sin(angle(0)),cos(angle(0)),0,0,0,1;
    RY_ << cos(angle(1)),0,sin(angle(1)),0,1,0,-sin(angle(1)),0,cos(angle(1));
    RX_ << 1,0,0,0,cos(angle(2)),-sin(angle(2)),0,sin(angle(2)),cos(angle(2));

    e1_ = UpdateError1();//e1 = (I-Hv)*m

    e2_ = UpdateError2();//e2 = e1 + Vv/c

    FVitual_ = UpdateFVitual();//Fv = -Kv * e2

    thrust_ = UpdateThrust();//T = - [Fv - m*g*ez]' (RY*RX *ez);
    RDesired_ = UpdateRotationDesired();//FVitual_ -> RDesired_
    omegaDesired_ = UpdateOmegaDesired();

    // Common::ShowVal("RZ_",RZ_);
    // Common::ShowVal("RY_",RY_);
    // Common::ShowVal("RX_",RX_);
    ShowInternal(8);
}

const Control::Quadrotor& 
HomographyGeometric::GetQuadrotor()const
{
    return curUavState_;
}

const Eigen::Matrix3d& 
HomographyGeometric::GetRDesired()const
{
    return RDesired_;
}

void 
HomographyGeometric::ShowInternal(int num) const
{
    std::cout << "-------------------HomographyFGeometric-------------------" <<std::endl;
    Common::ShowVal("e1_",e1_,num);
    Common::ShowVal("e2_",e2_,num);
    Common::ShowVal("FVitual_",FVitual_,num);
    Common::ShowVal("thrust_",thrust_,num);
    Common::ShowVal("RDesired_",RDesired_,num);
    Common::ShowVal("dot_RDesired_",dot_RDesired_,num);
    Common::ShowVal("omegaDesired_",omegaDesired_,num);

}

}