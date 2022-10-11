#include <mavros_interaction/MavrosInteraction.h>
#include <homo_msgs/HomographyResult.h>

using namespace Control;
using namespace std;



Eigen::Vector3d ToEulerAngles(const Eigen::Quaterniond& q) 
{

    Eigen::Vector3d angles;
 
    // roll (x-axis rotation)
    double sinr_cosp = 2 * (q.w() * q.x() + q.y() * q.z());
    double cosr_cosp = 1 - 2 * (q.x() * q.x() + q.y() * q.y());
    angles(2) = std::atan2(sinr_cosp, cosr_cosp);
 
    // pitch (y-axis rotation)
    double sinp = 2 * (q.w() * q.y() - q.z() * q.x());
    if (std::abs(sinp) >= 1)
        angles(1) = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        angles(1) = std::asin(sinp);
 
    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w() * q.z() + q.x() * q.y());
    double cosy_cosp = 1 - 2 * (q.y() * q.y() + q.z() * q.z());
    angles(0) = std::atan2(siny_cosp, cosy_cosp);
 
    return angles;
}


homo_msgs::HomographyResult HomographyCalculationPub(const Quadrotor::state& curState,Eigen::Vector3d& ref_posDesire_base,Eigen::Vector3d& n,double d)
{
    Eigen::Matrix3d enu_R_ned;
    enu_R_ned << 1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, -1.0;
    Eigen::Vector3d ref_pos_base(curState.pos);//

    Eigen::Vector3d ref_posError_base(ref_pos_base - ref_posDesire_base);
    Eigen::Vector3d refned_posError_basened(enu_R_ned*ref_posError_base);
    cout << "refned_posError_basened:"<<refned_posError_basened <<endl;

    Eigen::Matrix3d ref_R_base = Eigen::Matrix3d::Identity();
    ref_R_base = curState.orientation.toRotationMatrix();
    // cout << "R:"<<enu_R_ned <<endl;
    Eigen::Matrix3d refned_R_basened = Eigen::Matrix3d::Identity();;
    refned_R_basened = enu_R_ned.transpose()*ref_R_base*enu_R_ned;
    // cout << "R2:"<<enu_R_ned <<endl;
     

    Eigen::Matrix3d refned_homoTmp_basened;
    refned_homoTmp_basened = refned_R_basened.transpose() - (refned_R_basened.transpose() * refned_posError_basened * n.transpose()) / d;
    
    cout << "refned_homoTmp_basened:"<<refned_homoTmp_basened <<endl;

    homo_msgs::HomographyResult res;

    for(int i = 0;i<3;i++){
        for(int j = 0;j<3;j++){
                res.homography[i*3 + j] =refned_homoTmp_basened(i,j);
                
        }  
    }
    return res;
}


int main(int argc,char *argv[])
{
    ros::init(argc, argv, "homography_publish_node");
    ros::NodeHandle nh;
    ros::NodeHandle nhParam("~");//加“～”才能读取

    
    ros::Rate rate(60.0);
    MavrosInteraction uavInfo(nh,nhParam);
    ros::Publisher resultsPub = nh.advertise<homo_msgs::HomographyResult>("homography_pose", 1);



    Quadrotor::state curUavState;
    homo_msgs::HomographyResult homoMatrix;
    Eigen::Vector3d posDesire;
    posDesire << 0,0,4;
    Eigen::Vector3d n;
    n << 0,0,1;
    double d = 4.0;

    //初始化posDesire
    nhParam.param("posDesire/x",posDesire(0),0.0);
    nhParam.param("posDesire/y",posDesire(1),0.0);
    nhParam.param("posDesire/z",posDesire(2),1.0);
    //初始化n
    nhParam.param("n/x",n(0),0.0);
    nhParam.param("n/y",n(1),0.0);
    nhParam.param("n/z",n(2),1.0);
    //初始化d
    nhParam.param("d",d,1.0);
    while(ros::ok()){
        
        curUavState = uavInfo.GetState();
        //uavInfo.ShowUavState(5);
        homoMatrix = HomographyCalculationPub(curUavState,posDesire,n,d);
        resultsPub.publish(homoMatrix);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;

}