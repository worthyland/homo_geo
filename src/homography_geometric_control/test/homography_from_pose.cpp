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


homo_msgs::HomographyResult HomographyCalculationPub(const Quadrotor::state& curState,Eigen::Vector3d& pos_desire,Eigen::Matrix<double,1,3>& n,double d)
{
    Eigen::Matrix3d ned_R_enu;
    ned_R_enu << 1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, -1.0;
    Eigen::Vector3d pos_local(curState.pos);

    Eigen::Vector3d pos_error(pos_local - pos_desire);
    pos_error = ned_R_enu*pos_error;
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();;
    R = curState.orientation.toRotationMatrix();
    cout << "R:"<<R <<endl;
    R = ned_R_enu.transpose()*R*ned_R_enu;
    cout << "R2:"<<R <<endl;
    // cout << "posError:"<<pos_error <<endl;
    // cout << "R:"<<R <<endl;
    // Eigen::Vector3d ea1 = R.eulerAngles(2,1,0);
    // Eigen::Vector3d ea12 = ToEulerAngles(curState.orientation);
    // cout << "ea1:"<<ea1 <<endl;
    // cout << "ea12:"<<ea12 <<endl;

    Eigen::Matrix3d homoTmp;

    
    homoTmp = R.transpose() - (R.transpose() * pos_error * n) / d;
    cout << "homoTmp:"<<homoTmp <<endl;

    homo_msgs::HomographyResult res;
    int cout = 0;
    for(int i = 0;i<3;i++){
        for(int j = 0;j<3;j++){
                res.homography[cout] =homoTmp(i,j);
                cout++;
        }  
    }
    return res;
}


int main(int argc,char *argv[])
{
    ros::init(argc, argv, "homography_from_pose_node");
    ros::NodeHandle nh;
    ros::Rate rate(60.0);
    MavrosInteraction uavInfo(nh);
    ros::Publisher resultsPub = nh.advertise<homo_msgs::HomographyResult>("homography_pose", 1);

    Quadrotor::state curUavState;
    homo_msgs::HomographyResult homoMatrix;
    Eigen::Vector3d pos_desire;
    pos_desire << 0,0,4;
    Eigen::Matrix<double,1,3> n;
    n << 0,0,1;
    double d = 4.0;
    while(ros::ok()){
        curUavState = uavInfo.GetState();
        uavInfo.ShowUavState(5);
        homoMatrix = HomographyCalculationPub(curUavState,pos_desire,n,d);
        resultsPub.publish(homoMatrix);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;

}