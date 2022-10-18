#include <homography_geometric_control/HomographyGeometricControl.h>
#include <mavros_interaction/MavrosInteraction.h>
#include <so3_control/SO3Control.h>
#include <std_msgs/Float32MultiArray.h>

using namespace Control;
using namespace  std;
using namespace  HomographyGeometricControl;


int main(int argc, char *argv[]){

    ros::init(argc, argv, "homo_geo_node");
    ros::NodeHandle nh;
    ros::NodeHandle nhParam("~");

    MavrosInteraction uavInfo(nh,nhParam);//与mavros交互，获取无人机状态 ref_X_base
    HomographyGeometric outLoop(nh,nhParam);
    SO3Control attitudeControllor(nh,nhParam);

    ros::Publisher recordPub = nh.advertise<std_msgs::Float32MultiArray>
        ("record",10);


    ros::Rate rate(outLoop.GetControlRate());
    std::cout<<"wait for the connected!"<<std::endl;

    while(ros::ok() && !uavInfo.GetCurrentControlState().connected){
        std::cout<<"connecting.....: "<<std::endl;
        ros::spinOnce();
        rate.sleep();
    }


    while(ros::ok()){
        
        // uavInfo.ShowUavState(5);//通过mavros读取的
        outLoop(uavInfo.GetQuadrotor());
        attitudeControllor(outLoop.GetRDesired(),outLoop.GetOmegaDesired(),outLoop.GetQuadrotor());
        Eigen::Matrix3d enu_R_ned;
        enu_R_ned << 1.0, 0.0, 0.0,0.0, -1.0, 0.0, 0.0, 0.0, -1.0;
        //力矩计算表示在base_link_ned，推力计算为正，不需要转换
        uavInfo.ActuatorPub(outLoop.GetThrust(), attitudeControllor.GetTorque(),true);
        // uavInfo.ActuatorPub();

        std_msgs::Float32MultiArray outputRecord;
        outputRecord.data.push_back(attitudeControllor.GetTorque()(0));
        outputRecord.data.push_back(attitudeControllor.GetTorque()(1));
        outputRecord.data.push_back(attitudeControllor.GetTorque()(2));
        recordPub.publish(outputRecord);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}