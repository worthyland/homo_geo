#include <homography_geometric_control/HomographyGeometricControl.h>
#include <mavros_interaction/MavrosInteraction.h>
#include <so3_control/SO3Control.h>

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




    ros::Rate rate(200);
    std::cout<<"wait for the connected!"<<std::endl;

    while(ros::ok() && !uavInfo.GetCurrentControlState().connected){
        std::cout<<"connecting.....: "<<std::endl;
        ros::spinOnce();
        rate.sleep();
    }


    while(ros::ok()){

        //uavInfo.ShowUavState(5);
        outLoop(uavInfo.GetQuadrotor());
        attitudeControllor(outLoop.GetRDesired(),outLoop.GetOmegaDesired(),outLoop.GetQuadrotor());
        uavInfo.ActuatorPub(outLoop.GetThrust(),attitudeControllor.GetTorque());
        // uavInfo.ActuatorPub();

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}