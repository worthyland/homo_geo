#include <homography_geometric_control/HomographyGeometricControl.h>
#include <mavros_interaction/MavrosInteraction.h>
#include <so3_control/SO3Control.h>
#include<typeinfo>
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
    // double ControlRate;
    // nhParam.param("Control_rate",ControlRate,60.0);
    ros::Rate rate(attitudeControllor.GetControlRate());
    while(ros::ok()){

        //cout << controllor.GetControlRate() <<endl;
        //uavInfo.ShowUavState(5);
        outLoop(uavInfo.GetQuadrotor());
        attitudeControllor(outLoop.GetRDesired(),outLoop.GetQuadrotor());
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}