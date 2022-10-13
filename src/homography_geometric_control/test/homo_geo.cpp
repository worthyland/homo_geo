#include <homography_geometric_control/HomographyGeometricControl.h>
#include <mavros_interaction/MavrosInteraction.h>
#include <so3_control/SO3Control.h>
#include<typeinfo>
using namespace Control;
using namespace  std;
using namespace  HomographyGeometricControl;

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

int main(int argc, char *argv[]){

    ros::init(argc, argv, "homo_geo_node");
    ros::NodeHandle nh;
    ros::NodeHandle nhParam("~");

    //MavrosInteraction uavInfo(nh,nhParam);//与mavros交互，获取无人机状态 ref_X_base
    // HomographyGeometric outLoop(nh,nhParam);
    // SO3Control attitudeControllor(nh,nhParam);
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher mix_pub = nh.advertise<mavros_msgs::ActuatorControl>
        ("mavros/actuator_control",10);
    mavros_msgs::ActuatorControl mix_u;  

    mix_u.group_mix = 0;
    mix_u.controls[0] = 0.0;
    mix_u.controls[1] = 0.0;
    mix_u.controls[2] = 0.0;
    mix_u.controls[3] = std::max(0.0, std::min(0.95, 0.567));
    mix_u.controls[4] = 0.0;
    mix_u.controls[5] = 0.0;
    mix_u.controls[6] = 0.0;
    mix_u.controls[7] = -1.0;

    ros::Rate rate(200);
    std::cout<<"wait for the connected!"<<std::endl;

    while(ros::ok() && !current_state.connected){
        std::cout<<"connecting.....: "<<std::endl;
        ros::spinOnce();
        rate.sleep();
    }


    while(ros::ok()){

        //uavInfo.ShowUavState(5);
        // outLoop(uavInfo.GetQuadrotor());
        // attitudeControllor(outLoop.GetRDesired(),outLoop.GetOmegaDesired(),outLoop.GetQuadrotor());
        // uavInfo.ActuatorPub(attitudeControllor.GetTorque(),outLoop.GetThrust());
        // uavInfo.ActuatorPub();

        // mix_pub.publish(mix_u);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}