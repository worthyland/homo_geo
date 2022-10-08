#include <mavros_interaction/MavrosInteraction.h>
using namespace Control;


void
IMUUpdate(const sensor_msgs::Imu::ConstPtr& msg)
{

}


int main(int argc, char *argv[]){

    ros::init(argc, argv, "mavros_interaction_test_node");
 

    MavrosInteraction test;

    ros::Rate rate(60.0);
    while(ros::ok()){
         Eigen::Vector3d tmp1 = test.uav_.GetAcc();
         std::cout <<"acc:" <<tmp1(0) << ","<<tmp1(1)  << ","<<tmp1(2)  <<std::endl;
         Eigen::Vector3d tmp2 = test.uav_.GetOmega();
         std::cout <<"omage" <<tmp2(0) << ","<<tmp2(1)  << ","<<tmp2(2)  <<std::endl;
         Eigen::Quaterniond tmp = test.uav_.GetOrientation();
         std::cout <<"Orientation:" <<tmp.w() << ","<<tmp.x() << ","<<tmp.y()<<","<<tmp.z() <<std::endl;
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
