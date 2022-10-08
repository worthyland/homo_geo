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

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
