#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>


#include <ros/ros.h>

#include <uav_state/UAVState.h>
int main(int argc, char *argv[]){

    ros::init(argc, argv, "so3_control_test");
    ros::NodeHandle nh;

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(60.0);

    
    while(ros::ok()){
        
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
