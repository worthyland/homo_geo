#include <homography_geometric_control/HomographyGeometricControl.h>

#include <mavros_interaction/MavrosInteraction.h>
using namespace Control;





int main(int argc, char *argv[]){

    ros::init(argc, argv, "homo_geo_node");
    ros::NodeHandle nh;

    MavrosInteraction test(nh);

    ros::Rate rate(60.0);
    while(ros::ok()){
        test.ShowUavState(5);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}