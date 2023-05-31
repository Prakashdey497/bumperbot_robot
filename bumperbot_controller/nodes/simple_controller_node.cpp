#include <ros/ros.h>
#include "bumperbot_controller/simple_controller.h"


int main(int argc, char * argv[])
{
    ros::init(argc, argv, "simple_controller");
    ros::NodeHandle nh;
    ros::NodeHandle pnh_("~");

    double wheel_radius;
    double wheel_seperation;
    pnh_.getParam("wheel_radius",wheel_radius);
    pnh_.getParam("wheel_seperation",wheel_seperation);

    SimpleController controller(nh,wheel_radius,wheel_seperation);
    ros::spin();
    return 0;
}
