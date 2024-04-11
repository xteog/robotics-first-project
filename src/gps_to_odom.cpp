#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
//#include "nav_msgs/Odometry.h"

void navSatFixCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
    ROS_INFO("Latitude: %f", msg->latitude);
    ROS_INFO("Longitude: %f", msg->longitude);
    ROS_INFO("Altitude: %f", msg->altitude);
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "gps_to_odom");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("fix", 10, navSatFixCallback);

    ros::spin();

    /*
    ros::Publisher chatter_pub = n.advertise<nav_msgs::Odometry>("gps_odom", 1000);

    ros::Rate loop_rate(10);

    while (ros::ok())
    {

        chatter_pub.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();
    }
    */

    return 0;
}