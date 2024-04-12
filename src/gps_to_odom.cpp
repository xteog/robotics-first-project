#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "nav_msgs/Odometry.h"

void convertToOdom(const sensor_msgs::NavSatFix *, float *, float *);
void latLon_to_ECEF(float *);
void ECEF_to_ENU(float *);
void ENU_to_ONOM(float *, float *);

class gps_to_odom_node
{
    sensor_msgs::NavSatFix msg;

private:
    ros::NodeHandle n;

    ros::Subscriber sub;
    ros::Publisher pub;
    ros::Timer timer;

public:
    gps_to_odom()
    {
        sub = n.subscribe("/fix", 1, &gps_to_odom::navSatFixCallback, this);
        pub = n.advertise<nav_msgs::Odometry>("/gps_odom", 1);

        timer = n.createTimer(ros::Duration(1), &gps_to_odom::odomCallback, this);
    }

    void navSatFixCallback(const sensor_msgs::NavSatFix::ConstPtr &msg)
    {
        this->msg = *msg;
    }

    void odomCallback(const ros::TimerEvent &)
    {
        nav_msgs::Odometry msg;
        float odom[3];
        float reference[3];

        convertToOdom(&(this->msg), reference, odom);

        msg.pose.pose.position.x = odom[0];
        msg.pose.pose.position.y = odom[1];
        msg.pose.pose.position.z = odom[2];
        // msg.pose.pose.orientation = 0;

        pub.publish(msg);
    }
};

void convertToOdom(const sensor_msgs::NavSatFix *msg, float *reference, float *result)
{
    result[0] = msg->latitude;
    result[1] = msg->longitude;
    result[2] = msg->altitude;

    latLon_to_ECEF(result);
    ECEF_to_ENU(result);
    ENU_to_ONOM(reference, result);
}

void latLon_to_ECEF(float *arr) {}

void ECEF_to_ENU(float *arr) {}

void ENU_to_ONOM(float *reference, float *arr) {}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "gps_to_odom");

    gps_to_odom_node node;

    ros::spin();

    return 0;
}