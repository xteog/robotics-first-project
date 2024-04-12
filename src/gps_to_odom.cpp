#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "nav_msgs/Odometry.h"

#include <cmath>

#define PI 3.14159265358979323846
#define EQUATORIAL_RADIUS 6378137
#define POLAR_RADIUS 6356752.3142 

using namespace std;

void convertToOdom(const sensor_msgs::NavSatFix *, float *, float *);
void geodetic_to_ECEF(float *, float *);
void ECEF_to_ONOM(float *, float *, float *, float *);
float radians(float);

class gps_to_odom_node
{
    sensor_msgs::NavSatFix msg;

private:
    ros::NodeHandle n;

    ros::Subscriber sub;
    ros::Publisher pub;
    ros::Timer timer;

public:
    gps_to_odom_node()
    {
        sub = n.subscribe("/fix", 1, &gps_to_odom_node::navSatFixCallback, this);
        pub = n.advertise<nav_msgs::Odometry>("/gps_odom", 1);

        timer = n.createTimer(ros::Duration(1), &gps_to_odom_node::odomCallback, this);
    }

    void navSatFixCallback(const sensor_msgs::NavSatFix::ConstPtr &msg)
    {
        this->msg = *msg;
    }

    void odomCallback(const ros::TimerEvent &)
    {
        nav_msgs::Odometry msg;
        float odom[3];
        float reference[3] = {0};

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
    float geodetic[3], ref_ecef[3];
    float ecef[3] = {0};

    geodetic[0] = radians(msg->latitude);
    geodetic[1] = radians(msg->longitude);
    geodetic[2] = radians(msg->altitude);

    geodetic_to_ECEF(geodetic, ecef);
    geodetic_to_ECEF(reference, ref_ecef);

    ECEF_to_ONOM(geodetic, ecef, ref_ecef, result);
}

void geodetic_to_ECEF(float *geodetic, float *result)
{
    float x = geodetic[0];
    float y = geodetic[1];
    float z = geodetic[2];
    float N, e_sqr;

    e_sqr = 1 - POLAR_RADIUS / EQUATORIAL_RADIUS * POLAR_RADIUS / EQUATORIAL_RADIUS;
    N = EQUATORIAL_RADIUS / sqrt(1 - e_sqr * sin(x));

    result[0] = (N + z) * cos(x) * cos(y);
    result[1] = (N + z) * cos(x) * sin(y);
    result[2] = (N * (1 - e_sqr) + z) * sin(x);
}

void ECEF_to_ONOM(float *geodetic, float *ecef, float *reference, float *result)
{
    float x = ecef[0] - reference[0];
    float y = ecef[1] - reference[1];
    float z = ecef[2] - reference[2];

    result[0] = -sin(geodetic[1]) * x + cos(geodetic[1]) * y;
    result[1] = -sin(geodetic[0]) * cos(geodetic[1]) * x - sin(geodetic[0]) * sin(geodetic[1]) * y + cos(geodetic[0]) * z;
    result[2] = cos(geodetic[0]) * cos(geodetic[1]) * x + cos(geodetic[0]) * sin(geodetic[1]) * y + sin(geodetic[0]) * z;
}

float radians(float degrees)
{
    return degrees * (PI / 180);
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "gps_to_odom");

    gps_to_odom_node node;

    ros::spin();

    return 0;
}