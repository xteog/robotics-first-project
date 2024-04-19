#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "nav_msgs/Odometry.h"

#include <cmath>

#define PI 3.14159265358979323846
#define EQUATORIAL_RADIUS 6378137
#define POLAR_RADIUS 6356752.3142

using namespace std;

struct Vector
{
    float x;
    float y;
    float z;
};

void convertToOdom(const sensor_msgs::NavSatFix *, Vector *, Vector *);
void geodetic_to_ECEF(Vector *, Vector *);
void ECEF_to_ONOM(Vector *, Vector *, Vector *, Vector *);
float radians(float);
Vector pointsDirection(Vector, Vector);

class GpsToOdom
{
    sensor_msgs::NavSatFix msg;

private:
    ros::NodeHandle n;

    ros::Subscriber sub;
    ros::Publisher pub;
    ros::Timer timer;

    bool first;
    Vector reference;
    Vector prev_odom = {0, 0, 0};

public:
    GpsToOdom()
    {
        first = true;

        sub = n.subscribe("/fix", 1000, &GpsToOdom::navSatFixCallback, this);
        pub = n.advertise<nav_msgs::Odometry>("/gps_odom", 1);

        timer = n.createTimer(ros::Duration(1), &GpsToOdom::odomCallback, this);
    }

    void navSatFixCallback(const sensor_msgs::NavSatFix::ConstPtr &msg)
    {
        this->msg = *msg;
    }

    void odomCallback(const ros::TimerEvent &)
    {
        nav_msgs::Odometry msg;
        Vector odom, orientation;

        if (first)
        {
            reference.x = radians(this->msg.latitude);
            reference.y = radians(this->msg.longitude);
            reference.z = radians(this->msg.altitude);
            first = false;
        }

        convertToOdom(&(this->msg), &reference, &odom);

        msg.pose.pose.position.x = odom.x;
        msg.pose.pose.position.y = odom.y;
        msg.pose.pose.position.z = odom.z;
        orientation = pointsDirection(prev_odom, odom);
        msg.pose.pose.orientation.w = orientation.x;
        msg.pose.pose.orientation.z = orientation.y;

        prev_odom = odom;

        pub.publish(msg);
    }
};

void convertToOdom(const sensor_msgs::NavSatFix *msg, Vector *reference, Vector *result)
{
    Vector geodetic, ref_ecef, ecef;

    geodetic.x = radians(msg->latitude);
    geodetic.y = radians(msg->longitude);
    geodetic.z = radians(msg->altitude);

    geodetic_to_ECEF(&geodetic, &ecef);
    geodetic_to_ECEF(reference, &ref_ecef);

    ECEF_to_ONOM(&geodetic, &ecef, &ref_ecef, result);
}

void geodetic_to_ECEF(Vector *geodetic, Vector *result)
{
    float x = geodetic->x;
    float y = geodetic->y;
    float z = geodetic->z;
    float N, e_sqr;

    e_sqr = 1 - POLAR_RADIUS / EQUATORIAL_RADIUS * POLAR_RADIUS / EQUATORIAL_RADIUS;
    N = EQUATORIAL_RADIUS / sqrt(1 - e_sqr * sin(x));

    result->x = (N + z) * cos(x) * cos(y);
    result->y = (N + z) * cos(x) * sin(y);
    result->z = (N * (1 - e_sqr) + z) * sin(x);
}

void ECEF_to_ONOM(Vector *geodetic, Vector *ecef, Vector *reference, Vector *result)
{
    float x = ecef->x - reference->x;
    float y = ecef->y - reference->y;
    float z = ecef->z - reference->z;

    float lat = geodetic->x;
    float lon = geodetic->y;

    result->x = -sin(lon) * x + cos(lon) * y;
    result->y = -sin(lat) * cos(lon) * x - sin(lat) * sin(lon) * y + cos(lat) * z;
    result->z = cos(lat) * cos(lon) * x + cos(lat) * sin(lon) * y + sin(lat) * z;
}

Vector pointsDirection(Vector v, Vector s)
{
    float angle;
    Vector orientation;

    orientation.x = s.x - v.x;
    orientation.y = s.y - v.y;

    return orientation;
}

float radians(float degrees)
{
    return degrees * (PI / 180);
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "gps_to_odom");

    GpsToOdom node;

    ros::spin();

    return 0;
}