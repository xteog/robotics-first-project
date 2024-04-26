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

Vector convertToOdom(const sensor_msgs::NavSatFix *, Vector);
Vector geodetic_to_ECEF(Vector);
Vector ECEF_to_ONOM(Vector, Vector);
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

    Vector reference;
    Vector prev_odom = {0, 0, 0};

public:
    GpsToOdom()
    {
        sub = n.subscribe("/fix", 1000, &GpsToOdom::navSatFixCallback, this);
        pub = n.advertise<nav_msgs::Odometry>("/gps_odom", 1);

        timer = n.createTimer(ros::Duration(1), &GpsToOdom::odomCallback, this);

        n.getParam("latitude_reference", reference.x);
        n.getParam("longitude_reference", reference.y);
        n.getParam("altitude_reference", reference.z);
    }

    void navSatFixCallback(const sensor_msgs::NavSatFix::ConstPtr &msg)
    {
        this->msg = *msg;
    }

    void odomCallback(const ros::TimerEvent &)
    {
        nav_msgs::Odometry msg;
        Vector odom, orientation;

        odom = convertToOdom(&(this->msg), reference);

        msg.pose.pose.position.x = odom.x;
        msg.pose.pose.position.y = odom.y;
        msg.pose.pose.position.z = odom.z;
        
        orientation = pointsDirection(prev_odom, odom);
        msg.pose.pose.orientation.z = orientation.x;
        msg.pose.pose.orientation.w = orientation.y;

        msg.header.frame_id = "world";

        prev_odom = odom;

        pub.publish(msg);
    }
};

Vector convertToOdom(const sensor_msgs::NavSatFix *msg, Vector reference)
{
    Vector geodetic, ref_ecef, ecef, result;

    geodetic.x = radians(msg->latitude);
    geodetic.y = radians(msg->longitude);
    geodetic.z = msg->altitude;

    ecef = geodetic_to_ECEF(geodetic);

    result = ECEF_to_ONOM(ecef, reference);

    return result;
}

Vector geodetic_to_ECEF(Vector geodetic)
{
    Vector result;
    float x = geodetic.x;
    float y = geodetic.y;
    float z = geodetic.z;
    float N, e_sqr;

    e_sqr = 1 - POLAR_RADIUS / EQUATORIAL_RADIUS * POLAR_RADIUS / EQUATORIAL_RADIUS;
    N = EQUATORIAL_RADIUS / sqrt(1 - e_sqr * sin(x));

    result.x = (N + z) * cos(x) * cos(y);
    result.y = (N + z) * cos(x) * sin(y);
    result.z = (N * (1 - e_sqr) + z) * sin(x);

    return result;
}

Vector ECEF_to_ONOM(Vector ecef, Vector reference)
{
    Vector result, ref_ecef;
    float x, y, z;
    float lat, lon;

    ref_ecef = geodetic_to_ECEF(reference);

    x = ecef.x - ref_ecef.x;
    y = ecef.y - ref_ecef.y;
    z = ecef.z - ref_ecef.z;

    lat = reference.x;
    lon = reference.y;

    result.x = -sin(lon) * x + cos(lon) * y;
    result.y = -sin(lat) * cos(lon) * x - sin(lat) * sin(lon) * y + cos(lat) * z;
    result.z = cos(lat) * cos(lon) * x + cos(lat) * sin(lon) * y + sin(lat) * z;

    return result;
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

Vector normalize(Vector v){
    Vector result;
    float d;

    d = sqrt(pow(v.x, 2) + pow(v.y, 2))

    result.x = v.x / d;
    result.y = v.y / d;

    return result
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "gps_to_odom");

    GpsToOdom node;

    ros::spin();

    return 0;
}