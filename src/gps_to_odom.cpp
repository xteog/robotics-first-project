#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "nav_msgs/Odometry.h"
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <cmath>

#define PI 3.14159265358979323846
#define EQUATORIAL_RADIUS 6378137
#define POLAR_RADIUS 6356752.3142

using namespace std;

struct Vector
{
    double x;
    double y;
    double z;
};

Vector convertToOdom(const sensor_msgs::NavSatFix *, Vector);
Vector geodetic_to_ECEF(Vector);
Vector ECEF_to_ONOM(Vector, Vector);
double radians(double);
geometry_msgs::Quaternion pointsDirection(Vector, Vector);
Vector normalize(Vector);

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
    int count;

public:
    GpsToOdom()
    {
        sub = n.subscribe("/fix", 1000, &GpsToOdom::navSatFixCallback, this);
        pub = n.advertise<nav_msgs::Odometry>("/gps_odom", 1);

        // Uncomment this if you want use the timer.
        // timer = n.createTimer(ros::Duration(0.1), &GpsToOdom::odomCallback, this);  

        n.getParam("latitude_reference", reference.x);
        n.getParam("longitude_reference", reference.y);
        n.getParam("altitude_reference", reference.z);

        reference.x = radians(reference.x);
        reference.y = radians(reference.y);

        count = 0;
    }

    void navSatFixCallback(const sensor_msgs::NavSatFix::ConstPtr &msg)
    {
        this->msg = *msg;

        // Comment this if you want to use the timer 
        odomCallback();
    }

    void odomCallback() {
        nav_msgs::Odometry msg;
        Vector odom;
        geometry_msgs::Quaternion orientation;

        odom = convertToOdom(&(this->msg), reference);

        msg.pose.pose.position.x = odom.x;
        msg.pose.pose.position.y = odom.y;
        msg.pose.pose.position.z = odom.z;
        
        orientation = pointsDirection(prev_odom, odom);
        msg.pose.pose.orientation = orientation;

        msg.header.frame_id = "world";

        if (count > 100) {
            prev_odom = odom;
            count = 0;
        }
        count++;

        msg.header.stamp = this->msg.header.stamp;
        pub.publish(msg);
    }

    void odomCallback(const ros::TimerEvent &)
    {
        odomCallback();
    }
};

Vector convertToOdom(const sensor_msgs::NavSatFix *msg, Vector reference)
{
    Vector geodetic, ref_ecef, ecef, result;

    geodetic.x = radians((double) msg->latitude);
    geodetic.y = radians((double) msg->longitude);
    geodetic.z = msg->altitude;

    ecef = geodetic_to_ECEF(geodetic);

    result = ECEF_to_ONOM(ecef, reference);

    return result;
}

Vector geodetic_to_ECEF(Vector geodetic)
{
    Vector result;
    double x = geodetic.x;
    double y = geodetic.y;
    double z = geodetic.z;
    double N, e_sqr;

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
    double x, y, z;
    double lat, lon;

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

geometry_msgs::Quaternion pointsDirection(Vector v, Vector s)
{
    geometry_msgs::Quaternion orientation;
    tf2::Quaternion quat;
    
    double dx = s.x - v.x;
    double dy = s.y - v.y;
    
    quat.setRPY(0, 0, atan2(dy, dx));

    tf2::convert(quat, orientation);

    return orientation;
}

double radians(double degrees)
{
    return degrees * (PI / 180);
}

Vector normalize(Vector v){
    Vector result;
    double d;

    d = sqrt(pow(v.x, 2) + pow(v.y, 2));

    if (d == 0) {
        return v;
    }

    result.x = v.x / d;
    result.y = v.y / d;

    return result;
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "gps_to_odom");

    GpsToOdom node;

    ros::spin();

    return 0;
}