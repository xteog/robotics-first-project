#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_broadcaster.h"

class tf_sub_pub
{
private:
    ros::NodeHandle n;
    tf::TransformBroadcaster br;
    ros::Subscriber sub;

public:
    tf_sub_pub()
    {
        sub = n.subscribe("/odom", 1000, &tf_sub_pub::callback, this);
        // Penso sia /input_odom non /odom
    }

    void callback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        // Access pose from the Odometry message
        // geometry_msgs::PoseWithCovariance pose = msg->pose;
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z));
        transform.setRotation(tf::Quaternion(
            msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w));
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "wheel_odom"));
    }
};

int main(int argc, char **argv)
{
    // Node name: odom_to_tf
    ros::init(argc, argv, "odom_to_tf");
    tf_sub_pub my_tf_sub_bub;
    ros::spin();
    return 0;
}
