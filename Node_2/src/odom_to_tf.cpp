#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_broadcaster.h"

class tf_sub_pub {
public:
    tf_sub_pub() {
        sub = n.subscribe("/odom", 1000, &tf_sub_pub::callback, this);
    }


    void callback(const nav_msgs::Odometry::ConstPtr& msg) {
        // Access pose from the Odometry message
        geometry_msgs::Pose pose = msg->pose.pose;
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z));
        transform.setRotation(tf::Quaternion(
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w));
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "wheel_odom"));
    }

private:
    ros::NodeHandle n;
    tf::TransformBroadcaster br;
    ros::Subscriber sub;
};


int main(int argc, char** argv) {
    // Node name: odom_to_tf
    ros::init(argc, argv, "odom_to_tf"); 
    tf_sub_pub my_tf_sub_bub;
    ros::spin();
    return 0;
}