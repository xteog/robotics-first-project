#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_broadcaster.h"

class tf_sub_pub {
public:
    tf_sub_pub() {

        sub = n.subscribe("input_odom", 1000, &tf_sub_pub::callback, this); 
    }


    void callback(const nav_msgs::Odometry::ConstPtr& msg) {
        // Access pose from the Odometry message
        ROS_INFO_STREAM(msg);
        tf::Transform transform;
        n.getParam("root_frame", root); 
        n.getParam("child_frame", child);
        transform.setOrigin(tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z));
        transform.setRotation(tf::Quaternion(
            msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w));
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), root, child));
        ROS_INFO("Root param name: %s", root.c_str());
        ROS_INFO("Child param name: %s", child.c_str());
        ROS_INFO("prova");

    }

private:

    tf::TransformBroadcaster br;
    ros::Subscriber sub;
    ros::NodeHandle n;
    //ros::NodeHandle nh_private("~");
    std::string root; 
    std::string child;



};


int main(int argc, char** argv) {

    // Node name: odom_to_tf
    ros::init(argc, argv, "odom_to_tf"); 

    ROS_INFO("prova_2");
    tf_sub_pub my_tf_sub_bub;
    
    ros::spin();
    return 0;
}