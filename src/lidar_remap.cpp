#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/PointCloud2.h>

#include <dynamic_reconfigure/server.h>
#include <first_project/parametersConfig.h>  


class lidar_remap {
public:
    lidar_remap() {

        pub = n.advertise<sensor_msgs::PointCloud2>("pointcloud_remapped", 1000);
        f = boost::bind(&lidar_remap::paramCallback, this, _1, _2);
        server.setCallback(f);
        sub = n.subscribe<sensor_msgs::PointCloud2>("/os_cloud_node/points", 1, &lidar_remap::subCallback, this);

    }

    void paramCallback(first_project::parametersConfig &config, uint32_t level) {
        odom_source = config.odom_source;
     
    }


    void subCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
        sensor_msgs::PointCloud2 modifiedMsg = *msg;
        modifiedMsg.header.frame_id = odom_source;
        pub.publish(modifiedMsg);
        ROS_INFO("header param name: %s", odom_source.c_str());
    }

private:

    ros::NodeHandle n;
    ros::Subscriber sub;
    std::string odom_source;
    ros::Publisher pub;

    dynamic_reconfigure::Server<first_project::parametersConfig> server;
    dynamic_reconfigure::Server<first_project::parametersConfig>::CallbackType f;

};


int main(int argc, char** argv) {
    
    ros::init(argc, argv, "lidar_remap"); 

    lidar_remap my_lidar_remap;
    
    ros::spin();
    return 0;
}
