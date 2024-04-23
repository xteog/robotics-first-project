#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/PointCloud2.h>

#include <dynamic_reconfigure/server.h>
#include <first_project/parametersConfig.h>  

// default value setted to "wheel_odom"
std::string odom_source;

void paramCallback(first_project::parametersConfig &config, uint32_t level) {
  odom_source = config.odom_source;
     
}


void subCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
   sensor_msgs::PointCloud2 modifiedMsg = *msg;
   modifiedMsg.header.frame_id = odom_source;
  
}


int main(int argc, char **argv){
  	
	ros::init(argc, argv, "lidar_remap");

  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe<sensor_msgs::PointCloud2>("/os_cloud_node/points", 1, subCallback);
  ros::Publisher pub = n.advertise<sensor_msgs::PointCloud2>("pointcloud_remapped", 1000);

	dynamic_reconfigure::Server<first_project::parametersConfig> server;
  dynamic_reconfigure::Server<first_project::parametersConfig>::CallbackType f;
  

  f = boost::bind(&paramCallback, _1, _2);
  server.setCallback(f);


  ros::spin();

  return 0;
}


