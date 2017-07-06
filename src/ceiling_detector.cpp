#include <ros/ros.h>

// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>

//TF specific includes
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>



ros::Publisher pub;
ros::Publisher vis_pub;
void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg (*input, cloud);

  pcl::ModelCoefficients coefficients;
  pcl::PointIndices inliers;
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.01);

  seg.setInputCloud (cloud.makeShared ());
  seg.segment (inliers, coefficients); 
  
  // Publish the model coefficients
  pcl_msgs::ModelCoefficients ros_coefficients;
  pcl_conversions::fromPCL(coefficients, ros_coefficients);
  pub.publish (ros_coefficients);

  geometry_msgs::TransformStamped transformStamped;
  
  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = "ceiling";
  transformStamped.child_frame_id = "o3d3xx/camera_link";
  transformStamped.transform.translation.x = ros_coefficients.values[0];
  transformStamped.transform.translation.y = ros_coefficients.values[1];
  transformStamped.transform.translation.z = ros_coefficients.values[2];

  tf2::Quaternion q;
  transformStamped.transform.rotation.x = 1;
  transformStamped.transform.rotation.y = 0;
  transformStamped.transform.rotation.z = 0;
  transformStamped.transform.rotation.w = 0;

  static tf2_ros::TransformBroadcaster br;
  br.sendTransform(transformStamped);


}


int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "ceiling_detector");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/o3d3xx/camera/cloud", 1, cloud_cb);

  // Create a ROS publisher for the output model coefficients
  pub = nh.advertise<pcl_msgs::ModelCoefficients> ("ceiling", 1);
  vis_pub = nh.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );

  // Spin
  ros::spin ();
}

