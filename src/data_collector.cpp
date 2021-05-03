#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/PointCloud2.h"
#include <zivid_camera/Capture.h>
#include <zivid_camera/CaptureAssistantSuggestSettings.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>

//#include <pcl/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

const ros::Duration default_wait_duration{ 30 };
constexpr auto ca_suggest_settings_service_name = "/zivid_camera/capture_assistant/suggest_settings";

sensor_msgs::PointCloud2 pc2_zivid_msg;

void capture_assistant_suggest_settings()
{
  zivid_camera::CaptureAssistantSuggestSettings cass;
  cass.request.max_capture_time = ros::Duration{ 10.00 };
  cass.request.ambient_light_frequency =
      zivid_camera::CaptureAssistantSuggestSettings::Request::AMBIENT_LIGHT_FREQUENCY_NONE;

  ros::service::call(ca_suggest_settings_service_name, cass);
}

void capture()
{
  ROS_INFO("Calling capture service");
  zivid_camera::Capture capture;
  ros::service::call("/zivid_camera/capture", capture);
}

void on_points(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  pc2_zivid_msg = *msg;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "collect_data");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner(1);
  spinner.start();

  auto points_sub = nh.subscribe("/zivid_camera/points/xyzrgba", 10, on_points);
  ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("/point_cloud/cloud_transformed", 10);
  ros::Publisher pub_merged = nh.advertise<sensor_msgs::PointCloud2>("/point_cloud/cloud_merged", 10);

  tf::TransformListener listener;

  // setup zivid camera
  ros::service::waitForService(ca_suggest_settings_service_name, default_wait_duration);
  capture_assistant_suggest_settings();

  // capture positions
  float position[2][3] = {
    {0.506117809325929, -0.36538629946360257, 0.32621662675008856},
    {0.1960708791023984, 0.5577574555894433, -0.026876493187485948}};
  float orientation[2][4] = {
    {0.6849232912063599, -0.6465909588241828, 0.13713796034869008, -0.3065834786850864},
    {0.42426899813017876, 0.8014302253723145, 0.08913723921969446, 0.4120193811868045}};

  ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/iiwa/command/CartesianPose", 10);
  while (0 == pose_pub.getNumSubscribers()) {
      ros::Duration(0.1).sleep();
  }
  ros::Duration(0.2).sleep();

  sensor_msgs::PointCloud2 clouds[2];

  geometry_msgs::PoseStamped msg;
  msg.header.frame_id = "iiwa_link_0";
  for (int i = 0; i < 2; i++)
  {
    ROS_INFO("Moving to position");
    msg.header.stamp = ros::Time::now();
    msg.header.seq = i;
    msg.pose.position.x  = position[i][0];
    msg.pose.position.y  = position[i][1];
    msg.pose.position.z  = position[i][2];
    msg.pose.orientation.x  = orientation[i][0];
    msg.pose.orientation.y  = orientation[i][1];
    msg.pose.orientation.z  = orientation[i][2];
    msg.pose.orientation.w  = orientation[i][3];
    pose_pub.publish(msg);
    ros::Duration(20).sleep();
    // capture image
    capture();
    ros::Duration(5).sleep(); // wait for capture to finish
    // Convert point cloud to iiwa_link_0 frame
    sensor_msgs::PointCloud2 pc2_iiwa_msg;
    pcl_ros::transformPointCloud("iiwa_link_0", pc2_zivid_msg, pc2_iiwa_msg, listener);
    pub.publish(pc2_iiwa_msg);
    clouds[i] = pc2_iiwa_msg;
    // crop box filter

    ROS_INFO("Capturing finished");
  }

  // Stitch (not register) the clouds
  // Merge metadata
  sensor_msgs::PointCloud2 MergedCloud = clouds[0];
  sensor_msgs::PointCloud2 Cloud1 = clouds[1];

  MergedCloud.width += Cloud1.width;

  // Re-size the merged data array to make space for the new points
  uint64_t OriginalSize = MergedCloud.data.size();
  MergedCloud.data.resize(MergedCloud.data.size() + Cloud1.data.size());

  // Copy the new points from Cloud1 into the second half of the MergedCloud array
  std::copy(
    Cloud1.data.begin(),
    Cloud1.data.end(),
    MergedCloud.data.begin() + OriginalSize);

  MergedCloud.header.seq = 1;
  MergedCloud.header.stamp = ros::Time::now();
  pub_merged.publish(MergedCloud);
  ros::Duration(5).sleep(); // wait for last message to publish

  return 0;
}
