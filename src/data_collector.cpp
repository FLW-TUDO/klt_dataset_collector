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
  cass.request.max_capture_time = ros::Duration{ 1.20 };
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
  ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("/point_cloud/cloud_transformed", 1);

  tf::TransformListener listener;

  // setup zivid camera
  ros::service::waitForService(ca_suggest_settings_service_name, default_wait_duration);
  capture_assistant_suggest_settings();

  // capture positions
  float position[2][3] = {
    {0.653966303718863, 0.3916834478716141, -0.12949275619231873},
    {0.6185055234985145, -0.4643385077623673, 0.2250001532518916}};
  float orientation[2][4] = {
    {0.8857424855232239, 0.42670773623355057, 0.08713615390481165, 0.16058647412382923},
    {-0.45447058309696803, 0.8438899517059326, 0.23677301747020152, 0.15888606578577916}};

  ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/iiwa/command/CartesianPose", 10);
  while (0 == pose_pub.getNumSubscribers()) {
      ros::Duration(0.1).sleep();
  }
  ros::Duration(0.2).sleep();

  geometry_msgs::PoseStamped msg;
  msg.header.frame_id = "iiwa_link_0";
  msg.header.stamp = ros::Time::now();
  for (int i = 0; i < 2; i++)
  {
    ROS_INFO("Moving to position");
    msg.header.seq = i;
    msg.pose.position.x  = position[i][0];
    msg.pose.position.y  = position[i][1];
    msg.pose.position.z  = position[i][2];
    msg.pose.orientation.x  = orientation[i][0];
    msg.pose.orientation.y  = orientation[i][1];
    msg.pose.orientation.z  = orientation[i][2];
    msg.pose.orientation.w  = orientation[i][3];
    pose_pub.publish(msg);
    ros::Duration(5).sleep();
    // capture image
    capture();
    ros::Duration(2).sleep(); // wait for capture to finish
    // Convert point cloud to iiwa_link_0 frame
    sensor_msgs::PointCloud2 pc2_iiwa_msg;
    pcl_ros::transformPointCloud("iiwa_link_0", pc2_zivid_msg, pc2_iiwa_msg, listener);
    pub.publish(pc2_iiwa_msg);

    ROS_INFO("Capturing finished");
  }

  return 0;
}
