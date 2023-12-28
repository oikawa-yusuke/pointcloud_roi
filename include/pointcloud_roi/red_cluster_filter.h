#ifndef RED_CLUSTER_FILTER_H
#define RED_CLUSTER_FILTER_H

#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/message_filter.h>
#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <string>
#include <random>
#include <algorithm>

namespace pointcloud_roi
{

class RedClusterFilter
{
public:
  RedClusterFilter(ros::NodeHandle &nhp);

  void filter(const sensor_msgs::PointCloud2ConstPtr &pc);

private:
  std::string target_frame;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener;
  std::unique_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> pc_sub;
  std::unique_ptr<tf2_ros::MessageFilter<sensor_msgs::PointCloud2>> transform_filter;

  ros::Publisher pc_roi_pub;
  ros::Publisher roi_only_pub;
  ros::Publisher nonroi_only_pub;
  ros::Publisher roi_points_pub_;

  ros::Time time_, last_time_;

  // pcl voxel grid
  pcl::VoxelGrid<pcl::PointXYZRGB> vg_;

  // param
  double leafsize_;

  pcl::IndicesConstPtr filterRed(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr input, pcl::PointCloud<pcl::PointXYZRGB>::Ptr output = nullptr);
};

} // namespace pointcloud_roi

#endif // RED_CLUSTER_FILTER_H
