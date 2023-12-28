#include "pointcloud_roi/red_cluster_filter.h"
#include "pointcloud_roi/utils.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_eigen/tf2_eigen.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pointcloud_roi_msgs/PointcloudWithRoi.h>

namespace pointcloud_roi
{

RedClusterFilter::RedClusterFilter(ros::NodeHandle &nhp) : leafsize_(0.005)
{
  target_frame = nhp.param<std::string>("map_frame", "world");
  //bool transform_to_map_frame = nhp.param<bool>("publish_separate_clouds", false);
  //bool publish_combined_cloud = nhp.param<bool>("publish_combined_cloud", true);
  //bool publish_separate_clouds = nhp.param<bool>("publish_separate_clouds", false);

  tf_buffer.reset(new tf2_ros::Buffer(ros::Duration(tf2::BufferCore::DEFAULT_CACHE_TIME)));
  tf_listener.reset(new tf2_ros::TransformListener(*tf_buffer, nhp));
  pc_sub.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(nhp, "input", 1));
  transform_filter.reset(new tf2_ros::MessageFilter<sensor_msgs::PointCloud2>(*pc_sub, *tf_buffer, target_frame, 1000, nhp));
  transform_filter->registerCallback(&RedClusterFilter::filter, this);
  pc_roi_pub = nhp.advertise<pointcloud_roi_msgs::PointcloudWithRoi>("results", 1);
  roi_only_pub = nhp.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("roi_cloud", 1);
  nonroi_only_pub = nhp.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("nonroi_cloud", 1);
  roi_points_pub_ = nhp.advertise<geometry_msgs::PoseArray>("roi_point_array", 1);

  time_ = ros::Time::now();
  last_time_ = time_;

}

pcl::IndicesConstPtr RedClusterFilter::filterRed(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr input, pcl::PointCloud<pcl::PointXYZRGB>::Ptr output)
{
  if (output == nullptr)
    output.reset(new pcl::PointCloud<pcl::PointXYZRGB>);

  //inRange((30, 25, 15), (80, 255,255))
  //pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr color_cond(new pcl::ConditionAnd<pcl::PointXYZRGB>);
  //color_cond->addComparison(pcl::PackedHSIComparison<pcl::PointXYZRGB>::ConstPtr (new pcl::PackedHSIComparison<pcl::PointXYZRGB> ("h", pcl::ComparisonOps::GE, 0)));
  //color_cond->addComparison(pcl::PackedHSIComparison<pcl::PointXYZRGB>::ConstPtr (new pcl::PackedHSIComparison<pcl::PointXYZRGB> ("h", pcl::ComparisonOps::LE, 35)));
  //color_cond->addComparison(pcl::PackedHSIComparison<pcl::PointXYZRGB>::ConstPtr (new pcl::PackedHSIComparison<pcl::PointXYZRGB> ("s", pcl::ComparisonOps::GE, 50)));
  //color_cond->addComparison(pcl::PackedHSIComparison<pcl::PointXYZRGB>::ConstPtr (new pcl::PackedHSIComparison<pcl::PointXYZRGB> ("i", pcl::ComparisonOps::GE, 70)));
  // pcl::ConditionOr<pcl::PointXYZRGB>::Ptr color_cond(new pcl::ConditionOr<pcl::PointXYZRGB>);
  // color_cond->addComparison(pcl::PackedHSIComparison<pcl::PointXYZRGB>::ConstPtr (new pcl::PackedHSIComparison<pcl::PointXYZRGB> ("h", pcl::ComparisonOps::LT, 31)));
  // color_cond->addComparison(pcl::PackedHSIComparison<pcl::PointXYZRGB>::ConstPtr (new pcl::PackedHSIComparison<pcl::PointXYZRGB> ("h", pcl::ComparisonOps::GT, 37)));
  // color_cond->addComparison(pcl::PackedHSIComparison<pcl::PointXYZRGB>::ConstPtr (new pcl::PackedHSIComparison<pcl::PointXYZRGB> ("s", pcl::ComparisonOps::LT, 200)));
  // color_cond->addComparison(pcl::PackedHSIComparison<pcl::PointXYZRGB>::ConstPtr (new pcl::PackedHSIComparison<pcl::PointXYZRGB> ("i", pcl::ComparisonOps::LT, 150)));
  pcl::ConditionOr<pcl::PointXYZRGB>::Ptr color_cond(new pcl::ConditionOr<pcl::PointXYZRGB>);
  color_cond->addComparison(pcl::PackedHSIComparison<pcl::PointXYZRGB>::ConstPtr (new pcl::PackedHSIComparison<pcl::PointXYZRGB> ("h", pcl::ComparisonOps::LT, 31)));
  color_cond->addComparison(pcl::PackedHSIComparison<pcl::PointXYZRGB>::ConstPtr (new pcl::PackedHSIComparison<pcl::PointXYZRGB> ("h", pcl::ComparisonOps::GT, 37)));
  color_cond->addComparison(pcl::PackedHSIComparison<pcl::PointXYZRGB>::ConstPtr (new pcl::PackedHSIComparison<pcl::PointXYZRGB> ("s", pcl::ComparisonOps::LT, 100)));
  color_cond->addComparison(pcl::PackedHSIComparison<pcl::PointXYZRGB>::ConstPtr (new pcl::PackedHSIComparison<pcl::PointXYZRGB> ("i", pcl::ComparisonOps::LT, 100)));
    // pcl::ConditionOr<pcl::PointXYZRGB>::Ptr color_cond(new pcl::ConditionOr<pcl::PointXYZRGB>);
  // color_cond->addComparison(pcl::PackedHSIComparison<pcl::PointXYZRGB>::ConstPtr (new pcl::PackedHSIComparison<pcl::PointXYZRGB> ("h", pcl::ComparisonOps::GE, 31)));
  // color_cond->addComparison(pcl::PackedHSIComparison<pcl::PointXYZRGB>::ConstPtr (new pcl::PackedHSIComparison<pcl::PointXYZRGB> ("h", pcl::ComparisonOps::LE, 60)));
  // color_cond->addComparison(pcl::PackedHSIComparison<pcl::PointXYZRGB>::ConstPtr (new pcl::PackedHSIComparison<pcl::PointXYZRGB> ("s", pcl::ComparisonOps::GE, 50)));
  // color_cond->addComparison(pcl::PackedHSIComparison<pcl::PointXYZRGB>::ConstPtr (new pcl::PackedHSIComparison<pcl::PointXYZRGB> ("i", pcl::ComparisonOps::GE, 70)));


  pcl::ConditionalRemoval<pcl::PointXYZRGB> condrem(true);
  condrem.setInputCloud(input);
  condrem.setCondition(color_cond);
  condrem.filter(*output);
  return condrem.getRemovedIndices();
}

void RedClusterFilter::filter(const sensor_msgs::PointCloud2ConstPtr &pc)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg(*pc, *pcl_cloud);

  // down sampling
  pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr myConstCloud = pcl_cloud;
  vg_.setInputCloud(pcl_cloud);
  vg_.setLeafSize(leafsize_, leafsize_, leafsize_);
  vg_.filter(*pcl_cloud);

  pcl::IndicesConstPtr redIndices = filterRed(pcl_cloud);

  geometry_msgs::TransformStamped pcFrameTf;
  try
  {
    pcFrameTf = tf_buffer->lookupTransform(target_frame, pc->header.frame_id, pc->header.stamp);
  }
  catch (const tf2::TransformException &e)
  {
    ROS_ERROR_STREAM("Couldn't find transform to map frame: " << e.what());
    return;
  }
  ROS_INFO_STREAM("Transform for time " << pc->header.stamp << " successful");

  static Eigen::Isometry3d lastTfEigen;
  Eigen::Isometry3d tfEigen = tf2::transformToEigen(pcFrameTf);

  time_ = ros::Time::now();
  if (tfEigen.isApprox(lastTfEigen, 1e-2)) // Publish separate clouds only if not moved
  {
    const auto [inlier_cloud, outlier_cloud] = separateCloudByIndices<pcl::PointXYZRGB>(pcl_cloud, redIndices);
    roi_only_pub.publish(*inlier_cloud);
    //nonroi_only_pub.publish(*outlier_cloud);
  } else {
    last_time_ = time_;
  }
  lastTfEigen = tfEigen;

  pcl::transformPointCloud(*pcl_cloud, *pcl_cloud, tfEigen.matrix());

  ros::Duration ros_duration = time_ - last_time_;
  if (ros_duration.sec > 0.5) // 直前の移動からros時間で0.5秒経過したら
  {
    const auto [inlier_cloud, outlier_cloud] = separateCloudByIndices<pcl::PointXYZRGB>(pcl_cloud, redIndices);
    geometry_msgs::PoseArray roi_points;
    roi_points.header.stamp = pc->header.stamp;
    roi_points.header.frame_id = target_frame;
    for (auto point : *inlier_cloud)
    {
      geometry_msgs::Pose roi_point;
      roi_point.position.x = point.x;
      roi_point.position.y = point.y;
      roi_point.position.z = point.z;

      roi_points.poses.push_back(roi_point);
    }
    std::mt19937_64 get_rand_mt; // 64bit版メルセンヌ・ツイスタ
    std::shuffle( roi_points.poses.begin(), roi_points.poses.end(), get_rand_mt );
    if (roi_points.poses.size() > 100)
      roi_points.poses.resize(100); // 数が増えすぎると処理が重くなるので500娘に制限
    roi_points_pub_.publish(roi_points);
  }

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud_ds(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::VoxelGrid<pcl::PointXYZRGB> vg;
  vg.setInputCloud (pcl_cloud);
  vg.setLeafSize (0.01, 0.01f, 0.01f);
  //vg.setSaveLeafLayout(true);
  vg.filter (*pcl_cloud_ds);

  redIndices = filterRed(pcl_cloud_ds);
  pcl::IndicesPtr redIndicesRo(new std::vector<int>);

  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> ro;
  ro.setInputCloud(pcl_cloud_ds);
  ro.setIndices(redIndices);
  ro.filter(*redIndicesRo);

  pointcloud_roi_msgs::PointcloudWithRoi res;
  pcl::toROSMsg(*pcl_cloud_ds, res.cloud);
  res.cloud.header.frame_id = target_frame;
  res.cloud.header.stamp = pc->header.stamp;
  res.transform = pcFrameTf.transform;
  res.roi_indices.assign(redIndicesRo->begin(), redIndicesRo->end());
  pc_roi_pub.publish(res);
}

} // namespace pointcloud_roi
