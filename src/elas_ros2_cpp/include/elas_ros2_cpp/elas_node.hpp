#ifndef ELAS_ROS2_CPP__ELAS_NODE_HPP_
#define ELAS_ROS2_CPP__ELAS_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <opencv2/opencv.hpp>
#include "elas.h"  // from libelas third_party

#include <string>
#include <filesystem>

namespace elas_ros2_cpp
{

class ELASNode : public rclcpp::Node
{
public:
  ELASNode();

private:
  void infoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg);
  void stereoCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr & left_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr & right_msg);

  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> left_sub_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> right_sub_;

  using SyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_pub_;

  std::shared_ptr<Elas> elas_matcher_;

  bool intrinsics_loaded_ {false};
  double fx_, fy_, cx_, cy_;
  double baseline_;

  bool save_to_disk_;
  std::string save_dir_;
  int frame_count_;
  int save_every_;
};

}  // namespace elas_ros2_cpp

#endif  // ELAS_ROS2_CPP__ELAS_NODE_HPP_
