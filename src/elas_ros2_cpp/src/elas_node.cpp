#include "elas_ros2_cpp/elas_node.hpp"
#include <fstream>

namespace elas_ros2_cpp
{

ELASNode::ELASNode()
: Node("elas_live_node"),
  baseline_(0.095),  // corrected baseline value
  frame_count_(0),
  save_every_(10)
{
  left_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
    this, "/camera/camera/infra1/image_rect_raw");
  right_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
    this, "/camera/camera/infra2/image_rect_raw");

  sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
    SyncPolicy(10), *left_sub_, *right_sub_);
  sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(0.1));
  sync_->registerCallback(
    std::bind(&ELASNode::stereoCallback, this, std::placeholders::_1, std::placeholders::_2));

  info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    "/camera/camera/infra1/camera_info", 10,
    std::bind(&ELASNode::infoCallback, this, std::placeholders::_1));

  pc_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/custom/elas/points", 10);

  // ELAS parameters matching PRESET_ROBOTICS
  Elas::parameters param(Elas::ROBOTICS);
  elas_matcher_ = std::make_shared<Elas>(param);

  save_to_disk_ = true;
  save_dir_ = "/mnt/hs2/SLAM-nav2/campus_bulding2/elas_live_cpp_output";
  std::filesystem::create_directories(save_dir_);

  RCLCPP_INFO(this->get_logger(), "ELAS Live Node (C++) started — publishing /custom/elas/points");
}

void ELASNode::infoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg)
{
  if (!intrinsics_loaded_) {
    fx_ = msg->k[0];
    fy_ = msg->k[4];
    cx_ = msg->k[2];
    cy_ = msg->k[5];
    intrinsics_loaded_ = true;
    RCLCPP_INFO(this->get_logger(), "Intrinsics loaded, fx=%.2f", fx_);
  }
}

void ELASNode::stereoCallback(
  const sensor_msgs::msg::Image::ConstSharedPtr & left_msg,
  const sensor_msgs::msg::Image::ConstSharedPtr & right_msg)
{
  if (!intrinsics_loaded_) {
    return;
  }

  cv_bridge::CvImagePtr left_cv = cv_bridge::toCvCopy(left_msg, "mono8");
  cv_bridge::CvImagePtr right_cv = cv_bridge::toCvCopy(right_msg, "mono8");

  int32_t width = left_cv->image.cols;
  int32_t height = left_cv->image.rows;
  const int32_t dims[3] = {width, height, width};

  std::vector<float> disp_left(width * height);
  std::vector<float> disp_right(width * height);

  elas_matcher_->process(
    left_cv->image.data, right_cv->image.data,
    disp_left.data(), disp_right.data(), dims);

  std::vector<float> xs, ys, zs;
  xs.reserve(width * height);
  ys.reserve(width * height);
  zs.reserve(width * height);

  for (int v = 0; v < height; ++v) {
    for (int u = 0; u < width; ++u) {
      float d = disp_left[v * width + u];
      if (d <= 0) continue;

      float Z = static_cast<float>((fx_ * baseline_) / d);
      float X = static_cast<float>((u - cx_) * Z / fx_);
      float Y = static_cast<float>((v - cy_) * Z / fy_);

      xs.push_back(X);
      ys.push_back(Y);
      zs.push_back(Z);
    }
  }

  size_t num_points = xs.size();

  sensor_msgs::msg::PointCloud2 msg;
  msg.header.stamp = this->get_clock()->now();
  msg.header.frame_id = "camera_infra1_optical_frame";
  msg.height = 1;
  msg.width = num_points;
  msg.is_bigendian = false;
  msg.is_dense = true;

  sensor_msgs::PointCloud2Modifier modifier(msg);
  modifier.setPointCloud2Fields(
    3,
    "x", 1, sensor_msgs::msg::PointField::FLOAT32,
    "y", 1, sensor_msgs::msg::PointField::FLOAT32,
    "z", 1, sensor_msgs::msg::PointField::FLOAT32);
  modifier.resize(num_points);

  sensor_msgs::PointCloud2Iterator<float> iter_x(msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(msg, "z");

  for (size_t i = 0; i < num_points; ++i, ++iter_x, ++iter_y, ++iter_z) {
    *iter_x = xs[i];
    *iter_y = ys[i];
    *iter_z = zs[i];
  }

  pc_pub_->publish(msg);

  if (save_to_disk_ && (frame_count_ % save_every_ == 0)) {
    std::string filename = save_dir_ + "/elas_live_" +
      std::to_string(frame_count_) + ".bin";
    std::ofstream out(filename, std::ios::binary);
    out.write(reinterpret_cast<const char *>(disp_left.data()), disp_left.size() * sizeof(float));
    out.close();
  }

  frame_count_++;
  RCLCPP_INFO_THROTTLE(
    this->get_logger(), *this->get_clock(), 2000,
    "Frame %d: %zu points", frame_count_, num_points);
}

}  // namespace elas_ros2_cpp
