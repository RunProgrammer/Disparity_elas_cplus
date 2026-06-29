
#ifndef STEREO_DEPTH_CPP__STEREOBM_NODE_HPP_
#define STEREO_DEPTH_CPP__STEREOBM_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <string>
#include <filesystem>
#include <fstream>
#include <chrono>
#include <map>
#include <vector>
#include <utility>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <atomic>

namespace stereo_depth_cpp
{

// ---- Lightweight named timer -------------------------------------------
class TimeLog
{
public:
  void start(const std::string & name)
  {
    starts_[name] = std::chrono::steady_clock::now();
  }

  double stop(const std::string & name)  // returns ms
  {
    auto end = std::chrono::steady_clock::now();
    auto it = starts_.find(name);
    if (it == starts_.end()) return 0.0;
    double ms = std::chrono::duration<double, std::milli>(end - it->second).count();
    auto & acc = totals_[name];
    acc.first += ms;     // sum
    acc.second += 1;     // count
    last_[name] = ms;
    return ms;
  }

  double last(const std::string & name) const
  {
    auto it = last_.find(name);
    return (it == last_.end()) ? 0.0 : it->second;
  }

  double mean(const std::string & name) const
  {
    auto it = totals_.find(name);
    if (it == totals_.end() || it->second.second == 0) return 0.0;
    return it->second.first / it->second.second;
  }

  const std::map<std::string, std::pair<double, long>> & totals() const { return totals_; }

private:
  std::map<std::string, std::chrono::steady_clock::time_point> starts_;
  std::map<std::string, std::pair<double, long>> totals_;  // name -> {sum_ms, count}
  std::map<std::string, double> last_;
};
// -------------------------------------------------------------------------

// ---- Background disk-write job ------------------------------------------
struct WriteJob
{
  int frame_id {0};
  bool write_png {false};
  bool write_pcd {false};
  cv::Mat depth_mm;               // for PNG (deep-copied before enqueue)
  std::vector<float> xs, ys, zs;  // for PCD (moved in)
};
// -------------------------------------------------------------------------

class StereoBMNode : public rclcpp::Node
{
public:
  StereoBMNode();
  ~StereoBMNode();

private:
  void infoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg);
  void stereoCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr & left_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr & right_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr & rs_depth_msg);
  void printSummary();
  void savePCD(const std::vector<float> & xs,
               const std::vector<float> & ys,
               const std::vector<float> & zs);
  void writerLoop();  // background writer thread body

  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> left_sub_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> right_sub_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> rs_depth_sub_;

  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::Image>;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_pub_;
  cv::Ptr<cv::StereoBM> stereo_matcher_;

  bool intrinsics_loaded_ {false};
  double fx_, fy_, cx_, cy_;
  double baseline_;

  bool save_to_disk_;
  std::string save_dir_;
  int frame_count_;
  int save_every_;

  // Running accuracy stats vs RealSense
  double sum_abs_error_m_ {0.0};
  long valid_compare_pixels_ {0};
  long compared_frames_ {0};

  // Running stats — StereoBM's own output
  double sum_stereobm_depth_m_ {0.0};
  long stereobm_valid_pixels_ {0};

  // Running stats — RealSense's own output
  double sum_rs_depth_m_ {0.0};
  long rs_valid_pixels_ {0};

  double overall_max_error_m_ {0.0};
  double total_compute_ms_ {0.0};

  // ---- Timing / FPS ----
  TimeLog timer_;
  std::chrono::steady_clock::time_point last_cb_time_;
  bool have_last_cb_ {false};
  double sum_inter_cb_ms_ {0.0};   // for input/processing FPS
  long inter_cb_count_ {0};
  long published_count_ {0};
  std::chrono::steady_clock::time_point session_start_;
  bool session_started_ {false};

  // ---- PCD recording (20 s window, ASCII) ----
  bool pcd_recording_ {true};
  double pcd_record_seconds_ {20.0};
  bool pcd_done_ {false};
  std::string pcd_path_;
  std::ofstream pcd_stream_;
  long pcd_total_points_ {0};
  static constexpr size_t pcd_header_reserve_ = 256;  // bytes reserved for PCD header

  // ---- Background writer thread ----
  std::thread writer_thread_;
  std::mutex queue_mutex_;
  std::condition_variable queue_cv_;
  std::queue<WriteJob> write_queue_;
  std::atomic<bool> writer_running_ {false};
  size_t max_queue_size_ {30};     // backpressure cap
  long dropped_jobs_ {0};
};

}  // namespace stereo_depth_cpp

#endif  // STEREO_DEPTH_CPP__STEREOBM_NODE_HPP_