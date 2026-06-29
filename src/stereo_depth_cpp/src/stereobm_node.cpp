
#include "stereo_depth_cpp/stereobm_node.hpp"
#include <fstream>
#include <sstream>
#include <chrono>
#include <rclcpp/qos.hpp>
#include <opencv2/imgcodecs.hpp>
#include <cmath>
#include <iostream>

namespace stereo_depth_cpp
{

StereoBMNode::StereoBMNode()
: Node("stereobm_live_node"),
  baseline_(0.095),
  frame_count_(0),
  save_every_(1)
{
  auto qos = rclcpp::SensorDataQoS();

  left_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
    this, "/camera/camera/infra1/image_rect_raw", qos.get_rmw_qos_profile());
  right_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
    this, "/camera/camera/infra2/image_rect_raw", qos.get_rmw_qos_profile());
  rs_depth_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
    this, "/camera/camera/depth/image_rect_raw", qos.get_rmw_qos_profile());

  sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
    SyncPolicy(10), *left_sub_, *right_sub_, *rs_depth_sub_);
  sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(0.1));
  sync_->registerCallback(
    std::bind(&StereoBMNode::stereoCallback, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    "/camera/camera/infra1/camera_info", qos,
    std::bind(&StereoBMNode::infoCallback, this, std::placeholders::_1));

  rclcpp::QoS pub_qos(rclcpp::KeepLast(10));
  pub_qos.reliable();
  pc_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/custom/stereobm/points", pub_qos);

  stereo_matcher_ = cv::StereoBM::create(64, 15);

  save_to_disk_ = true;
  save_dir_ = "/mnt/hs2/SLAM-nav2/campus_bulding2/stereobm_live_cpp_output";
  std::filesystem::create_directories(save_dir_);

  // ---- PCD output file ----
  pcd_path_ = save_dir_ + "/stereobm_accumulated.pcd";

  // ---- Start background writer thread ----
  writer_running_ = true;
  writer_thread_ = std::thread(&StereoBMNode::writerLoop, this);

  RCLCPP_INFO(this->get_logger(),
    "StereoBM Live Node started — publishing /custom/stereobm/points, "
    "recording ASCII PCD for %.0f s, comparing live against RealSense depth",
    pcd_record_seconds_);
}

StereoBMNode::~StereoBMNode()
{
  // ---- Stop writer thread: drain queue, then join ----
  {
    std::unique_lock<std::mutex> lk(queue_mutex_);
    writer_running_ = false;
  }
  queue_cv_.notify_all();
  if (writer_thread_.joinable()) {
    writer_thread_.join();
  }

  // ---- Finalize PCD header (write real point count) before exit ----
  if (pcd_stream_.is_open()) {
    pcd_stream_.flush();
    pcd_stream_.close();

    std::fstream f(pcd_path_, std::ios::in | std::ios::out | std::ios::binary);
    if (f.is_open()) {
      std::ostringstream hdr;
      hdr << "# .PCD v0.7 - Point Cloud Data file format\n"
          << "VERSION 0.7\n"
          << "FIELDS x y z\n"
          << "SIZE 4 4 4\n"
          << "TYPE F F F\n"
          << "COUNT 1 1 1\n"
          << "WIDTH " << pcd_total_points_ << "\n"
          << "HEIGHT 1\n"
          << "VIEWPOINT 0 0 0 1 0 0 0\n"
          << "POINTS " << pcd_total_points_ << "\n"
          << "DATA ascii\n";
      std::string hstr = hdr.str();
      // Header was reserved to a fixed size; pad to match.
      if (hstr.size() < pcd_header_reserve_) {
        hstr.resize(pcd_header_reserve_, ' ');
      }
      hstr[pcd_header_reserve_ - 1] = '\n';
      f.seekp(0, std::ios::beg);
      f.write(hstr.data(), hstr.size());
      f.close();
    }
    RCLCPP_INFO(this->get_logger(),
      "PCD finalized: %s (%ld points)", pcd_path_.c_str(), pcd_total_points_);
  }

  printSummary();
}

void StereoBMNode::writerLoop()
{
  while (true) {
    WriteJob job;
    {
      std::unique_lock<std::mutex> lk(queue_mutex_);
      queue_cv_.wait(lk, [this] {
        return !write_queue_.empty() || !writer_running_;
      });

      if (write_queue_.empty() && !writer_running_) {
        break;  // shutting down and nothing left to write
      }

      job = std::move(write_queue_.front());
      write_queue_.pop();
    }

    // ---- PNG save (off the callback thread) ----
    if (job.write_png) {
      std::string filename = save_dir_ + "/stereobm_depth_" +
        std::to_string(job.frame_id) + ".png";
      cv::imwrite(filename, job.depth_mm);
    }

    // ---- PCD append (off the callback thread) ----
    if (job.write_pcd) {
      savePCD(job.xs, job.ys, job.zs);
    }
  }
}

void StereoBMNode::savePCD(const std::vector<float> & xs,
                           const std::vector<float> & ys,
                           const std::vector<float> & zs)
{
  if (!pcd_stream_.is_open()) {
    pcd_stream_.open(pcd_path_, std::ios::out | std::ios::binary | std::ios::trunc);
    if (!pcd_stream_.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open PCD file: %s", pcd_path_.c_str());
      return;
    }
    // Reserve a fixed-width header block; real counts written at destruction.
    std::string placeholder(pcd_header_reserve_, ' ');
    placeholder[pcd_header_reserve_ - 1] = '\n';
    pcd_stream_.write(placeholder.data(), placeholder.size());
  }

  for (size_t i = 0; i < xs.size(); ++i) {
    pcd_stream_ << xs[i] << ' ' << ys[i] << ' ' << zs[i] << '\n';
    ++pcd_total_points_;
  }
}

void StereoBMNode::printSummary()
{
  std::cout << "\n";
  std::cout << "==================================================================\n";
  std::cout << "                  STEREOBM vs REALSENSE — SUMMARY                 \n";
  std::cout << "==================================================================\n";
  std::cout << "  Frames processed:           " << frame_count_ << "\n";
  std::cout << "  Frames compared:            " << compared_frames_ << "\n";
  std::cout << "------------------------------------------------------------------\n";

  if (stereobm_valid_pixels_ > 0) {
    double mean_stereobm = sum_stereobm_depth_m_ / stereobm_valid_pixels_;
    std::cout << "  StereoBM  — mean depth:     " << mean_stereobm << " m\n";
    std::cout << "  StereoBM  — valid pixels:   " << stereobm_valid_pixels_ << "\n";
  } else {
    std::cout << "  StereoBM  — no valid pixels recorded\n";
  }

  if (rs_valid_pixels_ > 0) {
    double mean_rs = sum_rs_depth_m_ / rs_valid_pixels_;
    std::cout << "  RealSense — mean depth:     " << mean_rs << " m\n";
    std::cout << "  RealSense — valid pixels:   " << rs_valid_pixels_ << "\n";
  } else {
    std::cout << "  RealSense — no valid pixels recorded\n";
  }

  std::cout << "------------------------------------------------------------------\n";

  if (valid_compare_pixels_ > 0) {
    double mean_error = sum_abs_error_m_ / valid_compare_pixels_;
    double avg_compute = (frame_count_ > 0) ? (total_compute_ms_ / frame_count_) : 0.0;

    std::cout << "  Mean absolute error:        " << (mean_error * 1000.0) << " mm\n";
    std::cout << "  Max absolute error seen:    " << (overall_max_error_m_ * 1000.0) << " mm\n";
    std::cout << "  Compared pixel count:       " << valid_compare_pixels_ << "\n";
    std::cout << "  Avg compute time/frame:     " << avg_compute << " ms\n";
  } else {
    std::cout << "  No overlapping valid pixels found across the session.\n";
  }

  std::cout << "------------------------------------------------------------------\n";
  std::cout << "  PER-STAGE MEAN TIMING (ms)\n";
  for (const auto & kv : timer_.totals()) {
    double mean = (kv.second.second > 0) ? kv.second.first / kv.second.second : 0.0;
    std::cout << "    " << kv.first << std::string(
      (kv.first.size() < 22) ? 22 - kv.first.size() : 1, ' ')
      << ": " << mean << " ms  (n=" << kv.second.second << ")\n";
  }
  std::cout << "------------------------------------------------------------------\n";

  double proc_fps = (inter_cb_count_ > 0)
    ? 1000.0 / (sum_inter_cb_ms_ / inter_cb_count_) : 0.0;
  double session_s = (session_started_)
    ? std::chrono::duration<double>(
        std::chrono::steady_clock::now() - session_start_).count() : 0.0;
  double pub_fps = (session_s > 0.0) ? published_count_ / session_s : 0.0;

  std::cout << "  Input / processing FPS:     " << proc_fps << "\n";
  std::cout << "  Publish FPS:                " << pub_fps << "\n";
  std::cout << "  Frames published:           " << published_count_ << "\n";
  std::cout << "  PCD points saved:           " << pcd_total_points_ << "\n";
  std::cout << "  Write jobs dropped:         " << dropped_jobs_ << "\n";
  std::cout << "  PCD file:                   " << pcd_path_ << "\n";
  std::cout << "==================================================================\n";
  std::cout << "\n";
}

void StereoBMNode::infoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg)
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

void StereoBMNode::stereoCallback(
  const sensor_msgs::msg::Image::ConstSharedPtr & left_msg,
  const sensor_msgs::msg::Image::ConstSharedPtr & right_msg,
  const sensor_msgs::msg::Image::ConstSharedPtr & rs_depth_msg)
{
  if (!intrinsics_loaded_) {
    RCLCPP_WARN(this->get_logger(), "Intrinsics not loaded yet. Skipping frame.");
    return;
  }

  auto now = std::chrono::steady_clock::now();
  if (!session_started_) { session_start_ = now; session_started_ = true; }

  // ---- Inter-callback interval = input/processing FPS ----
  if (have_last_cb_) {
    double dt = std::chrono::duration<double, std::milli>(now - last_cb_time_).count();
    sum_inter_cb_ms_ += dt;
    inter_cb_count_++;
  }
  last_cb_time_ = now;
  have_last_cb_ = true;

  timer_.start("total_frame");

  // ---- Stage: cv_bridge conversion ----
  timer_.start("cv_bridge");
  cv_bridge::CvImagePtr left_cv = cv_bridge::toCvCopy(left_msg, "mono8");
  cv_bridge::CvImagePtr right_cv = cv_bridge::toCvCopy(right_msg, "mono8");
  cv_bridge::CvImagePtr rs_depth_cv = cv_bridge::toCvCopy(rs_depth_msg, "16UC1");
  timer_.stop("cv_bridge");

  int32_t width = left_cv->image.cols;
  int32_t height = left_cv->image.rows;

  // ---- Stage: StereoBM compute ----
  timer_.start("stereobm_compute");
  cv::Mat disparity_raw;
  stereo_matcher_->compute(left_cv->image, right_cv->image, disparity_raw);
  double compute_ms = timer_.stop("stereobm_compute");
  total_compute_ms_ += compute_ms;

  cv::Mat disparity_float;
  disparity_raw.convertTo(disparity_float, CV_32F, 1.0 / 16.0);

  // RealSense depth is 16UC1 in millimeters — same resolution as IR1/IR2
  cv::Mat rs_depth_m;
  rs_depth_cv->image.convertTo(rs_depth_m, CV_32F, 1.0 / 1000.0);  // mm -> m

  // Build a 16-bit depth image (millimeters), same convention as RealSense's depth/image_rect_raw
  cv::Mat depth_mm(height, width, CV_16UC1, cv::Scalar(0));

  std::vector<float> xs, ys, zs;
  xs.reserve(width * height);
  ys.reserve(width * height);
  zs.reserve(width * height);

  // Per-frame comparison accumulators
  double frame_sum_abs_error = 0.0;
  long frame_valid_pixels = 0;
  float frame_max_error = 0.0f;

  // ---- Stage: reprojection + comparison ----
  timer_.start("reproject_compare");
  for (int v = 0; v < height; ++v) {
    for (int u = 0; u < width; ++u) {

      // RealSense's own depth stats — tracked independently of StereoBM validity
      float rs_z_check = rs_depth_m.at<float>(v, u);
      if (rs_z_check > 0.0f) {
        sum_rs_depth_m_ += rs_z_check;
        rs_valid_pixels_++;
      }

      float d = disparity_float.at<float>(v, u);
      if (d <= 0) continue;

      float Z = static_cast<float>((fx_ * baseline_) / d);
      float X = static_cast<float>((u - cx_) * Z / fx_);
      float Y = static_cast<float>((v - cy_) * Z / fy_);

      xs.push_back(X);
      ys.push_back(Y);
      zs.push_back(Z);

      // Track StereoBM's own depth stats
      sum_stereobm_depth_m_ += Z;
      stereobm_valid_pixels_++;

      uint16_t z_mm = static_cast<uint16_t>(std::min(Z * 1000.0f, 65535.0f));
      depth_mm.at<uint16_t>(v, u) = z_mm;

      // Compare against RealSense's own depth at the same pixel, where both are valid
      if (rs_z_check > 0.0f) {
        float err = std::fabs(Z - rs_z_check);
        frame_sum_abs_error += err;
        frame_valid_pixels++;
        if (err > frame_max_error) frame_max_error = err;
        if (err > overall_max_error_m_) overall_max_error_m_ = err;
      }
    }
  }
  timer_.stop("reproject_compare");

  size_t num_points = xs.size();

  // ---- Stage: build + publish PointCloud2 ----
  timer_.start("build_publish");
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
  published_count_++;
  timer_.stop("build_publish");

  // ---- Decide what to write this frame, then hand off to writer thread ----
  bool want_png = save_to_disk_ && (frame_count_ % save_every_ == 0);

  bool within_pcd_window =
    std::chrono::duration<double>(now - session_start_).count() <= pcd_record_seconds_;
  bool want_pcd = pcd_recording_ && !pcd_done_ && within_pcd_window;

  // Close the PCD window once we pass the time limit (logged once).
  if (pcd_recording_ && !pcd_done_ && !within_pcd_window) {
    pcd_done_ = true;
    RCLCPP_INFO(this->get_logger(),
      "PCD recording window (%.0f s) complete — %ld points so far.",
      pcd_record_seconds_, pcd_total_points_);
  }

  if (want_png || want_pcd) {
    WriteJob job;
    job.frame_id  = frame_count_;
    job.write_png = want_png;
    job.write_pcd = want_pcd;
    if (want_png) job.depth_mm = depth_mm.clone();  // deep copy; depth_mm is local
    if (want_pcd) {
      job.xs = std::move(xs);   // safe: publish above already consumed them
      job.ys = std::move(ys);
      job.zs = std::move(zs);
    }

    {
      std::unique_lock<std::mutex> lk(queue_mutex_);
      // Backpressure: drop oldest if writer can't keep up, so the callback
      // never blocks and the FPS numbers stay honest.
      if (write_queue_.size() >= max_queue_size_) {
        write_queue_.pop();
        dropped_jobs_++;
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
          "Writer queue full — dropping oldest write job (disk too slow).");
      }
      write_queue_.push(std::move(job));
    }
    queue_cv_.notify_one();
  }

  double total_ms = timer_.stop("total_frame");

  // ---- Accuracy bookkeeping ----
  double frame_mean_error = 0.0;
  if (frame_valid_pixels > 0) {
    frame_mean_error = frame_sum_abs_error / frame_valid_pixels;
    sum_abs_error_m_ += frame_sum_abs_error;
    valid_compare_pixels_ += frame_valid_pixels;
    compared_frames_++;
  }

  // ---- Live FPS estimate ----
  double proc_fps = (inter_cb_count_ > 0)
    ? 1000.0 / (sum_inter_cb_ms_ / inter_cb_count_) : 0.0;
  double session_s = std::chrono::duration<double>(now - session_start_).count();
  double pub_fps = (session_s > 0.0) ? published_count_ / session_s : 0.0;

  if (frame_valid_pixels > 0) {
    double running_mean_error = sum_abs_error_m_ / valid_compare_pixels_;
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Frame %d | total %.1f ms (bridge %.1f | bm %.1f | reproj %.1f | pub %.1f) "
      "| %zu pts | %ld px cmp | frame err %.1f mm | running err %.1f mm | in/proc FPS %.1f | pub FPS %.1f",
      frame_count_, total_ms,
      timer_.last("cv_bridge"), timer_.last("stereobm_compute"),
      timer_.last("reproject_compare"), timer_.last("build_publish"),
      num_points, frame_valid_pixels,
      frame_mean_error * 1000.0, running_mean_error * 1000.0,
      proc_fps, pub_fps);
  } else {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Frame %d | total %.1f ms (bridge %.1f | bm %.1f | reproj %.1f | pub %.1f) "
      "| %zu pts | no overlap | in/proc FPS %.1f | pub FPS %.1f",
      frame_count_, total_ms,
      timer_.last("cv_bridge"), timer_.last("stereobm_compute"),
      timer_.last("reproject_compare"), timer_.last("build_publish"),
      num_points, proc_fps, pub_fps);
  }

  frame_count_++;
}

}  // namespace stereo_depth_cpp