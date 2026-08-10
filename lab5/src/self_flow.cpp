/*
 * @file self_flow.cpp
 * @brief Dense optical flow visualization (Farneback).
 */

#include <cv_bridge/cv_bridge.h>
#include <glog/logging.h>

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <iomanip>
#include <image_transport/image_transport.hpp>
#include <memory>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace {

struct FlowFrameStats {
  double mean_mag = 0.0;
  double max_mag = 0.0;
  double pct_moving = 0.0;
  double process_ms = 0.0;
};

FlowFrameStats computeFlowStats(const cv::Mat& flow, int spacing) {
  FlowFrameStats stats;
  if (flow.empty()) {
    return stats;
  }

  double sum_mag = 0.0;
  std::size_t count = 0;
  std::size_t moving = 0;
  constexpr double kMotionThresholdPx = 0.5;

  for (int y = 0; y < flow.rows; y += spacing) {
    for (int x = 0; x < flow.cols; x += spacing) {
      const cv::Point2f& displacement = flow.at<cv::Point2f>(y, x);
      const double mag = std::hypot(displacement.x, displacement.y);
      sum_mag += mag;
      stats.max_mag = std::max(stats.max_mag, mag);
      if (mag > kMotionThresholdPx) {
        ++moving;
      }
      ++count;
    }
  }

  if (count > 0) {
    stats.mean_mag = sum_mag / static_cast<double>(count);
    stats.pct_moving = 100.0 * static_cast<double>(moving) / static_cast<double>(count);
  }
  return stats;
}

}  // namespace

class SelfFlow : public rclcpp::Node {
 public:
  SelfFlow() : Node("self_flow") {
    declare_parameter<bool>("show_images", true);
    declare_parameter<std::string>("stats_output_dir", "");
    show_images_ = get_parameter("show_images").as_bool();
    stats_output_dir_ = get_parameter("stats_output_dir").as_string();
  }

  ~SelfFlow() override { printAndSaveStats(); }

  void showFlow(const cv::Mat& frame, const cv::Mat& flow, int spacing = 16) const {
    cv::Mat display;
    if (frame.channels() == 1) {
      cv::cvtColor(frame, display, cv::COLOR_GRAY2BGR);
    } else {
      display = frame.clone();
    }

    for (int y = 0; y < flow.rows; y += spacing) {
      for (int x = 0; x < flow.cols; x += spacing) {
        const cv::Point2f& displacement = flow.at<cv::Point2f>(y, x);
        const cv::Point start(x, y);
        const cv::Point end(cvRound(x + displacement.x), cvRound(y + displacement.y));
        cv::line(display, start, end, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
        cv::circle(display, start, 2, cv::Scalar(0, 255, 0), -1, cv::LINE_AA);
      }
    }

    if (show_images_) {
      cv::imshow("flow", display);
      cv::waitKey(10);
    }
  }

  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
    try {
      const cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;
      cv::Mat gray;
      cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

      // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      //  DELIVERABLE 8 | Optical Flow
      // ~~~~ begin solution
      if (!prev_gray_.empty()) {
        const auto t0 = std::chrono::steady_clock::now();
        cv::Mat flow;
        cv::calcOpticalFlowFarneback(prev_gray_,
                                     gray,
                                     flow,
                                     0.5,
                                     3,
                                     15,
                                     3,
                                     5,
                                     1.2,
                                     0);
        const auto t1 = std::chrono::steady_clock::now();
        FlowFrameStats frame_stats = computeFlowStats(flow, 16);
        frame_stats.process_ms =
            std::chrono::duration<double, std::milli>(t1 - t0).count();
        accumulateStats(frame_stats);
        showFlow(image, flow);

        if (flow_frames_ % 100 == 0) {
          LOG(INFO) << "Flow frame " << flow_frames_
                    << " mean_mag=" << frame_stats.mean_mag << " px"
                    << " max_mag=" << frame_stats.max_mag << " px"
                    << " moving=" << frame_stats.pct_moving << "%"
                    << " process_ms=" << frame_stats.process_ms;
        }
      } else {
        ++skipped_first_frame_;
      }
      prev_gray_ = gray.clone();
      // ~~~~ end solution
      // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    } catch (const cv_bridge::Exception& e) {
      RCLCPP_ERROR(get_logger(),
                   "Could not convert from '%s' to 'bgr8'.",
                   msg->encoding.c_str());
    }
  }

  void run() {
    if (show_images_) {
      cv::namedWindow("flow", cv::WINDOW_NORMAL);
    }

    image_transport::ImageTransport it(shared_from_this());
    sub_ = it.subscribe("/images_topic",
                        1,
                        std::bind(&SelfFlow::imageCallback, this, std::placeholders::_1));
  }

 private:
  void accumulateStats(const FlowFrameStats& stats) {
    ++flow_frames_;
    sum_mean_mag_ += stats.mean_mag;
    sum_max_mag_ += stats.max_mag;
    sum_pct_moving_ += stats.pct_moving;
    sum_process_ms_ += stats.process_ms;
    max_observed_mag_ = std::max(max_observed_mag_, stats.max_mag);
    max_process_ms_ = std::max(max_process_ms_, stats.process_ms);
  }

  void printAndSaveStats() const {
    if (flow_frames_ == 0) {
      LOG(WARNING) << "No optical-flow frames processed (need at least 2 images).";
      return;
    }

    const double avg_mean_mag = sum_mean_mag_ / static_cast<double>(flow_frames_);
    const double avg_max_mag = sum_max_mag_ / static_cast<double>(flow_frames_);
    const double avg_pct_moving = sum_pct_moving_ / static_cast<double>(flow_frames_);
    const double avg_process_ms = sum_process_ms_ / static_cast<double>(flow_frames_);

    LOG(INFO) << "=== Optical Flow Statistics (Farneback) ===";
    LOG(INFO) << "Flow frames processed: " << flow_frames_;
    LOG(INFO) << "Skipped first frame(s): " << skipped_first_frame_;
    LOG(INFO) << "Avg mean flow magnitude: " << avg_mean_mag << " px";
    LOG(INFO) << "Avg per-frame max magnitude: " << avg_max_mag << " px";
    LOG(INFO) << "Global max magnitude: " << max_observed_mag_ << " px";
    LOG(INFO) << "Avg moving samples (>0.5 px): " << avg_pct_moving << "%";
    LOG(INFO) << "Avg processing time: " << avg_process_ms << " ms";
    LOG(INFO) << "Max processing time: " << max_process_ms_ << " ms";

    if (stats_output_dir_.empty()) {
      return;
    }

    const std::string csv_path = stats_output_dir_ + "/flow_stats.csv";
    const std::string md_path = stats_output_dir_ + "/flow_stats.md";
    std::ofstream csv(csv_path, std::ios::app);
    if (csv) {
      csv << std::fixed << std::setprecision(4)
          << "Farneback,30fps_424x240_2018-10-01-18-35-06,"
          << flow_frames_ << ","
          << avg_mean_mag << ","
          << avg_max_mag << ","
          << max_observed_mag_ << ","
          << avg_pct_moving << ","
          << avg_process_ms << ","
          << max_process_ms_ << "\n";
    }

    std::ofstream md(md_path, std::ios::trunc);
    if (md) {
      md << "# Lab 5 — Dense Optical Flow Statistics\n\n";
      md << "Algorithm: **Farneback** (`calcOpticalFlowFarneback`)\n";
      md << "Bag: `30fps_424x240_2018-10-01-18-35-06` (424×240 @ 30 fps)\n\n";
      md << "| Flow frames | Avg mean mag (px) | Avg max mag (px) | Global max (px) | Avg moving % | Avg ms/frame | Max ms/frame |\n";
      md << "|------------:|------------------:|-----------------:|----------------:|-------------:|-------------:|-------------:|\n";
      md << std::fixed << std::setprecision(3)
          << "| " << flow_frames_
          << " | " << avg_mean_mag
          << " | " << avg_max_mag
          << " | " << max_observed_mag_
          << " | " << avg_pct_moving
          << " | " << avg_process_ms
          << " | " << max_process_ms_
          << " |\n";
    }

    LOG(INFO) << "Saved stats to " << csv_path << " and " << md_path;
  }

  bool show_images_ = true;
  std::string stats_output_dir_;
  cv::Mat prev_gray_;
  image_transport::Subscriber sub_;

  std::size_t flow_frames_ = 0;
  std::size_t skipped_first_frame_ = 0;
  double sum_mean_mag_ = 0.0;
  double sum_max_mag_ = 0.0;
  double sum_pct_moving_ = 0.0;
  double sum_process_ms_ = 0.0;
  double max_observed_mag_ = 0.0;
  double max_process_ms_ = 0.0;
};

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SelfFlow>();
  node->run();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
