/*
 * @file self_flow.cpp
 * @brief Dense optical flow visualization (Farneback).
 */

#include <cv_bridge/cv_bridge.h>
#include <glog/logging.h>

#include <cstdlib>
#include <functional>
#include <image_transport/image_transport.hpp>
#include <memory>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

class SelfFlow : public rclcpp::Node {
 public:
  SelfFlow() : Node("self_flow") {
    declare_parameter<bool>("show_images", true);
    show_images_ = get_parameter("show_images").as_bool();
  }

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
        showFlow(image, flow);
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
  bool show_images_ = true;
  cv::Mat prev_gray_;
  image_transport::Subscriber sub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SelfFlow>();
  node->run();
  rclcpp::spin(node);
  return EXIT_SUCCESS;
}
