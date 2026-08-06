#include "lk_feature_tracker.h"

#include <glog/logging.h>

#include <numeric>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <vector>

#include "ring_buffer.hpp"

using namespace cv;

/**
  LK feature tracker Constructor.
*/
LKFeatureTracker::LKFeatureTracker(bool show_images) : show_images_(show_images) {
  if (show_images_) {
    cv::namedWindow(window_name_, cv::WINDOW_NORMAL);
  }
}

void LKFeatureTracker::printStats() const {
  LOG(INFO) << "Avg. Keypoints 1 Size: " << avg_num_keypoints_img1_;
  LOG(INFO) << "Avg. Keypoints 2 Size: " << avg_num_keypoints_img2_;
  LOG(INFO) << "Avg. Number of matches: " << avg_num_matches_;
  LOG(INFO) << "Avg. Number of good matches: NA";
  LOG(INFO) << "Avg. Number of Inliers: " << avg_num_inliers_;
  LOG(INFO) << "Avg. Inliers ratio: " << avg_inlier_ratio_;
  LOG(INFO) << "Num. of samples: " << num_samples_;
}

LKFeatureTracker::~LKFeatureTracker() {
  printStats();
  if (show_images_) {
    cv::destroyWindow(window_name_);
  }
}

/** TODO This is the main tracking function. It takes in the current frame and
 * detects features that correspond to the previous frame.
 @param[in] frame Current image frame
*/
void LKFeatureTracker::trackFeatures(const cv::Mat& frame) {
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  //  DELIVERABLE 7 | Feature Tracking: Lucas-Kanade Tracker
  // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~  ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
  //
  // ~~~~ begin solution
  static constexpr int max_corners = 500;
  static constexpr double quality_level = 0.01;
  static constexpr double min_distance = 10.0;
  static constexpr int block_size = 3;
  static constexpr double harris_k = 0.04;

  Mat gray;
  cvtColor(frame, gray, COLOR_BGR2GRAY);

  if (prev_frame_.empty()) {
    prev_frame_ = gray.clone();
    goodFeaturesToTrack(gray,
                        prev_corners_,
                        max_corners,
                        quality_level,
                        min_distance,
                        Mat(),
                        block_size,
                        true,
                        harris_k);
    show(frame, prev_corners_, prev_corners_);
    return;
  }

  std::vector<Point2f> curr_corners;
  std::vector<uchar> status;
  std::vector<float> err;
  const TermCriteria criteria(TermCriteria::COUNT | TermCriteria::EPS, 30, 0.01);
  calcOpticalFlowPyrLK(prev_frame_,
                       gray,
                       prev_corners_,
                       curr_corners,
                       status,
                       err,
                       Size(21, 21),
                       3,
                       criteria);

  std::vector<Point2f> good_prev;
  std::vector<Point2f> good_curr;
  good_prev.reserve(status.size());
  good_curr.reserve(status.size());
  for (size_t i = 0; i < status.size(); ++i) {
    if (status[i]) {
      good_prev.push_back(prev_corners_[i]);
      good_curr.push_back(curr_corners[i]);
    }
  }

  unsigned int num_inliers = 0;
  if (good_prev.size() >= 8) {
    std::vector<uchar> inlier_mask;
    if (inlierMaskComputation(good_prev, good_curr, &inlier_mask)) {
      for (const uchar inlier : inlier_mask) {
        if (inlier) {
          ++num_inliers;
        }
      }
    }
  }

  const double new_num_samples = static_cast<double>(num_samples_) + 1.0;
  const double old_num_samples = static_cast<double>(num_samples_);
  avg_num_keypoints_img1_ =
      static_cast<float>((avg_num_keypoints_img1_ * old_num_samples +
                          static_cast<double>(prev_corners_.size())) /
                         new_num_samples);
  avg_num_keypoints_img2_ =
      static_cast<float>((avg_num_keypoints_img2_ * old_num_samples +
                          static_cast<double>(good_curr.size())) /
                         new_num_samples);
  avg_num_matches_ = static_cast<float>(
      (avg_num_matches_ * old_num_samples + static_cast<double>(good_prev.size())) /
      new_num_samples);
  avg_num_inliers_ =
      static_cast<float>((avg_num_inliers_ * old_num_samples +
                          static_cast<double>(num_inliers)) /
                         new_num_samples);
  avg_inlier_ratio_ = static_cast<float>(
      (avg_inlier_ratio_ * old_num_samples +
       (good_prev.empty()
            ? 0.0
            : static_cast<double>(num_inliers) /
                  static_cast<double>(good_prev.size()))) /
      new_num_samples);
  ++num_samples_;

  show(frame, good_prev, good_curr);

  prev_frame_ = gray.clone();
  prev_corners_ = good_curr;
  if (prev_corners_.size() < 50) {
    goodFeaturesToTrack(gray,
                        prev_corners_,
                        max_corners,
                        quality_level,
                        min_distance,
                        Mat(),
                        block_size,
                        true,
                        harris_k);
  }
  // ~~~~ end solution
  // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
  //                             end deliverable 7
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
}

bool LKFeatureTracker::inlierMaskComputation(const std::vector<Point2f>& pts1,
                                             const std::vector<Point2f>& pts2,
                                             std::vector<uchar>* inlier_mask) const {
  CHECK_NOTNULL(inlier_mask);

  bool mask_computed = true;  // always optimistic...
  static constexpr double max_dist_from_epi_line_in_px = 3.0;
  static constexpr double confidence_prob = 0.99;
  try {
    findFundamentalMat(pts1,
                       pts2,
                       FM_RANSAC,
                       max_dist_from_epi_line_in_px,
                       confidence_prob,
                       *inlier_mask);
  } catch (...) {
    LOG(WARNING) << "Inlier Mask could not be computed, this can happen if there"
                    "are not enough features tracked.";
    mask_computed = false;
  }
  return mask_computed;
}

/** TODO Display image with tracked features from prev to curr on the image
 * corresponding to 'frame'
 * @param[in] frame The current image frame, to draw the feature track on
 * @param[in] prev The previous set of keypoints
 * @param[in] curr The set of keypoints for the current frame
 */
void LKFeatureTracker::show(const cv::Mat& frame,
                            std::vector<cv::Point2f>& prev,
                            std::vector<cv::Point2f>& curr) {
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // ~~~~ begin solution
  if (!show_images_) {
    return;
  }

  Mat display = frame.clone();
  for (size_t i = 0; i < prev.size() && i < curr.size(); ++i) {
    if (prev[i] != curr[i]) {
      line(display, prev[i], curr[i], Scalar(0, 255, 0), 2, LINE_AA);
    }
    circle(display, curr[i], 4, Scalar(0, 0, 255), -1, LINE_AA);
  }
  imshow(window_name_, display);
  // ~~~~ end solution
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
}
