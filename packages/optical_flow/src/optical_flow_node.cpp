#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <cmath>
#include <map>
#include <random>
#include <optical_flow/dbscan.h>

class OpticalFlowNode : public rclcpp::Node {
public:
  OpticalFlowNode() : Node("optical_flow_node"), first_frame_(true) {
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/infra/gray/image_raw", 10,
      std::bind(&OpticalFlowNode::image_callback, this, std::placeholders::_1));

    publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/infra/optical_flow/image_raw", 10);

    term_criteria_ = cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 10, 0.03);
  }

private:
  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
    cv::Mat frame_gray;
    try {
      frame_gray = cv_bridge::toCvShare(msg, "mono8")->image;
    } catch (cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    if (first_frame_) {
      old_gray_ = frame_gray.clone();
      mask_ = cv::Mat::zeros(frame_gray.size(), CV_8UC3);
      cv::goodFeaturesToTrack(old_gray_, p0_, 100, 0.3, 7);
      first_frame_ = false;
      return;
    }

    cv::Mat diff, moving_mask;
    cv::absdiff(frame_gray, old_gray_, diff);
    cv::threshold(diff, moving_mask, 25, 255, cv::THRESH_BINARY);
    cv::erode(moving_mask, moving_mask, cv::Mat(), cv::Point(-1,-1), 1);
    cv::dilate(moving_mask, moving_mask, cv::Mat(), cv::Point(-1,-1), 2);

    cv::goodFeaturesToTrack(old_gray_, p0_, 500, 0.01, 3, moving_mask);
    if (p0_.empty()) {
      old_gray_ = frame_gray.clone();
      return;
    }

    std::vector<cv::Point2f> p1;
    std::vector<uchar> status;
    std::vector<float> err;

    cv::calcOpticalFlowPyrLK(old_gray_, frame_gray, p0_, p1, status, err, cv::Size(15,15), 2, term_criteria_);

    std::vector<cv::Point2f> good_new, good_old;
    std::vector<cv::Vec2f> optical_flow_vectors;
    std::vector<int> tracked_indices;

    for (size_t i = 0; i < status.size(); ++i) {
      if (status[i] && err[i] < 12.0 ) {
        good_new.push_back(p1[i]);
        good_old.push_back(p0_[i]);
        optical_flow_vectors.push_back(good_new.back() - good_old.back());
        tracked_indices.push_back(i);
      }
    }

    std::vector<clustering::Point> flow_points;
    for (size_t i = 0; i < good_new.size(); ++i) {
      const auto& vec = good_new[i] - good_old[i]; 
      const auto& pt = good_old[i];
      flow_points.emplace_back(std::vector<float>{vec.x, vec.y, pt.x, pt.y});
    }

    std::vector<int> cluster_labels;
    std::map<int, cv::Scalar> cluster_colors;
    if (!flow_points.empty()) {
      clustering::DBSCAN dbscan(15.0, 5);
      cluster_labels = dbscan.run(flow_points);
    
      for (size_t i = 0; i < cluster_labels.size(); ++i) {
        int cluster_id = cluster_labels[i];
        if (cluster_colors.find(cluster_id) == cluster_colors.end()) {
          cluster_colors[cluster_id] = getRandomColor();
        }
      }
    }

    std::map<int, std::vector<cv::Point2f>> clusters;
    for (size_t i = 0; i < good_new.size(); ++i) {
      int cluster_id = cluster_labels[i];
      if (cluster_id >= 0) {
        clusters[cluster_id].push_back(good_new[i]);
      }
    }

    cv::Mat display_img;
    cv::cvtColor(frame_gray, display_img, cv::COLOR_GRAY2BGR);

    for (const auto &cluster : clusters) {
      // Get the bounding box for each cluster
      cv::Rect bounding_box = cv::boundingRect(cluster.second);
      cv::Scalar color = cluster_colors[cluster.first];

      // Draw the bounding box
      cv::rectangle(display_img, bounding_box, color, 2);
    }

    auto out_msg = cv_bridge::CvImage(msg->header, "bgr8", display_img).toImageMsg();
    publisher_->publish(*out_msg);

    old_gray_ = frame_gray.clone();
    p0_ = good_new;
  }

  cv::Scalar getRandomColor() {
    static std::mt19937 rng{std::random_device{}()};
    std::uniform_int_distribution<int> dist(0, 255);
    return cv::Scalar(dist(rng), dist(rng), dist(rng));
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  cv::TermCriteria term_criteria_;
  bool first_frame_;
  std::vector<cv::Point2f> p0_;
  cv::Mat old_gray_, mask_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OpticalFlowNode>());
  rclcpp::shutdown();
  return 0;
}
