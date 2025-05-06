#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class OpticalFlowNode : public rclcpp::Node {
public:
  OpticalFlowNode() : Node("optical_flow_node"), first_frame_(true) {
    using std::placeholders::_1;

    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/infra/gray/image_raw", 10,
      std::bind(&OpticalFlowNode::image_callback, this, _1));

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
      first_frame_ = false;
      return;
    }

    cv::Mat diff, moving_mask;
    cv::absdiff(frame_gray, old_gray_, diff);
    cv::threshold(diff, moving_mask, 25, 255, cv::THRESH_BINARY);
    cv::erode(moving_mask, moving_mask, cv::Mat(), cv::Point(-1,-1), 1);
    cv::dilate(moving_mask, moving_mask, cv::Mat(), cv::Point(-1,-1), 2);

    cv::goodFeaturesToTrack(old_gray_, p0_, 50, 0.3, 7, moving_mask);
    if (p0_.empty()) {
      old_gray_ = frame_gray.clone();
      return;
    }

    std::vector<cv::Point2f> p1;
    std::vector<uchar> status;
    std::vector<float> err;

    cv::calcOpticalFlowPyrLK(old_gray_, frame_gray, p0_, p1, status, err, cv::Size(15,15), 2, term_criteria_);

    std::vector<cv::Point2f> good_new, good_old;
    for (size_t i = 0; i < status.size(); ++i) {
      if (status[i] && err[i] < 12.0 ) {
        good_new.push_back(p1[i]);
        good_old.push_back(p0_[i]);
      }
    }

    cv::Mat display_img;
    cv::cvtColor(frame_gray, display_img, cv::COLOR_GRAY2BGR);

    for (size_t i = 0; i < good_new.size(); ++i) {
      cv::circle(display_img, good_new[i], 5, cv::Scalar(0, 0, 255), -1);
    }

    cv::Mat output;
    cv::add(display_img, mask_, output);

    auto out_msg = cv_bridge::CvImage(msg->header, "bgr8", output).toImageMsg();
    publisher_->publish(*out_msg);

    old_gray_ = frame_gray.clone();
    p0_ = good_new;
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
