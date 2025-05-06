#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>
#include <libuvc/libuvc.h>

#include <iostream>
#include <thread>

class ThermalCameraNode : public rclcpp::Node {
public:
  ThermalCameraNode()
  : Node("thermal_camera_node"), gray_pub_(nullptr), bgr_pub_(nullptr), devh_(nullptr) {
    gray_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/infra/gray/image_raw", 10);
    bgr_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/infra/color/image_raw", 10);

    RCLCPP_INFO(this->get_logger(), "Thermal Camera Node started");
    RCLCPP_INFO(this->get_logger(), "Initializing libuvc...");

    if (uvc_init(&ctx_, NULL) < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to init libuvc");
      return;
    }

    if (uvc_find_device(ctx_, &dev_, 0x2bdf, 0x0102, NULL) < 0) {
      RCLCPP_ERROR(this->get_logger(), "Device not found");
      uvc_exit(ctx_);
      return;
    }

    if (uvc_open(dev_, &devh_) < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open device");
      uvc_unref_device(dev_);
      uvc_exit(ctx_);
      return;
    }

    print_device_formats();

    if (!try_streaming_formats(&ctrl_)) {
      RCLCPP_ERROR(this->get_logger(), "Couldn't find a compatible streaming format!");
      cleanup();
      return;
    }

    // 在不同的執行緒中處理影像
    streaming_thread_ = std::thread(&ThermalCameraNode::start_streaming, this);
  }

  ~ThermalCameraNode() {
    if (streaming_thread_.joinable()) {
      streaming_thread_.join();  // 確保執行緒結束後再清理
    }
    cleanup();
  }

private:
  void start_streaming() {
    if (uvc_start_streaming(devh_, &ctrl_, &ThermalCameraNode::frame_callback, this, 0) < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to start streaming!");
      cleanup();
      return;
    }
  }

  static void frame_callback(uvc_frame_t *frame, void *ptr) {
    auto *self = static_cast<ThermalCameraNode *>(ptr);

    // 在獨立執行緒處理影像
    std::thread gray_thread(&ThermalCameraNode::process_gray, self, frame);
    std::thread color_thread(&ThermalCameraNode::process_color, self, frame);

    gray_thread.join();
    color_thread.join();
  }

  static void process_gray(ThermalCameraNode *self, uvc_frame_t *frame) {
    uvc_frame_t *gray = uvc_allocate_frame(frame->width * frame->height);
    if (gray && uvc_yuyv2y(frame, gray) == UVC_SUCCESS) {
      cv::Mat gray_img(cv::Size(gray->width, gray->height), CV_8UC1, gray->data);
      auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", gray_img).toImageMsg();
      msg->header.stamp = self->now();
      self->gray_pub_->publish(*msg);
      uvc_free_frame(gray);
    }
  }

  static void process_color(ThermalCameraNode *self, uvc_frame_t *frame) {
    uvc_frame_t *bgr = uvc_allocate_frame(frame->width * frame->height * 3);
    if (bgr && uvc_any2bgr(frame, bgr) == UVC_SUCCESS) {
      cv::Mat bgr_img(cv::Size(bgr->width, bgr->height), CV_8UC3, bgr->data);
      cv::Mat colored;
      cv::applyColorMap(bgr_img, colored, cv::COLORMAP_JET);
      auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", colored).toImageMsg();
      msg->header.stamp = self->now();
      self->bgr_pub_->publish(*msg);
      uvc_free_frame(bgr);
    }
  }

  void print_device_formats() {
    const uvc_format_desc_t *format_desc = uvc_get_format_descs(devh_);
    while (format_desc) {
      const uvc_frame_desc_t *frame_desc = format_desc->frame_descs;
      while (frame_desc) {
        RCLCPP_INFO(this->get_logger(), "Format: %dx%d @ ~%dfps, Format Index: %d",
                    frame_desc->wWidth, frame_desc->wHeight,
                    10000000 / frame_desc->dwDefaultFrameInterval,
                    format_desc->bFormatIndex); 
  
        if (format_desc->bFormatIndex == UVC_FRAME_FORMAT_YUYV) {
          RCLCPP_INFO(this->get_logger(), "YUYV format supported!");
        }
  
        frame_desc = frame_desc->next;
      }
      format_desc = format_desc->next;
    }
  }

  bool try_streaming_formats(uvc_stream_ctrl_t *ctrl) {
    if (uvc_get_stream_ctrl_format_size(devh_, ctrl, UVC_FRAME_FORMAT_YUYV, 256, 192, 25) == 0)
      return true;

    const uvc_format_desc_t *format_desc = uvc_get_format_descs(devh_);
    while (format_desc) {
      const uvc_frame_desc_t *frame_desc = format_desc->frame_descs;
      while (frame_desc) {
        if (uvc_get_stream_ctrl_format_size(devh_, ctrl,
            UVC_FRAME_FORMAT_ANY, frame_desc->wWidth, frame_desc->wHeight, 0) == 0)
          return true;
        frame_desc = frame_desc->next;
      }
      format_desc = format_desc->next;
    }
    return false;
  }

  void cleanup() {
    if (devh_) uvc_close(devh_);
    if (dev_) uvc_unref_device(dev_);
    if (ctx_) uvc_exit(ctx_);
  }

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr gray_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr bgr_pub_;

  uvc_context_t *ctx_ = nullptr;
  uvc_device_t *dev_ = nullptr;
  uvc_device_handle_t *devh_ = nullptr;
  uvc_stream_ctrl_t ctrl_;

  std::thread streaming_thread_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ThermalCameraNode>());
  rclcpp::shutdown();
  return 0;
}
