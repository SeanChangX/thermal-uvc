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
  OpticalFlowNode() 
  : Node("optical_flow_node"), 
    first_frame_(true),
    frame_id_(0),
    next_track_id_(0),
    eps_(10.0),      // DBSCAN eps
    min_pts_(5)      // DBSCAN minPts
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/infra/gray/image_raw", 10,
      std::bind(&OpticalFlowNode::image_callback, this, std::placeholders::_1));

    publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/infra/optical_flow/image_raw", 10);

    term_criteria_ = cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 10, 0.03);
    RCLCPP_INFO(this->get_logger(), "[PARAM] eps=%.2f minPts=%d", eps_, min_pts_);
  }

private:
  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
    auto t_start = this->now();
    try {
      frame_gray = cv_bridge::toCvShare(msg, "mono8")->image;
    } catch (cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    frame_id_++;
    cv::applyColorMap(frame_gray, display_img, cv::COLORMAP_INFERNO);

    if (first_frame_) {
      old_gray_ = frame_gray.clone();
      mask_ = cv::Mat::zeros(frame_gray.size(), CV_8UC3);
      cv::goodFeaturesToTrack(old_gray_, p0_, 100, 0.3, 7);
      first_frame_ = false;
      active_tracks_.clear();
      for (const auto &pt : p0_) {
        TrackInfo tr;
        tr.id = next_track_id_++;
        tr.start_frame = frame_id_;
        tr.last_frame  = frame_id_;
        tr.point = pt;
        active_tracks_.push_back(tr);
      }
      return;
    }

    cv::Mat diff, moving_mask;
    cv::absdiff(frame_gray, old_gray_, diff);
    cv::threshold(diff, moving_mask, 25, 255, cv::THRESH_BINARY);
    cv::erode(moving_mask, moving_mask, cv::Mat(), cv::Point(-1,-1), 1);
    cv::dilate(moving_mask, moving_mask, cv::Mat(), cv::Point(-1,-1), 2);

    cv::goodFeaturesToTrack(old_gray_, p0_, 500, 0.01, 3, moving_mask);
    if (p0_.empty()) {
      for (const auto &tr : active_tracks_) {
        int len = tr.last_frame - tr.start_frame + 1;
        if (len > 1) {
          RCLCPP_INFO(this->get_logger(),
                      "[TRACK] id=%d start=%d end=%d len=%d",
                      tr.id, tr.start_frame, tr.last_frame, len);
        }
      }
      active_tracks_.clear();

      old_gray_ = frame_gray.clone();
      auto out_msg = cv_bridge::CvImage(msg->header, "bgr8", display_img).toImageMsg();
      publisher_->publish(*out_msg);
      return;
    }

    std::vector<cv::Point2f> p1;
    std::vector<uchar> status;
    std::vector<float> err;

    cv::calcOpticalFlowPyrLK(old_gray_, frame_gray, p0_, p1, status, err,
                         cv::Size(15,15), 2, term_criteria_);

    std::vector<cv::Point2f> good_new, good_old;
    std::vector<cv::Vec2f> optical_flow_vectors;
    std::vector<int> tracked_indices;

    std::vector<TrackInfo> updated_tracks;
    updated_tracks.reserve(active_tracks_.size());

    for (size_t i = 0; i < status.size(); ++i) {
      if (status[i] && err[i] < 12.0 ) {
        good_new.push_back(p1[i]);
        good_old.push_back(p0_[i]);
        optical_flow_vectors.push_back(good_new.back() - good_old.back());
        tracked_indices.push_back(i);

        if (i < active_tracks_.size()) {
          TrackInfo tr = active_tracks_[i];
          tr.last_frame = frame_id_;
          tr.point = p1[i];
          updated_tracks.push_back(tr);
        }
      } else {
        if (i < active_tracks_.size()) {
          const auto &tr = active_tracks_[i];
          int len = tr.last_frame - tr.start_frame + 1;
          if (len > 1) {
            RCLCPP_INFO(this->get_logger(),
                        "[TRACK] id=%d start=%d end=%d len=%d",
                        tr.id, tr.start_frame, tr.last_frame, len);
          }
        }
      }
    }
    active_tracks_.swap(updated_tracks);

    std::vector<clustering::Point> flow_points;
    double sum_flow_mag = 0.0;
    for (size_t i = 0; i < good_new.size(); ++i) {
      const auto& vec = good_new[i] - good_old[i]; 
      const auto& pt = good_old[i];
      flow_points.emplace_back(std::vector<float>{vec.x, vec.y, pt.x, pt.y});

      sum_flow_mag += std::hypot(vec.x, vec.y);
    }

    double avg_flow_mag = good_new.empty() ? 0.0 : sum_flow_mag / good_new.size();

    std::vector<int> cluster_labels;
    std::map<int, cv::Scalar> cluster_colors;
    std::map<int, int>        cluster_count;
    int noise_count = 0;

    std::map<int, double> cluster_flow_sum;

    if (!flow_points.empty()) {
      clustering::DBSCAN dbscan(eps_, min_pts_);
      cluster_labels = dbscan.run(flow_points);

      for (size_t i = 0; i < cluster_labels.size(); ++i) {
        int cluster_id = cluster_labels[i];

        if (cluster_id == -1) {
          noise_count++;
          continue;
        }

        cluster_count[cluster_id]++;

        const auto& vec = good_new[i] - good_old[i];
        cluster_flow_sum[cluster_id] += std::hypot(vec.x, vec.y);

        if (cluster_colors.find(cluster_id) == cluster_colors.end()) {
          cluster_colors[cluster_id] = getRandomColor();
        }
      }

      if (frame_id_ % 10 == 0) {
        RCLCPP_INFO(this->get_logger(), "[DBSCAN] Frame %d", frame_id_);
        for (auto &kv : cluster_count) {
          RCLCPP_INFO(this->get_logger(),
                      "   Cluster %d → %d pts",
                      kv.first, kv.second);
        }
        RCLCPP_INFO(this->get_logger(),
                    "   Noise     → %d pts",
                    noise_count);
      }
    }

    std::map<int, std::vector<cv::Point2f>> clusters;
    for (size_t i = 0; i < good_new.size(); ++i) {
      int cluster_id = (i < cluster_labels.size()) ? cluster_labels[i] : -1;
      if (cluster_id >= 0) {
        clusters[cluster_id].push_back(good_new[i]);
      }
    }

    static int det_global_id = 0;

    for (const auto &cluster : clusters) {
      int cid = cluster.first;
      const auto &pts = cluster.second;

      cv::Rect bounding_box = cv::boundingRect(pts);
      cv::Scalar color = cluster_colors[cid];

      cv::rectangle(display_img, bounding_box, color, 2);

      int size = static_cast<int>(pts.size());
      double avg_cluster_flow = 0.0;
      if (cluster_flow_sum.count(cid) && size > 0) {
        avg_cluster_flow = cluster_flow_sum[cid] / size;
      }

      RCLCPP_INFO(this->get_logger(),
                  "[CLUSTER] frame=%d cid=%d size=%d bbox=%.1f,%.1f,%.1f,%.1f avg_flow=%.3f",
                  frame_id_, cid, size,
                  static_cast<double>(bounding_box.x),
                  static_cast<double>(bounding_box.y),
                  static_cast<double>(bounding_box.width),
                  static_cast<double>(bounding_box.height),
                  avg_cluster_flow);
      int det_id = det_global_id++;
      double score = static_cast<double>(size);

      RCLCPP_INFO(this->get_logger(),
                  "[DET] frame=%d det_id=%d cid=%d bbox=%.1f,%.1f,%.1f,%.1f score=%.3f class=%d",
                  frame_id_, det_id, cid,
                  static_cast<double>(bounding_box.x),
                  static_cast<double>(bounding_box.y),
                  static_cast<double>(bounding_box.width),
                  static_cast<double>(bounding_box.height),
                  score,
                  0); 
    }

    auto t_end = this->now();
    double dt = (t_end - t_start).seconds();
    double fps = (dt > 0.0) ? 1.0 / dt : 0.0;

    size_t total_points = flow_points.size();
    size_t num_clusters = cluster_count.size();

    RCLCPP_INFO(this->get_logger(),
                "[FRAME] id=%d dt=%.4f fps=%.2f points=%zu clusters=%zu noise=%d avg_flow=%.3f",
                frame_id_, dt, fps,
                total_points,
                num_clusters,
                noise_count,
                avg_flow_mag);

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

  struct TrackInfo {
    int id;
    int start_frame;
    int last_frame;
    cv::Point2f point;
  };

  bool first_frame_;
  int frame_id_ = 0;
  int next_track_id_;
  double eps_;
  int min_pts_;

  std::vector<TrackInfo> active_tracks_;
  std::vector<cv::Point2f> p0_;

  cv::TermCriteria term_criteria_;

  cv::Mat old_gray_, mask_;
  cv::Mat frame_gray, display_img;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OpticalFlowNode>());
  rclcpp::shutdown();
  return 0;
}
