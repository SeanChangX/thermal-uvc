#include <vector>
#include <cmath>
#include <iostream>

namespace clustering {

enum PointStatus {
  UNCLASSIFIED,
  CLUSTERED,
  NOISE
};

struct Point {
  std::vector<float> data;
  PointStatus status;
  int cluster_id;

  Point(const std::vector<float>& d) : data(d), status(UNCLASSIFIED), cluster_id(-1) {}

  double distance(const Point& other) const {
    double dist_sq = 0;
    for (size_t i = 0; i < data.size(); ++i) {
      dist_sq += (data[i] - other.data[i]) * (data[i] - other.data[i]);
    }
    return std::sqrt(dist_sq);
  }
};

class DBSCAN {
public:
  DBSCAN(double eps, int minPts) : eps_(eps), minPts_(minPts), cluster_count_(0) {}

  std::vector<int> run(std::vector<Point>& points) {
    cluster_count_ = 0;
    for (auto& point : points) {
      if (point.status == PointStatus::UNCLASSIFIED) {
        if (expandCluster(points, point, cluster_count_)) {
          cluster_count_++;
        }
      }
    }

    std::vector<int> labels(points.size());
    for (size_t i = 0; i < points.size(); ++i) {
      labels[i] = points[i].cluster_id;
    }
    return labels;
  }

private:
  std::vector<int> regionQuery(const std::vector<Point>& points, const Point& p) const {
    std::vector<int> neighbors;
    for (size_t i = 0; i < points.size(); ++i) {
      if (p.distance(points[i]) <= eps_) {
        neighbors.push_back(i);
      }
    }
    return neighbors;
  }

  bool expandCluster(std::vector<Point>& points, Point& p, int clusterId) {
    std::vector<int> neighbors = regionQuery(points, p);
    if (neighbors.size() < static_cast<size_t>(minPts_)) {
      p.status = PointStatus::NOISE;
      return false;
    } else {
      p.status = PointStatus::CLUSTERED;
      p.cluster_id = clusterId;
      std::vector<int> seedList = neighbors;
      size_t index = 0;
      while (index < seedList.size()) {
        int currentPointIndex = seedList[index];
        Point& currentPoint = points[currentPointIndex];
        if (currentPoint.status == PointStatus::UNCLASSIFIED || currentPoint.status == PointStatus::NOISE) {
          if (currentPoint.status == PointStatus::UNCLASSIFIED) {
            std::vector<int> newNeighbors = regionQuery(points, currentPoint);
            if (newNeighbors.size() >= static_cast<size_t>(minPts_)) {
              seedList.insert(seedList.end(), newNeighbors.begin(), newNeighbors.end());
            }
          }
          currentPoint.status = PointStatus::CLUSTERED;
          currentPoint.cluster_id = clusterId;
        }
        index++;
      }
      return true;
    }
  }

  double eps_;
  int minPts_;
  int cluster_count_;
};

} // namespace clustering