// leg_detector_marker_detect.cpp
#include <memory>
#include <vector>
#include <cmath>
#include <string>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker.hpp"  // RViz2用のMarker

using std::placeholders::_1;

// クラスタ内の各点を表す構造体
struct ClusterPoint {
  int index;
  float range;
};

class LegDetector : public rclcpp::Node
{
public:
  LegDetector()
  : Node("leg_detector")
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10, std::bind(&LegDetector::scan_callback, this, _1));

    // RViz2用のMarkerパブリッシャーの作成
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);

    // 対象とする角度範囲 [-45°, 45°] (ラジアン)
    scan_angle_min_ = -M_PI / 4;
    scan_angle_max_ = M_PI / 4;
    // 対象となる距離の上限 [m]
    distance_threshold_ = 1.0;

    RCLCPP_INFO(this->get_logger(), "スキャン角度範囲: [%.2f, %.2f] [rad], 距離: %.2f m以内",
                scan_angle_min_, scan_angle_max_, distance_threshold_);
  }

private:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    const std::vector<float>& ranges = msg->ranges;
    float angle_increment = msg->angle_increment;
    float msg_angle_min = msg->angle_min;

    // 対象となる角度範囲に対応するインデックスを求める
    int start_index = std::max(0, static_cast<int>(std::ceil((scan_angle_min_ - msg_angle_min) / angle_increment)));
    int end_index   = std::min(static_cast<int>(ranges.size()) - 1,
                                 static_cast<int>(std::floor((scan_angle_max_ - msg_angle_min) / angle_increment)));

    if (start_index >= end_index) {
      RCLCPP_WARN(this->get_logger(), "指定した角度範囲に有効なデータがありません。");
      return;
    }

    // クラスタリング処理の準備
    std::vector<std::vector<ClusterPoint>> leg_clusters;
    std::vector<ClusterPoint> current_cluster;

    // 隣接する点間の差が diff_threshold 以内なら同一クラスタとみなす (単位:m)
    const float diff_threshold = 0.1;

    // 指定角度範囲内のデータのみ処理
    for (int i = start_index; i <= end_index; ++i)
    {
      float r = ranges[i];
      if (!std::isfinite(r)) {
        continue;
      }
      // 距離がdistance_threshold_以内以外は除外
      if (r > distance_threshold_) {
        continue;
      }

      if (current_cluster.empty()) {
        current_cluster.push_back({i, r});
      } else {
        float last_range = current_cluster.back().range;
        if (std::fabs(r - last_range) < diff_threshold) {
          current_cluster.push_back({i, r});
        } else {
          if (current_cluster.size() >= 3) {
            leg_clusters.push_back(current_cluster);
          }
          current_cluster.clear();
          current_cluster.push_back({i, r});
        }
      }
    }
    if (current_cluster.size() >= 3) {
      leg_clusters.push_back(current_cluster);
    }

    // 足検出結果の判定
    bool leg_detected = false;
    int marker_id = 0;  // 複数のマーカーを識別するためのID
    visualization_msgs::msg::Marker marker;

    // 検出した各クラスタについて処理
    for (const auto & cluster : leg_clusters)
    {
      int idx_start = cluster.front().index;
      int idx_end   = cluster.back().index;
      // クラスタの角度幅 [rad]
      float angle_width = (idx_end - idx_start) * angle_increment;

      // クラスタ内の平均距離の計算
      float sum_range = 0.0;
      for (const auto & point : cluster) {
        sum_range += point.range;
      }
      float mean_range = sum_range / cluster.size();

      // クラスタの開始角度と終了角度から中央角度を計算
      float angle_start = msg_angle_min + idx_start * angle_increment;
      float angle_end   = msg_angle_min + idx_end * angle_increment;
      float cluster_angle = (angle_start + angle_end) / 2;

      // 物理的な幅の計算（弦の長さ）
      float physical_width = 2 * mean_range * std::sin(angle_width / 2);

      // 足と判定する条件 (例: 幅0.05～0.3 m, 距離1.0 m以内)
      if ((physical_width > 0.05) && (physical_width < 0.3) && (mean_range < distance_threshold_)) {
        // 足が検出されたと判断
        leg_detected = true;
        // RViz2用のMarker作成
        //visualization_msgs::msg::Marker marker;
        marker.header.frame_id = msg->header.frame_id;
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "leg_detector";
        marker.id = marker_id++;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        // 極座標から直交座標 (x,y) を計算
        marker.pose.position.x = mean_range * std::cos(cluster_angle);
        marker.pose.position.y = mean_range * std::sin(cluster_angle);
        marker.pose.position.z = 0.0;
        marker.pose.orientation.w = 1.0;
        // マーカーサイズ (視認性のために調整)
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        // マーカーの色 (緑)
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;

        marker_pub_->publish(marker);
      }
    }

    // 足を検出している場合のみ「足検知」を表示し、
    // 検出がなかった場合は「不検出」と表示
    if (leg_detected) {
      RCLCPP_INFO(this->get_logger(), "足検知");
    }
    else {
      RCLCPP_INFO(this->get_logger(), "不検出");
      marker.action = visualization_msgs::msg::Marker::DELETE;
      marker_pub_->publish(marker);
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

  double scan_angle_min_;
  double scan_angle_max_;
  double distance_threshold_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LegDetector>());
  rclcpp::shutdown();
  return 0;
}
