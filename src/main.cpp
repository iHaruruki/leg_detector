// leg_detector_with_marker.cpp
#include <memory>
#include <vector>
#include <cmath>
#include <string>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker.hpp"  // Marker メッセージ

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
    // /scan トピックのサブスクライバー
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10, std::bind(&LegDetector::scan_callback, this, _1));

    // RViz2 に表示するための marker のパブリッシャー
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);

    // 対象とするスキャン角度を指定（例：-45°～45°）
    scan_angle_min_ = -M_PI / 4;  // -45° [rad]
    scan_angle_max_ = M_PI / 4;   //  45° [rad]

    // 対象となる距離の上限 [m]
    distance_threshold_ = 1.0;

    RCLCPP_INFO(this->get_logger(), "スキャン角度範囲: [%.2f, %.2f] [rad], 距離: %.2fm以内",
                scan_angle_min_, scan_angle_max_, distance_threshold_);
  }

private:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    const std::vector<float>& ranges = msg->ranges;
    float angle_increment = msg->angle_increment;
    float msg_angle_min = msg->angle_min;

    // 指定した角度範囲に対応するインデックスを算出
    int start_index = std::max(0, static_cast<int>(std::ceil((scan_angle_min_ - msg_angle_min) / angle_increment)));
    int end_index   = std::min(static_cast<int>(ranges.size()) - 1,
                       static_cast<int>(std::floor((scan_angle_max_ - msg_angle_min) / angle_increment)));

    if (start_index >= end_index) {
      RCLCPP_WARN(this->get_logger(), "指定した角度範囲に有効なデータがありません。");
      return;
    }

    // クラスタリングのための準備
    std::vector<std::vector<ClusterPoint>> leg_clusters;
    std::vector<ClusterPoint> current_cluster;

    // 隣接する点間の距離差がこの値以下なら同一クラスタとみなす（単位: m）
    const float diff_threshold = 0.1;

    // 指定した角度範囲内のデータを処理
    for (int i = start_index; i <= end_index; ++i)
    {
      float r = ranges[i];
      // 無限大や NaN は無視
      if (!std::isfinite(r)) {
        continue;
      }

      // 距離が distance_threshold_ 以内でなければ処理対象外とする
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

    // 各クラスタごとに足候補を判定し、RViz 用の Marker を作成
    bool leg_detected = false;
    int marker_id = 0;  // 複数の marker を表示する場合の識別子

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

      // クラスタの先頭と最後の角度を算出し、クラスタの中央角度を求める
      float angle_start = msg_angle_min + idx_start * angle_increment;
      float angle_end   = msg_angle_min + idx_end * angle_increment;
      float cluster_angle = (angle_start + angle_end) / 2;

      // 物理的な幅（弦の長さ）の計算
      float physical_width = 2 * mean_range * std::sin(angle_width / 2);

      // 足として判定する条件（例: 幅 0.05～0.3m, 距離1.0m以内）
      if ((physical_width > 0.05) && (physical_width < 0.3) && (mean_range < distance_threshold_)) {
        // 足検知があった場合、"足検知" とログに出力
        RCLCPP_INFO(this->get_logger(), "足検知");

        // RViz 用 Marker を作成
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = msg->header.frame_id; // センサのフレーム (例: "laser")
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "leg_detector";
        marker.id = marker_id++;  // 識別子はクラスタごとに付与

        // マーカーの種類を SPHERE に設定
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        // クラスタ中央の座標を計算 (極座標から直交座標に変換)
        marker.pose.position.x = mean_range * std::cos(cluster_angle);
        marker.pose.position.y = mean_range * std::sin(cluster_angle);
        marker.pose.position.z = 0.0;  // 平面上と仮定

        // マーカーの姿勢は単位四元数 (回転なし)
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        // マーカーの大きさ (検出箇所の見やすさのために調整)
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;

        // マーカーの色 (例: 緑色)
        marker.color.a = 1.0;  // 透過度 1.0 (不透明)
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;

        // Marker を RViz2 にパブリッシュ
        marker_pub_->publish(marker);

        leg_detected = true;
      }
    }

    if (!leg_detected) {
      RCLCPP_DEBUG(this->get_logger(), "指定角度・距離内のスキャンでは足は検出されませんでした。");
    }
  }

  // サブスクライバー
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;

  // Marker パブリッシャー (RViz2 用)
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

  // 対象とするスキャン角度の下限・上限 [rad]
  double scan_angle_min_;
  double scan_angle_max_;

  // 対象となる距離の上限 [m]
  double distance_threshold_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LegDetector>());
  rclcpp::shutdown();
  return 0;
}
