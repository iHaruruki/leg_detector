// leg_detector_limited.cpp
#include <memory>
#include <vector>
#include <cmath>
#include <string>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

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
    // サブスクライバーの作成
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10, std::bind(&LegDetector::scan_callback, this, _1));

    // 対象とするスキャン角度を指定（例：-45°～45°）
    scan_angle_min_ = -M_PI / 4;  // -45度 [rad]
    scan_angle_max_ = M_PI / 4;   //  45度 [rad]

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
    // index = (desired_angle - msg_angle_min) / angle_increment
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
      // 無限大やNaNは無視
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
          // クラスタが3点以上あれば採用する
          if (current_cluster.size() >= 3) {
            leg_clusters.push_back(current_cluster);
          }
          current_cluster.clear();
          current_cluster.push_back({i, r});
        }
      }
    }
    // 最後のクラスタを処理
    if (current_cluster.size() >= 3) {
      leg_clusters.push_back(current_cluster);
    }

    // 各クラスタごとに物理的な幅を計算して人の足候補かをチェック
    bool leg_detected = false;  // 足が検出されたかどうかのフラグ

    for (const auto & cluster : leg_clusters)
    {
      int idx_start = cluster.front().index;
      int idx_end   = cluster.back().index;
      // クラスタの角度幅 [rad]
      float angle_width = (idx_end - idx_start) * angle_increment;

      // クラスタ内の平均距離
      float sum_range = 0.0;
      for (const auto & point : cluster) {
        sum_range += point.range;
      }
      float mean_range = sum_range / cluster.size();

      // 物理的な幅（弦の長さ）の計算
      float physical_width = 2 * mean_range * std::sin(angle_width / 2);

      // 足と判定する条件（例: 幅0.05～0.3m, 距離1.0m以内）
      if ((physical_width > 0.05) && (physical_width < 0.3) && (mean_range < distance_threshold_)) {
        // 足が検出された場合、「足検知」とログ出力
        RCLCPP_INFO(this->get_logger(), "足検知");
        leg_detected = true;

        // 必要に応じて詳細情報をログ出力（オプション）
        /*
        RCLCPP_INFO(this->get_logger(), "Leg detected: Index [%d - %d], Range = %.2f m, Width = %.2f m",
                    idx_start, idx_end, mean_range, physical_width);
        */
      }
    }

    // 足が検出されなかった場合
    if (!leg_detected) {
      RCLCPP_DEBUG(this->get_logger(), "指定角度・距離内のスキャンでは足は検出されませんでした。");
    }
  }

  // サブスクライバー
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;

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
