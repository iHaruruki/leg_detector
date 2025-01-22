// leg_detector_with_angle_filter.cpp
#include <memory>
#include <vector>
#include <cmath>
#include <string>

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

    // 処理対象とするスキャン角度を指定（例：-45°～45°）
    scan_angle_min_ = -M_PI / 4;  // -45度
    scan_angle_max_ = M_PI / 4;   //  45度

    RCLCPP_INFO(this->get_logger(), "スキャン角度範囲: [%.2f, %.2f] [rad]", scan_angle_min_, scan_angle_max_);
  }

private:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    const std::vector<float>& ranges = msg->ranges;
    float angle_increment = msg->angle_increment;
    float msg_angle_min = msg->angle_min;
    float msg_angle_max = msg->angle_max;

    // 指定した角度範囲に対応するインデックスを計算
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
      // 無限大や NaN の値はスキップ
      if (!std::isfinite(r)) {
        continue;
      }

      if (current_cluster.empty()) {
        current_cluster.push_back({i, r});
      } else {
        float last_range = current_cluster.back().range;
        if (std::fabs(r - last_range) < diff_threshold) {
          current_cluster.push_back({i, r});
        } else {
          // クラスタの点数が3以上ならクラスタとして採用
          if (current_cluster.size() >= 3) {
            leg_clusters.push_back(current_cluster);
          }
          current_cluster.clear();
          current_cluster.push_back({i, r});
        }
      }
    }
    // ループ終了後、残ったクラスタを採用
    if (current_cluster.size() >= 3) {
      leg_clusters.push_back(current_cluster);
    }

    // クラスタごとに物理的な幅を計算して人の足候補かをチェック
    std::vector<std::string> detected_legs;
    for (const auto & cluster : leg_clusters)
    {
      int idx_start = cluster.front().index;
      int idx_end   = cluster.back().index;
      // クラスタの角度幅 [rad]
      float angle_width = (idx_end - idx_start) * angle_increment;

      // クラスタ内の距離の平均値
      float sum_range = 0.0;
      for (const auto & point : cluster) {
        sum_range += point.range;
      }
      float mean_range = sum_range / cluster.size();

      // 平均距離と角度幅から、近似的な物理的幅（弦の長さ）を計算
      float physical_width = 2 * mean_range * std::sin(angle_width / 2);

      // 人の足とみなす条件（例: 幅 0.05～0.3m, 距離 3.0m 未満）
      if ((physical_width > 0.05) && (physical_width < 0.3) && (mean_range < 3.0)) {
        detected_legs.push_back("Leg[" +
          std::to_string(idx_start) + "-" + std::to_string(idx_end) +
          "]: range = " + std::to_string(mean_range) +
          ", width = " + std::to_string(physical_width));
      }
    }

    // 検出結果をログに出力
    if (!detected_legs.empty()) {
      std::string log_msg = "検知した足候補: ";
      for (const auto & leg : detected_legs) {
        log_msg += leg + "; ";
      }
      RCLCPP_INFO(this->get_logger(), "%s", log_msg.c_str());
    } else {
      RCLCPP_DEBUG(this->get_logger(), "指定角度内のスキャンでは足は検出されませんでした。");
    }
  }

  // サブスクライバー
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  // 処理対象とするスキャン角度の下限・上限 [rad]
  double scan_angle_min_;
  double scan_angle_max_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  // LegDetector ノードのインスタンスを生成してスピン
  rclcpp::spin(std::make_shared<LegDetector>());
  rclcpp::shutdown();
  return 0;
}
