// Пример C++ узла: печатает расстояние до ближайшего препятствия по лидару.
// Запуск: ros2 run solution example_cpp --ros-args -p use_sim_time:=true

#include <algorithm>
#include <cmath>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class ExampleCpp : public rclcpp::Node
{
public:
  ExampleCpp() : Node("example_cpp")
  {
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
        float closest = std::numeric_limits<float>::infinity();
        for (float r : msg->ranges) {
          if (std::isfinite(r) && r >= msg->range_min) {
            closest = std::min(closest, r);
          }
        }
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "ближайшее препятствие: %.2f м", closest);
      });
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ExampleCpp>());
  rclcpp::shutdown();
  return 0;
}
