#include <string>
#include <vector>

#include "adore_ros2_msgs/msg/traffic_signals.hpp"

#include "std_msgs/msg/header.hpp"
#include <rclcpp/rclcpp.hpp>

namespace adore
{

using TrafficSignalsMsg = adore_ros2_msgs::msg::TrafficSignals;
using TrafficSignalMsg  = adore_ros2_msgs::msg::TrafficSignal;

class SimulatedTrafficSignal : public rclcpp::Node
{
public:

  SimulatedTrafficSignal();

private:

  struct TrafficLightConfig
  {
    double       x;
    double       y;
    double       red_duration;
    double       yellow_duration;
    double       green_duration;
    int32_t      state;
    rclcpp::Time last_transition_time;
  };

  void update_signals();

  rclcpp::Publisher<TrafficSignalsMsg>::SharedPtr publisher;
  rclcpp::TimerBase::SharedPtr                    timer;

  std::vector<TrafficLightConfig> traffic_lights;
};
} // namespace adore