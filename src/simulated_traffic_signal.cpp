#include "simulated_traffic_signal.hpp"
#include <unistd.h>
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

namespace adore
{

SimulatedTrafficSignal::SimulatedTrafficSignal() :
  Node( "traffic_signal_node" )
{
  // Declare parameters
  declare_parameter( "traffic_lights", std::vector<std::string>{} );
  std::vector<std::string> traffic_lights_names;
  get_parameter( "traffic_lights", traffic_lights_names );

  for( const auto &light_name : traffic_lights_names )
  {
    double x, y, red_duration, yellow_duration, green_duration;
    declare_parameter( light_name + ".x", 0.0 );
    declare_parameter( light_name + ".y", 0.0 );
    declare_parameter( light_name + ".red_duration", 5.0 );
    declare_parameter( light_name + ".yellow_duration", 2.0 );
    declare_parameter( light_name + ".green_duration", 5.0 );

    get_parameter( light_name + ".x", x );
    get_parameter( light_name + ".y", y );
    get_parameter( light_name + ".red_duration", red_duration );
    get_parameter( light_name + ".yellow_duration", yellow_duration );
    get_parameter( light_name + ".green_duration", green_duration );

    traffic_lights.emplace_back( TrafficLightConfig{ x, y, red_duration, yellow_duration, green_duration, TrafficSignalMsg::RED, now() } );
  }

  declare_parameter( "permanent_red", false );
  get_parameter( "permanent_red", permanent_red );

  // Publisher
  publisher = create_publisher<TrafficSignalsMsg>( "traffic_signals", 10 );

  // Subscirber
  subscriber_user_input = create_subscription<std_msgs::msg::String>( "user_input", 1, std::bind( &SimulatedTrafficSignal::user_input_callback, this, std::placeholders::_1 ) );

  // Timer
  timer = create_wall_timer( std::chrono::milliseconds( 100 ), std::bind( &SimulatedTrafficSignal::update_signals, this ) );
}

void
SimulatedTrafficSignal::update_signals()
{
  TrafficSignalsMsg msg;
  msg.header.stamp = now();

  for( auto &light : traffic_lights )
  {
    auto current_time = now();
    auto elapsed_time = ( current_time - light.last_transition_time ).seconds();

    switch( light.state )
    {
      case TrafficSignalMsg::RED:
        if( elapsed_time >= light.red_duration && !permanent_red )
        {
          light.state                = TrafficSignalMsg::GREEN;
          light.last_transition_time = current_time;
        }
        break;

      case TrafficSignalMsg::GREEN:
        if( elapsed_time >= light.green_duration )
        {
          light.state                = TrafficSignalMsg::YELLOW;
          light.last_transition_time = current_time;
        }
        break;

      case TrafficSignalMsg::YELLOW:
        if( elapsed_time >= light.yellow_duration )
        {
          light.state                = TrafficSignalMsg::RED;
          light.last_transition_time = current_time;
        }
        break;

      default:
        light.state = TrafficSignalMsg::UNKNOWN;
        break;
    }

    TrafficSignalMsg signal_msg;
    signal_msg.x               = light.x;
    signal_msg.y               = light.y;
    signal_msg.signal_group_id = 1; // Example ID
    signal_msg.state           = light.state;

    msg.signals.push_back( signal_msg );
  }

  msg.header.frame_id = "world";

  publisher->publish( msg );
}

void
SimulatedTrafficSignal::user_input_callback( const std_msgs::msg::String& msg )
{
  if ( msg.data == "turn green")
  {
    for (auto &light : traffic_lights)
    {
      std::cerr << "Turning traffic lights green" << std::endl;
      light.state = TrafficSignalMsg::GREEN;
      light.last_transition_time = now();
    }
  }
  
}

} // namespace adore

int
main( int argc, char **argv )
{
  rclcpp::init( argc, argv );
  rclcpp::spin( std::make_shared<adore::SimulatedTrafficSignal>() );
  rclcpp::shutdown();
  return 0;
}
