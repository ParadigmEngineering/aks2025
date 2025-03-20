#include "rclcpp/rclcpp.hpp"
#include "mavros_msgs/msg/rc_in.hpp"
#include <chrono>
#include <functional>
#include <memory>

using namespace std::chrono_literals;

class RCPublisher : public rclcpp::Node
{
public:
    RCPublisher() : Node("rc_publisher")
    {
        // Create a publisher for mavros/rc/in
        publisher_ = this->create_publisher<mavros_msgs::msg::RCIn>("mavros/rc/in", 10);

        timer_ = this->create_wall_timer(
            500ms, std::bind(&RCPublisher::publish_rc_input, this));

        // Register shutdown callback
        rclcpp::on_shutdown(std::bind(&RCPublisher::reset_rc_values, this));
    }

private:
    void publish_rc_input()
    {
        auto message = mavros_msgs::msg::RCIn();
        message.channels.resize(8, 1500);  // Default RC values (8 channels, all at 1500)
        message.channels[5] = 2000;        // Set channel 5 to 2000 (Autonomous mode)

        publisher_->publish(message);
        RCLCPP_INFO(this->get_logger(), "Published RC input with channel 5 set to 2000");
    }

    void reset_rc_values()
    {
        RCLCPP_WARN(this->get_logger(), "Node shutting down. Resetting RC values...");

        auto message = mavros_msgs::msg::RCIn();
        message.channels.resize(8, 1500); // Reset all channels to 1500 (Manual mode)

        // Publish multiple times to ensure MAVROS receives the message
        for (int i = 0; i < 5; i++) {
            publisher_->publish(message);
            rclcpp::sleep_for(200ms);  // Small delay to allow message transmission
        }

        RCLCPP_WARN(this->get_logger(), "RC values successfully reset.");
    }

    rclcpp::Publisher<mavros_msgs::msg::RCIn>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RCPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
