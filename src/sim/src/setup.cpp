#include "rclcpp/rclcpp.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

using namespace std;

// future Sam!!!! The idea here is to sub to the vehicle's current speed and then send it back to maintain the necessary message frequency to arm and enable offboard mode

class Setup : public rclcpp::Node
{
public:
    Setup() : Node("setup")
    {
        // Create subscription to the velocity topic
        subscription_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "/mavros/local_position/velocity", 10,
            std::bind(&Setup::publish_dummy, this, std::placeholders::_1));

        // Create a publisher to publish velocities
        publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/mavros/set_velocity/local", 10);

        // Create a client for the /mavros/cmd/arming service
        arm_client_ = this->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");

        // Create the service client to change the mode
        set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");

        services_timer_ = this->create_wall_timer(3s, [this]() {
            send_arm_command();
            change_to_offboard_mode();
        });

    }
private:

    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arm_client_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr services_timer_;
    rclcpp::TimerBase::SharedPtr dummy_timer_;

    void publish_dummy(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
        auto velocity_msg = geometry_msgs::msg::TwistStamped();
        velocity_msg.header = msg->header; // Preserve the original header
        velocity_msg.twist = msg->twist;   // Copy the twist (linear and angular velocity)

        publisher_->publish(velocity_msg);
    }

    void send_arm_command()
    {
        if (!arm_client_->wait_for_service(5s)) {
            RCLCPP_ERROR(this->get_logger(), "SetMode service not available");
            return;
        }

        auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
        request->value = true;

        // Send the service request asynchronously
        arm_client_->async_send_request(request,
            [this](rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture future) {
                auto response = future.get();
                if (response->success) {
                    RCLCPP_INFO(this->get_logger(), "Vehicle armed successfully!");
                } else {
                    RCLCPP_WARN(this->get_logger(), "Failed to arm/disarm the vehicle.");
                }
            });
    }

    void change_to_offboard_mode()
    {
        if (!set_mode_client_->wait_for_service(5s)) {
            RCLCPP_ERROR(this->get_logger(), "SetMode service not available");
            return;
        }

        auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
        request->base_mode = 0; // No change to base_mode
        request->custom_mode = "OFFBOARD";

        // Send the service request asynchronously
        set_mode_client_->async_send_request(request,
            [this](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future) {
                auto response = future.get();
                if (response->mode_sent) {
                    RCLCPP_INFO(this->get_logger(), "Mode changed to OFFBOARD successfully!");
                } else {
                    RCLCPP_WARN(this->get_logger(), "Failed to change mode to OFFBOARD.");
                }
            });
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Setup>());
    rclcpp::shutdown();
    return 0;
}
