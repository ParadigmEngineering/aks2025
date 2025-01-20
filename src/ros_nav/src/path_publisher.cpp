#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <fstream>
#include <vector>
#include <iostream>
#include <cmath>

//std::isfinite(msg->latitude) && std::isfinite(msg->longitude)
using namespace std;
using namespace std::chrono_literals;
using std::placeholders::_1;

class WaypointPublisher : public rclcpp::Node
{
public:
    WaypointPublisher() : Node("waypoint_publisher")
    {
        get_coords();
        // Subscription to the kart's GPS position
        gps_subscriber_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "/mavros/global_position/global", rclcpp::SensorDataQoS(), std::bind(&WaypointPublisher::gps_callback, this, _1));

        // Publisher for waypoints to the MAVROS setpoint topic
        waypoint_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/mavros/setpoint_position/local", 10);

        // Create the service client to change the mode
        set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");

        // Change the mode to OFFBOARD after a delay
        services_timer_ = this->create_wall_timer(3s, [this]() {
            RCLCPP_INFO(this->get_logger(), "Initiating OFFBOARD mode...");
            change_to_offboard_mode();
        });

        // Start a timer to publish dummy waypoints
        waypoint_timer_ = this->create_wall_timer(200ms, [this]() { publish_waypoint(); });
    }

private:
    void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Altitude: %.6f", msg->altitude);
            if (1) {
                RCLCPP_INFO(this->get_logger(), "got here!!!!");
                current_latitude_ = msg->latitude;
                current_longitude_ = msg->longitude;
                RCLCPP_INFO(this->get_logger(), "Current Position: Latitude: %.6f, Longitude: %.6f",
                            current_latitude_, current_longitude_);
            } else {
                RCLCPP_WARN(this->get_logger(), "Invalid GPS data received. Waiting for valid GPS signal...");
            }
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

        std::thread([this, request]() {
            auto result = set_mode_client_->async_send_request(request);
            try {
                if (result.wait_for(5s) == std::future_status::ready) {
                    auto response = result.get();
                    if (response->mode_sent) {
                        RCLCPP_INFO(this->get_logger(), "Successfully changed to OFFBOARD mode");
                        waypoint_timer_->cancel(); // Stop publishing dummy waypoints
                        waypoint_timer_ = this->create_wall_timer(1s, [this]() { publish_waypoint(); }); // Publish real waypoints
                    } else {
                        RCLCPP_ERROR(this->get_logger(), "Failed to change to OFFBOARD mode");
                    }
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Timeout while changing to OFFBOARD mode");
                }
            } catch (const std::exception &e) {
                RCLCPP_ERROR(this->get_logger(), "Exception in setting mode: %s", e.what());
            }
        }).detach(); // Detach the thread
    }

    void publish_waypoint()
    {
        // RCLCPP_INFO(this->get_logger(), "waypoint_latidtude: %.6f, waypoint_longitude: %.6f", waypoint_latitude, waypoint_longitude);
        RCLCPP_INFO(this->get_logger(), "current_latitude: %.6f, current_longitude: %.6f", current_latitude_, current_longitude_);
        
        get_next_waypoint();
        // Convert GPS (latitude, longitude) to a local coordinate system
        double waypoint_x = (waypoint_latitude - origin_latitude_) * meters_per_degree_;
        double waypoint_y = (waypoint_longitude - origin_longitude_) * meters_per_degree_;

        auto waypoint_msg = geometry_msgs::msg::PoseStamped();
        waypoint_msg.header.stamp = this->get_clock()->now();
        waypoint_msg.header.frame_id = "map"; // Replace with your local frame if needed
        waypoint_msg.pose.position.x = waypoint_x;
        waypoint_msg.pose.position.y = waypoint_y;
        waypoint_msg.pose.position.z = 0.0; // Assuming ground-level operation
        waypoint_msg.pose.orientation.w = 1.0; // Neutral orientation for simplicity

        RCLCPP_INFO(this->get_logger(), "Publishing Waypoint: X: %.2f, Y: %.2f", waypoint_x, waypoint_y);
        waypoint_publisher_->publish(waypoint_msg);
    }

    void get_next_waypoint()
    {
        // Ensure counter does not exceed vector size
        if (counter >= (int) latitude_vec.size()) {
            RCLCPP_INFO(this->get_logger(), "All waypoints have been processed.");
            waypoint_latitude = 0.0;
            waypoint_longitude = 0.0;
            return;
        }

        // If this is the first waypoint or we have reached the current waypoint
        if ((waypoint_latitude == 0.0 && waypoint_longitude == 0.0) ||
            (abs(waypoint_latitude - current_latitude_) < offset_latitude &&
            abs(waypoint_longitude - current_longitude_) < offset_longitude)) {
            waypoint_latitude = latitude_vec[counter];
            waypoint_longitude = longitude_vec[counter];
            RCLCPP_INFO(this->get_logger(), "Updating to next waypoint: Latitude: %.6f, Longitude: %.6f",
                        waypoint_latitude, waypoint_longitude);
            counter++;
        }
    }

    void get_coords()
    {
        // Get the path to the package's share directory
        std::string package_share_path = ament_index_cpp::get_package_share_directory("ros_nav");
        std::string latitude_file_path = package_share_path + "/data/latitudes.txt";
        std::string longitude_file_path = package_share_path + "/data/longitudes.txt";

        // Read latitudes from file
        ifstream read_latitude(latitude_file_path);
        if (!read_latitude.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Error: Unable to open %s", latitude_file_path.c_str());
            return;
        }
        double value;
        while (read_latitude >> value) {
            latitude_vec.push_back(value);
        }
        read_latitude.close();

        // Read longitudes from file
        ifstream read_longitude(longitude_file_path);
        if (!read_longitude.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Error: Unable to open %s", longitude_file_path.c_str());
            return;
        }
        while (read_longitude >> value) {
            longitude_vec.push_back(value);
        }
        read_longitude.close();

        // Check if latitude and longitude vectors are the same size
        if (latitude_vec.size() != longitude_vec.size()) {
            RCLCPP_ERROR(this->get_logger(), "Error: Latitude and Longitude vectors are not of equal size.");
            return;
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr waypoint_publisher_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
    rclcpp::TimerBase::SharedPtr waypoint_timer_;
    rclcpp::TimerBase::SharedPtr services_timer_;

    double current_latitude_ = 0.0;
    double current_longitude_ = 0.0;

    double waypoint_latitude = 0.0;
    double waypoint_longitude = 0.0;

    const double origin_latitude_ = 40.437688;
    const double origin_longitude_ = -86.944469;

    const double meters_per_degree_ = 111000.0;

    const double offset_latitude = 0.1;
    const double offset_longitude = 0.1;


    vector<double> longitude_vec;  // Vector to store longitudes
    vector<double> latitude_vec;  // Vector to store latitudes

    int counter = 0;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WaypointPublisher>());
    rclcpp::shutdown();
    return 0;
}
