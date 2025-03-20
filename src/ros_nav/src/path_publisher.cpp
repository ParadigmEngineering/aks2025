#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "mavros_msgs/msg/rc_in.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "geographic_msgs/msg/geo_pose_stamped.hpp"
#include <tf2/LinearMath/Quaternion.h> 
#include <fstream>
#include <vector>
#include <iostream>
#include <cmath>

using namespace std;
using namespace std::chrono_literals;
using std::placeholders::_1;

class WaypointPublisher : public rclcpp::Node
{
public:
    WaypointPublisher() : Node("waypoint_publisher")
    {
        // Subscribe to the control mode (RC/Autonomous)
        control_mode_subscriber_ = this->create_subscription<mavros_msgs::msg::RCIn>(
            "/mavros/rc/in", rclcpp::SensorDataQoS(), 
            std::bind(&WaypointPublisher::control_mode_callback, this, _1));

        RCLCPP_INFO(this->get_logger(), "Waypoint Publisher Node Initialized. Waiting for control mode...");
    }


private:
void control_mode_callback(const mavros_msgs::msg::RCIn::SharedPtr msg)
{
    if (msg->channels[5] == 1500) {
        if (is_autonomous) {
            RCLCPP_INFO(this->get_logger(), "Switching to RC mode. Stopping Autonomous Mode.");
            waypoint_timer_->cancel(); // Stop waypoint publishing
            services_timer_->cancel(); // Stop mode-changing service
        }
        is_autonomous = false;
        RCLCPP_INFO(this->get_logger(), "Kart is in RC mode");
    } 
    else if (msg->channels[5] == 2000) {
        if (!is_autonomous) {  // Only start if it's the first time switching to Autonomous mode
            is_autonomous = true;
            RCLCPP_INFO(this->get_logger(), "Going Autonomous!");
            
            // Start autonomous processes when mode is switched
            get_coords();
            gps_subscriber_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
                "/mavros/global_position/global", rclcpp::SensorDataQoS(),
                std::bind(&WaypointPublisher::gps_callback, this, _1));

            waypoint_publisher_ = this->create_publisher<geographic_msgs::msg::GeoPoseStamped>(
                "/mavros/setpoint_position/global", 10);

            set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");

            services_timer_ = this->create_wall_timer(3s, [this]() {
                RCLCPP_INFO(this->get_logger(), "Initiating OFFBOARD mode...");
                change_to_offboard_mode();
            });

            waypoint_timer_ = this->create_wall_timer(200ms, [this]() { publish_waypoint(); });
        }
    }
}
    void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
        if (isfinite(msg->latitude) && isfinite(msg->longitude)) {
            current_latitude_ = msg->latitude;
            current_longitude_ = msg->longitude;
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
        // RCLCPP_INFO(this->get_logger(), "waypoint_latitude: %.6f, waypoint_longitude: %.6f", waypoint_latitude, waypoint_longitude);
        
        get_next_waypoint();
        // Convert GPS (latitude, longitude) to a local coordinate system
        // double waypoint_x = (waypoint_latitude - origin_latitude_) * meters_per_degree_;
        // double waypoint_y = (waypoint_longitude - origin_longitude_) * meters_per_degree_;

        // Calculate heading
        double heading = calculate_heading(current_latitude_, current_longitude_, waypoint_latitude, waypoint_longitude);

        // Convert heading to quaternion
        tf2::Quaternion quaternion;
        quaternion.setRPY(0, 0, heading); // Roll and pitch are 0; yaw is the heading

        // Set message values
        auto waypoint_msg = geographic_msgs::msg::GeoPoseStamped();
        waypoint_msg.header.stamp = this->get_clock()->now();
        waypoint_msg.header.frame_id = "global"; // Replace with your local frame if needed
        waypoint_msg.pose.position.latitude = waypoint_latitude;
        waypoint_msg.pose.position.longitude = waypoint_longitude;
        waypoint_msg.pose.position.altitude = 0.0; // Assuming ground-level operation
        waypoint_msg.pose.orientation.x = quaternion.x();
        waypoint_msg.pose.orientation.y = quaternion.y();
        waypoint_msg.pose.orientation.z = quaternion.z();
        waypoint_msg.pose.orientation.w = quaternion.w();

        RCLCPP_INFO(this->get_logger(), "Publishing Waypoint: latitude: %.2f, longitude: %.2f, current counter = %d", waypoint_latitude, waypoint_longitude, counter);
        waypoint_publisher_->publish(waypoint_msg);
    }

    double calculate_heading(double lat1, double lon1, double lat2, double lon2)
    {
        // Convert degrees to radians
        lat1 = lat1 * M_PI / 180.0;
        lon1 = lon1 * M_PI / 180.0;
        lat2 = lat2 * M_PI / 180.0;
        lon2 = lon2 * M_PI / 180.0;

        double dlon = lon2 - lon1;

        // Calculate the heading in radians
        double y = sin(dlon) * cos(lat2);
        double x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dlon);
        double heading = atan2(y, x);

        // Normalize the heading to [0, 2π)
        if (heading < 0) {
            heading += 2 * M_PI;
        }

        return heading;
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
        double latitude_differential = abs(waypoint_latitude - current_latitude_);
        double longitude_differential = abs(waypoint_longitude - current_longitude_);
        RCLCPP_INFO(this->get_logger(), "latitude_differential = %.6f, longitude_differential = %.6f", latitude_differential, longitude_differential);
        // If this is the first waypoint or we have reached the current waypoint
        if ((waypoint_latitude == 0.0 && waypoint_longitude == 0.0) || (latitude_differential < offset_latitude && longitude_differential < offset_longitude)) {
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
    rclcpp::Subscription<mavros_msgs::msg::RCIn>::SharedPtr control_mode_subscriber_;
    rclcpp::Publisher<geographic_msgs::msg::GeoPoseStamped>::SharedPtr waypoint_publisher_;
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

    const double offset_latitude = 0.00004;
    const double offset_longitude = 0.00004;


    vector<double> longitude_vec;  // Vector to store longitudes
    vector<double> latitude_vec;  // Vector to store latitudes

    int counter = 0;
    bool is_autonomous = false;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WaypointPublisher>());
    rclcpp::shutdown();
    return 0;
}
