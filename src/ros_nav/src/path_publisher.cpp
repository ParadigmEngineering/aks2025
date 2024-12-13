#include "ament_index_cpp/get_package_share_directory.hpp"
#include <fstream>
#include <string>
#include <vector>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std;

class WaypointPublisher : public rclcpp::Node
{
public:
    WaypointPublisher() : Node("waypoint_publisher"), counter(0)
    {
        try {
            // Get the package's share directory
            string package_share_dir = ament_index_cpp::get_package_share_directory("ros_nav");
            string latitude_file = package_share_dir + "/data/latitudes.txt";
            string longitude_file = package_share_dir + "/data/longitudes.txt";

            // Read latitudes from file
            ifstream read_latitude(latitude_file);
            if (!read_latitude.is_open()) {
                RCLCPP_ERROR(this->get_logger(), "Error: Unable to open %s", latitude_file.c_str());
                return;
            }
            while (getline(read_latitude, latitude)) {
                latitude_vec.push_back(latitude);
            }
            read_latitude.close();

            // Read longitudes from file
            ifstream read_longitude(longitude_file);
            if (!read_longitude.is_open()) {
                RCLCPP_ERROR(this->get_logger(), "Error: Unable to open %s", longitude_file.c_str());
                return;
            }
            while (getline(read_longitude, longitude)) {
                longitude_vec.push_back(longitude);
            }
            read_longitude.close();

            // Check if latitude and longitude vectors are the same size
            if (latitude_vec.size() != longitude_vec.size()) {
                RCLCPP_ERROR(this->get_logger(), "Error: Latitude and Longitude vectors are not of equal size.");
                return;
            }

            subscriber_ = this->create_subscriber<std_msgs::msg::String>("/mavros/global_position/global", 5);
            publisher_ = this->create_publisher<std_msgs::msg::String>("waypoints", 5);
            timer_ = this->create_wall_timer(
                500ms, bind(&WaypointPublisher::timer_callback, this)
            );
        }
        catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Exception: %s", e.what());
        }
    }

private:
    void timer_callback()
    {
        if (latitude_vec.empty() || longitude_vec.empty()) {
            RCLCPP_WARN(this->get_logger(), "No waypoints available to publish.");
            return;
        }

        auto message = std_msgs::msg::String();
        message.data = latitude_vec[counter] + " " + longitude_vec[counter];
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        publisher_->publish(message);
        RCLCPP_INFO(this->get_logger(), "Subscribing: '%s", subscriber_.c_str());
        counter++;
        if (counter >= static_cast<int>(latitude_vec.size())) {
            counter = 0;
        }
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Subscriber<std_msgs::msg::String>::SharedPth subscriber_;
    int counter;
    vector<string> longitude_vec;
    vector<string> latitude_vec;
    string latitude;
    string longitude;
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(make_shared<WaypointPublisher>());
    rclcpp::shutdown();
    return 0;
}
