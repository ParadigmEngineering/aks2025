#include <fstream>
#include <string>
#include <vector>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std;
using namespace std::chrono_literals;



class WaypointPublisher : public rclcpp::Node
{
public:
    WaypointPublisher() : Node("waypoint_publisher"), counter(0) // Initialize counter to 0
    {
        //Read latitudes from file
        ifstream read_latitude("./aks2025/latitudes.txt"); 
        if (!read_latitude.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Error: Unable to open latitudes.txt");
        return;}
        while (getline(read_latitude, latitude)) 
        {
            
            latitude_vec.push_back(latitude);    
        }
        read_latitude.close(); 

        // Read longitudes from file
        ifstream read_longitude("./aks2025/longitudes.txt");
        while (getline(read_longitude, longitude)) 
        {
            longitude_vec.push_back(longitude);
            
        }
        read_longitude.close();
        
        // Check if latitude and longitude vectors are the same size
        if (latitude_vec.size() != longitude_vec.size()) {
            RCLCPP_ERROR(this->get_logger(), "Error: Latitude and Longitude vectors are not of equal size.");
            return;
        }
        
        
        publisher_ = this->create_publisher<std_msgs::msg::String>("waypoints", 5);
        timer_ = this->create_wall_timer(
            500ms, bind(&WaypointPublisher::timer_callback, this)
        );
    }

private:
    void timer_callback()
    {
        // Check if we have waypoints to publish
        if (latitude_vec.empty() || longitude_vec.empty()) {
            RCLCPP_WARN(this->get_logger(), "No waypoints available to publish.");
            return;
        }
        
        // Create and publish the waypoint message
        auto message = std_msgs::msg::String();
        message.data = latitude_vec[counter] + " " + longitude_vec[counter] ;
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        publisher_->publish(message);

        // Increment the counter and reset if needed
        counter++;
        if (counter >= latitude_vec.size()) {
            counter = 0; // Reset counter if we reach the end of the vector
        }
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    int counter; // Class-level counter
    vector<string> longitude_vec;  //Vector to store longitudes
    vector<string> latitude_vec ;  //Vector to store latitudes
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
