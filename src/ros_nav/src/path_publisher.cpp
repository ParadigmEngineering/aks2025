#include "rclcpp/rclcpp.hpp" // ROS 2 C++ Client Library for creating nodes, handling subscriptions, publishers, services, and timers
#include "sensor_msgs/msg/nav_sat_fix.hpp" // Header for the NavSatFix message, used for GPS data (latitude, longitude, altitude)
#include "geometry_msgs/msg/pose_stamped.hpp" // Header for the PoseStamped message, used to represent a position and orientation in 3D space with a timestamp
#include "mavros_msgs/srv/set_mode.hpp" // Header for the SetMode service, used to change the flight mode of a MAVLink-enabled vehicle
#include "ament_index_cpp/get_package_share_directory.hpp" //// Header for accessing the shared directory of a package, used to locate package-specific resources
#include <fstream> // Header for file input and output operations, used to read and write data from/to files
#include <vector> // Header for the standard vector container, used to store and manage dynamic arrays
#include <iostream> // Header for input and output stream operations, used for console input/output (e.g., std::cin, std::cout)
#include <cmath> // Header for mathematical functions, such as trigonometric, exponential, and logarithmic operations


using namespace std; // Brings all the names from the standard library (std) into the global namespace for convenience,  allowing you to use standard library features like cout, vector, etc., without the std:: prefix.
using namespace std::chrono_literals; //// Enables the use of convenient time duration literals (e.g., 10s, 500ms) from the std::chrono library without requiring the std::chrono_literals namespace prefix.


class WaypointPublisher : public rclcpp::Node // Defines a class named WaypointPublisher that inherits from rclcpp::Node, which is the base class for creating nodes in the ROS2 framework.
{
public: // what can be accessed outside the class
    WaypointPublisher() : Node("waypoint_publisher") // Constructor for the WaypointPublisher class, initializing it as a ROS2 node named "waypoint_publisher". The base class (rclcpp::Node) constructor is called with the node name to register the node within the ROS2 system.
    {
        get_coords(); //calls the get_coords function defined below.

        // Subscription to the kart's GPS position
        gps_subscriber_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(   //Initializes gps_subscriber_ as a subscriber to the "sensor_msgs::msg::NavSatFix" message type. // This subscription allows the node to receive GPS data from a topic. // The topic name and callback function would be specified in the full create_subscription call.
            "/mavros/global_position/global", rclcpp::SensorDataQoS(), //creates a subscription to the "/mavros/global_position/global" topic with a queue size of 10 messages and the callback function to be executed when a new message is received.
            [this](const sensor_msgs::msg::NavSatFix::SharedPtr msg) { //here we have the callback function. it takes a shared pointer to a received message as an argument.
                if (std::isfinite(msg->latitude) && std::isfinite(msg->longitude)) { //checks if the received GPS data is valid (finite values).
                    current_latitude_ = msg->latitude; //updates the current latitude and longitude values with the received GPS data.
                    current_longitude_ = msg->longitude;
                    RCLCPP_INFO(this->get_logger(), "Current Position: Latitude: %.6f, Longitude: %.6f",
                                current_latitude_, current_longitude_); //// Logs the current position of the node using the ROS2 logging system. It outputs a message with the current latitude and longitude values formatted to six decimal places. The `this->get_logger()` function retrieves the logger associated with the node, and `RCLCPP_INFO`  prints the message at the INFO log level.

                } else {
                    RCLCPP_WARN(this->get_logger(), "Invalid GPS data received. Waiting for valid GPS signal..."); //// Logs a warning message using the ROS2 logging system to indicate that invalid GPS data has been received. The `this->get_logger()` function retrieves the logger associated with the node, and `RCLCPP_WARN` prints the message at the WARN log level. This informs users or developers that the node is waiting for valid GPS data before proceeding.

                }
            }
        );

        // Publisher for waypoints to the MAVROS setpoint topic
        waypoint_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/mavros/setpoint_position/local", 10); //// Creates a publisher for the "/mavros/setpoint_position/local" topic, which publishes messages of type geometry_msgs::msg::PoseStamped. The topic is typically used to send local position setpoints to control 
// a robot or drone. The publisher is stored in the 'waypoint_publisher_' member variable, and the queue size 
// is set to 10, meaning up to 10 messages can be stored if the subscriber cannot keep up.


        // Create the service client to change the mode
        set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode"); // Creates a service client for the "/mavros/set_mode" service, which uses the mavros_msgs::srv::SetMode service type. This client allows the node to send requests to change the flight mode of a drone or robot. The created client is stored in the 'set_mode_client_' member variable, enabling the node to interact with the service when needed.

        // Change the mode to OFFBOARD after a delay
        services_timer_ = this->create_wall_timer(3s, [this]() {
            RCLCPP_INFO(this->get_logger(), "Initiating OFFBOARD mode...");
            change_to_offboard_mode();
        }); // // Creates a wall timer that triggers every 3 seconds. The timer calls a lambda function that logs the message "Initiating OFFBOARD mode..." using the ROS2 logging system. After logging, the function `change_to_offboard_mode()` is called to change the robot or drone to OFFBOARD mode. The timer is stored in the 'services_timer_' member variable, ensuring the callback is executed periodically.

        // Start a timer to publish dummy waypoints
        waypoint_timer_ = this->create_wall_timer(200ms, [this]() { publish_waypoint(); }); //// Creates a wall timer that triggers every 200 milliseconds. The timer calls a lambda function that invokes the `publish_waypoint()` method. This ensures that waypoints are published at regular intervals (every 200ms). The timer is stored in the 'waypoint_timer_' member variable, controlling the frequency at which waypoints are sent.

    }

private:
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
        RCLCPP_INFO(this->get_logger(), "waypoint_latidtude: %.6f, waypoint_longitude: %.6f", waypoint_latitude, waypoint_longitude);
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
        if (counter >= latitude_vec.size()) {
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
