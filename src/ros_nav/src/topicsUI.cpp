#include "rclcpp/rclcpp.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "QApplication"
#include "QLabel"
#include "QVBoxLayout"
#include "QWidget"
#include "QFont"
#include "memory"
#include "thread"
#include "iomanip"
#include "sstream"

using std::placeholders::_1;
using namespace std;
using namespace std::chrono_literals;

class ROS2StateMonitor : public QWidget, public rclcpp::Node
{
public:
    // Use shared pointer factory method
    static std::shared_ptr<ROS2StateMonitor> create() {
        return std::shared_ptr<ROS2StateMonitor>(new ROS2StateMonitor());
    }

    void update_state_ui(const mavros_msgs::msg::State& msg) {
        armed_label->setText(QString("Armed: %1").arg(msg.armed ? "YES" : "NO"));
        mode_label->setText(QString("Mode: %1").arg(QString::fromStdString(msg.mode)));
    }

    void update_gps_ui(const sensor_msgs::msg::NavSatFix& msg) {
        std::ostringstream ss;
    
        ss << std::fixed << std::setprecision(6) << "Latitude: " << msg.latitude << "°";
        lat_label->setText(QString::fromStdString(ss.str()));
        ss.str("");

        ss << std::fixed << std::setprecision(6) << "Longitutde: " << msg.longitude << "°";
        lon_label->setText(QString::fromStdString(ss.str()));
        ss.str("");

    }

    void update_velo_ui(const geometry_msgs::msg::TwistStamped& msg){
        std::ostringstream ss;
    
        ss << std::fixed << std::setprecision(2) << "x velocity: " << msg.twist.linear.x << "(m/s)";
        x_velo_label->setText(QString::fromStdString(ss.str()));
        ss.str("");

        ss << std::fixed << std::setprecision(2) << "y velocity: " << msg.twist.linear.y << "(m/s)";
        y_velo_label->setText(QString::fromStdString(ss.str()));
        ss.str("");
    }

private:
    ROS2StateMonitor() 
    : Node("mavros_state_monitor"), 
      connected_label(nullptr),
      armed_label(nullptr),
      guided_label(nullptr),
      manual_input_label(nullptr),
      mode_label(nullptr),
      system_status_label(nullptr),
      lat_label(nullptr),
      lon_label(nullptr),
      x_velo_label(nullptr),
      y_velo_label(nullptr)
    {
        // Initialize ROS2 subscriber
        subscription_ = this->create_subscription<mavros_msgs::msg::State>(
            "/mavros/state", 10,
            [this](const mavros_msgs::msg::State::SharedPtr msg) {
                // Use Qt signal mechanism to update UI thread-safely
                QMetaObject::invokeMethod(this, [this, msg]() {
                    update_state_ui(*msg);
                });
            });

        gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "/mavros/global_position/global", rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
                // Use Qt signal mechanism to update UI thread-safely
                QMetaObject::invokeMethod(this, [this, msg]() {
                    update_gps_ui(*msg);
                });
            });

        velo_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "/mavros/local_position/velocity_body", rclcpp::SensorDataQoS(),
            [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
                // Use Qt signal mechanism to update UI thread-safely
                QMetaObject::invokeMethod(this, [this, msg]() {
                    update_velo_ui(*msg);
                });
            });
        


        // Set up UI
        setup_ui();
    }

    void setup_ui() {
        setWindowTitle("MAVROS Monitor");
        setFixedSize(800, 200);

        QGridLayout *layout = new QGridLayout(this);

        QLabel *title_label = new QLabel("State Info");
        QFont title_font = title_label->font();
        title_font.setPointSize(16);
        title_font.setBold(true);
        title_label->setFont(title_font);

        QLabel *vel_label = new QLabel("Velocity Info");
        QFont vel_font = vel_label->font();
        vel_font.setPointSize(16);
        vel_font.setBold(true);
        vel_label->setFont(vel_font);

        QLabel *gps_label = new QLabel("GPS Info");
        QFont gps_font = vel_label->font();
        gps_font.setPointSize(16);
        gps_font.setBold(true);
        gps_label->setFont(gps_font);

        armed_label = create_state_label("Armed: N/A");
        mode_label = create_state_label("Mode: N/A");
        lat_label = create_state_label("Latitude: N/A");
        lon_label = create_state_label("Longitude: N/A");
        x_velo_label = create_state_label("x velocity: N/A");
        y_velo_label = create_state_label("y velocity: N/A");

        layout->addWidget(title_label, 0, 0);
        layout->addWidget(gps_label, 0, 2);
        layout->addWidget(vel_label, 0, 1);
        layout->addWidget(armed_label, 1, 0);
        layout->addWidget(mode_label, 2, 0);
        layout->addWidget(lat_label, 1, 2);
        layout->addWidget(lon_label, 2, 2);
        layout->addWidget(x_velo_label, 1, 1);
        layout->addWidget(y_velo_label, 2, 1);
        

        setLayout(layout);
    }

    QLabel* create_state_label(const QString& text) {
        QLabel *label = new QLabel(text);
        label->setAlignment(Qt::AlignLeft);
        QFont font = label->font();
        font.setPointSize(12);
        label->setFont(font);
        return label;
    }

    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr subscription_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>:: SharedPtr velo_sub_;

    QLabel *connected_label;
    QLabel *armed_label;
    QLabel *guided_label;
    QLabel *manual_input_label;
    QLabel *mode_label;
    QLabel *system_status_label;

    QLabel *lat_label;
    QLabel *lon_label;

    QLabel *x_velo_label;
    QLabel *y_velo_label;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    QApplication app(argc, argv);
    
    auto monitor = ROS2StateMonitor::create();
    monitor->show();
    
    // Start ROS2 spinner in separate thread
    std::thread ros_thread([monitor]() {
        rclcpp::spin(monitor);
    });
    
    int result = app.exec();
    
    rclcpp::shutdown();
    if(ros_thread.joinable()) ros_thread.join();
    
    return result;
}