#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

class SafetyNode: public rclcpp::Node{
    public:
        SafetyNode(): Node("safety_node"){
            drive_pub = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
                "/drive", 10
            );
            lidar_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
                "/scan", 10, std::bind(&SafetyNode::scan_callback, this, std::placeholders::_1)
            );
            odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
                "/ego_racecar/odom", 10, std::bind(&SafetyNode::odom_callback, this, std::placeholders::_1)
            );
        }

    private:
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
        rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub;
        void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg){
            RCLCPP_INFO(this->get_logger(), "recieved something");
        }

        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg){
            RCLCPP_INFO(this->get_logger(), "recieved something");
        }

};

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SafetyNode>());
    rclcpp::shutdown();
    return 0;
}