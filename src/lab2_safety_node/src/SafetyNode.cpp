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
        nav_msgs::msg::Odometry::SharedPtr odom;

        void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg){
            RCLCPP_INFO(this->get_logger(), "recieved something");
            if(!odom) return;
            std::vector<float> ranges = msg->ranges;
            float ttc = 100.0;
            float ttc_thresh = 1.5;
            for(unsigned int i = 0; i < ranges.size(); i++){
                if(!std::isnan(ranges[i]) && ranges[i] <= msg->range_max && ranges[i] >= msg->range_min){
                    float pt_angle = msg->angle_min + i * msg->angle_increment;
                    //RCLCPP_INFO(this->get_logger(), std::to_string(pt_angle));
                    float vel = cos(pt_angle) * odom->twist.twist.linear.x + sin(pt_angle) * odom->twist.twist.linear.y;
                   // RCLCPP_INFO(this->get_logger(), std::to_string(vel));
                    float dist = ranges[i];
                   // RCLCPP_INFO(this->get_logger(), std::to_string(dist));
                    if( vel != 0 && dist/vel > 0){
                        ttc = std::min(ttc, dist/vel);
                    }
                }
            }
            RCLCPP_INFO(this->get_logger(), std::to_string(ttc));
            if(ttc < ttc_thresh){
                RCLCPP_INFO(this->get_logger(), "brake");
                auto drive_msg_stamped =  ackermann_msgs::msg::AckermannDriveStamped();
                auto drive_msg = ackermann_msgs::msg::AckermannDrive();
                drive_msg.speed = 0.0;
                drive_msg_stamped.drive = drive_msg;
                drive_pub->publish(drive_msg_stamped);
            }
        }

        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg){
            RCLCPP_INFO(this->get_logger(), "recieved something");
            odom = msg;
        }

};

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SafetyNode>());
    rclcpp::shutdown();
    return 0;
}