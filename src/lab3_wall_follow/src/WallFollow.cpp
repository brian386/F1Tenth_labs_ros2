#include <rclcpp/rclcpp.hpp>
#include <string>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

#define PI 3.14159265358979323846

class WallFollow: public rclcpp::Node {
    public:
        WallFollow(): Node("wall_follow_node"){
            lidar_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
              lidarscan_topic, 10, std::bind(&WallFollow::scan_callback, this, std::placeholders::_1)  
            );
        }
    private:
        // PID CONTROL PARAMS
        // TODO: double kp =
        // TODO: double kd =
        // TODO: double ki =
        double servo_offset = 0.0;
        double prev_error = 0.0;
        double error = 0.0;
        double integral = 0.0;
        double L = 1.0;
        // Topics
        std::string lidarscan_topic = "/scan";
        std::string drive_topic = "/drive";
        /// TODO: create ROS subscribers and publishers
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub;

        double get_wall_angle(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg){
            float angle_increment = scan_msg->angle_increment;
            std::vector<float> ranges = scan_msg->ranges;
            float min_angle = scan_msg->angle_min; 

            //get 2 reference ranges and angles
            double angle_a = 50.0/180 * PI;
            double range_a = -1.0;
            double angle_b = 90.0/180 * PI; 
            double range_b = -1.0;
            for(unsigned int i = 0; i < ranges.size(); i++){
                double cur_angle = min_angle + i*angle_increment;
                bool in_range = ranges[i] <= scan_msg->range_max && ranges[i] >= scan_msg->range_min;
                if(range_b < 0 && cur_angle >= angle_b && in_range){
                    angle_b = cur_angle;
                    range_b = ranges[i];
                }
                if(range_a < 0 && cur_angle >= angle_a && in_range){
                    angle_a = cur_angle;
                    range_a = ranges[i];
                } 
            }

            double theta = angle_b - angle_a;
            double wall_angle = atan((range_a*cos(theta) - range_b)/(range_a*sin(theta)));
            RCLCPP_INFO(this->get_logger(), "angle: '%f'", wall_angle);
            return wall_angle; 
        }

        void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg){
            get_wall_angle(scan_msg);
        }

};

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WallFollow>());
    rclcpp::shutdown();
    return 0;
}