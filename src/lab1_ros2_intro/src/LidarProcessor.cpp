#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class LidarProcessor: public rclcpp::Node{
    public:
        LidarProcessor(): Node("lidar_processor") {
            closest_pub = this->create_publisher<std_msgs::msg::Float32>("/closest_point", 10);
            farthest_pub = this->create_publisher<std_msgs::msg::Float32>("/farthest_point", 10);
            subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
                "/scan", 10, std::bind(&LidarProcessor::scan_callback, this, std::placeholders::_1)
            );
        }
    private:
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr closest_pub;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr farthest_pub;

        void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg){
            std::vector<float> ranges = msg->ranges;
            float closest_point = 1000000000.0;
            float farthest_point = 0.0;
            for(unsigned int i = 0; i < ranges.size(); i++){
                if(ranges[i] >= msg->range_min && ranges[i] <= msg->range_max){
                    closest_point = std::min(closest_point, ranges[i]);
                    farthest_point = std::max(farthest_point, ranges[i]);
                }
            }
            auto closest_msg = std_msgs::msg::Float32();
            closest_msg.data = closest_point;
            auto farthest_msg = std_msgs::msg::Float32();
            farthest_msg.data = farthest_point;
            closest_pub->publish(closest_msg);
            farthest_pub->publish(farthest_msg);
            RCLCPP_INFO(this->get_logger(), "Closest: '%f', Farthest: '%f'", closest_point, farthest_point);
            RCLCPP_INFO(this->get_logger(), "Range: '%f', to '%f'", msg->range_min, msg->range_max);
        }
};

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarProcessor>());
    rclcpp::shutdown();
    return 0;
}