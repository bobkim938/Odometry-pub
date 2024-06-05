#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include <math.h>


class odom : public rclcpp::Node {
    public:
        odom() : Node("pose_odometry")
        {
            RCLCPP_INFO(this->get_logger(), "pose_odom node initialized");
            auto qos = rclcpp::QoS(10);  // 10 is the history depth
            qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
            encoder_subscription_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
                    "encoder_data", qos, std::bind(&odom::encoder_callback, this, std::placeholders::_1));
            odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
            last_time = this->now().seconds();
        }

    private:
        rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr encoder_subscription_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
        nav_msgs::msg::Odometry odom_msg = nav_msgs::msg::Odometry();

        // encoder data
        int encoder_data_left = 0;
        int encoder_data_right = 0;
        int interpolation_rate = 4; // 4 times interpolation rate
        int encoder_resolution = 1024; // 1024 counts per revolution
        double wheel_diameter = 0.15; // 15 cm
        double distance_left = 0.0;
        double distance_right = 0.0;
        double distance_avg = 0.0;
        double wheel_to_wheel = 0.3; // 30 cm
        double delta_theta = 0.0; // change in angles in radians

        double current_time = 0.0;
        double last_time = 0.0;
        double last_theta = 0.0;

        // robot pose
        tf2::Quaternion q;
        double prev_x = 0.0;
        double prev_y = 0.0;
        double x = 0.0;
        double y = 0.0;
        double theta = 0.0;
        double dx = 0.0;
        double dy = 0.0;
        double dt = 0.0;

        void encoder_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
            current_time = this->now().seconds();
            encoder_data_left = msg->data.at(0);
            encoder_data_right = msg->data.at(1);
            dt = current_time - last_time;
            RCLCPP_INFO(this->get_logger(), "Encoder_data_left: '%d'", encoder_data_left);
            RCLCPP_INFO(this->get_logger(), "Encoder_data_right: '%d'", encoder_data_right);
            distance_left = (encoder_data_left * (wheel_diameter * 3.14159)/2.0) / ((interpolation_rate * encoder_resolution)/2.0);
            distance_right = (encoder_data_right * (wheel_diameter * 3.14159)/2.0) / ((interpolation_rate * encoder_resolution)/2.0);
            distance_avg = (distance_left + distance_right) / 2.0;
            delta_theta = (distance_right - distance_left) / wheel_to_wheel;
            RCLCPP_INFO(this->get_logger(), "Distance_left: '%f'", distance_left);
            RCLCPP_INFO(this->get_logger(), "Distance_right: '%f'", distance_right);
            RCLCPP_INFO(this->get_logger(), "Distance_avg: '%f'", distance_avg);
            RCLCPP_INFO(this->get_logger(), "Delta_theta: '%f'", delta_theta);
            calculate_odom();
            last_time = current_time;
        }

        void calculate_odom() {
            last_theta = theta;
            theta += delta_theta;
            q.setRPY(0.0, 0.0, theta);
            x += distance_avg * cos(theta);
            y += distance_avg * sin(theta);
            dx = x - prev_x;
            dy = y - prev_y;
            if(theta > M_PI) {
                theta -= 2*M_PI;
            }
            else if(theta < -M_PI) {
                theta += 2*M_PI;
            }
            odom_msg.header.stamp = this->now();
            odom_msg.header.frame_id = "odom";
            odom_msg.child_frame_id = "base_link";
            odom_msg.pose.pose.position.x = x;
            odom_msg.pose.pose.position.y = y;
            odom_msg.pose.pose.position.z = 0.0;
            odom_msg.pose.pose.orientation.w = q.getW();
            odom_msg.pose.pose.orientation.x = q.getX();
            odom_msg.pose.pose.orientation.y = q.getY();
            odom_msg.pose.pose.orientation.z = q.getZ();
            odom_msg.twist.twist.linear.x = dx/dt;
            odom_msg.twist.twist.linear.y = dy/dt;
            odom_msg.twist.twist.angular.z = (theta - last_theta)/dt;
            this->odom_publisher_->publish(odom_msg);
            prev_x = x;
            prev_y = y;
        }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<odom>();
    rclcpp::spin(node);
    rclcpp::shutdown();
}
