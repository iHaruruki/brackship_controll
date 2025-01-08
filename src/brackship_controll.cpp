#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "../include/brackship_controll/serial.hpp"
#include <vector>

class BrackShipDriver : public rclcpp::Node
{
public:
    BrackShipDriver() : Node("brackship_driver"), x_(0.0), y_(0.0), theta_(0.0)
    {
        // シリアルポートの設定
        if (!serial_.InitSerial((char*)"/dev/ttyUSB0", B19200))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize serial port");
            rclcpp::shutdown();
        }

        // cmd_vel購読
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&BrackShipDriver::cmd_vel_callback, this, std::placeholders::_1));

        // エンコーダデータのパブリッシュ
        encoder_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

        // Transform Broadcasterの初期化
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&BrackShipDriver::read_encoder_data, this));
    }

private:
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        int right_speed = static_cast<int>(msg->linear.x + msg->angular.z);
        int left_speed = static_cast<int>(msg->linear.x - msg->angular.z);

        uint8_t command[7] = {0x02, 0x01, 0x07, 0xF0,
                              static_cast<uint8_t>(right_speed), static_cast<uint8_t>(left_speed), 0x03};
        serial_.Write2(command, 7);
    }

    void read_encoder_data()
    {
        uint8_t command[5] = {0x02, 0x01, 0x05, 0xF9, 0x03};
        serial_.Write2(command, 5);

        uint8_t response[10] = {0};
        serial_.Read2(response, 10);

        if (response[0] == 0x02 && response[9] == 0x03)
        {
            int right_count = (response[5] << 8) | response[6];
            int left_count = (response[7] << 8) | response[8];

            double wheel_separation = 0.5;  // 車輪間の距離[m]
            double wheel_radius = 0.15;     // 車輪の半径[m]
            double ticks_per_revolution = 500.0;
            double distance_per_tick = (2 * M_PI * wheel_radius) / ticks_per_revolution;

            double right_distance = right_count * distance_per_tick;
            double left_distance = left_count * distance_per_tick;
            double delta_distance = (right_distance + left_distance) / 2.0;
            double delta_theta = (right_distance - left_distance) / wheel_separation;

            // オドメトリの更新
            x_ += delta_distance * cos(theta_);
            y_ += delta_distance * sin(theta_);
            theta_ += delta_theta;

            // オドメトリメッセージの作成
            nav_msgs::msg::Odometry odom_msg;
            odom_msg.header.stamp = this->now();
            odom_msg.header.frame_id = "odom";
            odom_msg.child_frame_id = "base_link";

            odom_msg.pose.pose.position.x = x_;
            odom_msg.pose.pose.position.y = y_;
            odom_msg.pose.pose.position.z = 0.0;

            tf2::Quaternion q;
            q.setRPY(0, 0, theta_);
            odom_msg.pose.pose.orientation.x = q.x();
            odom_msg.pose.pose.orientation.y = q.y();
            odom_msg.pose.pose.orientation.z = q.z();
            odom_msg.pose.pose.orientation.w = q.w();

            odom_pub_->publish(odom_msg);

            // エンコーダデータのパブリッシュ
            auto joint_state = sensor_msgs::msg::JointState();
            joint_state.name = {"right_wheel", "left_wheel"};
            joint_state.position = {static_cast<double>(right_count), static_cast<double>(left_count)};
            encoder_pub_->publish(joint_state);

            // TFのブロードキャスト
            geometry_msgs::msg::TransformStamped odom_tf;
            odom_tf.header.stamp = this->now();
            odom_tf.header.frame_id = "odom";
            odom_tf.child_frame_id = "base_link";

            odom_tf.transform.translation.x = x_;
            odom_tf.transform.translation.y = y_;
            odom_tf.transform.translation.z = 0.0;

            odom_tf.transform.rotation.x = q.x();
            odom_tf.transform.rotation.y = q.y();
            odom_tf.transform.rotation.z = q.z();
            odom_tf.transform.rotation.w = q.w();

            tf_broadcaster_->sendTransform(odom_tf);
        }
    }

    CSerial serial_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr encoder_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;

    double x_, y_, theta_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BrackShipDriver>());
    rclcpp::shutdown();
    return 0;
}
