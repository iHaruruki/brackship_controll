#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include "../include/brackship_controll/serial.hpp"
#include <vector>

class BlackShipController : public rclcpp::Node
{
public:
    BlackShipController() : Node("blackship_controller"), x_(0.0), y_(0.0), theta_(0.0)
    {
        // シリアルポートの設定
        if (!serial_.InitSerial((char*)"/dev/ttyUSB0", B19200))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize serial port");
            rclcpp::shutdown();
        }

        // トピックの購読とパブリッシュ
        twist_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "joy_input", 10, std::bind(&BlackShipController::twistStampedCallback, this, std::placeholders::_1));

        encoder_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("encoder_publish", 10);
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

        // Transform Broadcasterの初期化
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // タイマーでエンコーダデータの読み取り
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&BlackShipController::readEncoderData, this));
    }

private:
    void twistStampedCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
    {
        double input_vel = limitVel(msg->twist.linear.x, msg->twist.angular.z);
        setSpeed(input_vel, msg->twist.angular.z);
    }

    double limitVel(double vel, double avel)
    {
        double max_v = 0.3;
        double max_w = 0.5;
        double limited_vel = std::max(std::min(vel, max_v), -max_v);
        double limited_avel = std::max(std::min(avel, max_w), -max_w);
        return limited_vel;
    }

    void setSpeed(double vel, double avel)
    {
        int left_speed = static_cast<int>((vel - avel) * 100);
        int right_speed = static_cast<int>((vel + avel) * 100);

        unsigned char motor_send_R[7] = {0x02, 0x01, 0x07, 0x00, static_cast<unsigned char>(right_speed), 0x00, 0x03};
        unsigned char motor_send_L[7] = {0x02, 0x02, 0x07, 0x00, static_cast<unsigned char>(left_speed), 0x00, 0x03};

        serial_.Write2(motor_send_R, 7);
        serial_.Write2(motor_send_L, 7);
    }

    void readEncoderData()
    {
        unsigned char encoder_send[5] = {0x02, 0x02, 0x05, 0x09, 0x03};
        unsigned char encoder_recv[10] = {0};

        serial_.Write2(encoder_send, 5);
        serial_.Read2(encoder_recv, 10);

        if (encoder_recv[0] == 0x02 && encoder_recv[9] == 0x03)
        {
            int encoder_right = (encoder_recv[5] << 8) | encoder_recv[6];
            int encoder_left = (encoder_recv[7] << 8) | encoder_recv[8];

            double wheel_separation = 0.5;  // 車輪間の距離[m]
            double wheel_radius = 0.15;     // 車輪の半径[m]
            double ticks_per_revolution = 500.0;
            double distance_per_tick = (2 * M_PI * wheel_radius) / ticks_per_revolution;

            double right_distance = encoder_right * distance_per_tick;
            double left_distance = encoder_left * distance_per_tick;
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

            // エンコーダデータのパブリッシュ
            sensor_msgs::msg::JointState encoder_msg;
            encoder_msg.header.stamp = this->now();
            encoder_msg.name = {"left_wheel", "right_wheel"};
            encoder_msg.position = {static_cast<double>(encoder_left), static_cast<double>(encoder_right)};
            encoder_pub_->publish(encoder_msg);
        }
    }

    CSerial serial_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr encoder_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;

    double x_, y_, theta_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BlackShipController>());
    rclcpp::shutdown();
    return 0;
}
