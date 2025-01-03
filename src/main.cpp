# include <rclcpp/rclcpp.hpp>
# include <geometry_msgs/msg/twist.hpp>
# include <sensor_msgs/msg/joint_state.hpp>
# include "serial.cpp"

#include <string>
#include <iostream>
#include <stdio.h>
#include <errno.h>

class Blackship{
public:
    Blackship(rclcpp::Node::SharedPtr node)
    : node_(node), prevr(0), prevl(0), curl(0), cnt_setspeed(0), enc_l(0), enc_r(0) {
        mCountEncoder = node_->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    }

    ~Blackship() {
        blackship_close();
    }

    bool blackship_open(const char* serial_port) {
        char errmsg[25 + 1];
        RCLCPP_INFO(node_->get_logger(), "Open Blackship Serial Port:%s", serial_port);
        if(!serial.InitSerial((char*)serial_port, 9600)){
            strerror_r(errno, errmsg, sizeof(errmsg));
            RCLCPP_ERROR(node_->get_logger(), "Failed to open Blackship Serial Port:%s, %s", serial_port, errmsg);
            return false;
        }
        //if(blackship_get_status() == false) return false;
        RCLCPP_INFO(node_->get_logger(), "Blackship Serial Port:%s done", serial_port);
        return true;
    }

    bool blackship_close(){
        char errmsg[255 + 1];
        RCLCPP_INFO(node_->get_logger(), "#Closing connection");
        if(!serial.CloseSerial()){
            strerror_r(errno, errmsg, sizeof(errmsg));
            RCLCPP_ERROR(node_->get_logger(), "Failed to close Blackship Serial Port");
            return false;
        }
        RCLCPP_INFO(node_->get_logger(), " #Closing connection done.");
        return true;
    }

    bool blackship_get_status(){
        unsigned char smsg[5];
        unsigned char rmsg[9];

        smsg[0] = 0x02; //BLACKSHIP_STX
        smsg[1] = 0x70; //BLACKSHIP to_CPU_ID
        smsg[2] = 0x05; //Num
        smsg[3] = 0x09; //BLACKSHIP_MODE_STATUS_REQUEST;
        smsg[4] = 0x03; //BLACKSHIP_ETX;

        serial.Write2(smsg, 5);
        serial.Read2(rmsg, 9);

        if(rmsg[0] = 0x02 && rmsg[1] == 0x99 && rmsg[2] == 0x09 && rmsg[3] == 0x09 && rmsg[4] == 0x70 && rmsg[8] == 0x03){
            if(rmsg[5] == 0) status_MotorPW = 'Off';
            else status_MotorPW = 'On';

            RCLCPP_INFO(node_->get_logger(), "Motor Power:%d", rmsg[5]);
            RCLCPP_INFO(node_->get_logger(), "Battery Voltage: %f[V]", ((int)rmsg[6]*0.04)+20);
            if(rmsg[6] < 21){
                RCLCPP_INFO(node_->get_logger(), "Charge battery immediately");
                return false;
            }
            RCLCPP_INFO(node_->get_logger(), "Battery Current: %f[A]", (int)rmsg[7]*0.1);

            return true;
        }

        // 正しい値が取得できなかった場合
        RCLCPP_ERROR(node_->get_logger(), "Failed to get status");
        return false;
    }

    unsigned int blackship_get_encoder() {
        unsigned char smsg[5];
        unsigned char rmsg[10];

        smsg[0] = 0x02; //BLACKSHIP_STX
        smsg[1] = 0x02; //BLACKSHIP to_CPU_ID
        smsg[2] = 0x05; //Num
        smsg[3] = 0xF9; //BLACKSHIP MODE_ENCODER_REQUEST;
        smsg[4] = 0x03; //BLACKSHIP_ETX;

        serial.Write2(smsg, 5);
        serial.Read2(rmsg, 10);

        enc_r = ((unsigned int)rmsg[5] << 8) | (unsigned int)rmsg[6];
        enc_l = ((unsigned int)rmsg[7] << 8) | (unsigned int)rmsg[8];

        RCLCPP_INFO(node_->get_logger(), "Encoder_right : %u", enc_r);
        RCLCPP_INFO(node_->get_logger(), "Encoder_left  : %u", enc_l);
        return enc_l, enc_r;
    }

    bool blackship_set_speed(int cmd_l, int cmd_r) {
        unsigned char smsg[7];

        cnt_setspeed++;

        smsg[0] = 0x02; //BLACKSHIP_STX
        smsg[1] = 0x02; //BLACKSHIP to_CPU_ID
        smsg[2] = 0x07; //Num
        smsg[3] = 0xF0; //BLACKSHIP_MODE_SPEED;
        smsg[4] = cmd_r;
        smsg[5] = cmd_l;
        smsg[6] = 0x03; //BLACKSHIP_ETX;

        if(cnt_setspeed % 2){
            return serial.Write2(smsg, 7);
        }
    }

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr mCountEncoder;
    rclcpp::Node::SharedPtr node_;
    CSerial serial;
    unsigned short prevr, prevl;
    int curr, curl;
    int cnt_setspeed;
    char status_MotorPW;
    unsigned int enc_r, enc_l;

private:

};
