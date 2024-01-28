#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "zlac8015d.h"

#define WHEEL_BASE 0.525
#define WHEEL_RAD 0.105; //meter
#define ONE_REV_TRAVEL 0.6597; //one_rev = 0.105m*2PI = 0.6597 
#define PULSE_PER_ROT 16385; //pulse per one rot = 16385 (dec)

class zlac_run : public rclcpp::Node{
public:
    zlac_run() : Node("odometry_and_twist_node"){   
        if (!initialized) {
            printf("===begin===\n");
            mot.begin("/dev/ttyUSB0", 115200, 0x01);
            printf("===set_vel_mode===\n");
            mot.set_vel_mode();
            printf("===enable===\n");
            mot.enable();
            mot.set_double_rpm(0, 0);
            initialized = true;
        }

        // make odometry publisher
        odometry_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

        // make twist subscriber
        twist_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&zlac_run::twist_callback, this, std::placeholders::_1));

        // make timer for 30hz odometry msg publish
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000 / 30),
            std::bind(&zlac_run::publish_odometry, this));
    }

private:
    ZLAC mot;
    struct MOT_DATA motorstat;    

    bool initialized = false;

    geometry_msgs::msg::Twist rcv_twist;

    void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg){
        rcv_twist.linear.x = msg->linear.x;
        rcv_twist.angular.z = msg->angular.z;
        // RCLCPP_INFO(this->get_logger(), "Received Twist: Linear X: '%.2f', Angular Z: '%.2f'",
        //             msg->linear.x, msg->angular.z);
        double L_rads = 9.5493 * ( rcv_twist.linear.x - (rcv_twist.angular.z*WHEEL_BASE)/2 );
        double R_rads = 9.5493 * ( rcv_twist.linear.x + (rcv_twist.angular.z*WHEEL_BASE)/2 );
        mot.set_double_rpm(L_rads, -R_rads);
    }

    void publish_odometry(){
        motorstat = mot.get_rpm();
        motorstat = mot.get_position();
        auto msg = nav_msgs::msg::Odometry();
        msg.header.stamp = this->get_clock()->now();
        msg.header.frame_id = "odom";
        msg.child_frame_id = "base_link";
        msg.twist.twist.linear.x = 0.10472 * (motorstat.rpm_L - motorstat.rpm_R) / 2 ;
        msg.twist.twist.angular.z = 0.10472 * (motorstat.rpm_R + motorstat.rpm_L) / WHEEL_BASE;
        odometry_publisher_->publish(msg);
    }

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<zlac_run>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
