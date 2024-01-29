#include <math.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

#include "zlac8015d.h"

#define WHEEL_BASE 0.525
#define WHEEL_RAD 0.105 //meter
#define ONE_REV_TRAVEL 0.6597 //one_rev = 0.105m*2PI = 0.6597 
#define PULSE_PER_ROT 16385 //pulse per one rot = 16385 (dec)

class zlac_run : public rclcpp::Node{
public:
    zlac_run() : Node("odometry_and_twist_node"){   
        if (!initialized) {
            mot.init("/dev/ttyUSB0", 115200, 0x01, false);
            motorstat_init = mot.get_rpm();
            motorstat_init = mot.get_position();
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
    ~zlac_run(){
        mot.terminate();
    }

private:
    bool initialized = false;

    struct MOT_DATA motorstat;
    struct MOT_DATA motorstat_init; //motor status data when init

    ZLAC mot;

    double cd = 0.1;                //odometry Covariance main Diagonal value
    double linx;                    //odometry twist.linear.x
    double angz;                    //odometry twist.angular.z
    int32_t ENCODER_DIFF_L;         //endocer diff counter
    int32_t ENCODER_DIFF_R;         //endocer diff counter
    double rot_L_dst;               //wheel rotation distance (meter) for calc odometry 
    double rot_R_dst;               //wheel rotation distance (meter) for calc odometry 
    double mean_rot_dist = 0.0;     //LR wheel mean rot distance (meter) for calc odometry
    double rot_theta_diff = 0.0;    //robot angular state for calc odometry 
    double rot_theta = 0.0;         //robot angular state for calc odometry 
    double pos_X = 0.0;             //robot 2D pos X
    double pos_Y = 0.0;             //robot 2D pos Y

    geometry_msgs::msg::Twist rcv_twist;

    void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg){
        rcv_twist.linear.x = msg->linear.x;
        rcv_twist.angular.z = msg->angular.z;
        // RCLCPP_INFO(this->get_logger(), "Received Twist: Linear X: '%.2f', Angular Z: '%.2f'",
        //             msg->linear.x, msg->angular.z);
        double L_rpm = 9.5493 * ( rcv_twist.linear.x - (rcv_twist.angular.z*WHEEL_BASE)/2 );
        double R_rpm = 9.5493 * ( rcv_twist.linear.x + (rcv_twist.angular.z*WHEEL_BASE)/2 );
        mot.set_double_rpm(L_rpm, -R_rpm);
    }

    void publish_odometry(){
        motorstat = mot.get_rpm();
        motorstat = mot.get_position();

        ENCODER_DIFF_L = motorstat.encoder_L - motorstat_init.encoder_L;
        ENCODER_DIFF_R = motorstat.encoder_R - motorstat_init.encoder_R;

        auto msg = nav_msgs::msg::Odometry();
        msg.header.stamp = this->get_clock()->now();
        msg.header.frame_id = "odom";
        msg.child_frame_id = "base_link";

        linx = 0.10472 * (motorstat.rpm_L - motorstat.rpm_R) / 2;
        angz = 0.10472 * (motorstat.rpm_R + motorstat.rpm_L) / WHEEL_BASE;

        msg.twist.twist.linear.x = linx;
        msg.twist.twist.angular.z = angz;

        // encoder / pulse per rot * 2pi * wheel rad = wheel rot dist (m)
        rot_L_dst = ENCODER_DIFF_L / PULSE_PER_ROT * 6.283185307 * WHEEL_RAD;
        rot_R_dst = ENCODER_DIFF_R / PULSE_PER_ROT * 6.283185307 * WHEEL_RAD;

        mean_rot_dist = (rot_L_dst + rot_R_dst) / 2.0;
        rot_theta_diff = (rot_R_dst - rot_L_dst) / WHEEL_BASE;
        rot_theta = rot_theta + rot_theta_diff;
        
        pos_X = pos_X + mean_rot_dist * cos(rot_theta / 2);
        pos_Y = pos_Y + mean_rot_dist * sin(rot_theta / 2);

        msg.pose.pose.position.x = pos_X;
        msg.pose.pose.position.y = pos_Y;
        
        tf2::Quaternion quat;
        quat.setRPY(0,0,rot_theta);
        msg.pose.pose.orientation = tf2::toMsg(quat);;

        msg.twist.covariance = { cd, 0, 0, 0, 0, 0,
                                0, cd, 0, 0, 0, 0,
                                0, 0, cd, 0, 0, 0,
                                0, 0, 0, cd, 0, 0,
                                0, 0, 0, 0, cd, 0,
                                0, 0, 0, 0, 0, cd};

        msg.pose.covariance = { cd, 0, 0, 0, 0, 0,
                                0, cd, 0, 0, 0, 0,
                                0, 0, cd, 0, 0, 0,
                                0, 0, 0, cd, 0, 0,
                                0, 0, 0, 0, cd, 0,
                                0, 0, 0, 0, 0, cd};
        
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
