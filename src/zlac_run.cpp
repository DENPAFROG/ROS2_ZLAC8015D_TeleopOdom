#include <math.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "zlac8015d.h"

//you can change subscribe topic name 
#define TWIST_SUB_TOPIC_NAME "cmd_vel"
//you can change odometry publish topic name and TF frame name
#define ODOM_PUB_TOPIC_NAME "odom"
#define ODOM_FRAME_ID "odom"
#define ODOM_CHILD_FRAME_ID "base_link"
//setting serial port, baudrate, MODBUS ID, debug_print_enable
#define SERIAL_PORT_NAME "/dev/ttyUSB0"
#define BAUDRATE 115200
#define MODBUS_ID 0x01
#define DEBUG_ENABLE false

#define JOINT_PUB_TOPIC_NAME "joint"

//setting odometry constant here (based on "ZLLG80ASM250-L" model)
#define WHEEL_RAD 0.105         //unit: meter
#define ONE_REV_TRAVEL 0.6597   //one_rev travel = 0.105m * 2PI = 0.6597m 
#define PULSE_PER_ROT 16385     //encoder pulse per one rot
#define WHEEL_BASE 0.525        //your robot's wheel to wheel distance

class zlac_run : public rclcpp::Node{
public:
    zlac_run() : Node("odometry_and_twist_node"){   
        if (!initialized) {
            mot.init(SERIAL_PORT_NAME, BAUDRATE, MODBUS_ID, DEBUG_ENABLE);
            motorstat_init = mot.get_rpm();
            motorstat_init = mot.get_position();
            initialized = true;
        }

        // make odometry publisher
        odometry_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>(ODOM_PUB_TOPIC_NAME, 10);

        // make jointstate publisher
        joint_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>(JOINT_PUB_TOPIC_NAME, 10);


        // make twist subscriber
        twist_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
            TWIST_SUB_TOPIC_NAME, 10, std::bind(&zlac_run::twist_callback, this, std::placeholders::_1));

        // make timer for 30hz odometry msg publish
        timer_odom = this->create_wall_timer(
            std::chrono::milliseconds(1000 / 30),
            std::bind(&zlac_run::publish_odometry, this));
        
        // make timer for 30hz jointstate msg publish
        timer_joint = this->create_wall_timer(
            std::chrono::milliseconds(1000 / 30),
            std::bind(&zlac_run::publish_jointstate, this));
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
    int32_t ENCODER_DIFF_L = 0;         //endocer diff counter
    int32_t ENCODER_DIFF_R = 0;         //endocer diff counter
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
        msg.header.frame_id = ODOM_FRAME_ID;
        msg.child_frame_id = ODOM_CHILD_FRAME_ID;

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
        
        pos_X = pos_X + mean_rot_dist * cos(rot_theta);
        pos_Y = pos_Y + mean_rot_dist * sin(rot_theta);

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

    void publish_jointstate(){
        auto message = sensor_msgs::msg::JointState();
        message.header.stamp = this->get_clock()->now();
        
        //resize array for joint amount
        size_t num_joints = 2;
        message.name.resize(num_joints);
        message.position.resize(num_joints);
        // message.velocity.resize(num_joints);
        // message.effort.resize(num_joints);

        message.name[0] = "wheel_L";
        message.position[0] = std::fmod(ENCODER_DIFF_L / PULSE_PER_ROT * 6.283185307, 6.283185307);
        // message.velocity[0] = velocity_for_joint1;
        // message.effort[0] = effort_for_joint1;

        message.name[1] = "wheel_R";
        message.position[1] = std::fmod(ENCODER_DIFF_R / PULSE_PER_ROT * 6.283185307, 6.283185307);
        // message.velocity[1] = velocity_for_joint2;
        // message.effort[1] = effort_for_joint2;

        joint_publisher_->publish(message);
    }

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_odom;
    rclcpp::TimerBase::SharedPtr timer_joint;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<zlac_run>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}