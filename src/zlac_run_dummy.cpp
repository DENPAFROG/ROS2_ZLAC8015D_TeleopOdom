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
//setting robot joint names
#define JOINT_PUB_TOPIC_NAME "joint_states"
#define JOINT_NAME_WHEEL_L "wheel_joint_L"
#define JOINT_NAME_WHEEL_R "wheel_joint_R"
//odometry Covariance main Diagonal value
#define CD 0.1
//setting odometry constant here (based on "ZLLG80ASM250-L" model)
#define WHEEL_RAD 0.100         //unit: meter
#define ONE_REV_TRAVEL 0.6283   //one_rev travel = 0.100m * 2PI = 0.6283m 
#define PULSE_PER_ROT 16385     //encoder pulse per one rot
#define WHEEL_BASE 0.525        //your robot's wheel to wheel distance
#define RAD2RPM 9.5493
#define RPM2RAD 0.1047
#define CIRCUMFERENCE 6.2832*WHEEL_RAD

class zlac_run : public rclcpp::Node{
public:
    zlac_run() : Node("virtual_zlac8015d_node"){
        if (!initialized) {
            printf("DUMMY MODE INIT");
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
    }

private:
    bool initialized = false;

    struct MOT_DATA motorstat;

    double rot_L_dst;               //wheel rotation distance (meter) for calc odometry 
    double rot_R_dst;               //wheel rotation distance (meter) for calc odometry 
    double mean_rot_dist = 0.0;     //LR wheel mean rot distance (meter) for calc odometry
    double mean_rot_dist_old = 0.0;     //LR wheel mean rot distance (meter) for calc odometry
    double mean_rot_dist_diff = 0.0;     //LR wheel mean rot distance (meter) diff per 30hz for calc odometry
    double rot_theta = 0.0;         //robot angular state for calc odometry 
    double pos_X = 0.0;             //robot 2D pos X
    double pos_Y = 0.0;             //robot 2D pos Y
    double rot_angle_L = 0.0;
    double rot_angle_R = 0.0;

    geometry_msgs::msg::Twist rcv_twist;

    void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg){
        rcv_twist.linear.x = msg->linear.x;
        rcv_twist.angular.z = msg->angular.z;
        //RCLCPP_INFO(this->get_logger(), "Received Twist: Linear X: '%.2f', Angular Z: '%.2f'", msg->linear.x, msg->angular.z);
        motorstat.rpm_L = ( rcv_twist.linear.x - (rcv_twist.angular.z * WHEEL_BASE / 2) ) * (60/(3.1415*2));
        motorstat.rpm_R = ( rcv_twist.linear.x + (rcv_twist.angular.z * WHEEL_BASE / 2) ) * (60/(3.1415*2));
        //printf("\n\nRPML:%lf|RPMR:%lf|",L_rpm, R_rpm);
    }

    void publish_odometry(){
        // printf("\nEL:%d|ER:%d|", ENCODER_DIFF_L, ENCODER_DIFF_R);
        auto msg = nav_msgs::msg::Odometry();
        msg.header.stamp = this->get_clock()->now();
        msg.header.frame_id = ODOM_FRAME_ID;
        msg.child_frame_id = ODOM_CHILD_FRAME_ID;
        msg.twist.twist.linear.x = ((motorstat.rpm_L + motorstat.rpm_R) / 2) / 60 * CIRCUMFERENCE;
        msg.twist.twist.angular.z = ((motorstat.rpm_R - motorstat.rpm_L) / WHEEL_BASE) / 60 * CIRCUMFERENCE;
        // encoder / pulse per rot * 2pi * wheel rad = wheel rot dist (m)
        rot_L_dst += CIRCUMFERENCE * motorstat.rpm_L / 180;
        rot_R_dst += CIRCUMFERENCE * motorstat.rpm_R / 180;
        rot_angle_L = std::fmod(rot_L_dst / WHEEL_RAD, 6.283185307);
        rot_angle_R = std::fmod(rot_R_dst / WHEEL_RAD, 6.283185307);
        // printf("\nDSTL:%lf|DSTR:%lf|", rot_L_dst, rot_R_dst);
        mean_rot_dist = (rot_L_dst + rot_R_dst) / 2.0;
        rot_theta = (rot_R_dst - rot_L_dst) / WHEEL_BASE;
        mean_rot_dist_diff = mean_rot_dist - mean_rot_dist_old;
        //printf("\nMMD:%lf|RT:%lf|", mean_rot_dist, rot_theta);
        pos_X += mean_rot_dist_diff * cos(rot_theta);
        pos_Y += mean_rot_dist_diff * sin(rot_theta);
        //printf("\nPX:%lf|PY:%lf|\n", pos_X, pos_Y);
        msg.pose.pose.position.x = pos_X;
        msg.pose.pose.position.y = pos_Y;
        tf2::Quaternion quat;
        quat.setRPY(0,0,rot_theta);
        msg.pose.pose.orientation = tf2::toMsg(quat);;
        msg.twist.covariance = { CD, 0, 0, 0, 0, 0,
                                0, CD, 0, 0, 0, 0,
                                0, 0, CD, 0, 0, 0,
                                0, 0, 0, CD, 0, 0,
                                0, 0, 0, 0, CD, 0,
                                0, 0, 0, 0, 0, CD};
        msg.pose.covariance = { CD, 0, 0, 0, 0, 0,
                                0, CD, 0, 0, 0, 0,
                                0, 0, CD, 0, 0, 0,
                                0, 0, 0, CD, 0, 0,
                                0, 0, 0, 0, CD, 0,
                                0, 0, 0, 0, 0, CD};
        odometry_publisher_->publish(msg);
        mean_rot_dist_old = mean_rot_dist;
    }

    void publish_jointstate(){
        auto message = sensor_msgs::msg::JointState();
        message.header.stamp = this->get_clock()->now();
        //resize array for joint amount
        size_t num_joints = 2;
        message.name.resize(num_joints);
        message.position.resize(num_joints);
        message.velocity.resize(num_joints);
        message.effort.resize(num_joints);
        message.name[0] = JOINT_NAME_WHEEL_L;
        message.position[0] = rot_angle_L;
        message.velocity[0] = motorstat.rpm_L * RPM2RAD;
        message.effort[0] = 0.0;
        message.name[1] = JOINT_NAME_WHEEL_R;
        message.position[1] = rot_angle_R;
        message.velocity[1] = motorstat.rpm_R * RPM2RAD;
        message.effort[1] = 0.0;
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
