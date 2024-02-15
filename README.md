# ROS2_ZLAC8015D_serial


**Controls "ZLAC8015D" motor driver through USB-RS485 conversion module and converts its data into a ROS2 message.**


SUB: "/cmd_vel" (geometry_msgs/msg/Twist)

PUB: "/odom"  (nav_msgs/msg/Odometry)  &  "/joint_states"  (sensor_msgs::msg::JointState)


effort data (motor current data or torque data) read and publishing function is not completed yet



## required packages

- rclcpp
- [serial](https://github.com/wjwwood/serial)



## How to use

```bash
cd <your_workspace>
git clone https://github.com/oxcarxierra/ROS2_ZLAC8015D_serial.git
colcon build --packages-select zlac8015d_serial
ros2 run zlac8015d_serial zlac_run
```



## Tip

if your USB-RS485 converter uses ch340 it maybe conflict with “brltty” linux service

recommended to disable the "brltty" Linux service



## etc.

(ZLTECH Dual-Channel Servo Driver ZLAC8015D)

Driver Info Link: http://www.zlrobotmotor.com/info/401.html


**Forked From https://github.com/oxcarxierra/ROS2_ZLAC8015D_serial**
