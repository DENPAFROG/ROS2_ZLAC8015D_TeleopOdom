# ROS2_ZLAC8015D_serial


**This program controls "ZLAC8015D" motor driver through USB-RS485 conversion module and converts its data into a ROS2 message.**


subscribe "/cmd_vel" (geometry_msgs/msg/Twist)

publish "/odom"  (nav_msgs/msg/Odometry)  and  "/joint_states"  (sensor_msgs::msg::JointState)

effort data (motor current data or torque data) read and publishing function is not completed yet


**ROS2 package for "RS485-USB" with "ZLAC8015D" motor driver communication**\

(ZLTECH Dual-Channel Servo Driver ZLAC8015D)

Driver Info Link: http://www.zlrobotmotor.com/info/401.html


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



**Forked From https://github.com/oxcarxierra/ROS2_ZLAC8015D_serial**
