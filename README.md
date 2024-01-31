# ROS2_ZLAC8015D_serial

**Forked From https://github.com/oxcarxierra/ROS2_ZLAC8015D_serial**


**integrate Teleop subscription and odometry publish**\

subscribe "/cmd_vel" (geometry_msgs/msg/Twist)

publish "/odom"  (nav_msgs/msg/Odometry)


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
