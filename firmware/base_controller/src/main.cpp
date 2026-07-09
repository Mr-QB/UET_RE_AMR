/**
 * @file main.cpp
 * @brief UET AMR Base Controller Firmware
 *
 * Main firmware for the differential drive base controller.
 * Communicates with ROS2 host via micro-ROS over UART.
 *
 * Architecture:
 *   ROS2 Host ←─UART/micro-ROS─→ This MCU ←─PWM/Encoder─→ Motor Driver
 *
 * Published Topics:
 *   /wheel_odom (uet_amr_msgs/WheelOdom)
 *   /battery/status (uet_amr_msgs/BatteryStatus)
 *
 * Subscribed Topics:
 *   /cmd_vel (geometry_msgs/Twist) — via micro-ROS
 */

#include <Arduino.h>
#include <micro_ros_arduino.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>

#include "motor_controller.h"
#include "encoder.h"
#include "pid_controller.h"

// ========== Configuration ==========
#define SERIAL_BAUD_RATE     115200
#define MICROROS_SERIAL_BAUD 115200
#define CONTROL_LOOP_HZ      100     // [Hz]
#define ODOM_PUBLISH_HZ      50      // [Hz]

// ========== micro-ROS handles ==========
rcl_node_t           node;
rclc_support_t       support;
rcl_allocator_t      allocator;
rclc_executor_t      executor;

rcl_subscription_t   cmd_vel_sub;
rcl_publisher_t      odom_pub;
rcl_timer_t          control_timer;

geometry_msgs__msg__Twist cmd_vel_msg;

// ========== Motor & Encoder ==========
MotorController left_motor(/* left motor pins */);
MotorController right_motor(/* right motor pins */);
Encoder left_encoder(/* encoder pins */);
Encoder right_encoder(/* encoder pins */);

// ========== Callbacks ==========
void cmd_vel_callback(const void * msg_in)
{
  const geometry_msgs__msg__Twist * twist =
    (const geometry_msgs__msg__Twist *)msg_in;

  // Convert Twist to wheel velocities (differential drive kinematics)
  double linear  = twist->linear.x;
  double angular = twist->angular.z;

  double left_vel  = (linear - angular * WHEEL_BASE_MM / 2000.0);  // [m/s]
  double right_vel = (linear + angular * WHEEL_BASE_MM / 2000.0);

  left_motor.setVelocity(left_vel);
  right_motor.setVelocity(right_vel);
}

void control_timer_callback(rcl_timer_t * /*timer*/, int64_t /*last_call_time*/)
{
  // Update PID and publish odometry
  left_motor.update();
  right_motor.update();
}

// ========== Setup ==========
void setup()
{
  Serial.begin(SERIAL_BAUD_RATE);
  set_microros_serial_transports(Serial);

  delay(2000);  // Wait for micro-ROS agent

  allocator = rcl_get_default_allocator();

  // Init support
  rclc_support_init(&support, 0, NULL, &allocator);

  // Init node
  rclc_node_init_default(&node, "amr_base_controller", "", &support);

  // Init subscriber: /cmd_vel
  rclc_subscription_init_default(
    &cmd_vel_sub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "/cmd_vel");

  // Init executor
  rclc_executor_init(&executor, &support.context, 2, &allocator);
  rclc_executor_add_subscription(
    &executor, &cmd_vel_sub, &cmd_vel_msg,
    &cmd_vel_callback, ON_NEW_DATA);
}

// ========== Loop ==========
void loop()
{
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
}
