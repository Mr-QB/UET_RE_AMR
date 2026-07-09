/**
 * @file motor_controller.h
 * @brief Differential drive motor controller with PID velocity control
 */

#pragma once

#include <Arduino.h>

class MotorController
{
public:
  /**
   * @param pwm_pin   PWM output pin to motor driver
   * @param dir_pin1  Direction pin 1
   * @param dir_pin2  Direction pin 2
   */
  MotorController(uint8_t pwm_pin, uint8_t dir_pin1, uint8_t dir_pin2);

  void begin();

  /**
   * @brief Set target velocity
   * @param velocity_ms Target velocity in [m/s], positive = forward
   */
  void setVelocity(double velocity_ms);

  /**
   * @brief Update PID loop — call at CONTROL_LOOP_HZ
   */
  void update();

  /**
   * @brief Get current measured velocity [m/s]
   */
  double getVelocity() const;

  /**
   * @brief Emergency stop
   */
  void stop();

  // PID gains — tune these for your hardware
  double kp = 1.5;
  double ki = 0.8;
  double kd = 0.05;

private:
  uint8_t pwm_pin_;
  uint8_t dir_pin1_;
  uint8_t dir_pin2_;

  double target_velocity_ = 0.0;
  double current_velocity_ = 0.0;
  double integral_ = 0.0;
  double prev_error_ = 0.0;

  unsigned long last_update_ms_ = 0;

  void setPWM(int pwm);  // -255 to 255
};
