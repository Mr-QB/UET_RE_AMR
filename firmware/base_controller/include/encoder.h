/**
 * @file encoder.h
 * @brief Quadrature encoder interface for wheel odometry
 */

#pragma once

#include <Arduino.h>

class Encoder
{
public:
  /**
   * @param pin_a         Encoder channel A pin (interrupt-capable)
   * @param pin_b         Encoder channel B pin
   * @param ticks_per_rev Encoder resolution (ticks per full revolution)
   * @param wheel_radius  Wheel radius [m]
   */
  Encoder(uint8_t pin_a, uint8_t pin_b,
          int ticks_per_rev = 1024,
          double wheel_radius = 0.065);

  void begin();

  /**
   * @brief Get cumulative position [rad]
   */
  double getPosition() const;

  /**
   * @brief Get current velocity [m/s]
   * @param dt Time delta since last call [s]
   */
  double getVelocity(double dt);

  /**
   * @brief Reset encoder count to zero
   */
  void reset();

  /**
   * @brief Get raw tick count
   */
  long getTicks() const;

private:
  uint8_t pin_a_;
  uint8_t pin_b_;
  int ticks_per_rev_;
  double wheel_radius_;

  volatile long tick_count_ = 0;
  long prev_tick_count_ = 0;

  // ISR helper
  void IRAM_ATTR onEncoderISR();
};
