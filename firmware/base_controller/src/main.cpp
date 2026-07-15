/**
 * @file main.cpp
 * @brief UET AGV Base Controller Firmware
 *
 * Main firmware for the differential drive base controller.
 */

#include <Arduino.h>
#include <Wire.h>

#include "encoder.h"
#include "controlMotor.h"


//declare encoder pin with encoder in brushless motor
//ENCODER A ==> YELLOW WIRE
//ENCODER B ==> GREEN WIRE
//ENCODER C ==> BLUE WIRE

#define EN_L_A 2 // encoder motor left
#define EN_L_B 3
#define EN_L_C 4

#define EN_R_A 5 // encoder motor right
#define EN_R_B 6
#define EN_R_C 7

#define STEPS_PER_ROTATION 90 // encoder steps per rotation
#define WHEEL_RADIUS 35.0 // wheel radius in meters

#define LEFT_MOTOR_CW true // left motor clockwise
#define RIGHT_MOTOR_CW false // right motor clockwise

#define LEFT_MOTOR_ADDR 0x60 // I2C address for left motor controller
#define RIGHT_MOTOR_ADDR 0x61 // I2C address for right motor controller

#define LEFT_MOTOR_DIR_PIN 9 // direction pin for left motor
#define RIGHT_MOTOR_DIR_PIN 10 // direction pin for right motor

#define LEFT_MOTOR_STOP_PIN 12 // stop pin for left motor
#define RIGHT_MOTOR_STOP_PIN 13 // stop pin for right motor

#define LOOP_FREQUENCY 800 // loop frequency in Hz

Encoder left;
Encoder right;

ControlMotor leftMotor;
ControlMotor rightMotor;

void setup() {
    
    delay(1000);
    Serial.begin(115200);
    delay(100);
    Wire.begin(); // initialize I2C communication

    left.Init(EN_L_A, EN_L_B, EN_L_C, STEPS_PER_ROTATION, WHEEL_RADIUS, LEFT_MOTOR_CW);
    right.Init(EN_R_A, EN_R_B, EN_R_C, STEPS_PER_ROTATION, WHEEL_RADIUS, RIGHT_MOTOR_CW);

    leftMotor.Init(LEFT_MOTOR_DIR_PIN, LEFT_MOTOR_ADDR, LEFT_MOTOR_STOP_PIN);
    rightMotor.Init(RIGHT_MOTOR_DIR_PIN, RIGHT_MOTOR_ADDR, RIGHT_MOTOR_STOP_PIN);

    leftMotor.start();
    rightMotor.start();
}



float leftSpeed = 0.0;
float rightSpeed = 0.0;

float leftMotorSpeed = 0.0;
float rightMotorSpeed = 0.0;

double leftVelocity = 0.0;
double rightVelocity = 0.0;
int32_t leftPos = 0;
int32_t rightPos = 0;

long long lastLoopTime = 0;

int32_t left_prev_pos = 0;
int32_t right_prev_pos = 0;

void loop() {
    long long currentTime = micros();
    if (currentTime - lastLoopTime >= 1000000 / LOOP_FREQUENCY) {
        lastLoopTime = currentTime;


        if (Serial.available() > 0) { // Check if data is available to read from the serial port
            String str = Serial.readStringUntil('\n');
            str.trim();
            if (str.length() > 0) {
                int control = str.toInt();

                if (control >= 0 && control <= 40) {
                    leftMotorSpeed = (control - 20) * 10;
                    rightMotorSpeed = (control - 20) * 10;
                    leftMotor.setSpeed(leftMotorSpeed);
                    rightMotor.setSpeed(rightMotorSpeed);
                } else {
                    Serial.println("Invalid input");
                }

                // leftSpeed = control;
                // rightSpeed = control;

            }
        }

        left.readEncoder(&leftPos, &leftVelocity);
        right.readEncoder(&rightPos, &rightVelocity);

        leftMotor.setSpeed(leftMotorSpeed);
        rightMotor.setSpeed(rightMotorSpeed);

        // leftMotor.setVelocity(leftSpeed, leftVelocity);
        // rightMotor.setVelocity(rightSpeed, rightVelocity);

        // leftMotor.smoothVelocity(leftSpeed, leftVelocity, 0.1f);
        // rightMotor.smoothVelocity(rightSpeed, rightVelocity, 0.1f);
        if(left_prev_pos != leftPos || right_prev_pos != rightPos){
            // Serial.print("Left Position: ");
            // Serial.print(leftPos);
            // Serial.print("|| Right Position: ");
            // Serial.print(rightPos);
            // Serial.print("|| Left Velocity: ");
            Serial.print(leftVelocity);
            Serial.print(" ");
            // Serial.print("|| Right Velocity: ");
            Serial.println(rightVelocity);
        }
        left_prev_pos = leftPos;
        right_prev_pos = rightPos;

    }
    
}
