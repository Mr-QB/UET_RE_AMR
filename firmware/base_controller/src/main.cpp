/**
 * @file main.cpp
 * @brief UET AMR Base Controller Firmware
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
#define WHEEL_RADIUS 0.08 // wheel radius in meters

#define LEFT_MOTOR_CW true // left motor clockwise
#define RIGHT_MOTOR_CW true // right motor clockwise

#define LEFT_MOTOR_ADDR 0x60 // I2C address for left motor controller
#define RIGHT_MOTOR_ADDR 0x61 // I2C address for right motor controller

#define LEFT_MOTOR_DIR_PIN 8 // direction pin for left motor
#define RIGHT_MOTOR_DIR_PIN 9 // direction pin for right motor

#define LEFT_MOTOR_STOP_PIN 10 // stop pin for left motor
#define RIGHT_MOTOR_STOP_PIN 11 // stop pin for right motor

#define LOOP_FREQUENCY 500 // loop frequency in Hz

Encoder left;
Encoder right;

ControlMotor leftMotor;
ControlMotor rightMotor;

void setup() {
    delay(1000); // wait for 1 second to allow the system to stabilize
    Serial.begin(115200);
    delay(100); // wait for 100 milliseconds to allow the serial port to initialize
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

                if (control >= -200 && control <= 200) {
                    leftMotorSpeed = control;
                    rightMotorSpeed = control;
                    leftMotor.setSpeed(leftMotorSpeed);
                    rightMotor.setSpeed(rightMotorSpeed);
                } else {
                    Serial.println("Invalid input. Please enter a value between -200 and 200.");
                }

                // leftSpeed = control;
                // rightSpeed = control;

            }
        }

        left.readEncoder(&leftPos, &leftVelocity);
        right.readEncoder(&rightPos, &rightVelocity);

        leftMotor.setSpeed(leftMotorSpeed);
        rightMotor.setSpeed(rightMotorSpeed);

        // leftMotor.setVelocity(leftSpeed);
        // rightMotor.setVelocity(rightSpeed);

        // leftMotor.smoothVelocity(leftSpeed, leftVelocity, 1.0f);
        // rightMotor.smoothVelocity(rightSpeed, rightVelocity, 1.0f);
        
        if(left_prev_pos != leftPos || right_prev_pos != rightPos){
            Serial.print("Left Position: ");
            Serial.print(leftPos);
            Serial.print("|| Right Position: ");
            Serial.print(rightPos);
            Serial.print("|| Left Velocity: ");
            Serial.print(leftVelocity);
            Serial.print("|| Right Velocity: ");
            Serial.println(rightVelocity);
        }
        left_prev_pos = leftPos;
        right_prev_pos = rightPos;
    }
}
