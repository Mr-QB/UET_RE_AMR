/**
 * @file main.cpp
 * @brief UET AGV Base Controller Firmware
 *
 * Main firmware for the differential drive base controller.
 */

#include <Arduino.h>
#include <Wire.h>
#include <string.h>
 
#include "encoder.h"
#include "controlMotor.h"
#include "odometry.h"
#include "protocol.h"
 
 
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
#define WHEEL_RADIUS 8.75 // wheel radius in centimeters
#define WHEEL_BASE 45.0 // TODO: measure the actual distance between the 2 wheels (same unit as WHEEL_RADIUS) and update this
 
#define LEFT_MOTOR_CW true // left motor clockwise
#define RIGHT_MOTOR_CW false // right motor clockwise
 
#define LEFT_MOTOR_ADDR 0x60 // I2C address for left motor controller
#define RIGHT_MOTOR_ADDR 0x61 // I2C address for right motor controller
 
#define LEFT_MOTOR_DIR_PIN 9 // direction pin for left motor
#define RIGHT_MOTOR_DIR_PIN 10 // direction pin for right motor
 
#define LEFT_MOTOR_STOP_PIN 12 // stop pin for left motor
#define RIGHT_MOTOR_STOP_PIN 13 // stop pin for right motor
 
#define LOOP_FREQUENCY 800 // loop frequency in Hz

#define SMOOTH_ACCEL 200.0f

Encoder left;
Encoder right;
 
ControlMotor leftMotor;
ControlMotor rightMotor;
 
Odometry odom;

Protocol proto; 
bool running = false;
bool useSmoothVelocity = false;
float batteryVoltage = 0.0f;
float currentDraw = 0.0f;
void onProtocolCommand(uint8_t cmd, const uint8_t* data, uint8_t len);

void setup() {
    delay(1000); // wait for 1 second to allow the system to stabilize
    Serial.begin(115200);
    delay(100); // wait for 100 milliseconds to allow the serial port to initialize
    Wire.begin(); // initialize I2C communication
 
    left.Init(EN_L_A, EN_L_B, EN_L_C, STEPS_PER_ROTATION, WHEEL_RADIUS, LEFT_MOTOR_CW);
    right.Init(EN_R_A, EN_R_B, EN_R_C, STEPS_PER_ROTATION, WHEEL_RADIUS, RIGHT_MOTOR_CW);
 
    leftMotor.Init(LEFT_MOTOR_DIR_PIN, LEFT_MOTOR_ADDR, LEFT_MOTOR_STOP_PIN);
    rightMotor.Init(RIGHT_MOTOR_DIR_PIN, RIGHT_MOTOR_ADDR, RIGHT_MOTOR_STOP_PIN);
 
    leftMotor.stop();
    rightMotor.stop();
 
    odom.Init(WHEEL_RADIUS, WHEEL_BASE, STEPS_PER_ROTATION);
    proto.begin(Serial);
    proto.onCommand(onProtocolCommand);
    proto.resetWatchdog(); // don't let the watchdog fire before the first packet arrives
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
 
double poseX = 0.0;
double poseY = 0.0;
double poseTheta = 0.0;


void onProtocolCommand(uint8_t cmd, const uint8_t* data, uint8_t len) {
    switch (cmd) {
        case CMD_PING:
            proto.sendAckPing();
            break;

        case CMD_START:
            running = true;
            leftMotor.start();
            rightMotor.start();
            proto.sendAckControl();
            break;

        case CMD_STOP:
            running = false;
            leftSpeed = 0;
            rightSpeed = 0;
            leftMotor.stop();
            rightMotor.stop();
            proto.sendAckControl();
            break;

        case CMD_RESET_ENCODER:
            left.reset();
            right.reset();
            proto.sendAckControl();
            break;

        case CMD_RESET_ODOMETRY:
            odom.reset();
            proto.sendAckControl();
            break;

        case CMD_SET_VELOCITY: {
            if (len < 8) break; // malformed payload, ignore
            float l, r;
            memcpy(&l, data + 0, 4);
            memcpy(&r, data + 4, 4);
            useSmoothVelocity = false;
            if (running) { leftSpeed = l; rightSpeed = r; }
            proto.sendAckControl();
            break;
        }

        case CMD_SET_SMOOTH_VELOCITY: {
            if (len < 8) break;
            float l, r;
            memcpy(&l, data + 0, 4);
            memcpy(&r, data + 4, 4);
            useSmoothVelocity = true;
            if (running) { leftSpeed = l; rightSpeed = r; }
            proto.sendAckControl();
            break;
        }

        // These arrive with len==0: master is asking for the current value.
        case CMD_ODOMETRY:
            proto.sendOdometry((float)poseX, (float)poseY, (float)poseTheta);
            break;

        case CMD_VELOCITY:
            proto.sendVelocity((float)leftVelocity, (float)rightVelocity);
            break;

        case CMD_ENCODER_POS:
            proto.sendEncoderPos(leftPos, rightPos);
            break;

        case CMD_STATUS:
            // TODO: replace with real battery/current sensing (e.g. analogRead + your divider/shunt math).
            proto.sendStatus(batteryVoltage, currentDraw);
            break;
    }
}
 

void loop() {
    // Parse any bytes waiting from master. Without this call, incoming packets
    // are never read off Serial and onProtocolCommand() never fires -- this was
    // the missing piece causing "connected but nothing responds to commands".
    proto.update();

    if (proto.checkWatchdog(PROTOCOL_DEFAULT_WATCHDOG_MS)) {
        running = false;
        leftSpeed = 0;
        rightSpeed = 0;
        leftMotor.stop();
        rightMotor.stop();
        proto.sendWatchdogTriggered();
    }

    long long currentTime = micros();
    if (currentTime - lastLoopTime >= 1000000 / LOOP_FREQUENCY) {
        lastLoopTime = currentTime;
 
        left.readEncoder(&leftPos, &leftVelocity);
        right.readEncoder(&rightPos, &rightVelocity);
 
        odom.update(leftPos, rightPos);
        odom.getPose(&poseX, &poseY, &poseTheta);
 
 
        if (!running) {
            leftSpeed = 0;
            rightSpeed = 0;
        }

        if (useSmoothVelocity) {
            leftMotor.smoothVelocity(leftSpeed, leftVelocity, SMOOTH_ACCEL);
            rightMotor.smoothVelocity(rightSpeed, rightVelocity, SMOOTH_ACCEL);
        } else {
            leftMotor.setVelocity(leftSpeed, leftVelocity);
            rightMotor.setVelocity(rightSpeed, rightVelocity);
        }
    }
    
}