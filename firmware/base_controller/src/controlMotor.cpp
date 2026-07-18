#include "controlMotor.h"
#include <Wire.h>

// This project uses MCP4725 DAC 12bit to control speed of motor.
// The ESC (Electronic Speed Controller) is controlled by analog voltage(1.1V to 4.2V converted to DAC value is 900 to 3400)
// In this project, motor is limited speed, so the DAC value is limited 900 to 1900.
// However, the 5V power supply for MCP4725 DAC is not stable, at 900 the voltage may be lower or higher than 1.1V. Therefore, you need to calibrate to know when motor runs with DAC value.
// You also need to calculate the dead band of the control value. For example, when the motor is off, setting the value to 10 does not make it run, but setting it to 15 starts the motor. After that, if you lower the value back to 10, the motor keeps running, and you have to decrease it to 5 for the motor to stop.

void ControlMotor::Init(uint8_t dirPin, uint8_t addr, uint8_t stopPin) {
    this->dirPin = dirPin;
    this->addr = addr;
    this->stopPin = stopPin;
    pinMode(dirPin, OUTPUT);
    digitalWrite(dirPin, HIGH); // set LOW to reverse, HIGH to forward, depending on your motor wiring
    if(stopPin != 255){  // Assuming 255 is used to indicate no stop pin
        pinMode(stopPin, OUTPUT);
        digitalWrite(stopPin, HIGH); // Assuming HIGH is the default state for stop pin
        delay(200);
    }
}

void ControlMotor::setSpeed(int speed) {
    if (speed > 200) {
        speed = 200;
    } else if (speed < -200) {
        speed = -200;
    }

    unsigned long now = millis();

    if (changingDirection) {
        if (now - dirChangeStartTime < dirChangeDelay) {
            setDAC(addr, offset_dac - deadband_dac);
            return;
        }
        changingDirection = false; // delay elapsed, dir pin already flipped, continue below
    }

    bool wantCW = (speed > 0) ? true : (speed < 0 ? false : cw);
    if (speed != 0 && wantCW != cw) {
        if (lastSpeed == 0) {
            // Motor is at rest, safe to flip direction now.
            digitalWrite(dirPin, wantCW ? HIGH : LOW);
            cw = wantCW;
            changingDirection = true;
            dirChangeStartTime = now;
            isRunning = false;  // motor is at rest during the turnaround
            kickstarting = false;
            integral = 0;    // reset PID: motor was at 0 before reversing, no reason to carry old accumulation
            prevError = 0;
            setDAC(addr, offset_dac - deadband_dac);
            lastSpeed = 0;
            return; // wait for dirChangeDelay to elapse on subsequent calls
        } else {
            // Motor still spinning the other way: refuse the flip for safety, command stop instead.
            speed = 0;
        }
    }

    if (speed == 0) {
        setDAC(addr, offset_dac - deadband_dac);
        lastSpeed = 0;
        isRunning = false;
        kickstarting = false;
        integral = 0; // motor at rest: no reason to carry PID accumulation forward
        prevError = 0;
        return;
    }

    if (kickstarting) {
        if (now - kickstartStartTime < kickstartDuration) {
            int16_t kickSpeed = (speed > 0) ? startThreshold : -startThreshold;
            uint16_t dacValue = map(abs(kickSpeed), 1, 200, offset_dac, offset_dac + 1000);
            setDAC(addr, dacValue);
            lastSpeed = speed; // report the real target so callers see progress
            return;
        }
        kickstarting = false;
        isRunning = true;
    }

    if (!isRunning && abs(speed) < startThreshold) {
        kickstarting = true;
        kickstartStartTime = now;
        kickstartTargetSpeed = speed;
        int16_t kickSpeed = (speed > 0) ? startThreshold : -startThreshold;
        uint16_t dacValue = map(abs(kickSpeed), 1, 200, offset_dac, offset_dac + 1000);
        setDAC(addr, dacValue);
        lastSpeed = speed;
        return;
    }

    // --- 6) Normal operation ---
    uint16_t dacValue = map(abs(speed), 1, 200, offset_dac, offset_dac + 1000);
    setDAC(addr, dacValue);
    lastSpeed = speed;
    isRunning = true;
}

void ControlMotor::setVelocity(float velocity, float currentVelocity) {
    if(abs(velocity) < 20) {
        setSpeed(0);
        integral = 0;
        prevError = 0;
        prevMeasurement = 0;
        return;
    }
    computeDt(); // measure real elapsed time since the last call instead of assuming a fixed dt

    float error = velocity - currentVelocity;
    integral += error * dt;
    if (integral > integralLimit) integral = integralLimit;
    else if (integral < -integralLimit) integral = -integralLimit;

    float derivative = -(currentVelocity - prevMeasurement) / dt;
    prevMeasurement = currentVelocity;
    prevError = error;

    float base = getBaseSpeed(velocity); // feed-forward term per target speed

    float output = base + Kp * error + Ki * integral + Kd * derivative;
    setSpeed((int)output);
}

float ControlMotor::computeDt() {
    unsigned long now = micros();

    if (lastUpdateMicros == 0) {
        lastUpdateMicros = now;
        return dt;
    }

    unsigned long elapsed = now - lastUpdateMicros;
    lastUpdateMicros = now;

    float measuredDt = elapsed / 1000000.0f;

    if (measuredDt <= 0.0f || measuredDt > 0.5f) {
        measuredDt = dt;
    }

    dt = measuredDt;
    return dt;
}

float ControlMotor::getBaseSpeed(float targetVelocity) {
    const float inMin = 53.6f, inMax = 81.8f, outMin = 40.0f, outMax = 60.0f;
    float sign = (targetVelocity < 0) ? -1.0f : 1.0f;
    float v = fabsf(targetVelocity);
    if (v < inMin) v = inMin;
    if (v > inMax) v = inMax;
    float base = outMin + (v - inMin) * (outMax - outMin) / (inMax - inMin);
    return sign * base;
}

void ControlMotor::smoothVelocity(float targetVelocity, float currentVelocity, float acceleration) {
    if(abs(targetVelocity) < 20) {
        setSpeed(0);
        smoothedSpeed = 0;
        integral = 0;
        prevError = 0;
        prevMeasurement = 0;
        return;
    }

    float step = acceleration * dt;


    if (smoothedSpeed < targetVelocity) {
        smoothedSpeed += step;
        if (smoothedSpeed > targetVelocity) smoothedSpeed = targetVelocity;
    } else if (smoothedSpeed > targetVelocity) {
        smoothedSpeed -= step;
        if (smoothedSpeed < targetVelocity) smoothedSpeed = targetVelocity;
    }

    setVelocity(smoothedSpeed, currentVelocity);
}

void ControlMotor::start() {
    digitalWrite(stopPin, HIGH); // Assuming HIGH allows the motor to run
}

void ControlMotor::stop() {
    setSpeed(0);
    digitalWrite(stopPin, LOW); // Assuming LOW stops the motor
    changingDirection = false;
    kickstarting = false;
    isRunning = false;
    smoothedSpeed = 0;     // otherwise the next smoothVelocity() call ramps from the old value
    prevMeasurement = 0;   // keep derivative term consistent with the reset integral/prevError
}

void setDAC(uint8_t addr, uint16_t value) {
    value &= 0x0FFF; 
    Wire.beginTransmission(addr);
    Wire.write(0x40);                 // Fast Mode Write
    Wire.write(value >> 4);           // 8 bit cao
    Wire.write((value & 0x0F) << 4);  // 4 bit thấp
    Wire.endTransmission();
}