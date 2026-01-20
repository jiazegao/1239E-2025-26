#pragma once

#include "custom/configs.hpp"

class Lever_PID {
    public:
        Lever_PID(pros::Motor* motor, pros::Rotation* sensor, double kP, double kI, double kD, double errorRange, double errorTimeout, double minOutput, double maxOutput, bool autoReset = true, bool* ballTrackingActivated = nullptr)
            : motor(motor), sensor(sensor), kP(kP), kI(kI), kD(kD), errorRange(errorRange), errorTimeout(errorTimeout), minOutput(minOutput), maxOutput(maxOutput), prevError(0), integral(0), target(0), autoReset(autoReset), ballTrackingActivated(ballTrackingActivated) { timer.hardReset(errorTimeout); motor->set_brake_mode(pros::E_MOTOR_BRAKE_HOLD); } // Constructor
    
        double calculate(double target, double measuredValue) {
            double error = target - measuredValue;
            integral += error;
            double derivative = error - prevError;
            prevError = error;
    
            // Calculate PID output
            double output = (kP * error) + (kI * integral) + (kD * derivative);
    
            // Clamp output to min/max range
            if (output > maxOutput) output = maxOutput;     
            else if (output < minOutput) output = minOutput;
    
            return output;
        }
        
        void mainloop() {
            // Main loop for PID control
            while (true) {

                if (!mainLoopLocked) {

                    // Get current measured value (e.g., from a sensor)
                    double measuredValue = sensor->get_position()/100.0;
                    double output = 0;

                    // If not within error range, reset timer
                    if (std::abs(target - measuredValue) > errorRange){
                        timer.reset();
                    }
                    else {
                        trapDoor.extend();
                    }

                    // If timer is up, stop motor
                    if (timer.timeIsUp()) {
                        output = 0;
                        if (autoReset) {
                            hardReset();
                            *ballTrackingActivated = true;
                        }
                    }
                    // Otherwise, calculate output
                    else {
                        // Calculate PID output
                        output = calculate(target, measuredValue);
                    }

                    // Apply output to the motor
                    motor->move(output);
                }
                
                pros::delay(20);
            }
        }

        void setTarget(double newTarget, double minOutput_ = -127, double maxOutput_ = 127) {
            target = newTarget;
            prevError = 0; // Reset previous error term when setpoint changes
            integral = 0; // Reset integral term when setpoint changes
            this->minOutput = minOutput_;
            this->maxOutput = maxOutput_;
        }

        void hardReset(double timeout = 1000) {

            // Lock mainloop
            mainLoopLocked = true;

            // Reset duration timer
            Timer t(timeout);

            // Reset lever
            motor->move(-127);
            pros::delay(150); // Allow time for motors to spin
            while ( !t.timeIsUp() && motor->get_actual_velocity() < -20 ) { pros::delay(20); }
            motor->move(0);
            pros::delay(200); // Allow time for motors to stop
            
            // Reset
            target = 0;
            prevError = 0;
            integral = 0;
            sensor->set_position(0);
            timer.reset();

            // Unlock mainloop
            mainLoopLocked = false;
        }

        void setCurrentPosition(double currentPosition) {
            target = currentPosition;
            sensor->set_position(currentPosition*100);
            prevError = 0;
            integral = 0;
        }

    private:
        double kP, kI, kD;
        double errorRange, errorTimeout;
        double minOutput, maxOutput;
        double prevError;
        double integral;
        pros::Motor* motor = nullptr;
        pros::Rotation* sensor = nullptr;
        double target;
        Timer timer;
        bool autoReset;
        bool* ballTrackingActivated;

        bool mainLoopLocked = false; // Flag to control main loop execution
};