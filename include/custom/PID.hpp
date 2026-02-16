#pragma once

#include "Tracking_Util.hpp"
#include "custom/configs.hpp"

enum LEVER_STAGE {INACTIVE, INTAKING, OUTTAKING, SCORING};
inline const int RESTING_POS = 910;
inline const int FORCE_TERMINATE_TIMEOUT = 5000;

class Lever_PID {
    public:
        Lever_PID(pros::Motor* motor, pros::adi::Potentiometer* sensor, double kP, double kI, double kD, double errorRange, double errorTimeout, double maxReverseSpeed, double maxScoreSpeed, bool autoReset = true, LEVER_STAGE* leverStage = nullptr)
            : motor(motor), sensor(sensor), kP(kP), kI(kI), kD(kD), errorRange(errorRange), errorTimeout(errorTimeout), maxReverseSpeed(maxReverseSpeed), maxScoreSpeed(maxScoreSpeed), prevError(0), integral(0), target(0), autoReset(autoReset), leverStage(leverStage) { forceStopTimer.hardReset(FORCE_TERMINATE_TIMEOUT), errorRangeTimer.hardReset(errorTimeout); motor->set_brake_mode(pros::E_MOTOR_BRAKE_HOLD); tickTimer.hardReset(1000); } // Constructor
    
        double calculate(double target, double measuredValue) {
            double error = target - measuredValue;
            integral += error;
            double derivative = error - prevError;
            prevError = error;
    
            // Calculate PID output
            double output = (kP * error) + (kI * integral) + (kD * derivative);
    
            return output;
        }
        
        void mainloop() {
            // Main loop for PID control
            while (true) {

                if (!mainLoopLocked && *leverStage == SCORING) {

                    // Get current measured value (e.g., from a sensor)
                    double measuredValue = getLeverPotentReading();
                    double output = 0.0;

                    // If not within error range, reset timer
                    if (std::abs(target - measuredValue) > errorRange){
                        errorRangeTimer.reset();
                    }
                    else {
                        trapDoor.retract(); // close trapdoor immediately if within errorRange
                    }

                    // If timer is up, stop motor
                    if (forceStopTimer.timeIsUp() || errorRangeTimer.timeIsUp()) {
                        output = 0.0;
                        motor->move(0);
                        if (autoReset) {
                            hardReset();
                            *leverStage = INACTIVE;
                        }
                    }
                    // Otherwise, move motor
                    else {
                        // Clamp velocity
                        double currSpeed = (measuredValue-previousValue) / tickTimer.elapsed(TimeUnit::SECOND);
                        tickTimer.reset();

                        if (currSpeed > maxScoreSpeed) motor->move_velocity(maxScoreSpeed/6);
                        else if (currSpeed < maxReverseSpeed) motor->move_velocity(maxReverseSpeed/6);
                        else {
                            // Calculate PID output
                            output = calculate(target, measuredValue);
                            motor->move(output);
                        }
                    }

                    // Apply output to the motor
                    previousValue = measuredValue;
                }
                
                pros::delay(20);
            }
        }

        void setTarget(double newTarget, double maxReverseSpeed_ = -300000, double maxScoreSpeed_ = 300000) {
            this->target = newTarget;
            this->prevError = 0; // Reset previous error term when setpoint changes
            this->integral = 0; // Reset integral term when setpoint changes
            this->maxReverseSpeed = maxReverseSpeed_;
            this->maxScoreSpeed = maxScoreSpeed_;
            this->previousValue = getLeverPotentReading();
            this->forceStopTimer.reset();
            this->errorRangeTimer.reset();
        }

        void hardReset(double timeout = 2000) {
            // Lock mainloop
            mainLoopLocked = true;

            // Reset duration timer
            Timer t(timeout);

            // Reset lever
            motor->move(-127);
            pros::delay(150); // Allow time for motors to spin
            while ( !t.timeIsUp() && getLeverPotentReading() >= RESTING_POS ) { pros::delay(20); }
            motor->move(0);
            
            // Reset
            target = 0;
            prevError = 0;
            integral = 0;
            
            // Unlock mainloop
            mainLoopLocked = false;
        }

        void setCurrentPosition(double currentPosition) {
            target = currentPosition;
            prevError = 0;
            integral = 0;
        }

        double kP, kI, kD;
        double errorRange, errorTimeout;
        double maxReverseSpeed, maxScoreSpeed;
        double prevError;
        double integral;
        pros::Motor* motor = nullptr;
        pros::adi::Potentiometer* sensor = nullptr;
        double target;
        Timer forceStopTimer, errorRangeTimer, tickTimer;
        bool autoReset;
        LEVER_STAGE* leverStage;
        double previousValue;

        bool mainLoopLocked = false; // Flag to control main loop execution
};