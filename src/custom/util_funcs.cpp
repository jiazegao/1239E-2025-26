
#include "custom/util_funcs.h"
#include "custom/configs.h"
#include "custom/auton.h"
#include "pros/misc.h"
#include "pros/motors.h"

// Indexer control
void frontIn() {
    frontMotor.move(127);
}
void frontOut() {
    frontMotor.move(-127);
}
void stopFront() {
    frontMotor.move(0);
}
void topIn() {
    topMotor.move(127);
}
void topOut() {
    topMotor.move(-127);
}
void stopTop() {
    topMotor.move(0);
}
void lockTop() {
    topMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
}
void unlockTop() {
    topMotor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
}
void lockFront() {
    frontMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
}
void unlockFront() {
    frontMotor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
}
bool topOpticIsRed() {
    return (345 < topOptic.get_hue() || topOptic.get_hue() < 20);
}
bool topOpticIsBlue() {
    return (200 < topOptic.get_hue() && topOptic.get_hue() < 240);
}

// Function for managing intake controls
void updateIntake() {

    // Motor Controls ----------------------------------------------------

    // Button B - Outtake
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
        frontOut();
        topIn();
    }
    // Button R2 - Score top
    else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
        frontIn();
        topOut();
    }
    // Button R1 - Score middle
    else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
        frontIn();
        topIn();
    }
    // Button L2 - Normal intake
    else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
        lockTop();
        stopTop();
        frontIn();
    }
    // Button L1 - Macro intake
    else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
        // If matches alliance color, lock top motor
        if ( (topOpticIsRed() && allianceColor == alliance_color::RED) || (topOpticIsBlue() && allianceColor == alliance_color::BLUE) ) {
            lockTop();
            stopTop();
            frontIn();
        }
        // Otherwise, keep intaking
        else {
            topOut();
            frontIn();
        }
    }
    // If no button is pressed, stop everything
    else {
        lockTop();
        lockFront();
        stopTop();
        stopFront();
    }

    // Miscellaneous Controls ---------------------------------------------

    // Button R1 - Pneumatics control
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
        middleMech.retract();
    }
    else {
        middleMech.extend();
    }
}

// Fucntion for managing pneumatics controls
void updatePneumatics() {
    // Button Y - Middle goal descore mech (Toggle)
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
        middleDescore.toggle();
    }
    // Button Left - Left descore arm (Toggle)
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
        leftDescoreArm.toggle();
    }
    // Button Down - Right descore arm (Toggle)
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
        rightDescoreArm.toggle();
    }
    // Button X - Match load mech (Toggle)
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
        matchLoadGate.toggle();
    }
}

// Tank drive
void updateTankDrive() { chassis.tank(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y), controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y)); }

// Display
void startControllerDisplay() {
    if (!controllerDiplsayStarted) {
        controllerDiplsayStarted = true;
        pros::Task  controllerScreenTask([&](){
            while (true) {
                controller.clear();
                pros::delay(50);
                controller.print(0, 0, "X: %f", chassis.getPose().x);
                pros::delay(50);
                controller.print(1, 0, "Y: %f", chassis.getPose().y);
                pros::delay(50);
                controller.print(2, 0, "Heading: %f", chassis.getPose().theta);
                pros::delay(100);
            }
        });
    }
}
void startBrainDisplay() {
    if (!brainDisplayStarted) {
        brainDisplayStarted = true;
        pros::Task brainScreenTask([&]() {
            while (true) {
                pros::lcd::print(0, "X: %f", chassis.getPose().x);
                pros::lcd::print(1, "Y: %f", chassis.getPose().y);
                pros::lcd::print(2, "Theta: %f", chassis.getPose().theta);
                pros::delay(50);
            }
        });
    }
}