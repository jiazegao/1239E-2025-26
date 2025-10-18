
#include "custom/util_funcs.h"
#include "custom/configs.h"
#include "custom/auton.h"
#include "pros/misc.h"
#include "pros/motors.h"

#include "auton_selector.h"

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
void slowTopOut() {
    topMotor.move(-20);
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
    return (345 < topOptic.get_hue() || topOptic.get_hue() < 15);
}
bool topOpticIsBlue() {
    return (195 < topOptic.get_hue() && topOptic.get_hue() < 225);
}

// Pneumatics functions
void openGate() {
    matchLoadGate.extend();
};
void closeGate() {
    matchLoadGate.retract();
};
void openMid() {
    middleMech.retract();
};
void closeMid() {
    middleMech.extend();
};
void extendLeftArm() {
    leftDescoreArm.extend();
};
void retractLeftArm() {
    leftDescoreArm.retract();
};
void extendRightArm() {
    rightDescoreArm.extend();
};
void retractRightArm() {
    rightDescoreArm.retract();
};

// Auton functions
void stopIntake() {
    middleMech.extend();
    lockTop();
    lockFront();
    stopTop();
    stopFront();
};
void stopTopScore() {
    stopIntake();
};
void stopMidScore() {
    stopIntake();
}
void stopOuttake() {
    stopIntake();
}
void startIntake() {
    stopIntake();
    lockTop();
    stopTop();
    frontIn();
};
void startTopScore() {
    stopIntake();
    frontIn();
    topOut();
};
void startMidScore() {
    stopIntake();
    middleMech.retract();
    frontIn();
    slowTopOut();
}
void startOuttake() {
    frontOut();
    topIn();
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
    // Button R1 - Score mid
    else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
        frontIn();
        slowTopOut();
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
    if (controllerScreenTask == nullptr) {
        controllerScreenTask = new pros::Task ([&](){
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
    if (brainScreenTask == nullptr) {
        brainScreenTask = new pros::Task ([&]() {
            while (true) {
                pros::lcd::print(0, "X: %f", chassis.getPose().x);
                pros::lcd::print(1, "Y: %f", chassis.getPose().y);
                pros::lcd::print(2, "Theta: %f", chassis.getPose().theta);
                pros::delay(50);
            }
        });
    }
}
void stopControllerDisplay() {
    if (controllerScreenTask != nullptr) {
        controllerScreenTask->remove();
        delete controllerScreenTask;
        controllerScreenTask = nullptr;
    }
}
void stopBrainDisplay() {
    if (brainScreenTask != nullptr) {
        brainScreenTask->remove();
        delete brainScreenTask;
        brainScreenTask = nullptr;
    }
}
void startControllerAutonSelectorDisplay() {
    if (controllerScreenTask == nullptr) {
        controllerScreenTask = new pros::Task ([&](){
            while (true) {
                controller.clear();
                pros::delay(50);
                controller.print(0, 0, "Color: %s", allianceColor == alliance_color::RED ? "RED" : "BLUE");
                pros::delay(50);
                controller.print(1, 0, "Type: %s", autonType == autonTypes::LEFT ? "LEFT" : autonType == autonTypes::RIGHT ? "RIGHT" : "SOLO_AWP");
                pros::delay(50);
                controller.print(2, 0, "Skills: %s", runningSkills ? "YES" : "NO");
                pros::delay(100);

                if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
                    toggle_color(nullptr);
                }
                if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
                    toggle_type(nullptr);
                }
                if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
                    toggle_skills(nullptr);
                }
            }
        });
    }
}