
#include "custom/util_funcs.hpp"

#include "RclTracking.hpp"
#include "configs.hpp"
#include "custom/auton_selector.hpp"
#include <cmath>
#include <numbers>

#include "MclTracking.hpp"
#include "pros/misc.h"
#include "pros/rtos.h"

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
    topMotor.move(-127);
}
void topOut(int velocity = 127) {
    topMotor.move(1 * std::abs(velocity));
}
void slowTopOut() {
    topMotor.move(35);
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
    return (335 < topOptic.get_hue() || topOptic.get_hue() < 25);
}
bool topOpticIsBlue() {
    return (185 < topOptic.get_hue() && topOptic.get_hue() < 235);
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
void extendMidDescore() {
    middleDescore.extend();
};
void retractMidDescore() {
    middleDescore.retract();
};
void extendLeftArm() {
    leftDescoreArm.extend();
};
void retractLeftArm() {
    leftDescoreArm.retract();
};

// Auton functions
void moveForward(double inches, int timeout, float maxSpeed, float minSpeed, bool async) {
    chassis.moveToPoint(chassis.getPose().x+inches*std::cos(vexToStd(chassis.getPose().theta)), chassis.getPose().y+inches*std::sin(vexToStd(chassis.getPose().theta)), timeout, {.forwards=inches > 0 ? true : false, .maxSpeed=maxSpeed, .minSpeed=minSpeed}, async);
}
void jiggle(int repeats, int time) {
    for (int i = 0; i < repeats; i++) {
        moveForward(3, time/repeats*2/3, 41, 40, false);
        moveForward(-0.1, time/repeats/3, 41, 40, false);
    }
}
void shake(int repeats, int time) {
    double orig_theta = chassis.getPose().theta;
    for (int i = 0; i < repeats; i++) {
        chassis.turnToHeading(orig_theta + 10, time/repeats/4, {.maxSpeed=60}, false);
        chassis.turnToHeading(orig_theta - 10, time/repeats/4, {.maxSpeed=60}, false);
        chassis.turnToHeading(orig_theta, time/repeats/4, {.maxSpeed=60}, false);
        moveForward(8, time/repeats/4, 81, 40, false);
    }
}

void stopIntake() {
    middleMech.extend();
    lockTop();
    lockFront();
    stopTop();
    stopFront();
};
void stopTopScore() {
    if (outtakeTaskRunning && colorOuttakeTask != nullptr) {
        outtakeTaskRunning = false;
        colorOuttakeTask->remove();
    }
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
void startTopScore(int velocity) {
    stopIntake();
    frontIn();
    topOut(velocity);
};
void startTopScore(alliance_color color) {
    stopTopScore();
    if (color == alliance_color::RED) {
        colorOuttakeTask = new pros::Task ([](){
            Timer timer(800);
            outtakeTaskRunning = true;
            while (outtakeTaskRunning) {
                // General Control
                if (!topOpticIsBlue() || topOptic.get_proximity() < 200) {
                    frontIn();
                    topOut(127);
                }
                else startOuttake();
                // Anti stuck
                if (topOptic.get_proximity() > 200) timer.reset();
                if (timer.timeIsUp()) {
                    startOuttake();
                    pros::delay(100);
                    frontIn();
                    topOut(127);
                    pros::delay(200);
                    timer.reset();
                }
                pros::delay(40);
            }
        });
    }
    else if (color == alliance_color::BLUE) {
        colorOuttakeTask = new pros::Task ([](){
            Timer timer(800);
            outtakeTaskRunning = true;
            while (outtakeTaskRunning) {
                // General Control
                if (!topOpticIsRed() || topOptic.get_proximity() < 200) {
                    frontIn();
                    topOut(127);
                }
                else startOuttake();
                // Anti stuck
                if (topOptic.get_proximity() > 200) timer.reset();
                if (timer.timeIsUp()) {
                    startOuttake();
                    pros::delay(100);
                    frontIn();
                    topOut(127);
                    pros::delay(200);
                    timer.reset();
                }
                pros::delay(40);
            }
        });
    }
    else if (color == alliance_color::NONE) {
        colorOuttakeTask = new pros::Task ([](){
            Timer timer(800);
            outtakeTaskRunning = true;
            while (outtakeTaskRunning) {
                // Anti stuck
                if (topOptic.get_proximity() > 200) timer.reset();
                if (timer.timeIsUp()) {
                    startOuttake();
                    pros::delay(100);
                    frontIn();
                    topOut(127);
                    pros::delay(200);
                    timer.reset();
                }
                pros::delay(40);
            }
        });
    }
}
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
double pivot(double curr_corrd, double pivot_coord) {
    return curr_corrd+std::abs(curr_corrd-pivot_coord)*std::abs(curr_corrd-pivot_coord)/(curr_corrd-pivot_coord);
}
double pivot_x(double pivot_coord) {
    return pivot(chassis.getPose().x, pivot_coord);
}
double pivot_y(double pivot_coord) {
    return pivot(chassis.getPose().y, pivot_coord);
}


// Function for managing intake controls
void updateIntake() {

    // Motor Controls ----------------------------------------------------

    // Button B - Outtake
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
        frontOut();
        topIn();
    }
    // Button A - Slow outtake
    else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
        frontMotor.move(-25);
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
bool descoreMacroActivated = false;
void updatePneumatics() {
    // Button X - Match load mech (Toggle)
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
        matchLoadGate.toggle();
        retractMidDescore();
    }
    // Button Down - Right descore arm (Toggle)
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
        leftDescoreArm.toggle();
        descoreMacroActivated = false; // Shutdown macro
    }
    // Button Right - Descore macro (Toggle)
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
        descoreMacroActivated = !descoreMacroActivated;
        extendLeftArm();
    }
    // Button Y - Middle descore mech (Toggle)
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
        middleDescore.toggle();
        closeGate();
    }

    // Descore macro update
    if (descoreMacroActivated) {
        controller.rumble(".");
        // Release descore arm if distance less than 15 cm
        if (descoreDist.get() < 140) {
            retractLeftArm();
            //controller.rumble("-");
        }
    }
}

// Tank drive
void updateTankDrive() { chassis.tank(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y), controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y)); }

// Display
void stopBrainDisplay() {
    if (brainScreenTask != nullptr) {
        brainScreenTask->remove();
        delete brainScreenTask;
        brainScreenTask = nullptr;
    }
}
void stopControllerDisplay() {
    if (controllerScreenTask != nullptr) {
        controllerScreenTask->remove();
        delete controllerScreenTask;
        controllerScreenTask = nullptr;
    }
}
void startBrainDisplay() {
    stopBrainDisplay();
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
void startBrainFBDisplay() {
    stopBrainDisplay();
    if (brainScreenTask == nullptr) {
        brainScreenTask = new pros::Task ([&]() {
            static lv_obj_t* image = lv_image_create(lv_screen_active());
            lv_obj_align(image, LV_ALIGN_CENTER, 0, 0);
            lv_image_set_src(image, &FB_Logo);
        });
    }
}
void startControllerDisplay() {
    stopControllerDisplay();
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
void startControllerAutonSelectorDisplay() {
    stopControllerDisplay();
    if (controllerScreenTask == nullptr) {
        controllerScreenTask = new pros::Task ([&](){
            while (true) {
                controller.clear();
                pros::delay(50);
                controller.print(0, 0, "Color: %s", allianceColor == alliance_color::RED ? "RED" : "BLUE");
                pros::delay(50);
                controller.print(1, 0, "Type: %s", autonType == autonTypes::LEFT ? "LEFT" : autonType == autonTypes::LEFT_RUSH ? "LEFT_RUSH" : autonType == autonTypes::RIGHT ? "RIGHT" : autonType == autonTypes::RIGHT_RUSH ? "RIGHT_RUSH" : autonType == autonTypes::SOLO_AWP ? "SOLO_AWP" : "NULL");
                pros::delay(50);
                controller.print(2, 0, "Skills: %s", runningSkills ? "YES" : "NO");
                pros::delay(100);
            }
        });
    }
}
void startControllerRclDisplay() {
    stopControllerDisplay();
    if (controllerScreenTask == nullptr) {
        controllerScreenTask = new pros::Task ([&](){
            while (true) {
                controller.clear();
                pros::delay(50);
                controller.print(0, 0, "RCL (%.1f, %.1f, %.1f)", RclMain.getRclPose().x, RclMain.getRclPose().y, RclMain.getRclPose().theta);
                pros::delay(50);
                controller.print(1, 0, "Sens:(L:%s, B:%s, R:%s)", left_rcl.getBotCoord(chassis.getPose()).first == CoordType::X ? "X" : "Y", back_rcl.getBotCoord(chassis.getPose()).first == CoordType::X ? "X" : "Y", right_rcl.getBotCoord(chassis.getPose()).first == CoordType::X ? "X" : "Y");
                pros::delay(50);
                controller.print(2, 0, "(%.1f, %.1f, %.1f)", left_rcl.getBotCoord(chassis.getPose()).second, back_rcl.getBotCoord(chassis.getPose()).second, right_rcl.getBotCoord(chassis.getPose()).second);
                pros::delay(100);
            }
        });
    }
}
void startControllerMatchDisplay() {
    stopControllerDisplay();
    if (controllerScreenTask == nullptr) {
        controllerScreenTask = new pros::Task ([&](){
            while (true) {
                controller.clear();
                pros::delay(50);
                controller.print(0, 0, "DESC_MAC: %s", descoreMacroActivated ? "TRUE" : "false");
                pros::delay(150);
            }
        });
    }
}

// Test Functions
void startControllerDistDataDisplay() {
    stopControllerDisplay();
    if (controllerScreenTask == nullptr) {
        controllerScreenTask = new pros::Task ([&](){
            while (true) {
                controller.clear();
                pros::delay(50);
                controller.print(0, 0, "Left Sens: %d", left_dist.get_distance());
                pros::delay(50);
                controller.print(1, 0, "Back Sens: %d", back_dist.get_distance());
                pros::delay(50);
                controller.print(2, 0, "Right Sens: %d", right_dist.get_distance());
                pros::delay(100);
            }
        });
    }
}
void startControllerOpticDisplay() {
    stopControllerDisplay();
    if (controllerScreenTask == nullptr) {
        controllerScreenTask = new pros::Task ([&](){
            while (true) {
                controller.clear();
                pros::delay(50);
                controller.print(0, 0, "Hue: %f", topOptic.get_hue());
                pros::delay(50);
                controller.print(1, 0, "Prox: %d", topOptic.get_proximity());
                pros::delay(100);
            }
        });
    }
}
void startControllerRCLUpdate() {
    stopControllerDisplay();
    if (controllerScreenTask == nullptr) {
        controllerScreenTask = new pros::Task ([&](){
            while (true) {
                controller.clear();
                pros::delay(50);
                controller.print(0, 0, "X: %.1f, %.1f", chassis.getPose().x, RclMain.getRclPose().x);
                pros::delay(50);
                controller.print(1, 0, "Y: %.1f, %.1f", chassis.getPose().y, RclMain.getRclPose().y);
                pros::delay(50);
                controller.print(2, 0, "Heading: %.1f, %.1f", chassis.getPose().theta, RclMain.getRclPose().theta);
                pros::delay(100);
            }
        });
    }
}

// Mcl Benchmark with Heading Conversion for LCD
inline Pose rawMcl = {0,0,0};
inline Timer MclT(15);
inline double MclRate = 0.0;
inline double MclComputeTime = 0.0;

void startMclBenchmark() {
    stopControllerDisplay();
    stopBrainDisplay();

    // Initial Sync
    chassis.setPose(0, 0, 270);
    RclMain.updateBotPose(&left_rcl);
    RclMain.updateBotPose(&right_rcl);
    RclMain.updateBotPose(&back_rcl);
    lemlib::Pose odomLast = chassis.getPose();
    MclMain.set_pose(odomLast.x, odomLast.y, odomLast.theta);

    if (controllerScreenTask == nullptr && brainScreenTask == nullptr) {

        controllerScreenTask = new pros::Task ([&](){
            while (true) {
                MclT.reset();
                rawMcl = MclMain.updateMcl();
                // Sync On Demand
                if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
                   MclMain.updateBotPose();
                }
                MclComputeTime = MclT.elapsed();
                pros::delay(7);
                MclRate = 1000.0 / (7+MclComputeTime);
            }
        });

        brainScreenTask = new pros::Task ([&](){
            while (true) {
                lemlib::Pose odomLast = chassis.getPose();
                lemlib::Pose RclPose = RclMain.getRclPose();
                // Display Stats
                pros::lcd::print(0, "Mcl Rate: %.1f Hz", MclRate);
                pros::lcd::print(1, "Net Compute Time: %.4f ms", MclComputeTime);
                pros::lcd::print(2, "MclPos: X:%.1f Y:%.1f T:%.1f", rawMcl.x, rawMcl.y, 90.0 - (rawMcl.theta * 180.0 / M_PI));
                pros::lcd::print(3, "OdomPos: X:%.1f Y:%.1f T:%.1f", odomLast.x, odomLast.y, odomLast.theta);
                pros::lcd::print(4, "RclPos: X:%.1f Y:%.1f T:%.1f", RclPose.x, RclPose.y, RclPose.theta);
                pros::lcd::print(5, "B: %d mm L: %d mm R:%d mm", back_dist.get(), left_dist.get(), right_dist.get());
                pros::delay(60);
            }
        });

    }
}