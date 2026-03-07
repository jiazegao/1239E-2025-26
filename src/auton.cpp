#include "custom/auton.hpp"
#include "custom/RclTracking.hpp"
#include "custom/configs.hpp"
#include "custom/util_funcs.hpp"
#include "custom/lever_control.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include <cmath>

void leftPush(float x_offset = 0, float y_offset = 0) {
    chassis.moveToPoint(-40+x_offset, 37.5+y_offset, 900, {}, false);
    chassis.turnToHeading(90, 600, {}, false);
    retractLeftArm();
    chassis.moveToPoint(-21+x_offset, 37.5+y_offset, 2000, {.minSpeed=127, .earlyExitRange=1}, false);
    chassis.turnToHeading(135, 2000, {.minSpeed=127}, false);
}
void rightPush(float x_offset = 0, float y_offset = 0) {
    chassis.moveToPoint(-40+x_offset, -37.5-y_offset, 900, {}, false);
    chassis.turnToHeading(270, 450, {}, false);
    retractLeftArm();
    chassis.moveToPoint(-24+x_offset, -37.5-y_offset, 2000, {.forwards=false, .minSpeed=127}, false);
}


void soloAWP(){
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.setPose(-47, 0, 0);
    MclMain.setObstacles(&soloAWP_obstacles, nullptr);
    startIntake();
    startMcl(-47, 0, 0, false, true, true, false);

    // Push teammate and get their preload
    chassis.moveToPoint(-47, 15, 400, {}, false);

    // Head towards the matchloader and intake
    chassis.moveToPoint(-47, -47, 1300, {.forwards=false}, true);
    pros::delay(400);
    openGate();
    chassis.turnToPoint(-65, -47, 400, {}, false);
    startIntake();
    chassis.moveToPoint(-65, -47, 1400, {.maxSpeed=80}, false);

    // Score the long goal
    chassis.moveToPoint(-24, -47.5, 1100, {.forwards=false}, true);
    pros::delay(300);
    score(900, 7);
    closeGate();
    
    // Intake 3 balls
    chassis.turnToHeading(20, 800, {.maxSpeed=100}, false);
    startIntake();

    // Intake 3 other balls
    chassis.moveToPoint(-23, 23, 1400, {.maxSpeed=110}, true);
    pros::delay(300);
    openGate();
    pros::delay(200);
    closeGate();
    pros::delay(300);
    openGate();

    // Score the mid goal
    chassis.turnToPoint(-8, 8, 400, {.forwards=false}, false);
    chassis.moveToPoint(-8, 8, 1000, {.forwards=false, .minSpeed=40}, true);
    chassis.turnToHeading(315, 200, {}, false);
    pros::delay(200);
    retractLift();
    score(1000, 7, 40);

    extendLift();
    openGate();
    chassis.moveToPoint(-47, 47, 1400, {}, false);
    chassis.turnToPoint(-65, 47, 500, {}, false);

    startIntake();
    chassis.moveToPoint(-65, 47, 1400, {.maxSpeed=80}, false);

    // Score again
    chassis.moveToPoint(-24, 47.5, 1100, {.forwards = false, .maxSpeed=110}, true);
    pros::delay(300);
    score(2000, 7);
}
void counterSAWP() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.setPose(-47, 0, 0);
    MclMain.setObstacles(&soloAWP_obstacles, nullptr);
    startIntake();
    startMcl(-47, 0, 0, false, true, true, false);
    frontMotor.move(127);

    // Push teammate and get their preload
    chassis.moveToPoint(-47, 15, 400, {}, false);
    startIntake();

    // Head towards the matchloader and intake
    chassis.moveToPoint(-47, -47, 1300, {.forwards=false}, true);
    pros::delay(400);
    openGate();
    chassis.turnToPoint(-65, -47, 350, {}, false);
    startIntake();
    chassis.moveToPoint(-65, -47, 1100, {.maxSpeed=80}, false);

    // Score the long goal
    chassis.moveToPoint(-24, -47.5, 1500, {.forwards=false, .maxSpeed=70}, true);
    pros::delay(900);
    score(800, 7, 100);
    closeGate();

    // Intake 3 balls
    chassis.turnToHeading(20, 800, {.maxSpeed=90}, false);
    startIntake();

    // Intake 3 other balls
    chassis.moveToPoint(-21, 19, 1000, {.minSpeed=40, .earlyExitRange=3}, true);
    pros::delay(1000);
    openGate();

    // Go to long goal
    chassis.moveToPoint(-42, 46, 1100, {}, false);

    // Score long goal
    chassis.moveToPoint(-24, 47, 1100, {.forwards = false, .maxSpeed=110}, true);
    pros::delay(400);
    score(800, 7, 100);

    // Intake from matchloader
    chassis.moveToPoint(-65, 47, 1700, {.maxSpeed=100}, true);
    pros::delay(800);
    startIntake();

    // Score mid
    chassis.moveToPoint(-47, 47, 400, {.forwards=false}, false);
    chassis.turnToPoint(-9, 9, 200, {.forwards=false}, false);
    chassis.moveToPoint(-9, 9, 1100, {.forwards=false}, true);
    pros::delay(900);
    retractLift();
    score(1000, 7, 70);

    // Descore
    closeGate();
    chassis.moveToPoint(-28, 34, 800, {}, false);
    chassis.turnToHeading(90, 400, {}, false);
    retractLeftArm();
    chassis.moveToPoint(-21, 37.5, 2000, {.minSpeed=127, .earlyExitRange=1}, false);
    chassis.turnToHeading(135, 2000, {.minSpeed=127}, false);
}
void leftControlRush() {
    
}

void leftFastRush() {
    
}

void rightControlRush() {
    
}

void rightFastRush() {
    
}

void leftv2() {
    
}

void rightv2() {

}

void right() {

}

void NAAuto() {
    
}

void left() {

}

void skills() {
    
}
// 90+ points
void skills_v2() {

}

// 80 points
void skills_v3() {
}

// 119 points
void skills_119() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.setPose(-44, 0, 270);
    startMcl(-44, 0, 270, true, true, false, true);

    // Clear Park Zone
    startIntake();
    moveForward(-5, 1500, 127, 1, false);
    chassis.moveToPoint(-80, 0, 1200, {.minSpeed=100, .earlyExitRange=4}, false);
    shake(4, 1000);
    moveForward(10, 500);
    shake(2, 1000);
    moveForward(10, 500);
    moveForward(-20, 1600);

    // Reset location
    chassis.turnToHeading(0, 600, {}, false);
    pros::delay(400);

    // Score top mid
    chassis.turnToHeading(270, 600, {}, false);
    chassis.moveToPose(-11, 11, 315, 1200, {.forwards=false, .lead=0.15}, false);
    chassis.moveToPoint(-17, 17, 1000, {.maxSpeed=30}, false);
    chassis.moveToPose(-9, 9, 315, 1000, {.forwards=false}, false);
    retractLift();
    score(3000, 7, 20);
    extendLift();

    // Refill at top left
    startIntake();
    chassis.turnToPoint(-47, 48, 200, {}, false);
    chassis.moveToPoint(-47, 48, 1400, {}, true);
    pros::delay(700);
    openGate();
    chassis.turnToPoint(-63, 47, 500, {}, false);
    startIntake();
    chassis.moveToPoint(-70, 47, 1000, {.maxSpeed=60}, false);
    jiggle(4, 2000);
    stopIntake();
    closeGate();

    // Score at top-right long goal
    chassis.turnToHeading(225, 200, {}, false);
    chassis.moveToPose(-25, 63, 270, 1200, {.forwards=false, .lead=0.5, .minSpeed=50}, false);
    chassis.moveToPoint(33, 63, 1500, {.forwards=false}, false);
    chassis.moveToPose(24, 48, 90, 1800, {.forwards=false, .lead = 0.2, .minSpeed=60}, false);
    chassis.swingToHeading(90, lemlib::DriveSide::LEFT, 800, {}, false);
    score(1200, 7, 80);

    // Refill at top-right loader then score again
    openGate();
    chassis.moveToPoint(70, 47, 1600, {.maxSpeed=70}, true);
    pros::delay(300);
    startIntake();
    pros::delay(1300);
    jiggle(4, 2000);
    chassis.moveToPoint(27, 47, 1400, {.forwards=false, .maxSpeed=110}, false);
    score(2000, 7, 40);
    closeGate();

    // Clear Park Zone
    chassis.turnToPoint(40, 0, 1200, {}, false);
    chassis.moveToPoint(40, 0, 2500, {}, false);
    startIntake();
    chassis.turnToHeading(90, 600, {}, false);
    moveForward(-5, 1500, 127, 1, false);
    chassis.moveToPoint(70, 0, 2200, {.minSpeed=100, .earlyExitRange=4}, false);
    moveForward(-20, 1600);

    // Reset location
    chassis.turnToHeading(0, 600, {}, false);
    pros::delay(400);

    // Score top mid
    chassis.turnToPoint(19, 19, 500, {}, false);
    chassis.moveToPoint(19, 19, 1200, {}, false);
    chassis.turnToPoint(13, 13, 800, {}, false);
    chassis.moveToPoint(13, 13, 800, {}, false);
    chassis.turnToPoint(0, 0, 500, {}, true);
    startOuttake();
    pros::delay(2000);

    // Pick up four balls
    chassis.moveToPoint(19, 19, 600, {.forwards=false}, false);
    chassis.turnToPoint(22, -23.5, 600, {}, false);
    startIntake();
    chassis.moveToPoint(22, -23.5, 2500, {.maxSpeed=70}, false);

    // Refill at bottom right
    chassis.turnToPoint(47, -48, 500, {}, false);
    chassis.moveToPoint(47, -48, 1300, {}, true);
    pros::delay(700);
    openGate();
    chassis.turnToPoint(63, -47, 500, {}, false);
    chassis.moveToPoint(70, -47, 1000, {.maxSpeed=65}, false);
    jiggle(4, 2000);
    stopIntake();
    closeGate();

    // Score at bottom-left long goal
    chassis.turnToHeading(45, 200, {}, false);
    chassis.moveToPose(25, -63, 90, 1200, {.forwards=false, .lead=0.5, .minSpeed=50}, false);
    chassis.moveToPoint(-33, -63, 1500, {.forwards=false}, false);
    chassis.moveToPose(-24, -48, 270, 1800, {.forwards=false, .lead = 0.2, .minSpeed=60}, false);
    chassis.swingToHeading(270, lemlib::DriveSide::LEFT, 800, {}, false);
    score(2000, 7, 100);

    // Refill at bottom-left loader then score again
    openGate();
    chassis.moveToPoint(-70, -47, 1600, {.maxSpeed=65}, true);
    pros::delay(300);
    startIntake();
    pros::delay(1300);
    jiggle(4, 2000);
    chassis.moveToPoint(-28, -47, 1400, {.forwards=false, .maxSpeed=90}, false);
    score(3000, 7, 40);
    closeGate();

    // Park
    chassis.moveToPoint(-67, -24, 800, {}, false);
    chassis.turnToHeading(0, 800, {}, false);
    chassis.moveToPoint(-63, 0, 1800, {.minSpeed=80, .earlyExitRange=3}, false);
}