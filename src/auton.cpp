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
    startMcl(-47, 0, 0, false, true, true, false);

    // Push teammate and get their preload
    startIntake();
    chassis.moveToPoint(-47, 10, 350, {}, false);

    // Head towards the matchloader and intake
    chassis.moveToPoint(-47, -47, 1300, {.forwards=false}, true);
    pros::delay(400);
    openGate();
    chassis.turnToPoint(-65, -47, 400, {}, false);
    startIntake();
    chassis.moveToPoint(-65, -47, 1100, {.maxSpeed=60}, false);

    // Score the long goal
    chassis.moveToPoint(-24, -47.5, 1100, {.forwards=false, .maxSpeed=110}, false);
    score(1500, 7);
    closeGate();
    
    // Intake 3 balls
    chassis.turnToHeading(10, 800, {.maxSpeed=100}, false);
    startIntake();
    chassis.moveToPoint(-23.5, -23.5, 700, {.maxSpeed=80}, true);
    pros::delay(300);
    openGate();
    pros::delay(300);
    closeGate();

    // Intake 3 other balls
    chassis.moveToPoint(-23, 23, 1100, {.maxSpeed=110}, true);
    pros::delay(700);
    openGate();
    retractLift();

    // Score the mid goal
    chassis.turnToHeading(320, 400, {}, true);
    chassis.moveToPoint(-8, 8, 1000, {.forwards=false, .maxSpeed=110}, false);
    score(1500, 7, 40);

    closeGate();
    extendLift();
    chassis.moveToPoint(-47, 47, 1100, {}, true);
    pros::delay(400);
    openGate();    
    chassis.turnToPoint(-65, 47, 500, {}, false);

    startIntake();
    chassis.moveToPoint(-65, 47, 1100, {.maxSpeed=100}, false);

    // Score again
    chassis.moveToPoint(-24, 47.5, 1100, {.forwards = false, .maxSpeed=110}, false);
    score(2000, 7);
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
    
}