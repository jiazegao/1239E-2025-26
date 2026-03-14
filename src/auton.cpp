#include "custom/auton.hpp"
#include "custom/MclTracking.hpp"
#include "custom/RclTracking.hpp"
#include "custom/configs.hpp"
#include "custom/util_funcs.hpp"
#include "custom/lever_control.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/pose.hpp"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include <cmath>

void leftPush() {
    // Push
    chassis.moveToPoint(-38, 36.5, 900, {}, true);
    chassis.turnToPoint(0, 37.5, 600, {}, true);
    retractLeftArm();
    chassis.moveToPoint(-16, 37.5, 2000, {.minSpeed=127}, true);
}
void rightPush() {
    // Push
    chassis.moveToPoint(-38, -36.5, 900, {}, true);
    chassis.turnToPoint(0, -37.5, 400, {.forwards=false}, true);
    retractLeftArm();
    chassis.moveToPoint(-16, -37.5, 2000, {.forwards=false, .minSpeed=127}, true);
}
void leftMidPush() {
    // Descore
    closeGate();
    chassis.moveToPoint(-17, 32, 600, {.minSpeed=127, .earlyExitRange=4}, true);
    chassis.turnToHeading(90, 700, {}, true);
    extendLift();
    retractLeftArm();
    chassis.moveToPoint(-15, 36, 1000, {.minSpeed=127}, true);
}
void rightMidPush() {
    // Descore
    closeGate();
    chassis.moveToPoint(-17, -32, 500, {.forwards=false, .minSpeed=127, .earlyExitRange=4}, true);
    chassis.turnToHeading(270, 700, {}, true);
    extendLift();
    retractLeftArm();
    chassis.moveToPoint(-15, -36, 1000, {.forwards=false, .minSpeed=127}, true);
}

int midScoreDelay = 0;
void setMidScoreDelay(int newDelay) {midScoreDelay = newDelay;}
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

    // Push teammate and get their preload
    chassis.moveToPoint(-47, 15, 300, {}, true);

    // Head towards the matchloader and intake
    chassis.moveToPoint(-47, -43, 1400, {.forwards=false}, true);
    pros::delay(400);
    openGate();
    chassis.turnToHeading(270, 400, {}, true);
    chassis.moveToPoint(-65, -47, 1200, {.maxSpeed=80}, true);

    // Score the long goal
    chassis.moveToPoint(-24, -46.5, 1800, {.forwards=false}, true);
    pros::delay(1400);
    closeGate();
    score(800, 7, 45);
    resetLever();

    // Intake 3 balls
    startIntake();
    chassis.turnToHeading(20, 700, {.maxSpeed=90}, true);
    chassis.moveToPoint(-22, -23, 900, {.maxSpeed=50}, true);
    pros::delay(600);
    openGate();

    // Intake 3 other balls
    chassis.moveToPoint(-20, 23, 600, {.minSpeed=70, .earlyExitRange=3}, true);
    closeGate();
    chassis.moveToPoint(-22, 23, 400, {.maxSpeed=50}, true);
    
    // Go to long goal
    chassis.moveToPoint(-40, 43, 1000, {.maxSpeed=70}, true);
    openGate();
    chassis.turnToHeading(270, 200, {}, false);    

    // Score long goal
    chassis.moveToPoint(-24, 47, 1800, {.forwards=false}, true);
    pros::delay(1200);
    score(800, 7, 45);
    resetLever();
    
    // Intake from matchloader
    chassis.moveToPoint(-65, 47, 1700, {.maxSpeed=60}, true);
    pros::delay(300);
    startIntake();

    // Score mid
    moveForward(-5, 700, 30, 1, true);
    chassis.moveToPoint(-12, 10, 1200, {.forwards=false}, true);
    pros::delay(300);
    retractLift();
    chassis.turnToPoint(0, 0, 200, {.forwards=false}, true);
    score(1000, 7, 30);
    resetLever();
    
    // leftMidPush();
}
void leftv2() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.setPose(-48.75, 16.125, 90);
    MclMain.setObstacles(&quadrant_dividers, nullptr);
    startMcl(-48.75, 16.125, 90, false, true, true, false);
    
    // Intake three balls
    startIntake();
    chassis.moveToPoint(-23, 21, 1100, {}, true);
    pros::delay(400);
    openGate();

    // Head to long goal
    chassis.turnToPoint(-40, 42, 400, {}, true);
    chassis.moveToPoint(-40, 42, 900, {}, true);

    // Score long goal
    chassis.turnToHeading(270, 200, {}, true);
    chassis.moveToPoint(-24, 47, 1800, {.forwards=false}, true);
    pros::delay(800);
    score(600, 7, 65);
    
    // Intake from matchloader
    chassis.moveToPoint(-65, 47, 1600, {.maxSpeed=60}, true);
    pros::delay(300);
    startIntake();

    // Score mid
    moveForward(-5, 800, 30, 1, true);
    chassis.moveToPoint(-12, 10, 1200, {.forwards=false}, true);
    pros::delay(300);
    retractLift();
    chassis.turnToPoint(0, 0, 200, {.forwards=false}, true);
    if (midScoreDelay > 0) pros::delay(midScoreDelay);
    score(1000, 7, 30);
    resetLever();

    // Push
    leftMidPush();
}
void rightv2() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.setPose(-48.75, -16.125, 90);
    MclMain.setObstacles(&quadrant_dividers, nullptr);
    startMcl(-48.75, -16.125, 90, false, false, true, true);
    
    // Intake three balls
    startIntake();
    chassis.moveToPoint(-23, -21, 1100, {}, true);
    pros::delay(400);
    openGate();

    // Head to long goal
    chassis.turnToPoint(-40, -39, 400, {}, true);
    chassis.moveToPoint(-40, -39, 900, {}, true);

    // Score long goal
    chassis.turnToHeading(270, 200, {}, true);
    chassis.moveToPoint(-24, -46, 1800, {.forwards=false}, true);
    pros::delay(800);
    score(600, 7, 65);
    
    // Intake from matchloader
    chassis.moveToPoint(-65, -47, 1600, {.maxSpeed=60}, true);
    pros::delay(300);
    startIntake();
    
    // Score mid
    moveForward(-4, 700, 30, 1, true);
    pros::delay(400);
    closeGate();
    chassis.turnToPoint(-13, -10, 700, {}, true);
    chassis.moveToPoint(-13, -10, 1400, {.maxSpeed=80}, true);
    chassis.turnToPoint(0, 0, 300, {}, true);
    if (midScoreDelay > 0) pros::delay(midScoreDelay);
    startOuttake(300);
    jiggle(3, 2000, 1.0, 2.0);

    // Push
    rightMidPush();
}
void leftControlRush() {
    
}
void rightControlRush() {
    
}
void leftFastRush() {

}
void rightFastRush() {

}

// 119 points
void skills_119() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.setPose(-54, 0, 270);
    startMcl(-54, 0, 270, true, true, false, true);

    // Clear Park Zone
    startIntake();
    antiStuckOn = false;
    moveForward(3.0, 800, 127, 20, false);
    pros::delay(500);
    leftMotors.move(127);
    rightMotors.move(127);
    pros::delay(800);
    shake(3, 2400);
    chassis.turnToHeading(270, 400, {}, false);
    moveForward(-20, 1300, 127, 60, false);
    antiStuckOn = true;

    // Score top mid
    chassis.moveToPoint(-21, 10, 1300, {.forwards=false}, false);
    chassis.swingToHeading(315, lemlib::DriveSide::LEFT, 900, {.maxSpeed=70}, false);
    chassis.turnToPoint(-23.5, 23.5, 400, {}, false);
    MclMain.setDistSyncProp(1e-6);
    moveForward(7.0, 1000, 127, 1, false);
    MclMain.setDistSyncProp(0.30);
    chassis.moveToPoint(-7, 7, 800, {.forwards=false}, true);
    pros::delay(300);
    hoodLock = true;
    retractLift();
    openHood();
    chassis.turnToPoint(0, 0, 300, {.forwards=false}, true);
    startIntake();
    pros::delay(700);
    scoreReserve(3000, 0,15);

    // Refill at top left
    resetLever();
    startIntake();        
    chassis.moveToPoint(-23.5, 23.5, 300, {.minSpeed=60, .earlyExitRange=3}, true);
    chassis.moveToPoint(-23.5, 23.5, 400, {.maxSpeed=35}, true);
    chassis.moveToPoint(-47, 42, 1100, {.maxSpeed=70}, true);
    hoodLock = false;
    extendLift();
    chassis.turnToHeading(270, 200, {}, true);
    
    chassis.moveToPoint(-24, 47, 1000, {.forwards=false}, true);
    // Cool tech
    hoodLock = true; 
    openHood();
    pros::delay(1200);
    scoreReserve(800, 0, 50);
    hoodLock = false; 
    closeHood();
    openGate();
    startIntake();
    chassis.moveToPoint(-63, 47, 2400, {.maxSpeed=60}, false);
    jiggle(2, 1400);

    // Score at top-right long goal
    chassis.turnToHeading(225, 200, {}, false);
    chassis.moveToPose(-25, 63, 270, 1000, {.forwards=false, .lead=0.5, .minSpeed=127}, false);
    closeGate();
    chassis.moveToPoint(33, 63, 1200, {.forwards=false}, false);
    chassis.moveToPoint(24, 46, 1300, {.forwards=false}, true);
    chassis.swingToHeading(90, lemlib::DriveSide::LEFT, 800, {.minSpeed=100}, true);
    pros::delay(400);
    scoreReserve(1000, 0, 40);

    // Refill at top-right loader then score again
    openGate();
    startIntake();
    chassis.moveToPoint(63, 47, 2400, {.maxSpeed=60}, false);
    jiggle(2, 1400);
    chassis.moveToPoint(24, 47, 1500, {.forwards=false, .maxSpeed=110}, true);
    pros::delay(600);
    scoreReserve(1400, 0, 20);
    closeGate();

    // Park (?)
    startIntake();
    chassis.moveToPoint(67, 24, 700, {}, false);
    chassis.turnToHeading(180, 500, {}, false);
    chassis.moveToPoint(65, 18, 800, {.maxSpeed=80}, false);
    openGate();
    chassis.moveToPoint(65, -24, 2000, {.maxSpeed=80}, true);
    pros::delay(500);
    closeGate();

    // Refill at bottom right
    chassis.turnToPoint(40, -47, 500, {}, false);
    chassis.moveToPoint(40, -47, 1200, {}, true);
    chassis.turnToHeading(90, 500, {}, true);
    chassis.moveToPoint(24, -47, 1000, {.forwards=false}, true);
    openGate();
    pros::delay(1200);
    scoreReserve(1000, 0, 50);
    startIntake();
    chassis.moveToPoint(63, -47, 2400, {.maxSpeed=60}, false);
    jiggle(2, 1400);

    // Score at bottom-left long goal
    chassis.turnToHeading(45, 200, {}, false);
    chassis.moveToPose(25, -63, 90, 1000, {.forwards=false, .lead=0.5, .minSpeed=127}, false);
    closeGate();
    chassis.moveToPoint(-33, -63, 1200, {.forwards=false}, false);
    chassis.moveToPoint(-24, -46, 1300, {.forwards=false}, true);
    chassis.swingToHeading(270, lemlib::DriveSide::LEFT, 800, {.minSpeed=100}, true);
    pros::delay(400);
    scoreReserve(1000, 0, 40);

    // Refill at bottom-left loader then score again
    openGate();
    startIntake();
    chassis.moveToPoint(-63, -47, 2400, {.maxSpeed=60}, false);
    jiggle(2, 1400);
    chassis.moveToPoint(-24, -47, 1300, {.forwards=false, .maxSpeed=110}, true);
    pros::delay(600);
    scoreReserve(1400, 0, 20);
    closeGate();

    // Score low mid
    startIntake();
    chassis.turnToPoint(-23.5, -23.5, 800, {}, false);
    chassis.moveToPoint(-23.5, -23.5, 1000, {}, false);
    chassis.moveToPoint(-11, -9, 1000, {.maxSpeed=80}, false);
    chassis.turnToPoint(0, 0, 300, {}, true);
    startOuttake(300);
    jiggle(2, 2000, 1.0, 2.0);
    startIntake();

    // Park
    chassis.moveToPoint(-40, 0, 1000, {.forwards=false}, false);
    chassis.turnToHeading(270, 700, {}, false);
    chassis.moveToPoint(-63, 0, 1800, {.minSpeed=127}, false);
}
