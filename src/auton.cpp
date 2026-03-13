#include "custom/auton.hpp"
#include "custom/RclTracking.hpp"
#include "custom/configs.hpp"
#include "custom/util_funcs.hpp"
#include "custom/lever_control.hpp"
#include "lemlib/chassis/chassis.hpp"
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
    chassis.moveToPoint(-27, 32, 900, {}, false);
    chassis.turnToHeading(90, 400, {}, true);
    extendLift();
    retractLeftArm();
    chassis.moveToPoint(-15, 36, 1000, {.minSpeed=127}, true);
}
void rightMidPush() {
    // Descore
    closeGate();
    chassis.moveToPoint(-27, -31, 900, {.forwards=false}, false);
    chassis.turnToHeading(270, 400, {}, true);
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
    chassis.turnToPoint(-65, -47, 300, {}, true);
    chassis.moveToPoint(-65, -47, 1200, {.maxSpeed=80}, true);

    // Score the long goal
    chassis.moveToPoint(-24, -46.5, 1800, {.forwards=false}, true);
    pros::delay(1400);
    closeGate();
    score(700, 7, 55);
    resetLever();

    // Intake 3 balls
    startIntake();
    chassis.turnToHeading(20, 700, {.maxSpeed=90}, true);
    chassis.moveToPoint(-22, -23, 900, {.maxSpeed=50}, true);
    pros::delay(600);
    openGate();

    // Intake 3 other balls
    chassis.moveToPoint(-22, 23, 600, {.minSpeed=70, .earlyExitRange=3}, true);
    closeGate();
    chassis.moveToPoint(-23, 23, 400, {.maxSpeed=50}, true);
    
    // Go to long goal
    chassis.moveToPoint(-40, 43, 1000, {.maxSpeed=70}, true);
    openGate();
    chassis.turnToHeading(270, 200, {}, false);    

    // Score long goal
    chassis.moveToPoint(-24, 47, 1800, {.forwards=false}, true);
    pros::delay(1200);
    score(700, 7, 55);
    resetLever();
    
    // Intake from matchloader
    chassis.moveToPoint(-65, 47, 1600, {.maxSpeed=60}, true);
    pros::delay(300);
    startIntake();

    // Score mid
    retractLift();
    moveForward(-5, 800, 30, 1, true);
    chassis.moveToPoint(-12, 10, 1200, {.forwards=false}, false);
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
    chassis.moveToPoint(-23, 23, 1100, {}, true);
    pros::delay(500);
    openGate();

    // Head to long goal
    chassis.turnToPoint(-40, 43, 400, {}, true);
    chassis.moveToPoint(-40, 43, 1000, {}, false);

    // Score long goal
    chassis.turnToHeading(270, 300, {}, false);
    chassis.moveToPoint(-24, 47, 1800, {.forwards=false}, true);
    pros::delay(800);
    score(1300, 7, 55);
    
    // Intake from matchloader
    chassis.moveToPoint(-65, 47, 1600, {.maxSpeed=60}, true);
    startIntake();

    // Score mid
    chassis.moveToPoint(-11, 9, 1300, {.forwards=false}, false);
    retractLift();
    closeGate();
    chassis.turnToPoint(0, 0, 200, {.forwards=false}, true);
    if (midScoreDelay > 0) pros::delay(midScoreDelay);
    score(1200, 7, 25);
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
    chassis.moveToPoint(-23, -23, 1100, {}, true);
    pros::delay(500);
    openGate();

    // Head to long goal
    chassis.turnToPoint(-40, -43, 400, {}, true);
    chassis.moveToPoint(-40, -43, 1000, {}, false);

    // Score long goal
    chassis.turnToHeading(270, 300, {}, false);
    chassis.moveToPoint(-24, -47, 1800, {.forwards=false}, true);
    pros::delay(800);
    score(1300, 7, 55);
    
    // Intake from matchloader
    chassis.moveToPoint(-65, -47, 1600, {.maxSpeed=60}, true);
    startIntake();
    
    // Score mid
    chassis.moveToPoint(-50, -47, 600, {.forwards=false}, true);
    pros::delay(400);
    closeGate();
    chassis.turnToPoint(-15, -10, 400, {}, true);
    chassis.moveToPoint(-15, -10, 1200, {}, true);
    chassis.turnToPoint(0, 0, 300, {}, false);
    startOuttake(250);
    pros::delay(1600);

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
    chassis.setPose(-44, 0, 270);
    startMcl(-44, 0, 270, true, true, false, true);

    // Clear Park Zone
    startIntake();
    moveForward(-5, 800, 127, 1, true);
    chassis.moveToPoint(-56, 0, 2000, {.maxSpeed=70}, false);
    pros::delay(1500);
    leftMotors.move(127);
    rightMotors.move(127);
    pros::delay(800);
    shake(6, 2800);
    chassis.turnToHeading(270, 400, {}, false);
    moveForward(-20, 1300, 127, 60, false);

    // Score top mid
    chassis.moveToPoint(-20, 9, 1000, {.forwards=false}, false);
    chassis.swingToHeading(315, lemlib::DriveSide::LEFT, 1200, {.maxSpeed=60}, false);
    chassis.turnToPoint(-23.5, 23.5, 250, {}, false);
    moveForward(6.5, 800, 30, 1, false);
    chassis.moveToPoint(-8, 8, 800, {.forwards=false}, false);
    chassis.turnToPoint(0, 0, 300, {.forwards=false}, true);
    stopIntake();
    retractLift();
    openHood();
    hoodLock = true;
    stopIntake();
    startIntake();
    pros::delay(400);
    startIntake();
    pros::delay(400);
    scoreReserve(1800, 0, 20);

    // Refill at top left
    resetLever();
    startIntake();        
    chassis.moveToPoint(-23.5, 23.5, 300, {.minSpeed=60, .earlyExitRange=3}, true);
    chassis.moveToPoint(-23.5, 23.5, 400, {.maxSpeed=35}, true);
    chassis.moveToPoint(-47, 42, 800, {}, true);
    hoodLock = false;
    extendLift();
    chassis.turnToHeading(270, 200, {}, true);
    chassis.moveToPoint(-24, 47, 1000, {.forwards=false}, true);
    pros::delay(800);
    scoreReserve(600, 0, 70);
    openGate();
    startIntake();
    chassis.moveToPoint(-70, 47, 1000, {.maxSpeed=70}, false);
    jiggle(3, 2000);

    // Score at top-right long goal
    chassis.turnToHeading(225, 200, {}, false);
    chassis.moveToPose(-25, 63, 270, 1000, {.forwards=false, .lead=0.5, .minSpeed=127}, false);
    closeGate();
    chassis.moveToPoint(33, 63, 1200, {.forwards=false}, false);
    chassis.moveToPoint(24, 47, 1000, {.forwards=false}, false);
    chassis.swingToHeading(90, lemlib::DriveSide::LEFT, 500, {.minSpeed=100}, true);
    scoreReserve(700, 0, 70);

    // Refill at top-right loader then score again
    openGate();
    startIntake();
    chassis.moveToPoint(70, 47, 1000, {.maxSpeed=70}, false);
    jiggle(3, 2000);
    chassis.moveToPoint(27, 47, 1100, {.forwards=false, .maxSpeed=110}, true);
    pros::delay(800);
    scoreReserve(1100, 0, 30);
    closeGate();

    // Clear Park Zone
    chassis.turnToPoint(39, 0, 600, {}, false);
    chassis.moveToPoint(39, 4, 1200, {}, false);
    chassis.turnToPoint(56, 0, 800, {}, false);
    pros::delay(300);

    // Clear Park Zone
    startIntake();
    chassis.moveToPoint(56, 0, 2000, {.maxSpeed=70}, false);
    pros::delay(1500);
    leftMotors.move(127);
    rightMotors.move(127);
    pros::delay(800);
    shake(6, 2800);
    chassis.turnToHeading(90, 400, {}, false);
    moveForward(-20, 1200, 127, 60, false);

    // Score bottom  mid
    chassis.turnToPoint(19, 16, 500, {}, false);
    chassis.moveToPoint(19, 16, 1400, {.maxSpeed=60}, true);
    chassis.turnToPoint(12,12   , 500, {}, true);
    chassis.moveToPoint(12, 12, 1000, {.maxSpeed = 50}, false);
    chassis.turnToPoint(0, 0, 500, {}, true);
    moveForward(-1, 500, {}, true);
    startOuttake(200);
    pros::delay(3000);

    // Pick up four balls
    chassis.moveToPoint(19, 19, 600, {.forwards=false}, false);
    chassis.turnToPoint(22, -22.5, 500, {}, true);
    startIntake();
    chassis.moveToPoint(22, -22.5, 700, {.minSpeed=60, .earlyExitRange=3}, false);
    chassis.moveToPoint(22, -22.5, 500, {.maxSpeed=40}, true);

    // Refill at top left
    startIntake();        
    chassis.moveToPoint(40, -43, 900, {}, true);
    pros::delay(400);
    extendLift();
    openGate();
    chassis.turnToHeading(90, 200, {}, true);
    chassis.moveToPoint(24, -47, 1000, {.forwards=false}, true);
    pros::delay(800);
    scoreReserve(600, 0, 70);
    startIntake();
    chassis.moveToPoint(70, -47, 1000, {.maxSpeed=70}, false);
    jiggle(3, 2000);

    // Score at top-right long goal
    chassis.turnToHeading(45, 200, {}, false);
    chassis.moveToPose(25, -63, 90, 1000, {.forwards=false, .lead=0.5, .minSpeed=127}, false);
    closeGate();
    chassis.moveToPoint(-33, -63, 1200, {.forwards=false}, false);
    chassis.moveToPoint(-24, -47, 1000, {.forwards=false}, false);
    chassis.swingToHeading(270, lemlib::DriveSide::LEFT, 500, {.minSpeed=100}, true);
    scoreReserve(700, 0, 80);

    // Refill at bottom-left loader then score again
    openGate();
    startIntake();
    chassis.moveToPoint(-70, -47, 1000, {.maxSpeed=70}, false);
    jiggle(3, 2000);
    chassis.moveToPoint(-27, -47, 1100, {.forwards=false, .maxSpeed=110}, true);
    pros::delay(800);
    scoreReserve(1100, 0, 30);
    closeGate();

    // Park
    chassis.moveToPoint(-67, -24, 800, {}, false);
    chassis.turnToHeading(0, 400, {}, false);
    chassis.moveToPoint(-63, 0, 1800, {.minSpeed=80}, false);
}
