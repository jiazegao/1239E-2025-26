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
    chassis.moveToPoint(-27, 30, 900, {}, false);
    chassis.turnToHeading(90, 400, {}, true);
    extendLift();
    retractLeftArm();
    chassis.moveToPoint(-15, 36, 1000, {.minSpeed=127}, true);
}
void rightMidPush() {
    // Descore
    closeGate();
    chassis.moveToPoint(-27, -33, 900, {.forwards=false}, false);
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
    chassis.moveToPoint(-47, -46, 1400, {.forwards=false}, true);
    pros::delay(400);
    openGate();
    chassis.turnToPoint(-65, -47, 300, {}, true);
    chassis.moveToPoint(-65, -47, 1300, {.maxSpeed=80}, true);

    // Score the long goal
    chassis.moveToPoint(-24, -46.5, 1800, {.forwards=false}, true);
    pros::delay(1200);
    closeGate();
    score(700, 7, 55);
    resetLever();

    // Intake 3 balls
    startIntake();
    chassis.turnToHeading(20, 700, {.maxSpeed=90}, true);
    chassis.moveToPoint(-22, -23, 900, {.maxSpeed=50}, true);

    // Intake 3 other balls
    chassis.moveToPoint(-22, 23, 600, {.minSpeed=70, .earlyExitRange=3}, true);
    chassis.moveToPoint(-23, 23, 400, {.maxSpeed=50}, true);
    
    // Go to long goal
    chassis.moveToPoint(-42, 44, 1000, {.maxSpeed=70}, true);
    openGate();
    chassis.turnToHeading(270, 200, {}, false);    

    // Score long goal
    chassis.moveToPoint(-24, 47, 1800, {.forwards=false}, true);
    pros::delay(1100);
    score(800, 7, 55);
    resetLever();
    
    // Intake from matchloader
    chassis.moveToPoint(-65, 47, 1600, {.maxSpeed=70}, true);
    pros::delay(300);
    startIntake();

    // Score mid
    chassis.moveToPoint(-12, 10, 1300, {.forwards=false}, false);
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
    chassis.moveToPoint(-23, 23, 1100, {}, true);
    pros::delay(500);
    openGate();

    // Head to long goal
    chassis.turnToPoint(-40, 44, 400, {}, true);
    chassis.moveToPoint(-40, 44, 1000, {}, false);

    // Score long goal
    chassis.turnToHeading(270, 300, {}, false);
    chassis.moveToPoint(-24, 47, 1800, {.forwards=false}, true);
    pros::delay(800);
    score(1300, 7, 55);
    
    // Intake from matchloader
    chassis.moveToPoint(-65, 47, 1800, {.maxSpeed=60}, true);
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
    chassis.turnToPoint(-40, -44, 400, {}, true);
    chassis.moveToPoint(-40, -44, 1000, {}, false);

    // Score long goal
    chassis.turnToHeading(270, 300, {}, false);
    chassis.moveToPoint(-24, -47, 1800, {.forwards=false}, true);
    pros::delay(800);
    score(1300, 7, 55);
    
    // Intake from matchloader
    chassis.moveToPoint(-65, -47, 1600, {.maxSpeed=65}, true);
    startIntake();
    
    // Score mid
    chassis.moveToPoint(-50, -47, 600, {.forwards=false}, true);
    pros::delay(400);
    closeGate();
    chassis.turnToPoint(-15, -10, 400, {}, true);
    chassis.moveToPoint(-15, -10, 1200, {}, true);
    chassis.turnToPoint(0, 0, 300, {}, false);
    startOuttake(250);
    pros::delay(1400);
    stopIntake();

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
    MclMain.setObstacles(&quadrant_dividers, nullptr);
    startMcl(-44, 0, 270, true, true, false, true);

    // Clear Park Zone
    startIntake();
    moveForward(-5, 800, 127, 1, true);
    chassis.moveToPoint(-58, 0, 2000, {.maxSpeed=70}, false);
    pros::delay(1000);
    shake(3, 1000);
    leftMotors.move(127);
    rightMotors.move(127);
    pros::delay(800);
    shake(6, 2600);
    chassis.turnToHeading(90, 400, {}, false);
    moveForward(-20, 1400, 127, 1, false);

    // Score top mid
    chassis.moveToPoint(-19, 10, 1000, {.forwards=false}, false);
    chassis.swingToHeading(315, lemlib::DriveSide::LEFT, 800, {.maxSpeed=90}, false);
    moveForward(5, 800, 25, 1, false);
    chassis.moveToPoint(-8, 8, 1000, {.forwards=false}, false);
    chassis.turnToPoint(0, 0, 300, {.forwards=false}, false);
    retractLift();
    openHood();
    startIntake();
    pros::delay(600);
    score(2500, 7, 20);

    // Refill at top left
    startIntake();        
    chassis.moveToPoint(-47, 45, 1300, {}, true);
    pros::delay(400);
    extendLift();
    openGate();
    chassis.turnToHeading(270, 200, {}, true);
    chassis.moveToPoint(-24, 47, 1000, {.forwards=false}, true);
    pros::delay(1500);
    score(800, 7, 80);
    startIntake();
    chassis.moveToPoint(-70, 47, 1500, {.maxSpeed=60}, false);
    jiggle(4, 1800);
    stopIntake();

    // Score at top-right long goal
    chassis.turnToHeading(225, 200, {}, false);
    chassis.moveToPose(-25, 63, 270, 1000, {.forwards=false, .lead=0.5, .minSpeed=127}, false);
    closeGate();
    chassis.moveToPoint(33, 63, 1200, {.forwards=false}, false);
    chassis.moveToPoint(24, 47, 600, {.forwards=false}, false);
    chassis.moveToPoint(24, 47, 1000, {.forwards=false}, false);
    chassis.swingToHeading(90, lemlib::DriveSide::LEFT, 600, {.minSpeed=80}, false);
    score(1200, 7, 80);

    // Refill at top-right loader then score again
    openGate();
    startIntake();
    chassis.moveToPoint(70, 47, 1400, {.maxSpeed=70}, false);
    jiggle(4, 1800);
    chassis.moveToPoint(27, 47, 1100, {.forwards=false, .maxSpeed=110}, false);
    score(1200, 7, 40);
    closeGate();

    // Clear Park Zone
    chassis.turnToPoint(39, 0, 900, {}, false);
    chassis.moveToPoint(39, 0, 1500, {}, false);

    // Clear Park Zone
    startIntake();
    moveForward(-5, 800, 127, 1, true);
    chassis.moveToPoint(58, 0, 2000, {.maxSpeed=70}, false);
    pros::delay(1000);
    shake(3, 1000);
    leftMotors.move(127);
    rightMotors.move(127);
    pros::delay(800);
    shake(6, 2600);
    chassis.turnToHeading(270, 400, {}, false);
    moveForward(-20, 1400, 127, 1, false);

    // Score top mid
    chassis.turnToPoint(19, 19, 500, {}, false);
    chassis.moveToPoint(19, 19, 1000, {}, false);
    chassis.turnToPoint(10, 10, 600, {}, false);
    chassis.moveToPoint(10, 10, 1000, {}, false);
    chassis.turnToPoint(0, 0, 500, {}, true);
    frontMotor.move(-20);
    pros::delay(2000);

    // Pick up four balls
    chassis.moveToPoint(19, 19, 600, {.forwards=false}, false);
    chassis.turnToPoint(22, -23.5, 500, {}, true);
    startIntake();
    chassis.moveToPoint(22, -23.5, 500, {.minSpeed=60, .earlyExitRange=3}, false);
    chassis.moveToPoint(22, -23.5, 800, {.maxSpeed=60}, true);
    pros::delay(600);
    openGate();

    // Refill at top left
    startIntake();        
    chassis.moveToPoint(47, -46, 1300, {}, true);
    pros::delay(400);
    extendLift();
    openGate();
    chassis.turnToHeading(90, 200, {}, true);
    chassis.moveToPoint(24, -47, 1000, {.forwards=false}, true);
    pros::delay(500);
    score(800, 7, 80);
    startIntake();
    chassis.moveToPoint(70, -47, 1500, {.maxSpeed=60}, false);
    jiggle(4, 1800);
    stopIntake();


    // Score at bottom-left long goal
    chassis.turnToHeading(45, 200, {}, false);
    chassis.moveToPose(25, -63, 90, 1000, {.forwards=false, .lead=0.5, .minSpeed=127}, false);
    closeGate();
    chassis.moveToPoint(-33, -63, 1200, {.forwards=false}, false);
    chassis.moveToPose(-24, -48, 270, 1800, {.forwards=false, .lead = 0.2, .minSpeed=60}, false);
    chassis.swingToHeading(270, lemlib::DriveSide::LEFT, 600, {}, false);
    score(1200, 7, 100);

    // Refill at bottom-left loader then score again
    openGate();
    startIntake();
    chassis.moveToPoint(-70, -47, 1400, {.maxSpeed=70}, false);
    jiggle(4, 1800);
    chassis.moveToPoint(-28, -47, 1400, {.forwards=false, .maxSpeed=90}, false);
    score(1200, 7, 40);
    closeGate();

    // Park
    chassis.moveToPoint(-67, -24, 800, {}, false);
    chassis.turnToHeading(0, 600, {}, false);
    chassis.moveToPoint(-63, 4, 1800, {.minSpeed=80}, false);
}
