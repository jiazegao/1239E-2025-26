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
    chassis.moveToPoint(-36, 38, 500, {}, true);
    chassis.turnToHeading(90, 400, {}, true);
    retractLeftArm();
    chassis.moveToPoint(-11, 38, 500, {}, true);
    chassis.moveToPoint(-11, 38, 1000, {.maxSpeed=40}, true);
    chassis.turnToHeading(120, 12000, {.maxSpeed=30}, false);
}
void rightPush() {
    // Push
    chassis.moveToPoint(-35, -37.5, 500, {}, true);
    chassis.turnToHeading(270, 300, {}, true);
    retractLeftArm();
    chassis.moveToPoint(-10, -38, 400, {.forwards=false}, true);
    chassis.moveToPoint(-10, -38, 1000, {.forwards=false, .maxSpeed=40}, true);
    chassis.turnToHeading(240, 12000, {.maxSpeed=30}, false);
}
void leftMidPush() {
    // Descore
    closeGate();
    chassis.moveToPoint(-24, 34.25, 1500, {}, true);
    chassis.turnToHeading(90, 400, {}, true);
    retractLeftArm();
    chassis.moveToPoint(-11, 34, 500, {}, true);
    chassis.moveToPoint(-11, 34, 1000, {.maxSpeed=40}, true);
    chassis.turnToHeading(120, 12000, {.maxSpeed=30}, false);
}
void rightMidPush() {
    // Descore
    closeGate();
    chassis.moveToPoint(-24, -37, 400, {.forwards=false}, true);
    chassis.turnToHeading(270, 400, {}, true);
    retractLeftArm();
    chassis.moveToPoint(-10, -38, 600, {.forwards=false}, true);
    chassis.moveToPoint(-10, -38, 1000, {.forwards=false, .maxSpeed=40}, true);
    chassis.turnToHeading(240, 12000, {.maxSpeed=30}, false);
}

void leftStackStart() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.setPose(-48.75, 16.125, 90);
    MclMain.setObstacles(&left_dividers, nullptr);
    startMcl(-48.75, 16.125, 90, false, true, true, false);
    
    // Intake three balls
    startIntake();
    chassis.moveToPoint(-23, 24, 1000, {}, true);
    pros::delay(450);
    openGate();
}
void rightStackStart() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.setPose(-48.75, -16.125, 90);
    MclMain.setObstacles(&right_dividers, nullptr);
    startMcl(-48.75, -16.125, 90, false, false, true, true);
    
    // Intake three balls
    startIntake();
    chassis.moveToPoint(-23, -22, 1000, {}, true);
    pros::delay(450);
    openGate();
}
void leftFastStartToGoal() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.setPose(-48.75, 16.125, 90);
    MclMain.setObstacles(&left_dividers, nullptr);
    startMcl(-48.75, 16.125, 90, false, true, true, false);
    
    // Intake three balls
    startIntake();
    chassis.moveToPoint(-23, 22, 700, {}, true);
    pros::delay(450);
    openGate();

    // Head to matchloader
    chassis.turnToPoint(-44, 38, 300, {}, true);
    chassis.moveToPoint(-44, 38, 800, {}, true);
    chassis.moveToPoint(-24, 46, 700, {.forwards=false}, true);
    chassis.swingToHeading(270, lemlib::DriveSide::LEFT, 200, {.minSpeed=127}, true);
    chassis.moveToPoint(-20, 47, 400, {.forwards=false}, true);
}
void rightFastStartToGoal() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.setPose(-48.75, -16.125, 90);
    MclMain.setObstacles(&right_dividers, nullptr);
    startMcl(-48.75, -16.125, 90, false, false, true, true);
    
    // Intake three balls
    startIntake();
    chassis.moveToPoint(-23, -22, 700, {}, true);
    pros::delay(400);
    openGate();

    // Head to matchloader
    chassis.turnToPoint(-44, -38, 300, {}, true);
    chassis.moveToPoint(-44, -38, 800, {}, true);
    chassis.moveToPoint(-24, -46, 700, {.forwards=false}, true);
    chassis.swingToHeading(270, lemlib::DriveSide::RIGHT, 200, {.minSpeed=127}, true);
    chassis.moveToPoint(-20, -47, 400, {.forwards=false}, true);
}

int midScoreDelay = 0;
void setMidScoreDelay(int newDelay) {midScoreDelay = newDelay;}
// NOT USING
void soloAWP(){
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.setPose(-47, 0, 0);
    MclMain.setObstacles(&soloAWP_obstacles_right, nullptr);
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
// TUNED
void SAWPRight_slowScr() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.setPose(-47, 0, 0);
    MclMain.setObstacles(&soloAWP_obstacles_right, nullptr);
    startIntake();
    startMcl(-47, 0, 0, false, true, true, false);

    // Push teammate and get their preload
    chassis.moveToPoint(-47, 15, 300, {}, true);

    // Head towards the matchloader and intake
    chassis.moveToPoint(-47, -44, 1400, {.forwards=false}, true);
    pros::delay(450);
    openGate();
    chassis.turnToHeading(270, 400, {}, true);
    chassis.moveToPoint(-70, -47, 1200, {.maxSpeed=50}, true);

    // Score the long goal
    chassis.moveToPoint(-24, -47, 1800, {.forwards=false}, true);
    pros::delay(1400);
    closeGate();
    hoodLock = true;
    openHood();
    score(900, 7, 45);
    frontMotor.move(-127);

    // Intake 3 balls
    chassis.turnToHeading(20, 800, {.maxSpeed=80}, true);
    pros::delay(300);
    resetLever();
    chassis.moveToPoint(-22, -23, 700, {.maxSpeed=50}, true);
    startIntake();
    pros::delay(300);
    openGate();
    hoodLock = false;
    closeHood();

    // Intake 3 other balls
    chassis.moveToPoint(-22, 23, 600, {.minSpeed=70, .earlyExitRange=3}, true);
    closeGate();
    chassis.moveToPoint(-23, 23, 500, {.maxSpeed=50}, true);
    pros::delay(200);
    openGate();
    lever_antistuck_on = true;
    // Go to long goal
    chassis.moveToPoint(-43, 41, 1000, {.maxSpeed=80}, true);
    chassis.turnToHeading(270, 150, {}, false);

    // Score long goal
    chassis.moveToPoint(-24, 47, 1300, {.forwards=false}, true);
    pros::delay(600);
    score(900, 7, 45);
    lever_antistuck_on = false;

    // Intake from matchloader
    chassis.moveToPoint(-70, 47, 400, {.minSpeed=80}, false);
    resetLever();
    chassis.moveToPoint(-70, 47, 900, {.maxSpeed=50}, true);
    pros::delay(300);
    startIntake();

    // Score mid
    moveForward(-5, 400, 30, 1, true);
    chassis.moveToPoint(-11, 10, 1100, {.forwards=false}, true);
    pros::delay(750);
    retractLift();
    score(500, 3, 25);
    chassis.turnToPoint(0, 0, 200, {.forwards=false}, true);
    pros::delay(1000);
    resetLever();
}
void SAWPRight_fastScr() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.setPose(-47, 0, 0);
    MclMain.setObstacles(&soloAWP_obstacles_right, nullptr);
    startIntake();
    startMcl(-47, 0, 0, false, true, true, false);

    // Push teammate and get their preload
    chassis.moveToPoint(-47, 15, 300, {}, true);

    // Head towards the matchloader and intake
    chassis.moveToPoint(-47, -44, 1400, {.forwards=false}, true);
    pros::delay(400);
    openGate();
    chassis.turnToHeading(270, 400, {}, true);
    chassis.moveToPoint(-70, -47, 1000, {.maxSpeed=50}, true);

    // Score the long goal
    chassis.moveToPoint(-24, -47, 1800, {.forwards=false}, true);
    pros::delay(900);
    closeGate();
    hoodLock = true;
    openHood();
    score(900, 7, 45);
    frontMotor.move(-127);

    // Intake 3 balls
    chassis.turnToHeading(20, 800, {.maxSpeed=80}, true);
    pros::delay(300);
    resetLever();
    chassis.moveToPoint(-22, -23, 700, {.maxSpeed=50}, true);
    startIntake();
    pros::delay(300);
    openGate();
    hoodLock = false;
    closeHood();

    // Intake 3 other balls
    chassis.moveToPoint(-22, 23, 600, {.minSpeed=70, .earlyExitRange=3}, true);
    closeGate();
    chassis.moveToPoint(-23, 23, 500, {.maxSpeed=50}, true);
    pros::delay(200);
    openGate();
    
    // Go to long goal
    chassis.moveToPoint(-43, 41, 1000, {.maxSpeed=80}, true);
    chassis.turnToHeading(270, 150, {}, false);

    // Score long goal
    chassis.moveToPoint(-24, 47, 1300, {.forwards=false}, true);
    pros::delay(600);
    score(800, 7, 80);
    
    // Intake from matchloader
    chassis.moveToPoint(-70, 47, 400, {.minSpeed=80}, false);
    resetLever();
    chassis.moveToPoint(-70, 47, 1100, {.maxSpeed=50}, true);
    pros::delay(300); //Change later
    startIntake();

    // Score mid
    moveForward(-5, 400, 30, 1, true);
    chassis.moveToPoint(-11, 10, 1100, {.forwards=false}, true);
    pros::delay(700);
    retractLift();
    score(400, 4, 35);
    chassis.turnToPoint(0, 0, 200, {.forwards=false}, true);
    pros::delay(1000);
    resetLever();
}
void SAWPLeft() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.setPose(-47, 0, 180);
    MclMain.setObstacles(&soloAWP_obstacles_left, nullptr);
    startIntake();
    startMcl(-47, 0, 180, false, false, true, true);

    // Push teammate and get their preload
    chassis.moveToPoint(-47, -15, 300, {}, true);

    // Head towards the matchloader and intake
    chassis.moveToPoint(-47, 44, 1400, {.forwards=false}, true);
    pros::delay(400);
    openGate();
    chassis.turnToHeading(270, 400, {}, true);
    chassis.moveToPoint(-70, 47, 1200, {.maxSpeed=50}, true);

    // Score the long goal
    chassis.moveToPoint(-24, 47, 1800, {.forwards=false}, true);
    pros::delay(900);
    closeGate();
    hoodLock = true;
    openHood();
    score(800, 7, 45);
    frontMotor.move(-127);

    // Intake 3 balls
    chassis.turnToHeading(160, 800, {.maxSpeed=60}, true);
    pros::delay(300);
    resetLever();
    chassis.moveToPoint(-22, 23, 700, {.maxSpeed=50}, true);
    startIntake();
    pros::delay(300);
    openGate();
    hoodLock = false;
    closeHood();

    // Intake 3 other balls
    chassis.moveToPoint(-22, -23, 600, {.minSpeed=70, .earlyExitRange=3}, true);
    closeGate();
    chassis.moveToPoint(-23, -23, 500, {.maxSpeed=50}, true);
    pros::delay(200);
    openGate();
    
    // Go to long goal
    chassis.moveToPoint(-43, -41, 1000, {.maxSpeed=80}, true);
    chassis.turnToHeading(270, 150, {}, false);

    // Score long goal
    chassis.moveToPoint(-24, -47, 1300, {.forwards=false}, true);
    pros::delay(600);
    score(800, 7, 45);
    
    // Intake from matchloader
    chassis.moveToPoint(-70, -47, 400, {.minSpeed=80}, false);
    resetLever();
    chassis.moveToPoint(-70, -47, 1100, {.maxSpeed=50}, true);
    pros::delay(300);
    startIntake();

    // Score mid
    moveForward(-4, 600, 30, 1, true);
    pros::delay(400);
    closeGate();
    chassis.turnToPoint(-10, -10, 500, {}, true);
    chassis.moveToPoint(-10, -10, 1400, {.maxSpeed=80}, true);
    pros::delay(1000);
    startOuttake(200);
    chassis.turnToPoint(0, 0, 200, {}, true);
    moveForward(8, 800, 60, 1, true);
}
void specSAWP() {
    leftFastStartToGoal();
    score(800, 7, 45);
    
    // Intake from matchloader
    chassis.moveToPoint(-70, 47, 400, {.minSpeed=80}, false);
    resetLever();
    chassis.moveToPoint(-70, 47, 1000, {.maxSpeed=45}, true);
    pros::delay(300);
    startIntake();

    // Score mid
    moveForward(-5, 400, 30, 1, true);
    chassis.moveToPoint(-11, 10, 1100, {.forwards=false}, true);
    pros::delay(700);
    retractLift();
    score(400, 7, 35);
    chassis.turnToPoint(0, 0, 200, {.forwards=false}, true);
    pros::delay(500);
    resetLever();
    closeGate();

    // Intake 3 balls
    chassis.turnToHeading(210, 500, {}, true);
    chassis.moveToPoint(-23, -23, 400, {.minSpeed=70, .earlyExitRange=3}, true);
    closeGate();
    chassis.moveToPoint(-23, -23, 600, {.maxSpeed=50}, true);
    pros::delay(200);
    openGate();
    
    // Go to long goal
    chassis.moveToPoint(-43, -41, 1000, {.maxSpeed=80}, true);
    chassis.turnToHeading(270, 150, {}, false);

    // Score long goal
    chassis.moveToPoint(-24, -47, 1300, {.forwards=false}, true);
    pros::delay(600);
    score(800, 7, 45);
    
    // Intake from matchloader
    chassis.moveToPoint(-70, -47, 400, {.minSpeed=80}, false);
    resetLever();
    chassis.moveToPoint(-70, -47, 1100, {.maxSpeed=50}, true);
    pros::delay(300);
    startIntake();

    // Score mid
    moveForward(-4, 600, 30, 1, true);
    pros::delay(400);
    closeGate();
    chassis.turnToPoint(-10, -10, 500, {}, true);
    chassis.moveToPoint(-10, -10, 1400, {.maxSpeed=80}, true);
    pros::delay(1000);
    startOuttake(200);
    chassis.turnToPoint(0, 0, 200, {}, true);
    moveForward(8, 800, 60, 1, true);
}
// TUNED
void leftv2() {    
    leftFastStartToGoal();
    score(800, 7, 65);
    
    // Intake from matchloader
    chassis.moveToPoint(-70, 47, 450, {.minSpeed=80}, false);
    hoodLock = false;
    resetLever();
    chassis.moveToPoint(-70, 47, 800, {.maxSpeed=45}, true);
    pros::delay(300);
    startIntake();

    // Score mid
    moveForward(-5, 400, 30, 1, true);
    chassis.moveToPoint(-11, 10, 1100, {.forwards=false}, true);
    pros::delay(700);
    retractLift();
    score(500, 4, 27);
    chassis.turnToPoint(0, 0, 200, {.forwards=false}, true);
    pros::delay(500);
    resetLever();
    
    leftMidPush();
}
// TEST AT PRAC FIELD ////////////////////////////
void rightv2() {
    rightFastStartToGoal();
    score(800, 7, 45);
    
    // Intake from matchloader
    chassis.moveToPoint(-70, -47, 400, {.minSpeed=80}, false);
    resetLever();
    chassis.moveToPoint(-70, -47, 1000, {.maxSpeed=45}, true);
    pros::delay(300);
    startIntake();
    
    // Score mid
    moveForward(-4, 700, 30, 1, true);
    pros::delay(400);
    closeGate();
    chassis.turnToPoint(-10, -10, 600, {}, true);
    chassis.moveToPoint(-10, -10, 1400, {.maxSpeed=80}, true);
    pros::delay(700);
    startOuttake(250);
    chassis.turnToPoint(0, 0, 300, {}, true);
    moveForward(9.5, 800, 60, 1, true);
    pros::delay(1800);

    // Push
    rightMidPush();
}
// TUNED
void left7() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.setPose(-48.75, 16.125, 90);
    MclMain.setObstacles(&left_dividers, nullptr);
    startMcl(-48.75, 16.125, 90, false, true, true, false);
    
    // Intake three balls
    startIntake();
    chassis.moveToPoint(-23, 22, 700, {}, true);
    pros::delay(400);
    openGate();

    // Head to matchloader
    chassis.turnToPoint(-52, 41, 300, {}, true);
    chassis.moveToPoint(-52, 41, 900, {}, true);
    chassis.turnToHeading(270, 150, {}, true);
    chassis.moveToPoint(-70, 47, 1200, {.maxSpeed=60}, true);

    // Score the long goal
    chassis.moveToPoint(-22, 47, 1900, {.forwards=false}, true);
    pros::delay(550);
    hoodLock = true;
    trapDoor.extend();
    startIntake();
    pros::delay(1000);
    closeGate();
    score(800, 7, 45);
    hoodLock = false;

    // Push
    closeGate();
    leftPush();
}
// TUNED
void right7() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.setPose(-48.75, -16.125, 90);
    MclMain.setObstacles(&right_dividers, nullptr);
    startMcl(-48.75, -16.125, 90, false, false, true, true);
    
    // Intake three balls
    startIntake();
    chassis.moveToPoint(-23, -22, 700, {}, true);
    pros::delay(400);
    openGate();

    // Head to matchloader
    chassis.turnToPoint(-52, -41, 300, {}, true);
    chassis.moveToPoint(-52, -41, 900, {}, true);
    chassis.turnToHeading(270, 150, {}, true);
    chassis.moveToPoint(-70, -47, 1200, {.maxSpeed=60}, true);

    // Score the long goal
    chassis.moveToPoint(-22, -47, 1900, {.forwards=false}, true);
    pros::delay(550);
    hoodLock = true;
    trapDoor.extend();
    startIntake();
    pros::delay(1000);
    closeGate();
    score(900, 7, 45);
    hoodLock = false;

    // Push
    closeGate();
    rightPush();
}
void left6() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.setPose(-48.75, 16.125, 90);
    MclMain.setObstacles(&left_dividers, nullptr);
    startMcl(-48.75, 16.125, 90, false, true, true, false);
    
    // Intake three balls
    startIntake();
    chassis.moveToPoint(-23, 22, 700, {}, true);
    pros::delay(400);
    openGate();

    // Head to matchloader
    chassis.turnToPoint(-52, 43, 300, {}, true);
    chassis.moveToPoint(-52, 43, 900, {}, true);
    chassis.turnToHeading(270, 150, {}, true);
    chassis.moveToPoint(-70, 47, 1200, {.maxSpeed=60}, true);

    // Score the long goal
    chassis.moveToPoint(-22, 47, 1400, {.forwards=false}, true);
    pros::delay(700);
    score(700, 7, 45);
    // Push
    closeGate();
    leftPush();
}
void right6() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.setPose(-48.75, -16.125, 90);
    MclMain.setObstacles(&right_dividers, nullptr);
    startMcl(-48.75, -16.125, 90, false, false, true, true);
    
    // Intake three balls
    startIntake();
    chassis.moveToPoint(-23, -22, 700, {}, true);
    pros::delay(400);
    openGate();

    // Head to matchloader
    chassis.turnToPoint(-52, -41, 300, {}, true);
    chassis.moveToPoint(-52, -41, 900, {}, true);
    chassis.turnToHeading(270, 150, {}, true);
    chassis.moveToPoint(-70, -47, 1200, {.maxSpeed=60}, true);

    // Score the long goal
    chassis.moveToPoint(-22, -47, 1400, {.forwards=false}, true);
    pros::delay(700);
    score(700, 7, 45);
    hoodLock = false;

    // Push
    closeGate();
    rightPush();
}
// TUNED
void left4() {
    leftFastStartToGoal();
    score(800, 7, 45);
    leftPush();
}
// TUNED
void right4() {
    rightFastStartToGoal();
    score(800, 7, 45);
    rightPush();
}

void leftDescoreAuto() {
    Timer t(9000);
    lever_antistuck_on = false;

    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.setPose(-47, 0, 180);
    MclMain.setObstacles(&left_dividers, nullptr);
    startIntake();
    startMcl(-47, 0, 180, false, false, true, true);

    // Head towards the matchloader and intake
    chassis.moveToPoint(-47, 44.5, 1100, {.forwards=false}, true);
    pros::delay(100);
    openGate();
    chassis.turnToHeading(270, 400, {}, true);
    chassis.moveToPoint(-70, 47, 1200, {.maxSpeed=50}, true);

    // Pick up two balls
    chassis.moveToPoint(-32, 58, 900, {.forwards=false}, true);
    pros::delay(400);
    closeGate();
    chassis.turnToHeading(90, 500, {}, true);
    startPriming();
    chassis.moveToPoint(-12, 58, 900, {}, true);
    pros::delay(500);
    MclMain.setDistSyncProp(0.00f);
    antiStuckOn = false;
    chassis.turnToPoint(-6, 47, 400, {}, true);
    chassis.moveToPoint(-6, 47, 700, {.maxSpeed=30}, true);
    chassis.swingToHeading(180, lemlib::DriveSide::RIGHT, 700, {.minSpeed=127}, true);
    MclMain.setDistSyncProp(0.10f);
    openGate();
    pros::delay(600);
    closeGate();
    chassis.turnToHeading(270, 500, {.minSpeed=70}, true);
    chassis.moveToPoint(-20, 57, 700, {}, true);
    chassis.moveToPoint(-6, 57, 800, {.forwards=false}, true);
    chassis.swingToHeading(270, lemlib::DriveSide::RIGHT, 300, {}, false);
    antiStuckOn = true;

    // Descore
    while (!t.timeIsUp()) {pros::delay(20);}
    retractLeftArm();
    chassis.moveToPoint(-41, 58, 900, {}, true);
    chassis.turnToPoint(-20, 47, 200, {.forwards=false}, true);
    chassis.moveToPoint(-20, 47, 700, {.forwards=false, .maxSpeed=80}, true);
    chassis.swingToHeading(270, lemlib::DriveSide::RIGHT, 200, {.minSpeed=127}, true);
    chassis.moveToPoint(-18, 47, 700, {.forwards=false}, true);
    score(450, 7, 90);
    startIntake();
    // Score top mid
    chassis.turnToHeading(160, 400, {.maxSpeed=80}, true);
    pros::delay(150);
    resetLever();
    startIntake();
    chassis.moveToPoint(-25, 25, 1000, {.maxSpeed=80}, true);
    pros::delay(400);
    openGate();
    pros::delay(800);
    chassis.turnToHeading(315, 700, {}, true);
    chassis.moveToPoint(-11, 11, 1000, {.forwards=false}, true);
    pros::delay(600);
    retractLift();
    score(400, 4, 55);
    chassis.turnToPoint(0, 0, 300, {.forwards=false}, true);
}
void rightDescoreAuto() {
    Timer t(9000);
    rightStackStart();

    // Get two more balls
    antiStuckOn = false;
    lever_antistuck_on = false;
    chassis.turnToPoint(-9.5, -40, 400, {}, true);
    chassis.moveToPoint(-9.5, -40, 1300, {.maxSpeed=40}, true);
    closeGate();
    MclMain.setDistSyncProp(0.00f);
    pros::delay(800);
    startPriming();
    chassis.swingToHeading(180, lemlib::DriveSide::RIGHT, 600, {.minSpeed=127}, true);
    openGate();
    MclMain.setDistSyncProp(0.10f);

    // Get matchload
    chassis.moveToPoint(-32, -32, 900, {.forwards=false}, true);
    antiStuckOn = true;
    chassis.turnToPoint(-46, -44, 500, {}, true);
    chassis.moveToPoint(-46, -44, 900, {}, true);
    chassis.turnToHeading(270, 400, {}, true);
    chassis.moveToPoint(-70, -47, 1100, {.maxSpeed=50}, true);

    // Descore
    chassis.moveToPoint(-32, -36, 1000, {.forwards=false}, true);
    chassis.turnToHeading(270, 500, {}, true);
    chassis.moveToPoint(-9, -36, 1100, {.forwards=false}, false);
    while (!t.timeIsUp()) {pros::delay(20);}
    retractLeftArm();
    chassis.moveToPoint(-41, -36, 900, {}, true);
    chassis.turnToPoint(-20, -47, 200, {.forwards=false}, true);
    chassis.moveToPoint(-20, -47, 700, {.forwards=false, .maxSpeed=80}, true);
    chassis.swingToHeading(270, lemlib::DriveSide::RIGHT, 200, {.minSpeed=127}, true);
    chassis.moveToPoint(-18, -47, 400, {.forwards=false}, true);
    score(450, 7, 85);
    startIntake();

    // Score bottom mid
    chassis.turnToHeading(45, 800, {}, true);
        closeGate();
    chassis.moveToPoint(-15, -15, 400, {.maxSpeed=80}, true);

   // resetLever();
    chassis.moveToPoint(-9, -11, 900, {.maxSpeed=80}, true);
    pros::delay(600);
    startOuttake(300);
    chassis.turnToPoint(0, 0, 200, {}, true);
    moveForward(8, 800, 60, 1, false);
    pros::delay(2000);
}

// 119 points
void skills_119() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.setPose(-54, 0, 270);
    startMcl(-54, 0, 270, true, true, false, true);

    // Clear Park Zone
    startIntake();
    antiStuckOn = false;
    moveForward(5, 1000, 30, 1, false);
    pros::delay(800);
    leftMotors.move(127);
    rightMotors.move(127);
    pros::delay(800);
    shake(3, 1800);
    chassis.turnToHeading(270, 400, {}, false);
    moveForward(-20, 1300, 127, 60, false);
    antiStuckOn = true;

    // Score top mid
    chassis.moveToPoint(-21, 10, 1300, {.forwards=false}, false);
    chassis.swingToHeading(315, lemlib::DriveSide::LEFT, 900, {.maxSpeed=70}, false);
    chassis.turnToHeading(315, 400, {}, false);
    moveForward(6.5, 1000, 127, 1, false);
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
    jiggle(2, 1200);

    // Score at top-right long goal
    chassis.turnToHeading(225, 200, {}, false);
    chassis.moveToPose(-25, 63, 270, 1000, {.forwards=false, .lead=0.5, .minSpeed=127}, false);
    closeGate();
    chassis.moveToPoint(0, 63, 700, {.forwards=false}, true);
    chassis.moveToPoint(24, 45, 800, {.forwards=false}, true);
    chassis.swingToHeading(90, lemlib::DriveSide::LEFT, 800, {.minSpeed=100}, true);
    moveForward(-5, 1000, 127, 1, true);
    scoreReserve(1000, 0, 40);

    // Refill at top-right loader then score again
    openGate();
    startIntake();
    chassis.moveToPoint(63, 47, 2400, {.maxSpeed=60}, false);
    jiggle(2, 1200);
    chassis.moveToPoint(24, 47, 1500, {.forwards=false, .maxSpeed=110}, true);
    pros::delay(600);
    scoreReserve(1400, 0, 20);
    closeGate();

    moveForward(5, 300);

    // Clear Park Zone
    chassis.turnToPoint(38.5, 10, 800, {}, false);
    chassis.moveToPoint(38.5, 10, 1300, {}, false);
    pros::delay(200);

    MclMain.set_pose(RclMain.updateBotPose(&left_rcl).second, RclMain.updateBotPose(&back_rcl).second, chassis.getPose().theta);

    chassis.moveToPoint(38.5, 0, 900, {}, false);
    chassis.turnToHeading(90, 700, {}, false);

    // Clear Park Zone
    startIntake();
    antiStuckOn = false;
    chassis.moveToPoint(56, 0, 2000, {.maxSpeed=70}, false);
    pros::delay(800);
    leftMotors.move(127);
    rightMotors.move(127);
    pros::delay(800);
    shake(3, 1800);
    chassis.turnToHeading(90, 400, {}, false);
    moveForward(-20, 1300, 127, 60, false);
    antiStuckOn = true;

    // Score bottom  mid
    chassis.turnToPoint(19, 16, 500, {}, false);
    chassis.moveToPoint(19, 16, 1400, {.maxSpeed=60}, true);
    chassis.turnToPoint(11, 11, 500, {}, true);
    chassis.moveToPoint(11, 11, 1000, {.maxSpeed = 50}, false);
    chassis.turnToPoint(0, 0, 500, {}, true);
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
    openGate();
    scoreReserve(1000, 0, 50);
    startIntake();
    chassis.moveToPoint(64, -47, 1800, {.maxSpeed=60}, false);
    jiggle(2, 2000);
    
    // Score at bottom-left long goal
    chassis.turnToHeading(45, 200, {}, false);
    chassis.moveToPose(25, -63, 90, 1000, {.forwards=false, .lead=0.4, .minSpeed=127}, false);
    closeGate();
    chassis.moveToPoint(0, -63, 700, {.forwards=false}, true);
    chassis.moveToPoint(-24, -45, 800, {.forwards=false}, true);
    chassis.swingToHeading(270, lemlib::DriveSide::LEFT, 800, {.minSpeed=100}, true);
    moveForward(-5, 1000, 127, 1, true);
    scoreReserve(1000, 0, 40);

    // Refill at bottom-left loader then score again
    openGate();
    startIntake();
    chassis.moveToPoint(-63, -47, 2000, { .maxSpeed = 60, .minSpeed=15}, true);
    jiggle(2, 1200);
    chassis.moveToPoint(-24, -47, 1300, {.forwards=false, .maxSpeed=110}, true);
    pros::delay(600);
    scoreReserve(1400, 0, 20);
    closeGate();
    startIntake();

    // Park
    chassis.moveToPoint(-67, -24, 800, {}, false);
    chassis.turnToHeading(0, 400, {}, false);
    chassis.moveToPoint(-63, 0, 1800, {.minSpeed=80}, false);
}
