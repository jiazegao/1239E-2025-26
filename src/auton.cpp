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
    chassis.moveToPoint(-38, 36.5, 900, {}, false);
    chassis.turnToPoint(0, 37.5, 600, {}, false);
    retractLeftArm();
    chassis.moveToPoint(-16, 37.5, 2000, {.minSpeed=127}, false);
    chassis.turnToHeading(135, 2000, {.minSpeed=127}, false);
}
void rightPush(float x_offset = 0, float y_offset = 0) {
    // Push
    chassis.moveToPoint(-38, -36.5, 900, {}, false);
    chassis.turnToPoint(0, -37.5, 400, {.forwards=false}, false);
    retractLeftArm();
    chassis.moveToPoint(-18, -37.5, 2000, {.minSpeed=127}, false);
}
void leftMidPush() {
    // Descore
    closeGate();
    chassis.moveToPoint(-27, 30, 700, {}, false);
    extendLift();
    chassis.turnToHeading(90, 400, {}, false);
    retractLeftArm();
    chassis.moveToPoint(-18, 36, 1000, {.minSpeed=127, .earlyExitRange=3}, false);
    chassis.turnToHeading(135, 1000, {.minSpeed=127}, false);
}
void rightMidPush() {
    // Descore
    closeGate();
    chassis.moveToPoint(-27, -30, 700, {.forwards=false}, false);
    extendLift();
    chassis.turnToHeading(270, 400, {}, false);
    retractLeftArm();
    chassis.moveToPoint(-18, -36, 1000, {.forwards=false, .minSpeed=127, .earlyExitRange=3}, false);
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

    // Push teammate and get their preload
    chassis.moveToPoint(-47, 15, 300, {}, false);

    // Head towards the matchloader and intake
    chassis.moveToPoint(-47, -46, 1400, {.forwards=false}, true);
    pros::delay(400);
    openGate();
    chassis.turnToPoint(-65, -47, 300, {}, false);
    chassis.moveToPoint(-65, -47, 1100, {.maxSpeed=80}, false);

    // Score the long goal
    chassis.moveToPoint(-24, -46.5, 1300, {.forwards=false}, true);
    pros::delay(1000);
    closeGate();
    score(700, 7);
    resetLever();

    // Intake 3 balls
    startIntake();
    chassis.turnToHeading(20, 600, {.maxSpeed=90}, true);
    chassis.moveToPoint(-22, -23, 1000, {.maxSpeed=40}, true);

    // Intake 3 other balls
    chassis.moveToPoint(-23, 23, 600, {.minSpeed=70, .earlyExitRange=3}, true);
    chassis.moveToPoint(-24, 23, 400, {.maxSpeed=50}, true);
    pros::delay(200);
    openGate();

    // Go to long goal
    chassis.moveToPoint(-42, 42, 1100, {.maxSpeed=70}, true);    

    // Score long goal
    chassis.moveToPoint(-24, 44, 1000, {.forwards=false}, true);
    pros::delay(400);
    score(400, 7, 100);
    frontMotor.move(-80);
    pros::delay(200);
    
    // Intake from matchloader
    chassis.moveToPoint(-65, 47, 1600, {.maxSpeed=90}, true);
    pros::delay(50);
    resetLever();
    pros::delay(500);
    startIntake();

    // Score mid
    chassis.moveToPoint(-11, 10, 1200, {.forwards=false}, false);
    retractLift();
    chassis.turnToPoint(0, 0, 300, {.forwards=false}, true);
    score(800, 7, 50);
    resetLever();
    
    leftMidPush();
}
void leftv2() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.setPose(-48.75, 16.125, 90);
    MclMain.setObstacles(&quadrant_dividers, nullptr);
    startIntake();
    startMcl(-48.75, 16.125, 90, false, true, true, false);
    // Intake three balls
    stopIntake();
    startIntake();
    chassis.moveToPoint(-19, 24, 800, {.minSpeed = 60, .earlyExitRange = 2}, true);
    pros::delay(350);
    openGate();

    // Intake two balls
    closeGate();
    chassis.turnToPoint(-9, 44, 200, {}, true);
    openGate();
    chassis.moveToPoint(-9, 44, 900, {.maxSpeed=95}, false);
    
    // Head to long goal
    chassis.moveToPose(-36, 45, 140, 2600, {.forwards=false, .minSpeed=90}, false);
    
    // Score long goal
    chassis.moveToPoint(-24, 47, 1000, {.forwards=false}, true);
    pros::delay(400);
    score(400, 7, 100);
    frontMotor.move(-80);
    pros::delay(200);
    
    // Intake from matchloader
    chassis.moveToPoint(-65, 47, 1600, {.maxSpeed=90}, true);
    pros::delay(50);
    resetLever();
    pros::delay(500);
    startIntake();

    // Score mid
    chassis.moveToPoint(-11, 10, 1200, {.forwards=false}, false);
    retractLift();
    chassis.turnToPoint(0, 0, 300, {.forwards=false}, true);
    score(800, 7, 50);
    resetLever();

    // Push
    leftMidPush();
}
void rightv2() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.setPose(-48.75, -16.125, 90);
    MclMain.setObstacles(&quadrant_dividers, nullptr);
    startIntake();
    startMcl(-48.75, -16.125, 90, false, false, true, true);
    // Intake three balls
    stopIntake();
    startIntake();
    chassis.moveToPoint(-19, -24, 800, {.minSpeed = 60, .earlyExitRange = 2}, true);
    pros::delay(350);
    openGate();

    // Intake two balls
    closeGate();
    chassis.turnToPoint(-9, -44, 200, {}, true);
    openGate();
    chassis.moveToPoint(-9, -44, 900, {.maxSpeed=95}, false);
    
    // Head to long goal
    chassis.moveToPose(-36, -45, 40, 2600, {.forwards=false, .minSpeed=90}, false);
    
    // Score long goal
    chassis.moveToPoint(-24, -47, 1000, {.forwards=false}, true);
    pros::delay(400);
    score(400, 7, 100);
    frontMotor.move(-80);
    pros::delay(200);
    
    // Intake from matchloader
    chassis.moveToPoint(-65, -47, 1600, {.maxSpeed=90}, true);
    pros::delay(50);
    resetLever();
    pros::delay(500);
    startIntake();

    // Score mid
    moveForward(-10, 400);
    chassis.turnToPoint(-14, -14, 500, {}, false);
    chassis.moveToPoint(-14, -14, 1200, {}, false);
    chassis.turnToPoint(0, 0, 300, {}, true);
    startOuttake(200);
    pros::delay(1000);
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
    moveForward(-5, 800, 127, 1, false);
    chassis.moveToPoint(-58, 0, 2000, {.maxSpeed=70}, false);
    pros::delay(1000);
    moveForward(-5, 500, 50, 1, false);
    leftMotors.move(127);
    rightMotors.move(127);
    pros::delay(1000);
    shake(3, 3000);
    chassis.moveToPoint(-40, 0, 1500, {.forwards=false, .maxSpeed=70}, false);
    MclMain.set_pose(RclMain.updateBotPose(&front_rcl).second, RclMain.updateBotPose(&left_rcl).second, chassis.getPose().theta);

    // Score top mid
    chassis.moveToPoint(-13, 11, 1100, {.forwards=false}, false);
    chassis.swingToHeading(315, lemlib::DriveSide::LEFT, 800, {.maxSpeed=90}, false);
    chassis.moveToPoint(-17.5, 17.5, 800, {.maxSpeed=50}, false);
    chassis.moveToPoint(-9, 9, 600, {.forwards=false}, false);
    retractLift();
    openHood();
    pros::delay(600);
    score(2000, 7, 40);

    // Refill at top left
    startIntake();        
    chassis.moveToPoint(-47, 47, 1400, {}, true);
    pros::delay(300);
    extendLift();
    openGate();
    chassis.turnToPoint(-63, 47, 500, {}, false);
    startIntake();
    chassis.moveToPoint(-70, 47, 1000, {.maxSpeed=60}, false);
    jiggle(4, 2000);
    stopIntake();
    closeGate();

    // Score at top-right long goal
    chassis.turnToHeading(225, 200, {}, false);
    chassis.moveToPose(-25, 63, 270, 1000, {.forwards=false, .lead=0.5, .minSpeed=128}, false);
    chassis.moveToPoint(33, 63, 1200, {.forwards=false}, false);
    chassis.moveToPose(20, 47, 90, 1800, {.forwards=false, .lead = 0.2, .minSpeed=60}, false);
    chassis.swingToHeading(90, lemlib::DriveSide::LEFT, 600, {}, false);
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
    chassis.turnToPoint(30, 0, 900, {}, false);
    chassis.moveToPoint(30, 5, 1500, {}, false);
    startIntake();
    chassis.turnToHeading(90, 500, {}, false);
    startIntake();
    moveForward(40, 1800, 127, 60, false);
    shake(4, 1000);
    moveForward(10, 500);
    shake(2, 1000);
    moveForward(10, 500);
    moveForward(-20, 1200);

    // Score top mid
    chassis.turnToPoint(19, 19, 500, {}, false);
    chassis.moveToPoint(19, 19, 1000, {}, false);
    chassis.turnToPoint(13, 13, 600, {}, false);
    chassis.moveToPoint(13, 13, 600, {}, false);
    chassis.turnToPoint(0, 0, 500, {}, true);
    startOuttake(100);
    pros::delay(2000);

    // Pick up four balls
    chassis.moveToPoint(19, 19, 600, {.forwards=false}, false);
    chassis.turnToPoint(22, -23.5, 500, {}, false);
    startIntake();
    chassis.moveToPoint(22, -23.5, 500, {.minSpeed=60, .earlyExitRange=3}, false);
    chassis.moveToPoint(22, -23.5, 1000, {.maxSpeed=60}, false);

    // Refill at bottom right
    chassis.turnToPoint(47, -48, 400, {}, false);
    chassis.moveToPoint(47, -48, 1000, {}, true);
    pros::delay(700);
    openGate();
    chassis.turnToPoint(63, -47, 500, {}, false);
    chassis.moveToPoint(70, -47, 1000, {.maxSpeed=65}, false);
    jiggle(4, 2000);
    stopIntake();
    closeGate();

    // Score at bottom-left long goal
    chassis.turnToHeading(45, 200, {}, false);
    chassis.moveToPose(25, -63, 90, 1000, {.forwards=false, .lead=0.5, .minSpeed=128}, false);
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

