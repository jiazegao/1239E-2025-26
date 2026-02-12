#include "custom/auton.hpp"
#include "custom/RclTracking.hpp"
#include "custom/configs.hpp"
#include "custom/util_funcs.hpp"
#include "custom/lever_control.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include <cmath>

void leftPush(double x_offset = 0, double y_offset = 0) {
    chassis.moveToPoint(-40+x_offset, 37.5+y_offset, 900, {}, false);
    chassis.turnToHeading(90, 600, {}, false);
    retractLeftArm();
    chassis.moveToPoint(-21+x_offset, 37.5+y_offset, 2000, {.minSpeed=127, .earlyExitRange=1}, false);
    chassis.turnToHeading(135, 2000, {.minSpeed=127}, false);
}
void rightPush(double x_offset = 0, double y_offset = 0) {
    chassis.moveToPoint(-40+x_offset, -37.5-y_offset, 900, {}, false);
    chassis.turnToHeading(270, 450, {}, false);
    retractLeftArm();
    chassis.moveToPoint(-24+x_offset, -37.5-y_offset, 2000, {.forwards=false, .minSpeed=127}, false);
}

// Push + 5 Long + 4 Mid + 5 Long
void soloAWP(){
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.setPose(-47, 0, 0);
    RclMain.setRclPose(chassis.getPose());

    RclMain.updateBotPose(&left_rcl);
    RclMain.updateBotPose(&back_rcl);

    // Push teammate and get their preload
    startIntake();
    chassis.moveToPoint(-47, 5, 350, {}, false);

    // Head towards the matchloader and intake
    chassis.moveToPoint(-47, -47, 1200, {.forwards=false}, true);
    pros::delay(400);
    openGate();
    chassis.turnToPoint(-65, -47, 400, {}, false);
    startIntake();
    chassis.moveToPoint(-65, -47, 1100, {.maxSpeed=60}, false);
    RclMain.updateBotPose(&left_rcl);
    RclMain.updateBotPose(&fl_rcl);
    RclMain.updateBotPose(&fr_rcl);

    // Score the long goal
    chassis.moveToPoint(-24, -47.5, 1800, {.forwards=false, .maxSpeed=110}, true);
    pros::delay(1200);
    scoreColor(1000);
    RclMain.updateBotPose(&left_rcl);

    // Intake 3 balls
    chassis.turnToHeading(10, 800, {.maxSpeed=100}, false);
    startIntake();
    moveForward(17, 700, 80, 1, true);
    pros::delay(300);
    openGate();
    pros::delay(300);
    closeGate();

    // Intake 3 other balls
    chassis.turnToHeading(0, 200, {}, false);
    RclMain.updateBotPose(&left_rcl);
    chassis.moveToPoint(-22, 25, 1200, {.maxSpeed=110}, true);
    pros::delay(700);
    openGate();

    // Score the mid goal
    chassis.turnToHeading(320, 400, {}, true);
    chassis.moveToPoint(chassis.getPose().x+17, chassis.getPose().y-17, 1200, {.forwards=false, .maxSpeed=110}, true);
    retractLift();
    pros::delay(500);
    score(1000, 4, FAST_MID_SCORE);
    startIntake();

    closeGate();
    chassis.moveToPoint(-47, -48, 1200, {}, true);
    pros::delay(400);
    openGate();    
    chassis.turnToPoint(-65, 48, 550, {}, false);

    RclMain.updateBotPose(&right_rcl);
    startIntake();
    chassis.moveToPoint(-65, 48, 900, {.maxSpeed=80}, false);
    RclMain.updateBotPose(&right_rcl);
    RclMain.updateBotPose(&fl_rcl);
    RclMain.updateBotPose(&fr_rcl);

    // Score again
    chassis.moveToPoint(-24, 47.5, 1800, {.forwards = false, .maxSpeed=110}, true);
    pros::delay(1200);
    scoreColor(1000);
}
// Three balls -> Match loader -> Long goal -> Push
// ---
void leftControlRush() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.setPose(-48.75, 16.125, 90);
    RclMain.setRclPose(chassis.getPose());

    RclMain.updateBotPose();
    RclMain.updateBotPose(&left_rcl);
    RclMain.updateBotPose(&back_rcl);

    // Intake three balls
    startIntake();
    chassis.moveToPoint(-24, 24, 750, {}, true);
    pros::delay(450);
    openGate();

    // Head to matchloader
    chassis.turnToPoint(-48, 48, 400, {}, false);
    chassis.moveToPoint(-48, 48, 900, {}, false);
    chassis.turnToPoint(-70, 47, 350, {}, false);
    chassis.moveToPoint(-70, 47, 900, {.maxSpeed=60}, false);
    RclMain.updateBotPose(&right_rcl);
    RclMain.updateBotPose(&fl_rcl);
    RclMain.updateBotPose(&fr_rcl);

    // Move towards long goal sand score
    chassis.turnToPoint(-24, 47.5, 200, {.forwards=false}, false);
    chassis.moveToPoint(-24, 47.5, 1800, {.forwards=false, .maxSpeed=110}, true);
    pros::delay(1200);
    scoreColor(1000);
    RclMain.updateBotPose(&right_rcl);
    RclMain.updateBotPose(&fl_rcl);
    RclMain.updateBotPose(&fr_rcl);

    // Push
    closeGate();
    leftPush(0, 0);
}
// Match loader -> Long goal -> Push
// WORKS
void leftFastRush() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.setPose(-47, 13.25, 180);
    RclMain.setRclPose(chassis.getPose());

    RclMain.updateBotPose();
    RclMain.updateBotPose(&right_rcl);
    RclMain.updateBotPose(&back_rcl);

    // Head towards the matchloader and intake
    openGate();
    chassis.moveToPoint(-47, 47, 1100, {.forwards=false}, false);
    chassis.turnToHeading(270, 300, {});
    startIntake();
    chassis.moveToPoint(-70, 47, 1200, {.maxSpeed=70}, false);
    RclMain.updateBotPose(&right_rcl);
    RclMain.updateBotPose(&fl_rcl);
    RclMain.updateBotPose(&fr_rcl);

    // Score the long goal
    chassis.moveToPoint(-24, 48, 1800, {.forwards=false, .maxSpeed=110}, true);
    pros::delay(1200);
    scoreColor(1000);
    RclMain.updateBotPose(&right_rcl);
    RclMain.updateBotPose(&fl_rcl);
    RclMain.updateBotPose(&fr_rcl);

    // Push
    closeGate();
    leftPush(0, 0);
}
// Three balls -> Match loader -> Long goal -> Push
// WORKS
void rightControlRush() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.setPose(-48.75, -16.125, 90);
    RclMain.setRclPose(chassis.getPose());

    RclMain.updateBotPose();
    RclMain.updateBotPose(&right_rcl);
    RclMain.updateBotPose(&back_rcl);

    // Intake three balls
    startIntake();
    chassis.moveToPoint(-24, -24, 900, {}, true);
    pros::delay(450);
    openGate();

    // Head to matchloader
    chassis.turnToPoint(-50, -47, 500, {}, false);
    chassis.moveToPoint(-50, -47, 1100, {}, false);
    chassis.turnToPoint(-70, -47, 300, {}, false);
    chassis.moveToPoint(-70, -47, 900, {.maxSpeed=70}, false);
    RclMain.updateBotPose(&left_rcl);
    RclMain.updateBotPose(&fl_rcl);
    RclMain.updateBotPose(&fr_rcl);

    // Move towards long goal and score
    chassis.turnToPoint(-24, -47.5, 200, {.forwards=false}, false);
    chassis.moveToPoint(-24, -47.5, 1800, {.forwards=false, .maxSpeed=110}, true);
    pros::delay(1200);
    scoreColor(1000);

    // Push
    closeGate();
    rightPush(0, 0);
}
// Match loader -> Long goal -> Push
// ---
void rightFastRush() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.setPose(-47, -13.25, 0);
    RclMain.setRclPose(chassis.getPose());

    RclMain.updateBotPose();
    RclMain.updateBotPose(&left_rcl);
    RclMain.updateBotPose(&back_rcl);

    // Head towards the matchloader and intake
    openGate();
    chassis.moveToPoint(-47, -44.5, 900, {.forwards=false}, false);
    chassis.turnToHeading(270, 300, {});
    startIntake();
    chassis.moveToPoint(-70, -47, 1100, {.maxSpeed=80}, false);
    RclMain.updateBotPose(&left_rcl);
    RclMain.updateBotPose(&fl_rcl);
    RclMain.updateBotPose(&fr_rcl);

    // Score the long goal
    chassis.moveToPoint(-24, -47, 1800, {.forwards=false, .maxSpeed=110}, true);
    pros::delay(1200);
    scoreColor(1000);
    RclMain.updateBotPose(&left_rcl);
    RclMain.updateBotPose(&fl_rcl);
    RclMain.updateBotPose(&fr_rcl);

    // Push
    closeGate();
    rightPush(0, 0);
}
// Three balls -> Two balls -> Long goal -> Match loader -> Top mid -> Push
// WORKS
void leftv2() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.setPose(-48.75, 16.125, 90);
    RclMain.setRclPose(chassis.getPose());

    RclMain.updateBotPose();
    RclMain.updateBotPose(&left_rcl);
    RclMain.updateBotPose(&back_rcl);

    // Intake three balls
    stopIntake();
    startIntake();
    chassis.moveToPoint(-19, 24, 800, {}, true);
    pros::delay(350);
    openGate();

    // Intake two balls
    chassis.turnToPoint(-9, 44, 200, {}, true);
    closeGate();
    chassis.moveToPoint(-9, 44, 900, {.maxSpeed=95}, false);

    // Head to long goal
    chassis.moveToPose(-36, 45, 140, 2600, {.forwards=false, .minSpeed=90}, false);
    chassis.turnToPoint(-24, 47, 500, {.forwards=false}, true);
    pros::delay(250);
    openGate();
    startOuttake();
    chassis.moveToPoint(-24, 47, 1400, {.forwards=false, .maxSpeed=100}, true);
    // Score
    pros::delay(500);
    scoreAll(1000);

    // Refill at match loader
    openGate();
    chassis.moveToPoint(-72, 47, 2500, {.maxSpeed=40}, true);
    startOuttake();
    pros::delay(350);
    startIntake();
    pros::delay(1250);
    RclMain.updateBotPose(&right_rcl);
    RclMain.updateBotPose(&fl_rcl);
    RclMain.updateBotPose(&fr_rcl);
    // Score top mid
    chassis.turnToPoint(-10, 10, 500, {.forwards=false}, false);
    retractLift();
    chassis.moveToPoint(-10, 10, 1700, {.forwards=false, .maxSpeed = 110}, false);
    scoreAll(1000, FAST_MID_SCORE);

    // Push
    chassis.moveToPoint(-38, 36.5, 1200, {}, false);
    chassis.turnToPoint(0, 37.5, 500, {}, false);
    retractLeftArm();
    chassis.moveToPoint(-16, 37.5, 2000, {.minSpeed=127}, false);
    chassis.turnToHeading(135, 2000, {.minSpeed=127}, false);
}
// ---
void rightv2() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.setPose(-48.75, -16.125, 90);
    RclMain.setRclPose(chassis.getPose());

    RclMain.updateBotPose();
    RclMain.updateBotPose(&right_rcl);
    RclMain.updateBotPose(&back_rcl);

    // Intake three balls
    stopIntake();
    startIntake();
    chassis.moveToPoint(-19, -24, 1000, {}, true);
    pros::delay(300);
    openGate();

    // Intake two balls
    chassis.turnToPoint(-9, -44, 350, {}, true);
    closeGate();
    chassis.moveToPoint(-9, -44, 1300, {.maxSpeed=70}, false);

    // Head to long goal
    chassis.moveToPose(-40, -44, 40, 2000, {.forwards=false, .minSpeed=100}, false);
    chassis.swingToHeading(270, lemlib::DriveSide::RIGHT, 1500, {.direction = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .minSpeed=127}, false);
    chassis.moveToPoint(-28, -47.5, 1100, {.forwards=false, .maxSpeed=100}, true);
    // Score
    pros::delay(450);
    startTopScore(allianceColor);
    closeGate();
    pros::delay(1550);
    RclMain.updateBotPose(&left_rcl);
    stopTopScore();

    // Refill at match loader
    startIntake();
    openGate();
    chassis.moveToPoint(-63, -47, 1400, {.maxSpeed=70}, false);
    pros::delay(350);
    RclMain.updateBotPose(&left_rcl);
    // Score lower mid
    chassis.moveToPoint(-47.5, -47.5, 1200, {.forwards=false}, true);
    pros::delay(600);
    closeGate();
    chassis.turnToPoint(-17, -12, 600, {}, false);
    chassis.moveToPoint(-17, -12, 400, {}, true);
    chassis.moveToPoint(-17, -12, 1200, {.maxSpeed = 50}, true);
    chassis.turnToPoint(0, 0, 300, {}, true);
    pros::delay(800);
    startOuttake();
    frontMotor.move(-90);
    pros::delay(1000);
    stopIntake();

    // Push
    chassis.moveToPoint(-36, -37, 1100, {.forwards=false}, false);
    chassis.turnToPoint(-15.5, -37, 800, {.forwards=false}, false);
    retractLeftArm();
    chassis.moveToPoint(-15.5, -37, 1200, {.forwards=false, .maxSpeed=80}, false);
    chassis.moveToPoint(-15.5, -37, 9999, {.forwards=false}, false);
    chassis.turnToHeading(225, 700, {.minSpeed=127});
}

void skills() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.setPose(-47, -13.25, 180);
    RclMain.setRclPose(chassis.getPose());

    RclMain.updateBotPose();

    // Head towards bottom-left match loader
    chassis.moveToPoint(-47, -47, 1800, {}, false);
    chassis.turnToHeading(270, 700, {});
    startIntake();
    openGate();
    chassis.moveToPoint(-68.5, -47, 1200, {.maxSpeed=50}, false);
    jiggle(4, 2500);
    RclMain.updateBotPose(&left_rcl);

    // Back off (#1)
    stopIntake();
    closeGate();
    chassis.moveToPoint(-49, -47,  1000, {.forwards=false}, false);
    chassis.turnToHeading(155, 800, {}, false);

    // Score bottom-right
    chassis.moveToPose(-16, -63, 90, 1500, {.lead=0.4, .minSpeed=55}, false);
    chassis.moveToPoint(20, -61, 2000, {.minSpeed=30, .earlyExitRange=12}, false);
     
    chassis.moveToPoint(40, -61, 1000, {.maxSpeed=60}, false);
    chassis.turnToHeading(0, 800, {});
    RclMain.updateBotPose(&right_rcl);
    chassis.moveToPoint(38, -48, 1200, {}, true);
    chassis.turnToHeading(90, 600, {}, false);

    chassis.moveToPoint(24, -48, 1000, {.forwards=false, .maxSpeed=70}, false);
    startTopScore(alliance_color::NONE);
    pros::delay(2750);
    
    // Head towards bottom-right match loader
    stopTopScore();
    startIntake();
    openGate();
    chassis.moveToPoint(68, -48, 1200, {.maxSpeed=50}, false);
    jiggle(4, 2500);
    RclMain.updateBotPose(&right_rcl);

    // Score the long goal
    chassis.moveToPoint(24, -48, 1100, {.forwards=false, .maxSpeed=70}, false);
    startTopScore(alliance_color::NONE);
    closeGate();
    RclMain.updateBotPose(&right_rcl);
    pros::delay(3000);

    // Back off (#2)
    chassis.moveToPoint(48, -48, 1500, {}, false);
    chassis.turnToHeading(0, 600, {}, false);
    stopTopScore();
    pros::delay(200);
    RclMain.updateBotPose(&back_rcl);

    // Head towards top-right match loader
    chassis.moveToPoint(48, 48, 1300, {.minSpeed=80}, false);
    chassis.moveToPoint(48, 48, 1000, {.maxSpeed=60}, false);
    RclMain.updateBotPose(&right_rcl);
    chassis.turnToHeading(90, 500, {}, false);
    pros::delay(200);
    RclMain.updateBotPose(&left_rcl);
    startIntake();
    openGate();
    chassis.moveToPoint(68, 47, 1200, {.maxSpeed=50}, false); //
    jiggle(4, 2500);
    RclMain.updateBotPose(&left_rcl);

    // Back off (#3)
    stopIntake();
    closeGate();
    chassis.moveToPoint(50, 49, 1000, {.forwards=false}, false);
    RclMain.updateBotPose(&left_rcl);
    chassis.turnToHeading(335, 700, {}, false);

    // Score the top-left goal
    chassis.moveToPose(10, 64, 270, 1500, {.lead=0.5, .minSpeed=55}, false);
    chassis.moveToPoint(-20, 64.5, 2000, {.minSpeed=40, .earlyExitRange=12}, false);
    chassis.moveToPoint(-38, 64.5, 1000, {.maxSpeed=60}, false);
    chassis.turnToHeading(180, 800, {});
    RclMain.updateBotPose(&back_rcl);
    RclMain.updateBotPose(&right_rcl);
    chassis.moveToPoint(-38, 48, 1200, {}, false);
    chassis.turnToHeading(270, 800, {}, false);
    
    chassis.moveToPoint(-22, 48, 1100, {.forwards=false, .maxSpeed=70}, false);
    startTopScore(alliance_color::NONE);
    RclMain.updateBotPose(&right_rcl);
    pros::delay(3000);
    
    // Intake from top-left match loader
    stopTopScore();
    startIntake();
    openGate();
    chassis.moveToPoint(-68, 47, 1200, {.maxSpeed=50}, false);
    jiggle(4, 2500);
    RclMain.updateBotPose(&right_rcl);

    // Score the long goal
    chassis.moveToPoint(-20, 48, 1500, {.forwards=false, .maxSpeed=70}, false);
    startTopScore(alliance_color::NONE);
    closeGate();
    RclMain.updateBotPose(&right_rcl);
    pros::delay(3000);

    // Back off (#4)
    chassis.moveToPoint(-42, 48, 1000, {});
    stopTopScore();
        // Park
    chassis.moveToPoint(-55, 48, 1000);
    chassis.moveToPose(-70, 18, 200, 3500, {.lead = .25}, false);
    //chassis.moveToPose(-64, 18, 180, 2500, {.maxSpeed = 100}, false);
    odomLift.extend();
    startOuttake();
    pros::delay(50);
    leftMotors.move(127);
    rightMotors.move(127);
    pros::delay(800);
    leftMotors.move(0);
    rightMotors.move(0);
}
// 90+ points
void skills_v2() {

    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.setPose(-48.75, 16.125, 90);
    RclMain.setRclPose(chassis.getPose());

    RclMain.updateBotPose();
    RclMain.updateBotPose(&left_rcl);
    RclMain.updateBotPose(&back_rcl);

    // Intake three balls
    stopIntake();
    startIntake();
    chassis.moveToPoint(-18, 26, 1800, {.maxSpeed=70}, true);
    pros::delay(1000);
    openGate();

    // Score two red
    chassis.turnToPoint(-12, 9, 500,{.forwards=false}, false);
    startOuttake();
    frontMotor.move(-25);
    chassis.moveToPose(-6, 6, 315, 3000, {.forwards=false}, true);
    pros::delay(1200);
    middleMech.retract();
    startMidScore();
    closeGate();
    pros::delay(1400); // middle goal score time
    stopMidScore();
    startOuttake();
    pros::delay(400);
    middleMech.extend();

    // Empty top-left loader
    startIntake();
    chassis.turnToPoint(-47, 48, 200, {}, false);
    chassis.moveToPoint(-47, 48, 1400, {}, true);
    pros::delay(700);
    openGate();
    chassis.turnToPoint(-63, 47, 500, {}, false);
    startIntake();
    chassis.moveToPoint(-70, 47, 1000, {.maxSpeed=60}, false);
    RclMain.updateBotPose(&right_rcl);
    jiggle(4, 2000);
    stopIntake();
    closeGate();

    // Score at top-right long goal
    chassis.turnToHeading(225, 200, {}, false);
    chassis.moveToPose(-25, 61, 270, 1200, {.forwards=false, .lead=0.5, .minSpeed=50}, false);
    chassis.moveToPoint(38, 61, 1500, {.forwards=false}, false);
    chassis.turnToHeading(180, 700, {}, false);
    pros::delay(400);
    RclMain.updateBotPose(&left_rcl);
    RclMain.updateBotPose(&back_rcl);
    chassis.moveToPoint(38 , 48, 1200, {}, false);
    chassis.turnToPoint(27, 48, 500, {.forwards=false}, false);
    chassis.moveToPoint(27, 48, 800, {.forwards=false, .maxSpeed=110}, true);
    pros::delay(500);
    startTopScore();
    startTopScore(alliance_color::NONE);
    pros::delay(3000);
    RclMain.updateBotPose(&left_rcl);
    stopTopScore();

    // Refill at top-right loader then score again
    openGate();
    chassis.moveToPoint(70, 47, 1600, {.maxSpeed=70}, true);
    pros::delay(300);
    startIntake();
    pros::delay(1300);
    jiggle(4, 2000);
    RclMain.updateBotPose(&left_rcl);
    chassis.moveToPoint(27, 48, 1400, {.forwards=false, .maxSpeed=110}, true);
    pros::delay(400);
    startTopScore();
    startTopScore(alliance_color::NONE);
    pros::delay(2700);
    RclMain.updateBotPose(&left_rcl);
    stopTopScore();
    closeGate();

    // Clear park zone and get red balls
    chassis.moveToPoint(67, 26, 1000, {}, false);
    odomLift.extend();
    chassis.swingToPoint(180, 600, lemlib::DriveSide::RIGHT, false);
    chassis.turnToHeading(180, 300, {}, false);

    startIntake();
    RclMain.setMaxSyncPerSec(100);
    chassis.moveToPoint(68, -30, 2200, {.minSpeed=100, .earlyExitRange=4}, true);
    pros::delay(1700);
    openGate();
    odomLift.retract();
    pros::delay(500);
    closeGate();
    chassis.turnToHeading(180, 400, {}, false);
    RclMain.setMaxSyncPerSec(6.0);

    // Reset location
    chassis.turnToHeading(270, 700, {}, false);
    chassis.moveToPoint(chassis.getPose().x-7, chassis.getPose().y, 1200, {}, false);
    chassis.turnToHeading(270, 300, {}, false);
    pros::delay(400);
    RclMain.updateBotPose(&left_rcl);
    RclMain.updateBotPose(&back_rcl);

    // Get one more red ball then score everything in top mid
    startIntake();
    chassis.turnToPoint(30, -21, 700, {}, false);
    chassis.moveToPoint(30, -21, 1000, {.maxSpeed=80}, false);
    chassis.turnToPoint(11.5, -11.5, 800, {.forwards=false}, false);
    chassis.moveToPose(11, -11, 135, 2400, {.forwards=false}, true);
    pros::delay(400);
    openGate();
    startOuttake();
    frontMotor.move(-60);
    pros::delay(700);
    middleMech.retract();
    startMidScore();
    pros::delay(3000); // middle goal score time
    stopMidScore();
    startOuttake();
    pros::delay(400);
    middleMech.extend();
    closeGate();

    // Empty bottom-right loader
    chassis.turnToHeading(135, 300, {}, false);
    moveForward(52, 1200, 127, 1, true);
    pros::delay(700);
    startIntake();
    openGate();
    chassis.turnToPoint(63, -47, 500, {}, false);
    chassis.moveToPoint(70, -47, 1000, {.maxSpeed=65}, false);
    RclMain.updateBotPose(&right_rcl);
    jiggle(4, 2000);
    stopIntake();
    closeGate();

    // Score at bottom-left long goal
    chassis.turnToHeading(45, 200, {}, false);
    chassis.moveToPose(25, -61, 90, 1200, {.forwards=false, .lead=0.5, .minSpeed=50}, false);
    chassis.moveToPoint(-38, -61, 1500, {.forwards=false}, false);
    chassis.turnToHeading(0, 700, {}, false);
    RclMain.updateBotPose(&left_rcl);
    RclMain.updateBotPose(&back_rcl);
    chassis.moveToPoint(-38 , -47.5, 1200, {}, false);
    chassis.turnToPoint(-28, -47.5, 500, {.forwards=false}, false);
    chassis.moveToPoint(-28, -47.5, 800, {.forwards=false, .maxSpeed=110}, true);
    pros::delay(500);
    startTopScore();
    startTopScore(alliance_color::NONE);
    pros::delay(3000);
    RclMain.updateBotPose(&left_rcl);
    stopTopScore();

    // Refill at bottom-left loader then score again
    openGate();
    chassis.moveToPoint(-70, -47, 1600, {.maxSpeed=65}, true);
    pros::delay(300);
    startIntake();
    pros::delay(1300);
    jiggle(4, 2000);
    RclMain.updateBotPose(&left_rcl);
    chassis.moveToPoint(-28, -47.5, 1400, {.forwards=false, .maxSpeed=90}, true);
    pros::delay(400);
    startTopScore();
    startTopScore(alliance_color::NONE);
    pros::delay(2700);
    RclMain.updateBotPose(&left_rcl);
    stopTopScore();
    closeGate();

    // Park
    chassis.moveToPoint(-67, -24, 1000, {}, false);
    odomLift.extend();
    chassis.swingToPoint(0, 700, lemlib::DriveSide::RIGHT, false);
    chassis.turnToHeading(0, 400, {}, false);
    pros::delay(200);
    chassis.waitUntilDone();

    startIntake();
    chassis.moveToPoint(-68, 0, 1300, {.minSpeed=80, .earlyExitRange=3}, true);
}

// 80 points
void skills_v3() {

    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.setPose(-48.75, 16.125, 90);
    RclMain.setRclPose(chassis.getPose());

    RclMain.updateBotPose();
    RclMain.updateBotPose(&left_rcl);
    RclMain.updateBotPose(&back_rcl);

    // Intake three balls
    stopIntake();
    startIntake();
    chassis.moveToPoint(-18, 26, 1800, {.maxSpeed=70}, true);
    pros::delay(1000);
    openGate();

    // Score two red
    chassis.turnToPoint(-12, 9, 500,{.forwards=false}, false);
    startOuttake();
    frontMotor.move(-25);
    chassis.moveToPose(-6, 6, 315, 3000, {.forwards=false}, true);
    pros::delay(1200);
    middleMech.retract();
    startMidScore();
    closeGate();
    pros::delay(1400); // middle goal score time
    stopMidScore();
    startOuttake();
    pros::delay(400);
    middleMech.extend();

    // Empty top-left loader
    startIntake();
    chassis.turnToPoint(-47, 48, 200, {}, false);
    chassis.moveToPoint(-47, 48, 1400, {}, true);
    pros::delay(700);
    openGate();
    chassis.turnToPoint(-63, 47, 500, {}, false);
    startIntake();
    chassis.moveToPoint(-70, 47, 1000, {.maxSpeed=60}, false);
    RclMain.updateBotPose(&right_rcl);
    jiggle(4, 2000);
    stopIntake();
    closeGate();

    // Score at top-right long goal
    chassis.turnToHeading(225, 200, {}, false);
    chassis.moveToPose(-25, 61, 270, 1200, {.forwards=false, .lead=0.5, .minSpeed=50}, false);
    chassis.moveToPoint(38, 61, 1500, {.forwards=false}, false);
    chassis.turnToHeading(180, 700, {}, false);
    pros::delay(400);
    RclMain.updateBotPose(&left_rcl);
    RclMain.updateBotPose(&back_rcl);
    chassis.moveToPoint(38 , 48, 1200, {}, false);
    chassis.turnToPoint(24, 48, 500, {.forwards=false}, false);
    chassis.moveToPoint(24, 48, 1200, {.forwards=false, .maxSpeed=70}, true);
    pros::delay(500);
    startTopScore();
    startTopScore(alliance_color::NONE);
    pros::delay(3000);
    RclMain.updateBotPose(&left_rcl);
    stopTopScore();

    // Refill at top-right loader then score again
    openGate();
    chassis.moveToPoint(70, 47, 1600, {.maxSpeed=70}, true);
    pros::delay(300);
    startIntake();
    pros::delay(1300);
    jiggle(4, 2000);
    RclMain.updateBotPose(&left_rcl);
    chassis.moveToPoint(24, 48, 1700, {.forwards=false, .maxSpeed=70}, true);
    pros::delay(400);
    startTopScore();
    startTopScore(alliance_color::NONE);
    pros::delay(2700);
    RclMain.updateBotPose(&left_rcl);
    stopTopScore();
    closeGate();

    // Get one more red ball then score everything in top mid
    startIntake();
    chassis.turnToPoint(24, -22, 1000, {}, false);
    chassis.moveToPoint(24, -22, 4000, {.maxSpeed=80}, false);

    chassis.turnToPoint(10.5, -10.5, 800, {.forwards=false}, false);
    openGate();
    startOuttake();
    frontMotor.move(-45);
    chassis.moveToPose(8, -8, 135, 3400, {.forwards=false}, true);
    pros::delay(1700);
    middleMech.retract();
    startMidScore();
    pros::delay(2000); // middle goal score time
    jiggle(2, 3000, 3.0, 3.0);
    stopMidScore();
    startOuttake();
    pros::delay(400);
    middleMech.extend();
    closeGate();

    // Empty bottom-right loader
    //chassis.turnToPoint(40, -48, 400, {}, false);
    //chassis.moveToPoint(40, -48, 1600, {}, true);
    chassis.turnToHeading(135, 300, {}, false);
    moveForward(52, 1200, 127, 1, true);
    pros::delay(700);
    startIntake();
    openGate();
    chassis.turnToPoint(63, -47, 600, {}, false);
    RclMain.updateBotPose(&right_rcl);
    chassis.moveToPoint(70, -47, 1000, {.maxSpeed=65}, false);
    RclMain.updateBotPose(&right_rcl);
    jiggle(4, 2000);
    stopIntake();
    closeGate();

    // Score at bottom-left long goal
    chassis.turnToHeading(45, 200, {}, false);
    chassis.moveToPose(25, -61, 90, 1200, {.forwards=false, .lead=0.5, .minSpeed=50}, false);
    chassis.moveToPoint(-38, -61, 1500, {.forwards=false}, false);
    chassis.turnToHeading(0, 700, {}, false);
    RclMain.updateBotPose(&left_rcl);
    RclMain.updateBotPose(&back_rcl);
    chassis.moveToPoint(-38 , -48, 1200, {}, false);
    chassis.turnToPoint(-24, -48, 500, {.forwards=false}, false);
    chassis.moveToPoint(-24, -48, 800, {.forwards=false, .maxSpeed=110}, true);
    pros::delay(500);
    startTopScore();
    startTopScore(alliance_color::NONE);
    pros::delay(3000);
    RclMain.updateBotPose(&left_rcl);
    stopTopScore();

    // Refill at bottom-left loader then score again
    openGate();
    chassis.moveToPoint(-70, -47, 1600, {.maxSpeed=65}, true);
    pros::delay(300);
    startIntake();
    pros::delay(1300);
    jiggle(4, 2000);
    RclMain.updateBotPose(&left_rcl);
    chassis.moveToPoint(-24, -48, 1400, {.forwards=false, .maxSpeed=90}, true);
    pros::delay(400);
    startTopScore();
    startTopScore(alliance_color::NONE);
    pros::delay(2700);
    RclMain.updateBotPose(&left_rcl);
    stopTopScore();
    closeGate();

    // Park
    chassis.moveToPoint(-67, -24, 1000, {}, false);
    odomLift.extend();
    chassis.swingToPoint(0, 700, lemlib::DriveSide::RIGHT, false);
    chassis.turnToHeading(0, 400, {}, false);
    pros::delay(200);
    chassis.waitUntilDone();

    chassis.turnToHeading(0, 800, {}, false);
}

// 119 points
void skills_119() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.setPose(-63, 16, 180);
    RclMain.setRclPose(chassis.getPose());

    RclMain.updateBotPose();
    RclMain.updateBotPose(&right_rcl);
    RclMain.updateBotPose(&back_rcl);

    // Clear Park Zone
    odomLift.extend();
    startIntake();
    RclMain.setMaxSyncPerSec(40.0);
    chassis.moveToPoint(-63, -23, 2200, {.minSpeed=100, .earlyExitRange=4}, true);
    pros::delay(1700);
    openGate();
    odomLift.retract();
    pros::delay(500);
    closeGate();
    RclMain.setMaxSyncPerSec(6.0);

    // Reset location
    chassis.turnToHeading(90, 300, {}, false);
    chassis.moveToPoint(chassis.getPose().x-7, chassis.getPose().y, 1000, {}, false);
    chassis.turnToHeading(270, 300, {}, false);
    pros::delay(400);
    RclMain.updateBotPose(&right_rcl);
    RclMain.updateBotPose(&back_rcl);

    // Get one ball from stack
    chassis.turnToPoint(-30, -20, 300, {}, false);
    chassis.moveToPoint(-30, -20, 900, {.maxSpeed=80}, false);

    // Score top mid
    chassis.turnToPoint(-18, 10, 400, {}, false);
    chassis.moveToPoint(-18, 10, 1400, {}, false);
    chassis.moveToPose(-18, 18, 315, 1200, {.lead=0.5}, false);
    chassis.moveToPoint(-11, 11, 1000, {.forwards=false}, false);
    openGate();
    startOuttake();
    frontMotor.move(-100);
    pros::delay(300);
    middleMech.retract();
    pros::delay(200);
    startMidScore();
    pros::delay(3000);
    stopMidScore();
    closeGate();

    // Refill at top left
    startIntake();
    chassis.turnToPoint(-47, 48, 200, {}, false);
    chassis.moveToPoint(-47, 48, 1400, {}, true);
    pros::delay(700);
    openGate();
    chassis.turnToPoint(-63, 47, 500, {}, false);
    startIntake();
    chassis.moveToPoint(-70, 47, 1000, {.maxSpeed=60}, false);
    RclMain.updateBotPose(&right_rcl);
    jiggle(4, 2000);
    stopIntake();
    closeGate();

    // Score at top-right long goal
    chassis.turnToHeading(225, 200, {}, false);
    chassis.moveToPose(-25, 63, 270, 1200, {.forwards=false, .lead=0.5, .minSpeed=50}, false);
    chassis.moveToPoint(38, 63, 1500, {.forwards=false}, false);
    chassis.turnToHeading(180, 700, {}, false);
    pros::delay(400);
    RclMain.updateBotPose(&left_rcl);
    RclMain.updateBotPose(&back_rcl);
    chassis.moveToPoint(38 , 48, 1200, {}, false);
    chassis.turnToPoint(27, 47, 500, {.forwards=false}, false);
    chassis.moveToPoint(27, 47, 800, {.forwards=false, .maxSpeed=110}, true);
    pros::delay(500);
    startTopScore();
    startTopScore(alliance_color::NONE);
    pros::delay(3000);
    RclMain.updateBotPose(&left_rcl);
    stopTopScore();

    // Refill at top-right loader then score again
    openGate();
    chassis.moveToPoint(70, 47, 1600, {.maxSpeed=70}, true);
    pros::delay(300);
    startIntake();
    pros::delay(1300);
    jiggle(4, 2000);
    RclMain.updateBotPose(&left_rcl);
    chassis.moveToPoint(27, 47, 1400, {.forwards=false, .maxSpeed=110}, true);
    pros::delay(400);
    startTopScore();
    startTopScore(alliance_color::NONE);
    pros::delay(2700);
    RclMain.updateBotPose(&left_rcl);
    stopTopScore();
    closeGate();

    // Clear park zone and get red balls
    chassis.moveToPoint(67, 26, 1000, {}, false);
    odomLift.extend();
    chassis.swingToPoint(180, 600, lemlib::DriveSide::RIGHT, false);
    chassis.turnToHeading(180, 300, {}, false);
    RclMain.updateBotPose(&back_rcl);

    startIntake();
    RclMain.setMaxSyncPerSec(40.0);
    chassis.moveToPoint(63, -23, 2200, {.minSpeed=100, .earlyExitRange=4}, true);
    pros::delay(1700);
    openGate();
    odomLift.retract();
    pros::delay(500);
    closeGate();
    RclMain.setMaxSyncPerSec(6.0);

    // Reset location
    chassis.turnToHeading(270, 700, {}, false);
    chassis.moveToPoint(chassis.getPose().x-7, chassis.getPose().y, 800, {}, false);
    chassis.turnToHeading(270, 300, {}, false);
    pros::delay(400);
    RclMain.updateBotPose(&left_rcl);
    RclMain.updateBotPose(&back_rcl);

    // Pick up one more red ball
    chassis.turnToPoint(36.5, -11, 400, {}, false);
    chassis.moveToPoint(36.5, -11, 1000, {}, false);
    chassis.turnToPoint(29, -18.5, 600, {}, false);
    chassis.moveToPose(29, -18.5, 225, 1700, {.lead=0.1}, false);
    
    // Pick up one red then score everything in bottom mid
    chassis.moveToPoint(36.5, -11, 800, {.forwards=false}, false);
    chassis.turnToPoint(19, 19, 600, {}, false);
    chassis.moveToPoint(19, 19, 1600, {.maxSpeed=80}, false);
    chassis.turnToPoint(12, 12, 800, {}, false);
    chassis.moveToPose(12, 12, 225, 1200, {}, false);
    startOuttake();
    frontMotor.move(-50);
    pros::delay(3000);
    stopIntake();

    // Pick up three balls
    chassis.moveToPoint(19, 19, 600, {.forwards=false}, false);
    chassis.turnToPoint(23.5, -23.5, 600, {}, false);
    startIntake();
    chassis.moveToPoint(23.5, -23.5, 2500, {.maxSpeed=70}, false);

    // Refill at bottom right
    chassis.turnToPoint(47, -48, 500, {}, false);
    chassis.moveToPoint(47, -48, 1300, {}, true);
    pros::delay(700);
    openGate();
    chassis.turnToPoint(63, -47, 500, {}, false);
    chassis.moveToPoint(70, -47, 1000, {.maxSpeed=65}, false);
    RclMain.updateBotPose(&right_rcl);
    jiggle(4, 2000);
    stopIntake();
    closeGate();

    // Score at bottom-left long goal
    chassis.turnToHeading(45, 200, {}, false);
    chassis.moveToPose(25, -63, 90, 1200, {.forwards=false, .lead=0.5, .minSpeed=50}, false);
    chassis.moveToPoint(-38, -63, 1500, {.forwards=false}, false);
    chassis.turnToHeading(0, 700, {}, false);
    RclMain.updateBotPose(&left_rcl);
    RclMain.updateBotPose(&back_rcl);
    chassis.moveToPoint(-38 , -47, 1200, {}, false);
    chassis.turnToPoint(-28, -47, 500, {.forwards=false}, false);
    chassis.moveToPoint(-28, -47, 800, {.forwards=false, .maxSpeed=110}, true);
    pros::delay(500);
    startTopScore();
    startTopScore(alliance_color::NONE);
    pros::delay(3000);
    RclMain.updateBotPose(&left_rcl);
    stopTopScore();

    // Refill at bottom-left loader then score again
    openGate();
    chassis.moveToPoint(-70, -47, 1600, {.maxSpeed=65}, true);
    pros::delay(300);
    startIntake();
    pros::delay(1300);
    jiggle(4, 2000);
    RclMain.updateBotPose(&left_rcl);
    chassis.moveToPoint(-28, -47, 1400, {.forwards=false, .maxSpeed=90}, true);
    pros::delay(400);
    startTopScore();
    startTopScore(alliance_color::NONE);
    pros::delay(2700);
    RclMain.updateBotPose(&left_rcl);
    stopTopScore();
    closeGate();

    // Park
    chassis.moveToPoint(-67, -24, 1000, {}, false);
    odomLift.extend();
    chassis.swingToPoint(0, 700, lemlib::DriveSide::RIGHT, false);
    chassis.turnToHeading(0, 400, {}, false);
    RclMain.setMaxSyncPerSec(40.0);
    chassis.moveToPoint(-63, 0, 1800, {.minSpeed=80, .earlyExitRange=3}, false);
    RclMain.setMaxSyncPerSec(6.0);
}