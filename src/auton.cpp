#include "custom/auton.hpp"
#include "custom/RclTracking.hpp"
#include "custom/configs.hpp"
#include "custom/util_funcs.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include <cmath>

void leftPush(double x_offset = 0, double y_offset = 0) {
    //chassis.moveToPoint(-53, 47, 600, {}, false);
    //chassis.turnToHeading(130, 500, {}, false);
    //chassis.moveToPoint(-37, 35, 550, {}, false);
    chassis.moveToPoint(-38+x_offset, 37.5+y_offset, 900, {}, false);
    chassis.turnToHeading(270, 450, {}, false);
    retractLeftArm();
    chassis.moveToPoint(-21+x_offset, 37.5+y_offset, 2000, {.minSpeed=127, .earlyExitRange=1}, false);
    chassis.turnToHeading(135, 2000, {.minSpeed=127}, false);
}
void rightPush(double x_offset = 0, double y_offset = 0) {
    //chassis.moveToPoint(-50, -47, 300, {}, false);
    //chassis.turnToHeading(240, 250, {}, false);
    //chassis.moveToPoint(-39, -37, 450, {.forwards=false}, false);
    chassis.moveToPoint(-38+x_offset, -37.5-y_offset, 900, {}, false);
    chassis.turnToHeading(270, 450, {}, false);
    retractLeftArm();
    chassis.moveToPoint(-24+x_offset, -37.5-y_offset, 2000, {.forwards=false, .minSpeed=127}, false);
}

// WORKS
void soloAWP(){
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.setPose(-47, 0, 0);
    RclMain.setRclPose(chassis.getPose());

    // Circle_Obstacle allianceBot(-52, 19, 6, 15000);
    RclMain.updateBotPose(&left_rcl);
    RclMain.updateBotPose(&back_rcl);

    // Push teammate and get their preload
    startIntake();
    chassis.moveToPoint(-47, 5, 350, {}, false);

    // Head towards the matchloader and intake
    chassis.moveToPoint(-47, -45, 1400, {.forwards=false}, true);
    pros::delay(400);
    openGate();
    chassis.turnToPoint(-70, -47, 400, {}, false);
    startIntake();
    chassis.moveToPoint(-70, -47, 1100, {.maxSpeed=127}, false);
    RclMain.updateBotPose(&left_rcl);

    // Score the long goal
    chassis.moveToPoint(-24, -47.5, 2300, {.forwards=false, .maxSpeed=127}, true);
    pros::delay(200);
    startOuttake();
    pros::delay(100);
    startTopScore(alliance_color::NONE);
    closeGate();
    pros::delay(2000);
    RclMain.updateBotPose(&left_rcl);
    stopTopScore();

    // Back off
    startTopScore(allianceColor == alliance_color::RED ? alliance_color::BLUE : alliance_color::RED);
    
    // Intake 3 balls
    chassis.turnToHeading(10, 700, {.maxSpeed=100}, false);
    stopTopScore();
    startIntake();
    //chassis.moveToPoint(-30, -20, 800, {.maxSpeed=100}, true);
    moveForward(17, 700, 80, 1, true);
    pros::delay(300);
    openGate();
    pros::delay(300);
    closeGate();

    // Intake 3 other balls
    chassis.turnToHeading(0, 200, {}, false);
    RclMain.updateBotPose(&left_rcl);
    chassis.moveToPoint(-24, 25, 1200, {.maxSpeed=127}, true);
    pros::delay(700);
    openGate();

    // Score the mid goal
    chassis.turnToPoint(-10, 10, 400, {.forwards=false}, true);
    stopIntake();
    startOuttake();
    frontMotor.move(-50);    
    chassis.moveToPoint(-10, 10, 1200, {.forwards=false, .maxSpeed=127}, true);
    pros::delay(350);
    startMidScore();
    chassis.waitUntilDone();
    pros::delay(900); // middle goal score time
    middleMech.extend();
    stopMidScore();
    startIntake();

    /*
    chassis.turnToHeading(0, 500, {}, false);
    RclMain.updateBotPose(&left_rcl);
    chassis.turnToPoint(-12, 12, 200, {}, false);
    chassis.moveToPoint(-12, 12, 950, {}, true);
    pros::delay(750);
    startOuttake();
    frontMotor.move(-110);
    pros::delay(200);
    stopIntake();
    chassis.turnToPoint(-10, 10, 350, {.forwards=false}, true);
    chassis.moveToPoint(-10, 10, 800, {.forwards=false}, true);
    startMidScore();
    chassis.waitUntilDone();
    pros::delay(900); // middle goal score time
    middleMech.extend();
    stopMidScore();
    startIntake();
    */
    
    // Move towards long goal and score
    /*
    closeGate();
    chassis.turnToPoint(-48, 49, 200, {}, false);
    chassis.moveToPoint(-48, 49, 1150, {}, true);
    //pros::delay(700);
    pros::delay(400);
    openGate();
    chassis.turnToPoint(-70, 49, 450, {}, false);
    startIntake();
    chassis.moveToPoint(-70, 49, 1100, {.maxSpeed=70}, false);
    RclMain.updateBotPose(&right_rcl);
    */

    closeGate();
    chassis.turnToHeading(315, 150, {}, false);
    RclMain.setMaxSyncPerSec(0.01);
    moveForward(51, 1150, 127, 1, true);
    //pros::delay(700);
    pros::delay(400);
    openGate();
    chassis.turnToPoint(-70, 47, 550, {}, false);
    RclMain.setMaxSyncPerSec(6.0);
    RclMain.updateBotPose(&right_rcl);
    startIntake();
    chassis.moveToPoint(-70, 47.5, 1000, {.maxSpeed=127}, false);
    RclMain.updateBotPose(&right_rcl);

    // Score again
    chassis.moveToPoint(-24, 48, 1800, {.forwards = false, .maxSpeed=127}, true);
    pros::delay(200);
    startOuttake();
    pros::delay(100);
    startTopScore(alliance_color::NONE);
}
// Three balls -> Two balls -> Match loader -> Long goal -> Push
// ---
void right(){
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.setPose(-48.75, -16.125, 90);
    RclMain.setRclPose(chassis.getPose());

    RclMain.updateBotPose(&right_rcl);
    RclMain.updateBotPose(&back_rcl);

    // Intake three balls
    stopIntake();
    startIntake();
    //topMotor.move_velocity(50);
    chassis.moveToPoint(-19, -24, 1000, {}, true);
    pros::delay(300);
    openGate();

    // Intake two balls
    chassis.turnToPoint(-9, -44, 350, {}, true);
    closeGate();
    chassis.moveToPoint(-9, -44, 1300, {.maxSpeed=70}, false);
    chassis.moveToPoint(-19, -19, 850, {.forwards=false}, false);

    // Move towards long goal and refill
    chassis.turnToPoint(-42, -47, 350, {}, false);
    chassis.moveToPoint(-42, -47, 1100, {}, false);
    openGate();
    chassis.turnToPoint(-68, -47, 450, {}, false);

    // Refill at matchloader
    startIntake();
    openGate();
    chassis.moveToPoint(-68, -47, 1000, {.maxSpeed=70}, false);
    pros::delay(300);
    RclMain.updateBotPose(&left_rcl);

    // Score again
    chassis.moveToPoint(-28, -47.5, 1100, {.forwards=false, .maxSpeed=100}, true);
    pros::delay(500);
    //closeGate();
    startTopScore(allianceColor);
    pros::delay(2300);
    RclMain.updateBotPose(&left_rcl);
    stopTopScore();

    // Push
    closeGate();
    chassis.moveToPoint(-40, -37, 1200, {.minSpeed = 60, .earlyExitRange = 1}, false);
    chassis.turnToPoint(-15.5, -37, 600, {.forwards=false}, false);
    retractLeftArm();
    chassis.moveToPoint(-15.5, -37, 1200, {.forwards=false, .maxSpeed=80}, false);
    chassis.moveToPoint(-15.5, -37, 9999, {.forwards=false}, false);
    chassis.turnToHeading(225, 700, {.minSpeed=127});
}
// Three balls -> Two balls -> Top mid -> Match loader -> Long goal -> Push
// ---
void left() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.setPose(-48.75, 16.125, 90);
    RclMain.setRclPose(chassis.getPose());

    RclMain.updateBotPose();
    RclMain.updateBotPose(&left_rcl);
    RclMain.updateBotPose(&back_rcl);

    // Intake three balls
    stopIntake();
    startIntake();
    //topMotor.move_velocity(50);
    chassis.moveToPoint(-19, 24, 700, {}, true);
    pros::delay(300);
    openGate();

    // Intake two balls
    chassis.turnToPoint(-10, 44, 200, {}, true);
    closeGate();
    chassis.moveToPoint(-10, 44, 900, {.maxSpeed=95}, false);
    // Backoff and score middle goal
    chassis.turnToPoint(-18, 18, 150, {.forwards=false}, false);
    chassis.moveToPoint(-18, 18, 900, {.forwards=false}, false);
    stopIntake();
    chassis.turnToPoint(-10.5, 10.5, 450, {.forwards=false}, false);
    openGate();
    startOuttake();
    frontMotor.move(-70);
    chassis.moveToPoint(-10.5, 10.5, 900, {.forwards=false, .maxSpeed = 100}, true);
    pros::delay(200);
    middleMech.retract();
    startMidScore();
    pros::delay(1000); // middle goal score time
    stopMidScore();
    middleMech.extend();
    closeGate();

    // Move towards long goal and score
    chassis.moveToPoint(-47, 47, 900, {}, false);
    openGate();
    chassis.turnToPoint(-68, 48, 350, {}, false);

    // Refill at matchloader
    startIntake();
    openGate();
    chassis.moveToPoint(-68, 48, 1000, {.maxSpeed=70}, false);
    pros::delay(300);
    RclMain.updateBotPose(&right_rcl);

    // Score again
    chassis.moveToPoint(-28, 48, 1100, {.forwards = false, .maxSpeed=110}, true);
    pros::delay(400);
    //closeGate();
    startTopScore(allianceColor);
    pros::delay(1600);
    RclMain.updateBotPose(&right_rcl);
    stopTopScore();

    // Push
    closeGate();
    leftPush();
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
    //topMotor.move_velocity(50);
    chassis.moveToPoint(-24, 24, 900, {}, true);
    pros::delay(450);
    openGate();

    // Head to matchloader
    chassis.turnToPoint(-48, 48, 500, {}, false);
    chassis.moveToPoint(-48, 48, 1000, {}, false);
    chassis.turnToPoint(-70, 47, 300, {}, false);
    chassis.moveToPoint(-70, 47, 1000, {.maxSpeed=70}, false);
    RclMain.updateBotPose(&right_rcl);

    // Move towards long goal sand score
    chassis.turnToPoint(-24, 48, 200, {.forwards=false}, false);
    chassis.moveToPoint(-28, 48, 1300, {.forwards=false, .maxSpeed=110}, true);
    pros::delay(500);
    startTopScore(allianceColor);
    startTopScore();
    pros::delay(2300);
    RclMain.updateBotPose(&right_rcl);
    stopTopScore();

    // Push
    closeGate();
    leftPush(3, -0.5);
}
// Match loader -> Long goal -> Push
// ---
void leftFastRush() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.setPose(-47, 13.25, 180);
    RclMain.setRclPose(chassis.getPose());

    RclMain.updateBotPose();
    RclMain.updateBotPose(&right_rcl);
    RclMain.updateBotPose(&back_rcl);

    // Head towards the matchloader and intake
    openGate();
    chassis.moveToPoint(-47, 47, 1200, {.forwards=false}, false);
    chassis.turnToHeading(270, 300, {});
    startIntake();
    chassis.moveToPoint(-70, 47, 1300, {.maxSpeed=70}, false);
    RclMain.updateBotPose(&right_rcl);

    // Score the long goal
    chassis.moveToPoint(-26, 48, 1100, {.forwards=false, .maxSpeed=110}, true);
    pros::delay(400);
    startTopScore(allianceColor);
    closeGate();
    pros::delay(1400);
    RclMain.updateBotPose(&right_rcl);

    // Push
    closeGate();
    leftPush(1, -1);
}
// ---
void NAAuto() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.setPose(-47, 13.25, 0);
    RclMain.setRclPose(chassis.getPose());

    RclMain.updateBotPose();
    RclMain.updateBotPose(&left_rcl);

    // Head towards the matchloader and intake
    openGate();
    chassis.moveToPoint(-47, 39, 1200, {.minSpeed=60, .earlyExitRange=3}, false);
    chassis.turnToHeading(275, 400, {});
    startIntake();
    chassis.moveToPoint(-63.4, 48, 900, {.maxSpeed=70}, false);
    pros::delay(350);
    RclMain.updateBotPose(&right_rcl);

    // Score the long goal
    chassis.moveToPoint(-28, 48, 1100, {.forwards=false, .maxSpeed=100}, true);
    pros::delay(450);
    startTopScore(allianceColor);
    closeGate();
    pros::delay(1300);
    RclMain.updateBotPose(&right_rcl);

    // Push
    closeGate();
    chassis.moveToPoint(-48, 59, 1200, {.minSpeed = 60, .earlyExitRange = 1}, false);
    chassis.turnToPoint(-24, 58, 600, {.forwards=false}, false);
    retractLeftArm();
    chassis.moveToPoint(-24, 58, 1200, {.forwards=false, .maxSpeed=80}, false);
    chassis.moveToPoint(-24, 58, 9999, {.forwards=false}, false);
    chassis.turnToHeading(225, 700, {.minSpeed=127});
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

    // Move towards long goal and score
    chassis.turnToPoint(-24, -47.5, 200, {.forwards=false}, false);
    chassis.moveToPoint(-24, -47.5, 1300, {.forwards=false, .maxSpeed=110}, true);
    pros::delay(450);
    startTopScore(allianceColor);
    pros::delay(2300);
    RclMain.updateBotPose(&left_rcl);
    stopTopScore();

    // Push
    closeGate();
    rightPush(3, 1.5);
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

    // Score the long goal
    chassis.moveToPoint(-23, -47, 1500, {.forwards=false, .maxSpeed=110}, true);
    pros::delay(200);
    startTopScore(allianceColor);
    closeGate();
    pros::delay(1300);
    RclMain.updateBotPose(&left_rcl);

    // Push
    closeGate();
    rightPush(0, -1);
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
    chassis.moveToPose(-38, 49, 140, 2600, {.forwards=false, .minSpeed=80}, false);
    chassis.turnToPoint(-24, 49.5, 600, {.forwards=false, .maxSpeed=100}, true);
    pros::delay(350);
    openGate();
    startOuttake();
    chassis.moveToPoint(-24, 49.5, 2000, {.forwards=false, .maxSpeed=100}, true);
    // Score
    startTopScore(alliance_color::NONE);
    pros::delay(2000);
    RclMain.updateBotPose(&right_rcl);
    stopTopScore();

    // Refill at match loader
    openGate();
    chassis.moveToPoint(-72, 46, 1500, {.maxSpeed=70}, true);
    startOuttake();
    pros::delay(400);
    startIntake();
    pros::delay(1100);
    RclMain.updateBotPose(&right_rcl);
    // Score top mid
    chassis.turnToPoint(-15, 11, 500, {.forwards=false}, false);
    chassis.moveToPoint(-15, 11, 1700, {.forwards=false, .maxSpeed = 110}, true);
    pros::delay(450);
    startOuttake();
    frontMotor.move(-127);
    pros::delay(300);
    middleMech.retract();
    startMidScore();
    pros::delay(800); // middle goal score time
    closeGate();
    pros::delay(1000);
    stopMidScore();
    middleMech.extend();

    // Push
    chassis.moveToPoint(-23, 39, 1400, {}, false);
    chassis.turnToPoint(0, 39, 600, {}, false);
    retractLeftArm();
    chassis.moveToPoint(-21.5, 39, 2000, {.minSpeed=127}, false);
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
    //MclMain.set_pose(chassis.getPose().x, chassis.getPose().y, chassis.getPose().theta);
	//MclMain.startTracking();

    // Head towards bottom-left match loader
    chassis.moveToPoint(-47, -47, 1800, {}, false);
    chassis.turnToHeading(270, 700, {});
    startIntake();
    openGate();
    chassis.moveToPoint(-68.5, -47, 1200, {.maxSpeed=60}, false);
    chassis.moveToPoint(-61.5, -47, 300, {.forwards=false, .maxSpeed=60}, false);
    chassis.moveToPoint(-68.5, -47, 1000, {.maxSpeed=60}, false);
    RclMain.updateBotPose(&left_rcl);
    //MclMain.set_pose(chassis.getPose().x, chassis.getPose().y, chassis.getPose().theta);
    pros::delay(1000);

    // Back off (#1)
    stopIntake();
    closeGate();
    //chassis.moveToPoint(-38, -48, 1000, {.forwards=false});
    //chassis.turnToHeading(180, 800, {}, false);
    chassis.moveToPoint(-49, -47,  1000, {.forwards=false}, false);
    chassis.turnToHeading(145, 800, {}, false);

    // Score bottom-right
    //chassis.moveToPoint(-38, -62, 1200, {}, false);
    //chassis.turnToHeading(90, 800, {});
    chassis.moveToPose(-15, -61, 90, 1500, {.lead=0.4, .minSpeed=55}, false);
    chassis.moveToPoint(20, -61, 2000, {.minSpeed=30, .earlyExitRange=12}, false);
     
    chassis.moveToPoint(40, -61, 1000, {.maxSpeed=60}, false);
    chassis.turnToHeading(0, 600, {});
    chassis.moveToPoint(38, -48, 1200, {}, true);
    //RclMain.updateBotPose(&right_rcl);
    pros::delay(150);
   // RclMain.updateBotPose(&back_rcl);
    chassis.turnToHeading(90, 600, {}, false);

    chassis.moveToPoint(24, -48, 1000, {.forwards=false, .maxSpeed=70}, false);
    startTopScore();
    pros::delay(2750);
    
    // Head towards bottom-right match loader
    stopTopScore();
    startIntake();
    openGate();
    chassis.moveToPoint(68, -48, 1200, {.maxSpeed=60}, false);
    chassis.moveToPoint(61.5, -48, 300, {.forwards=false, .maxSpeed=60}, false);
    chassis.moveToPoint(78, -48, 1000, {.maxSpeed=60}, false);
    pros::delay(1500);
    RclMain.updateBotPose(&right_rcl);
    //MclMain.set_pose(chassis.getPose().x, chassis.getPose().y, chassis.getPose().theta);

    // Score the long goal
    chassis.moveToPoint(24, -48, 1100, {.forwards=false, .maxSpeed=70}, false);
    
    startTopScore();
    closeGate();
    RclMain.updateBotPose(&right_rcl);
    //MclMain.set_pose(chassis.getPose().x, chassis.getPose().y, chassis.getPose().theta);
    pros::delay(3000);

    // Back off (#2)
    chassis.moveToPoint(47, -48, 1500, {}, false);
    chassis.turnToHeading(0, 600, {}, false);
    stopTopScore();
    pros::delay(200);
    RclMain.updateBotPose(&back_rcl);
    //MclMain.set_pose(chassis.getPose().x, chassis.getPose().y, chassis.getPose().theta);

    // Head towards top-right match loader
    chassis.moveToPoint(45, 48, 1300, {.minSpeed=80}, false);
    chassis.moveToPoint(45, 48, 1000, {.maxSpeed=60}, false);
    RclMain.updateBotPose(&right_rcl);
    //MclMain.set_pose(chassis.getPose().x, chassis.getPose().y, chassis.getPose().theta);
    chassis.turnToHeading(90, 500, {}, false);
    pros::delay(200);
    RclMain.updateBotPose(&left_rcl);
    //MclMain.set_pose(chassis.getPose().x, chassis.getPose().y, chassis.getPose().theta);
    startIntake();
    openGate();
    chassis.moveToPoint(68, 47, 1200, {.maxSpeed=60}, false); //
    chassis.moveToPoint(61.5, 47, 300, {.forwards=false, .maxSpeed=60}, false); //
    chassis.moveToPoint(78, 47, 1000, {.maxSpeed=60}, false); //
    pros::delay(1500);

    // Back off (#3)
    stopIntake();
    closeGate();
    //chassis.moveToPoint(45, 42, 1000, {.forwards=false});
    //chassis.turnToHeading(0, 800, {}, false);
    chassis.moveToPoint(49, 49, 1000, {.forwards=false}, false);
    RclMain.updateBotPose(&left_rcl);
    //MclMain.set_pose(chassis.getPose().x, chassis.getPose().y, chassis.getPose().theta);
    chassis.turnToHeading(325, 700, {}, false);

    // Score the top-left goal
    //chassis.moveToPoint(38, 56, 1200, {}, false);
    //chassis.turnToHeading(270, 800, {});
    chassis.moveToPose(12, 62.5, 270, 1500, {.lead=0.4, .minSpeed=55}, false);
    chassis.moveToPoint(-20, 62.25, 2000, {.minSpeed=40, .earlyExitRange=12}, false);
    chassis.moveToPoint(-38, 62.25, 1000, {.maxSpeed=60}, false);
    chassis.turnToHeading(180, 800, {});
    RclMain.updateBotPose(&back_rcl);
    RclMain.updateBotPose(&right_rcl);
    //MclMain.set_pose(chassis.getPose().x, chassis.getPose().y, chassis.getPose().theta);
    chassis.moveToPoint(-38, 48, 1200, {}, false);
    chassis.turnToHeading(270, 800, {}, false);
    
    chassis.moveToPoint(-22, 48, 1100, {.forwards=false, .maxSpeed=70}, false);
    startTopScore();
    RclMain.updateBotPose(&right_rcl);
    //MclMain.set_pose(chassis.getPose().x, chassis.getPose().y, chassis.getPose().theta);
    pros::delay(3000);
    
    // Intake from top-left match loader
    stopTopScore();
    startIntake();
    openGate();
    chassis.moveToPoint(-68, 47, 1200, {.maxSpeed=60}, false);
    chassis.moveToPoint(-61.5, 47, 400, {.forwards=false, .maxSpeed=60}, false);
    chassis.moveToPoint(-70, 47, 1300, {.maxSpeed=60}, false);     
    pros::delay(2000);

    // Score the long goal
    chassis.moveToPoint(-20, 48, 1500, {.forwards=false, .maxSpeed=70}, false);
    startTopScore();
    closeGate();
    RclMain.updateBotPose(&right_rcl);
    //MclMain.set_pose(chassis.getPose().x, chassis.getPose().y, chassis.getPose().theta);
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
    chassis.turnToPoint(-63, 48, 500, {}, false);
    startIntake();
    chassis.moveToPoint(-70, 48, 1000, {.maxSpeed=60}, false);
    RclMain.updateBotPose(&right_rcl);
    jiggle(4, 2000);
    stopIntake();
    closeGate();

    // Score at top-right long goal
    chassis.turnToHeading(225, 200, {}, false);
    chassis.moveToPose(-25, 61, 270, 1200, {.forwards=false, .lead=0.5, .minSpeed=50}, false);
    chassis.moveToPoint(38, 61, 1500, {.forwards=false}, false);
    chassis.turnToHeading(180, 700, {}, false);
    pros::delay(300);
    RclMain.updateBotPose(&left_rcl);
    RclMain.updateBotPose(&back_rcl);
    chassis.moveToPoint(38 , 47.5, 1200, {}, false);
    chassis.turnToPoint(27, 47.5, 500, {.forwards=false}, false);
    chassis.moveToPoint(27, 47.5, 800, {.forwards=false, .maxSpeed=110}, true);
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
    chassis.moveToPoint(27, 47.5, 1400, {.forwards=false, .maxSpeed=110}, true);
    pros::delay(400);
    startTopScore();
    startTopScore(alliance_color::NONE);
    pros::delay(2700);
    RclMain.updateBotPose(&left_rcl);
    stopTopScore();
    closeGate();

    // Clear park zone and get red balls
    chassis.moveToPoint(67, 26, 1400, {}, false);
    odomLift.extend();
    chassis.swingToPoint(180, 600, lemlib::DriveSide::RIGHT, false);
    chassis.turnToHeading(180, 300, {}, false);

    startIntake();
    chassis.moveToPoint(68, -20, 2200, {.minSpeed=90, .earlyExitRange=4}, true);
    pros::delay(1700);
    openGate();
    odomLift.retract();
    pros::delay(500);
    closeGate();
    chassis.turnToHeading(180, 400, {}, false);

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
    chassis.turnToPoint(10.5, -10.5, 800, {.forwards=false}, false);
    chassis.moveToPose(9, -9, 135, 2400, {.forwards=false}, true);
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
    chassis.turnToPoint(40, -48, 400, {}, false);
    chassis.moveToPoint(40, -48, 1600, {}, true);
    pros::delay(700);
    startIntake();
    openGate();
    chassis.turnToPoint(63, -48, 500, {}, false);
    chassis.moveToPoint(70, -48, 1000, {.maxSpeed=65}, false);
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
    chassis.moveToPoint(-70, -47.5, 1600, {.maxSpeed=65}, true);
    pros::delay(300);
    startIntake();
    pros::delay(1300);
    jiggle(4, 2000);
    chassis.moveToPoint(-28, -47.5, 1400, {.forwards=false, .maxSpeed=90}, true);
    pros::delay(400);
    startTopScore();
    startTopScore(alliance_color::NONE);
    pros::delay(2700);
    RclMain.updateBotPose(&left_rcl);
    stopTopScore();
    closeGate();

    // Park
    chassis.moveToPoint(-67, -24, 1400, {}, false);
    odomLift.extend();
    chassis.swingToPoint(0, 700, lemlib::DriveSide::RIGHT, false);
    chassis.turnToHeading(0, 400, {}, false);
    pros::delay(200);
    chassis.waitUntilDone();

    startIntake();
    chassis.moveToPoint(-68, 0, 1300, {.minSpeed=80, .earlyExitRange=3}, true);
}