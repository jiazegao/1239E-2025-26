#include "custom/auton.hpp"
#include "custom/RclTracking.hpp"
#include "custom/configs.hpp"
#include "custom/util_funcs.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include <cmath>

// #7
void soloAWP(){
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.setPose(-47, 0, 0);
    RclMain.setRclPose(chassis.getPose());

    RclMain.updateBotPose();
    RclMain.updateBotPose(&left_rcl);

    // Push teammate and get their preload
    startIntake();
    chassis.moveToPoint(-47, 5, 300, {}, false);

    // Head towards the matchloader and intake
    chassis.moveToPoint(-47, -28, 1200, {.forwards=false, .minSpeed=100, .earlyExitRange=4}, true);
    pros::delay(400);
    openGate();
    chassis.turnToHeading(270, 300, {});
    chassis.turnToPoint(-64, -45, 300, {}, false);
    startIntake();
    chassis.moveToPoint(-64, -47, 900, {.maxSpeed=70}, false);
    pros::delay(100);
    RclMain.updateBotPose(&left_rcl);
    
    // Score the long goal
    chassis.moveToPoint(-24, -47.5, 1100, {.forwards=false, .maxSpeed=100}, true);
    pros::delay(380);
    startTopScore(allianceColor);
    closeGate();
    pros::delay(600);
    RclMain.updateBotPose(&left_rcl);
    pros::delay(1150);
    stopTopScore();

    // Back off
    startTopScore(allianceColor == alliance_color::RED ? alliance_color::BLUE : alliance_color::RED);
    // Intake 3 balls
    chassis.swingToPoint(-22, -22.5, lemlib::DriveSide::RIGHT, 650, {}, false);
    stopIntake();
    stopTopScore();
    startIntake();
    chassis.moveToPoint(-27, -22.5, 1250, {.maxSpeed=95}, true);
    pros::delay(500);
    openGate();
    pros::delay(400);
    closeGate();

    // Intake 3 other balls
    chassis.turnToHeading(0, 700, {}, false);
    RclMain.updateBotPose(&left_rcl);
    chassis.moveToPoint(-21.5, 27.5, 500, {.minSpeed=90}, true);
    chassis.moveToPoint(-21.5, 27.5, 500, {.maxSpeed=70}, true);
    pros::delay(500);
    openGate();

    // Score the mid goal
    pros::delay(400);
    stopIntake();
    startOuttake();
    frontMotor.move(-60);
    chassis.turnToPoint(-10, 10, 400, {.forwards=false}, false);
    chassis.moveToPoint(-10, 10, 1000, {.forwards=false, .maxSpeed=100}, true);
    pros::delay(200);
    startMidScore();
    chassis.waitUntilDone();
    pros::delay(800); // middle goal score time
    middleMech.extend();
    stopMidScore();
    startIntake();
    
    // Move towards long goal and score
    closeGate();
    chassis.turnToPoint(-47, 47, 150, {}, false);
    chassis.moveToPoint(-47, 47, 1300, {}, true);
    pros::delay(700);
    openGate();
    chassis.turnToPoint(-68, 47, 500, {}, false);
    startIntake();
    chassis.moveToPoint(-68, 47, 900, {.maxSpeed=70}, false);
    pros::delay(150);
    RclMain.updateBotPose(&right_rcl);

    // Score again
    chassis.moveToPoint(-24, 47.5, 1100, {.forwards = false, .maxSpeed=100}, true);
    pros::delay(450);
    startTopScore(allianceColor);
}
// Three balls -> Two balls -> Match loader -> Long goal -> Push
// #2
void right(){
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.setPose(-48.75, -16.125, 90);
    RclMain.setRclPose(chassis.getPose());

    RclMain.updateBotPose();
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
    chassis.turnToPoint(-23, -37, 600, {.forwards=false}, false);
    retractLeftArm();
    chassis.moveToPoint(-23, -37, 1200, {.forwards=false, .maxSpeed=80}, false);
    chassis.moveToPoint(-23, -37, 9999, {.forwards=false}, false);
    chassis.turnToHeading(225, 700, {.minSpeed=127});
}
// Three balls -> Two balls -> Top mid -> Match loader -> Long goal -> Push
// #6
void left() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.setPose(-48.75, 16.125, 90);
    RclMain.setRclPose(chassis.getPose());

    RclMain.updateBotPose();
    RclMain.updateBotPose(&back_rcl);

    // Intake three balls
    stopIntake();
    startIntake();
    //topMotor.move_velocity(50);
    chassis.moveToPoint(-19, 24, 1000, {}, true);
    pros::delay(300);
    openGate();

    // Intake two balls
    chassis.turnToPoint(-9, 44, 350, {}, true);
    closeGate();
    chassis.moveToPoint(-9, 44, 1300, {.maxSpeed=70}, false);
    // Backoff and score middle goal
    chassis.moveToPoint(-18, 18, 1100, {.forwards=false}, false);
    stopIntake();
    openGate();
    chassis.turnToPoint(pivot_x(-10.5), pivot_y(10.5), 400, {}, false);
    startOuttake();
    frontMotor.move(-55);
    chassis.moveToPoint(-10.5, 10.5, 1000, {.forwards=false, .maxSpeed = 70}, true);
    pros::delay(450);
    middleMech.retract();
    startMidScore();
    pros::delay(1250); // middle goal score time
    stopMidScore();
    middleMech.extend();
    closeGate();

    // Move towards long goal and score
    chassis.turnToPoint(-42, 46, 250, {}, false);
    chassis.moveToPoint(-42, 46, 1300, {}, false);
    openGate();
    chassis.turnToPoint(-70, 47, 400, {}, false);

    // Refill at matchloader
    startIntake();
    openGate();
    chassis.moveToPoint(-70, 47, 1000, {.maxSpeed=70}, false);
    pros::delay(300);
    RclMain.updateBotPose(&right_rcl);

    // Score again
    chassis.moveToPoint(-28, 47.5, 1100, {.forwards = false, .maxSpeed=100}, true);
    pros::delay(500);
    //closeGate();
    startTopScore(allianceColor);
    pros::delay(2300);
    RclMain.updateBotPose(&right_rcl);
    stopTopScore();

    // Push
    closeGate();
    chassis.moveToPoint(-40, 37, 1200, {.minSpeed = 60, .earlyExitRange = 1}, false);
    chassis.turnToPoint(-13, 37, 600, {}, false);
    retractLeftArm();
    chassis.moveToPoint(-13, 37, 1200, {.maxSpeed=90}, false);
    chassis.moveToPoint(-13, 37, 9999, {}, false);
    chassis.turnToHeading(135, 700, {.minSpeed=127});
}
// Three balls -> Match loader -> Long goal -> Push
// #4
void leftControlRush() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.setPose(-48.75, 16.125, 90);
    RclMain.setRclPose(chassis.getPose());

    RclMain.updateBotPose();
    RclMain.updateBotPose(&back_rcl);

    // Intake three balls
    startIntake();
    //topMotor.move_velocity(50);
    chassis.moveToPoint(-24, 24, 1500, {.minSpeed=50, .earlyExitRange=3}, true);
    pros::delay(450);
    openGate();

    // Head to matchloader
    chassis.turnToPoint(-48, 48, 500, {}, false);
    chassis.moveToPoint(-48, 48, 1100, {}, false);
    chassis.turnToPoint(-67, 48, 300, {}, false);
    chassis.moveToPoint(-67, 48, 700, {.maxSpeed=60}, false);
    pros::delay(300);
    RclMain.updateBotPose(&right_rcl);

    // Move towards long goal sand score
    chassis.turnToPoint(pivot_x(-24), pivot_y(47), 200, {}, false);
    chassis.moveToPoint(-28, 47, 1300, {.forwards=false, .maxSpeed=100}, true);
    pros::delay(500);
    startTopScore(allianceColor);
    pros::delay(2650);
    RclMain.updateBotPose(&right_rcl);
    stopTopScore();

    // Push
    closeGate();
    chassis.moveToPoint(-40, 37, 1200, {.minSpeed = 60, .earlyExitRange = 1}, false);
    chassis.turnToPoint(-15, 37, 600, {}, false);
    retractLeftArm();
    chassis.moveToPoint(-15, 37, 1200, {.maxSpeed=80}, false);
    chassis.moveToPoint(-15, 37, 9999, {}, false);
    chassis.turnToHeading(135, 700, {.minSpeed=127});
}
// Match loader -> Long goal -> Push
// #5
void leftFastRush() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.setPose(-47, 13.25, 0);
    RclMain.setRclPose(chassis.getPose());

    RclMain.updateBotPose();
    RclMain.updateBotPose(&left_rcl);

    // Head towards the matchloader and intake
    openGate();
    chassis.moveToPoint(-47, 38, 1200, {.minSpeed=60, .earlyExitRange=3}, false);
    chassis.turnToHeading(275, 400, {});
    startIntake();
    chassis.moveToPoint(-63.4, 47, 900, {.maxSpeed=70}, false);
    pros::delay(350);
    RclMain.updateBotPose(&right_rcl);

    // Score the long goal
    chassis.moveToPoint(-28, 47, 1100, {.forwards=false, .maxSpeed=100}, true);
    pros::delay(450);
    startTopScore(allianceColor);
    closeGate();
    pros::delay(1300);
    RclMain.updateBotPose(&right_rcl);

    // Push
    closeGate();
    chassis.moveToPoint(-40, 37, 1200, {.minSpeed = 60, .earlyExitRange = 1}, false);
    chassis.turnToPoint(-15, 37, 600, {}, false);
    retractLeftArm();
    chassis.moveToPoint(-15, 37, 1200, {.maxSpeed=80}, false);
    chassis.moveToPoint(-15, 37, 9999, {}, false);
    chassis.turnToHeading(135, 700, {.minSpeed=127});
}
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
    chassis.turnToPoint(-23, 58, 600, {.forwards=false}, false);
    retractLeftArm();
    chassis.moveToPoint(-23, 58, 1200, {.forwards=false, .maxSpeed=80}, false);
    chassis.moveToPoint(-23, 58, 9999, {.forwards=false}, false);
    chassis.turnToHeading(225, 700, {.minSpeed=127});
}
// Three balls -> Match loader -> Long goal -> Push
// DONE TUNING
void rightControlRush() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.setPose(-48.75, -16.125, 90);
    RclMain.setRclPose(chassis.getPose());

    RclMain.updateBotPose();
    RclMain.updateBotPose(&back_rcl);
    RclMain.updateBotPose(&right_rcl);

    // Intake three balls
    startIntake();
    //topMotor.move_velocity(50);
    chassis.moveToPoint(-24, -24, 1500, {.minSpeed=50, .earlyExitRange=3}, true);
    pros::delay(450);
    openGate();

    // Head to matchloader
    chassis.turnToPoint(-48, -45, 500, {}, false);
    chassis.moveToPoint(-48, -45, 1100, {}, false);
    chassis.turnToPoint(-67, -47, 300, {}, false);
    chassis.moveToPoint(-67, -47, 900, {.maxSpeed=70}, false);
    pros::delay(200);
    RclMain.updateBotPose(&left_rcl);

    // Move towards long goal and score
    chassis.turnToPoint(pivot_x(-24), pivot_y(-47), 200, {}, false);
    chassis.moveToPoint(-28, -47, 1300, {.forwards=false, .maxSpeed=100}, true);
    pros::delay(500);
    startTopScore(allianceColor);
    pros::delay(2650);
    RclMain.updateBotPose(&left_rcl);
    stopTopScore();

    // Push
    closeGate();
    chassis.moveToPoint(-40, -37, 1200, {.minSpeed = 60, .earlyExitRange = 1}, false);
    chassis.turnToPoint(-23, -37, 600, {.forwards=false}, false);
    retractLeftArm();
    chassis.moveToPoint(-23, -37, 1200, {.forwards=false, .maxSpeed=80}, false);
    chassis.moveToPoint(-23, -37, 9999, {.forwards=false}, false);
    chassis.turnToHeading(225, 700, {.minSpeed=127});
}
// Match loader -> Long goal -> Push
// DONE TUNING
void rightFastRush() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.setPose(-47, -13.25, 180);
    RclMain.setRclPose(chassis.getPose());

    RclMain.updateBotPose();
    RclMain.updateBotPose(&right_rcl);

    // Head towards the matchloader and intake
    openGate();
    chassis.moveToPoint(-47, -38, 1200, {.minSpeed=60, .earlyExitRange=3}, false);
    chassis.turnToHeading(265, 400, {});
    startIntake();
    chassis.moveToPoint(-70, -47, 1100, {.maxSpeed=70}, false);
    pros::delay(350);
    RclMain.updateBotPose(&left_rcl);

    // Score the long goal
    chassis.moveToPoint(-28, -47, 1100, {.forwards=false, .maxSpeed=100}, true);
    pros::delay(500);
    startTopScore(allianceColor);
    closeGate();
    pros::delay(1400);
    RclMain.updateBotPose(&left_rcl);

    // Push
    closeGate();
    chassis.moveToPoint(-40, -37, 1200, {.minSpeed = 60, .earlyExitRange = 1}, false);
    chassis.turnToPoint(-23, -37, 600, {.forwards=false}, false);
    retractLeftArm();
    chassis.moveToPoint(-23, -37, 1200, {.forwards=false, .maxSpeed=80}, false);
    chassis.moveToPoint(-23, -37, 9999, {.forwards=false}, false);
    chassis.turnToHeading(225, 700, {.minSpeed=127});
}
// Three balls -> Two balls -> Long goal -> Match loader -> Top mid -> Push
// #3
void leftv2() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.setPose(-48.75, 16.125, 90);
    RclMain.setRclPose(chassis.getPose());

    RclMain.updateBotPose();
    RclMain.updateBotPose(&back_rcl);

    // Intake three balls
    stopIntake();
    startIntake();
    chassis.moveToPoint(-19, 24, 1000, {}, true);
    pros::delay(300);
    openGate();

    // Intake two balls
    chassis.turnToPoint(-9, 44, 350, {}, true);
    closeGate();
    chassis.moveToPoint(-9, 44, 1300, {.maxSpeed=70}, false);

    // Head to long goal
    chassis.moveToPose(-40, 44, 140, 2000, {.forwards=false, .minSpeed=100}, false);
    chassis.swingToHeading(270, lemlib::DriveSide::LEFT, 1500, {.direction = lemlib::AngularDirection::CW_CLOCKWISE, .minSpeed=127}, false);
    chassis.moveToPoint(-28, 47.5, 1100, {.forwards=false, .maxSpeed=100}, true);
    // Score
    pros::delay(450);
    startTopScore(allianceColor);
    openGate();
    pros::delay(1550);
    RclMain.updateBotPose(&right_rcl);
    stopTopScore();

    // Refill at match loader
    startIntake();
    chassis.moveToPoint(-70, 47.5, 400, {.minSpeed=70}, false);
    chassis.moveToPoint(-70, 47.5, 1000, {.maxSpeed=55}, false);
    pros::delay(350);
    RclMain.updateBotPose(&right_rcl);
    // Score top mid
    chassis.turnToPoint(-10, 10, 600, {.forwards=false}, false);
    chassis.moveToPoint(-10, 10, 400, {.forwards=false}, true);
    startOuttake();
    frontMotor.move(-55);
    chassis.moveToPoint(-10, 10, 1000, {.forwards=false, .maxSpeed = 80}, true);
    pros::delay(700);
    middleMech.retract();
    startMidScore();
    pros::delay(1500); // middle goal score time
    stopMidScore();
    middleMech.extend();
    closeGate();

    // Push
    chassis.moveToPoint(-30, 36, 1200, {}, false);
    chassis.turnToPoint(-13, 37, 800, {}, false);
    retractLeftArm();
    chassis.moveToPoint(-13, 37, 1200, {.maxSpeed=90}, false);
    chassis.moveToPoint(-13, 37, 9999, {}, false);
    chassis.turnToHeading(135, 700, {.minSpeed=127});
}
// Unnecessary
void rightv2() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.setPose(-48.75, -16.125, 90);
    RclMain.setRclPose(chassis.getPose());

    RclMain.updateBotPose();
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
    chassis.moveToPoint(-70, -47, 1400, {.maxSpeed=70}, false);
    pros::delay(250);
    RclMain.updateBotPose(&left_rcl);
    // Score lower mid
    chassis.moveToPoint(-47.5, -47.5, 1200, {.forwards=false}, true);
    pros::delay(600);
    closeGate();
    chassis.turnToPoint(-20, -10, 600, {}, false);
    chassis.moveToPoint(-20, -10, 400, {}, true);
    chassis.moveToPoint(-20, -10, 1200, {.maxSpeed = 50}, true);
    pros::delay(800);
    startOuttake();
    frontMotor.move(-90);
    pros::delay(1000);
    stopIntake();

    // Push
    chassis.moveToPoint(-36, -34, 1100, {.forwards=false}, false);
    chassis.turnToPoint(-22, -34, 800, {.forwards=false}, false);
    retractLeftArm();
    chassis.moveToPoint(-22, -34, 1200, {.forwards=false, .maxSpeed=80}, false);
    chassis.moveToPoint(-22, -34, 9999, {.forwards=false}, false);
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
    jiggle(2, 2800);
    RclMain.updateBotPose(&left_rcl);

    // Back off (#1)
    stopIntake();
    closeGate();
    chassis.moveToPoint(-49, -47,  1000, {.forwards=false}, false);
    chassis.turnToHeading(145, 800, {}, false);

    // Score bottom-right
    chassis.moveToPose(-15, -61, 90, 1500, {.lead=0.4, .minSpeed=55}, false);
    chassis.moveToPoint(20, -61, 2000, {.minSpeed=30, .earlyExitRange=12}, false);
     
    chassis.moveToPoint(40, -61, 1000, {.maxSpeed=60}, false);
    chassis.turnToHeading(0, 600, {});
    pros::delay(200);
    RclMain.updateBotPose(&right_rcl);
    RclMain.updateBotPose(&back_rcl);
    chassis.moveToPoint(38, -47.5, 1200, {}, true);
    chassis.turnToHeading(90, 600, {}, false);

    chassis.moveToPoint(24, -47.5, 1000, {.forwards=false, .maxSpeed=70}, false);
    startTopScore();
    pros::delay(2750);
    
    // Head towards bottom-right match loader
    stopTopScore();
    startIntake();
    openGate();
    chassis.moveToPoint(68, -47, 1200, {.maxSpeed=60}, false);
    jiggle(2, 2800);
    RclMain.updateBotPose(&right_rcl);

    // Score the long goal
    chassis.moveToPoint(24, -47.5, 1100, {.forwards=false, .maxSpeed=70}, false);
    
    startTopScore();
    closeGate();
    RclMain.updateBotPose(&right_rcl);
    pros::delay(3000);

    // Back off (#2)
    chassis.moveToPoint(47, -47.5, 1500, {}, false);
    chassis.turnToHeading(0, 600, {}, false);
    stopTopScore();
    pros::delay(200);
    RclMain.updateBotPose(&back_rcl);

    // Head towards top-right match loader
    chassis.moveToPoint(45, 47, 1300, {.minSpeed=80}, false);
    chassis.moveToPoint(45, 47, 1000, {.maxSpeed=60}, false);
    RclMain.updateBotPose(&right_rcl);
    chassis.turnToHeading(90, 500, {}, false);
    pros::delay(200);
    RclMain.updateBotPose(&left_rcl);
    startIntake();
    openGate();
    chassis.moveToPoint(68, 47, 1200, {.maxSpeed=60}, false); //
    jiggle(2, 2800);
    // Back off (#3)
    stopIntake();
    closeGate();
    chassis.moveToPoint(53, 49, 1000, {.forwards=false}, false);
    RclMain.updateBotPose(&left_rcl);
    chassis.turnToHeading(325, 700, {}, false);

    // Score the top-left goal
    chassis.moveToPose(12, 64, 270, 1500, {.lead=0.5, .minSpeed=55}, false);
    chassis.moveToPoint(-20, 63, 2000, {.minSpeed=40, .earlyExitRange=12}, false);
    chassis.moveToPoint(-38, 63, 1000, {.maxSpeed=60}, false);
    chassis.turnToHeading(180, 800, {});
    RclMain.updateBotPose(&back_rcl);
    RclMain.updateBotPose(&right_rcl);
    chassis.moveToPoint(-38, 47.5, 1200, {}, false);
    chassis.turnToHeading(270, 800, {}, false);
    
    chassis.moveToPoint(-22, 47.5, 1100, {.forwards=false, .maxSpeed=70}, false);
    startTopScore();
    RclMain.updateBotPose(&right_rcl);
    pros::delay(3000);
    
    // Intake from top-left match loader
    stopTopScore();
    startIntake();
    openGate();
    chassis.moveToPoint(-68, 47, 1200, {.maxSpeed=60}, false);
    jiggle(2, 2800);

    // Score the long goal
    chassis.moveToPoint(-20, 47.5, 1500, {.forwards=false, .maxSpeed=70}, false);
    startTopScore();
    closeGate();
    RclMain.updateBotPose(&right_rcl);
    pros::delay(3000);

    // Back off (#4)
    chassis.moveToPoint(-42, 48, 1000, {});
    stopTopScore();
    // Park
    chassis.moveToPoint(-55, 48, 1000);
    chassis.moveToPose(-70, 18, 200, 3500, {.lead = .25}, false);
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
    chassis.moveToPoint(-22, 23, 1000, {.maxSpeed=80}, true);
    pros::delay(500);
    openGate();

    // Score two red
    chassis.turnToPoint(-12, 9, 450,{.forwards=false}, false);
    startOuttake();
    frontMotor.move(-50);
    chassis.moveToPose(-6, 6, 315, 2400, {.forwards=false}, true);
    pros::delay(800);
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
    chassis.turnToPoint(-45, 48, 200, {}, false);
    chassis.moveToPoint(-45, 48, 1100, {}, true);
    pros::delay(700);
    openGate();
    chassis.turnToPoint(-63, 47.5, 550, {}, false);
    startIntake();
    chassis.moveToPoint(-63, 47.5, 1000, {.maxSpeed=65}, false);
    RclMain.updateBotPose(&right_rcl);
    jiggle(3, 2400);
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
    chassis.moveToPoint(38 , 47.5, 700, {}, false);
    chassis.turnToPoint(28, 47.5, 500, {.forwards=false}, false);
    chassis.moveToPoint(28, 47.5, 1000, {.forwards=false, .maxSpeed=80}, true);
    pros::delay(500);
    startTopScore(alliance_color::NONE);
    pros::delay(3000);
    RclMain.updateBotPose(&left_rcl);
    stopTopScore();

    // Refill at top-right loader then score again
    openGate();
    startIntake();
    chassis.moveToPoint(70, 47, 1200, {.maxSpeed=70}, false);
    jiggle(3, 2400);
    chassis.moveToPoint(28, 47.5, 1400, {.forwards=false, .maxSpeed=90}, true);
    pros::delay(400);
    startTopScore(alliance_color::NONE);
    pros::delay(2700);
    RclMain.updateBotPose(&left_rcl);
    stopTopScore();
    closeGate();

    // Clear park zone and get red balls
    chassis.moveToPoint(67, 24, 1400, {}, false);
    odomLift.extend();
    chassis.swingToPoint(180, 700, lemlib::DriveSide::RIGHT, false);
    chassis.turnToHeading(180, 300, {}, false);
    chassis.waitUntilDone();

    startIntake();
    leftMotors.move(127);
    rightMotors.move(127);
    pros::delay(1200);
    chassis.turnToHeading(180, 400, {}, false);
    chassis.moveToPoint(64, -38, 1000, {.minSpeed=75}, true);
    pros::delay(600);
    openGate();
    odomLift.retract();
    pros::delay(400);
    closeGate();

    jiggle(2, 2000, 4.0, 4.0);

    // Reset location
    chassis.turnToHeading(270, 700, {}, false);
    chassis.moveToPoint(chassis.getPose().x-7, chassis.getPose().y, 1200, {}, false);
    chassis.turnToHeading(270, 300, {}, false);
    pros::delay(200);
    RclMain.updateBotPose(&left_rcl);
    RclMain.updateBotPose(&back_rcl);

    // Get one more red ball then score everything in top mid
    startIntake();
    chassis.turnToPoint(30, -21, 500, {}, false);
    chassis.moveToPoint(30, -21, 1000, {.maxSpeed=60}, false);
    chassis.turnToPoint(10.5, -10.5, 700, {.forwards=false}, false);
    openGate();
    startOuttake();
    frontMotor.move(-55);
    chassis.moveToPose(8, -8, 135, 3400, {.forwards=false}, true);
    pros::delay(1300);
    middleMech.retract();
    startMidScore();
    pros::delay(4500); // middle goal score time
    stopMidScore();
    startOuttake();
    pros::delay(400);
    middleMech.extend();
    closeGate();

    // Empty bottom-right loader
    chassis.turnToPoint(45, -50, 200, {}, false);
    chassis.moveToPoint(45, -50, 1100, {}, true);
    pros::delay(700);
    startIntake();
    openGate();
    chassis.turnToPoint(63, -48, 550, {}, false);
    chassis.moveToPoint(63, -48, 1000, {.maxSpeed=65}, false);
    RclMain.updateBotPose(&right_rcl);
    jiggle(3, 2400);
    stopIntake();
    closeGate();

    // Score at bottom-left long goal
    chassis.turnToHeading(45, 200, {}, false);
    chassis.moveToPose(25, -61, 90, 1200, {.forwards=false, .lead=0.5, .minSpeed=50}, false);
    chassis.moveToPoint(-38, -61, 1500, {.forwards=false}, false);
    chassis.turnToHeading(0, 700, {}, false);
    pros::delay(200);
    RclMain.updateBotPose(&left_rcl);
    RclMain.updateBotPose(&back_rcl);
    chassis.moveToPoint(-38 , -47.5, 800, {}, false);
    chassis.turnToPoint(-28, -47.5, 500, {.forwards=false}, false);
    chassis.moveToPoint(-28, -47.5, 1000, {.forwards=false, .maxSpeed=80}, true);
    pros::delay(500);
    startTopScore(alliance_color::NONE);
    pros::delay(3000);
    RclMain.updateBotPose(&left_rcl);
    stopTopScore();

    // Refill at bottom-left loader then score again
    openGate();
    startIntake();
    chassis.moveToPoint(-63,-47, 1200, {.maxSpeed=55}, false);
    jiggle(3, 2400);
    chassis.moveToPoint(-28, -47.5, 1400, {.forwards=false, .maxSpeed=90}, true);
    pros::delay(400);
    startTopScore(alliance_color::NONE);
    pros::delay(2700);
    RclMain.updateBotPose(&left_rcl);
    stopTopScore();
    closeGate();

    // Park
    chassis.moveToPoint(-67, -24, 1400, {}, false);
    odomLift.extend();
    chassis.swingToPoint(0, 700, lemlib::DriveSide::RIGHT, false);
    chassis.turnToHeading(0, 300, {}, false);
    chassis.waitUntilDone();

    startIntake();
    leftMotors.move(127);
    rightMotors.move(127);
    pros::delay(1200);
    leftMotors.move(0);
    rightMotors.move(0);

    chassis.turnToHeading(0, 800, {}, false);
}

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
    chassis.moveToPoint(-22, 23, 1000, {.maxSpeed=80}, true);
    pros::delay(1000);
    openGate();

    // Score two red
    chassis.turnToPoint(-12, 9, 450,{.forwards=false}, false);
    startOuttake();
    frontMotor.move(-50);
    chassis.moveToPose(-6, 6, 315, 2400, {.forwards=false}, true);
    pros::delay(800);
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
    chassis.turnToPoint(-45, 48, 200, {}, false);
    chassis.moveToPoint(-45, 48, 1100, {}, true);
    pros::delay(700);
    openGate();
    chassis.turnToPoint(-63, 47.5, 550, {}, false);
    startIntake();
    chassis.moveToPoint(-63, 47.5, 1000, {.maxSpeed=65}, false);
    RclMain.updateBotPose(&right_rcl);
    jiggle(3, 2400);
    stopIntake();
    closeGate();

    // Score at top-right long goal
    chassis.turnToHeading(225, 200, {}, false);
    chassis.moveToPose(-25, 61, 270, 1200, {.forwards=false, .lead=0.5, .minSpeed=50}, false);
    chassis.moveToPoint(38, 61, 1500, {.forwards=false}, false);
    chassis.turnToHeading(180, 700, {}, false);
    RclMain.updateBotPose(&left_rcl);
    RclMain.updateBotPose(&back_rcl);
    chassis.moveToPoint(38 , 47, 700, {}, false);
    chassis.turnToPoint(28, 47, 500, {.forwards=false}, false);
    chassis.moveToPoint(28, 47, 1000, {.forwards=false, .maxSpeed=80}, true);
    pros::delay(500);
    startTopScore();
    pros::delay(3000);
    RclMain.updateBotPose(&left_rcl);
    stopTopScore();

    // Refill at top-right loader then score again
    openGate();
    startIntake();
    chassis.moveToPoint(63, 46, 1200, {.maxSpeed=55}, false);
    jiggle(3, 2400);
    chassis.moveToPoint(28, 48, 1400, {.forwards=false, .maxSpeed=90}, true);
    pros::delay(400);
    startTopScore();
    pros::delay(2700);
    RclMain.updateBotPose(&left_rcl);
    stopTopScore();
    closeGate();

    chassis.moveToPoint(40, 40, 1400, {}, false);
    chassis.turnToHeading(180, 700, {}, false);
    pros::delay(200);
    RclMain.updateBotPose(&left_rcl);
    RclMain.updateBotPose(&back_rcl);

    // Get one more red ball then score everything in top mid
    startIntake();
    chassis.turnToPoint(24, -22, 1000, {}, false);
    chassis.moveToPoint(24, -22, 4000, {.maxSpeed=80}, false);

    chassis.turnToHeading(0, 1000, {}, false);
    pros::delay(400);
    RclMain.updateBotPose(&right_rcl);
    RclMain.updateBotPose(&back_rcl);

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
    chassis.turnToPoint(45, -50, 200, {}, false);
    chassis.moveToPoint(45, -50, 1100, {}, true);
    pros::delay(700);
    startIntake();
    openGate();
    chassis.turnToPoint(63, -48, 550, {}, false);
    chassis.moveToPoint(63, -48, 1000, {.maxSpeed=65}, false);
    RclMain.updateBotPose(&right_rcl);
    jiggle(3, 2400);
    stopIntake();
    closeGate();

    // Score at bottom-left long goal
    chassis.turnToHeading(45, 200, {}, false);
    chassis.moveToPose(25, -61, 90, 1200, {.forwards=false, .lead=0.5, .minSpeed=50}, false);
    chassis.moveToPoint(-38, -61, 1500, {.forwards=false}, false);
    chassis.turnToHeading(0, 700, {}, false);
    pros::delay(200);
    RclMain.updateBotPose(&left_rcl);
    RclMain.updateBotPose(&back_rcl);
    chassis.moveToPoint(-38 , -47.5, 800, {}, false);
    chassis.turnToPoint(-28, -47.5, 500, {.forwards=false}, false);
    chassis.moveToPoint(-28, -47.5, 1000, {.forwards=false, .maxSpeed=80}, true);
    pros::delay(500);
    startTopScore();
    pros::delay(3000);
    RclMain.updateBotPose(&left_rcl);
    stopTopScore();

    // Refill at bottom-left loader then score again
    openGate();
    startIntake();
    chassis.moveToPoint(-63,-47, 1200, {.maxSpeed=55}, false);
    jiggle(3, 2400);
    chassis.moveToPoint(-28, -47.5, 1400, {.forwards=false, .maxSpeed=90}, true);
    pros::delay(400);
    startTopScore();
    pros::delay(2700);
    RclMain.updateBotPose(&left_rcl);
    stopTopScore();
    closeGate();

    // Park
    chassis.moveToPoint(-67, -24, 1400, {}, false);
    odomLift.extend();
    chassis.swingToPoint(0, 700, lemlib::DriveSide::RIGHT, false);
    chassis.turnToHeading(0, 300, {}, false);
    chassis.waitUntilDone();

    startIntake();
    leftMotors.move(127);
    rightMotors.move(127);
    pros::delay(1200);
    leftMotors.move(0);
    rightMotors.move(0);

    chassis.turnToHeading(0, 800, {}, false);
}