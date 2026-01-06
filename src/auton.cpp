#include "custom/auton.hpp"
#include "custom/RclTracking.hpp"
#include "custom/configs.hpp"
#include "custom/util_funcs.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include <cmath>

void soloAWP(){
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.setPose(-47, 0, 0);
    RclMain.setRclPose(chassis.getPose());

    RclMain.updateBotPose();

    // Push teammate and get their preload
    startIntake();
    chassis.moveToPoint(-47, 40, 500, {.minSpeed=127}, false);

    // Head towards the matchloader and intake
    chassis.moveToPoint(-47, -39, 1600, {.forwards=false, .minSpeed=60, .earlyExitRange=3}, true);
    pros::delay(400);
    openGate();
    chassis.turnToHeading(265, 400, {});
    startIntake();
    //pros::delay(200);
    chassis.moveToPoint(-64, -47, 900, {.maxSpeed=70}, false);
    pros::delay(400);

    // Score the long goal
    chassis.moveToPoint(-24, -46, 800, {.forwards=false}, true);
    pros::delay(350);
    startTopScore();
    closeGate();
    pros::delay(800);
    RclMain.updateBotPose(&left_rcl);
    pros::delay(500);

    // Back off
    
   // chassis.moveToPoint(-40, -47, 600, {.minSpeed=60, .earlyExitRange=3});
    //chassis.turnToHeading(45, 800, {.minSpeed=60, .earlyExitRnge=3});
    
    stopTopScore();
    // Intake 3 balls
    chassis.moveToPoint(-22, -22, 1300, {.maxSpeed=65, .minSpeed=40, .earlyExitRange=4}, true);
    // Intake 3 other balls
    startIntake();
    pros::delay(1200);
    openGate();
    //chassis.turnToHeading(0, 300, {.minSpeed=60, .earlyExitRange=3});
 
    // Intake 3 other balls
    chassis.turnToHeading(0, 300, {}, false);
    pros::delay(200);
    RclMain.updateBotPose(&left_rcl);
    chassis.moveToPoint(-24, 29, 1400, {}, true);
    pros::delay(40);
    closeGate();
    pros::delay(800);
    openGate();
    // Score the mid goal
    pros::delay(500);
    chassis.moveToPoint(-24, 24, 500, {.forwards=false}, true);
    stopIntake();
    //  chassis.turnToPoint(-21, 15,700,  {.forwards = false});    
    topMotor.move(127);
    frontMotor.move(-25);
    /*frontMotor.move(0);
    topMotor.move(0);*/
    //chassis.turnToHeading(315, 800, {}, false);
   
   // chassis.turnToHeading(330, 400, {}, false);
    //chassis.moveToPoint(-18, 9, 1000, {.forwards=false, .maxSpeed=70}, false);
    chassis.turnToPoint(pivot_x(-10), pivot_y(10), 400, {}, false);
    chassis.moveToPoint(-10, 10, 1500, {.forwards=false, .maxSpeed=70}, true);
    pros::delay(500);
    //startTopScore();
    startMidScore();
    chassis.waitUntilDone();
    pros::delay(700); // middle goal score time
    middleMech.extend();
    stopMidScore();
    stopTopScore();
    startIntake();
    //stopTopScore();
    
    // Move towards long goal and score
    chassis.turnToPoint(-48, 48, 200, {}, false);
    chassis.moveToPoint(-48, 48, 1200, {}, false);
    openGate();
    //closeGate();
    chassis.turnToPoint(-68, 48, 500, {}, false);
    /*chassis.moveToPoint(-27, 51, 1100, {.forwards=false, .maxSpeed=80}, false);
    startTopScore();
    pros::delay(1000);
    stopTopScore();*/

    // Refill at matchloader
    startIntake();
    //RclMain.updateBotPose();
    chassis.moveToPoint(-68, 48, 900, {.maxSpeed=70}, false);
    pros::delay(200);
    RclMain.updateBotPose(&right_rcl);

    // Score again
    chassis.moveToPoint(-25, 48, 900, {.forwards = false, .maxSpeed=100}, true);
    pros::delay(400);
    //closeGate();
    startTopScore(allianceColor);
}
// Three balls -> Two balls -> Match loader -> Long goal -> Push
void right(){
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.setPose(-48.75, -16.125, 90);
    RclMain.setRclPose(chassis.getPose());

    // Intake three balls
    stopIntake();
    startIntake();
    //topMotor.move_velocity(50);
    chassis.moveToPoint(-19, -24, 1000, {}, true);
    pros::delay(300);
    openGate();

    // Intake two balls
    chassis.turnToPoint(-8, -44, 350, {}, true);
    closeGate();
    chassis.moveToPoint(-8, -44, 1300, {.maxSpeed=70}, false);
    chassis.moveToPoint(-19, -19, 850, {.forwards=false}, false);

    // Move towards long goal and score
    chassis.turnToPoint(-42, -46, 350, {}, false);
    chassis.moveToPoint(-42, -46, 1100, {}, false);
    openGate();
    chassis.turnToPoint(-68, -47, 450, {}, false);
    /*chassis.moveToPoint(-27, 51, 1100, {.forwards=false, .maxSpeed=80}, false);
    startTopScore();
    pros::delay(1000);
    stopTopScore();	*/

    // Refill at matchloader
    startIntake();
    openGate();
    chassis.moveToPoint(-68, -47, 900, {.maxSpeed=70, .minSpeed=50}, false);
    pros::delay(400);
    pros::delay(400);

    // Score again
    chassis.moveToPoint(-25, -47, 1100, {.forwards=false, .maxSpeed=100}, true);
    pros::delay(500);
    //closeGate();
    startTopScore(allianceColor);
    pros::delay(2900);
    stopTopScore();

    // Push
    closeGate();
    chassis.moveToPoint(-46, -35.5, 1200, {.minSpeed=60, .earlyExitRange=1}, false);
    chassis.turnToPoint(-16, -35.5, 500, {.forwards=false}, false);
    retractLeftArm();
    chassis.moveToPoint(-16, -35.5, 1000, {.forwards=false, .maxSpeed=60}, false);
    chassis.moveToPoint(-16, -35.5, 9999, {.forwards=false}, false);
}
// Three balls -> Two balls -> Top mid -> Match loader -> Long goal -> Push
void left() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.setPose(-48.75, 16.125, 90);
    RclMain.setRclPose(chassis.getPose());

    // Intake three balls
    stopIntake();
    startIntake();
    //topMotor.move_velocity(50);
    chassis.moveToPoint(-19, 24, 1000, {}, true);
    pros::delay(300);
    openGate();

    // Intake two balls
    chassis.turnToPoint(-8, 44, 350, {}, true);
    closeGate();
    chassis.moveToPoint(-8, 44, 1300, {.maxSpeed=70}, false);
    // Backoff and score middle goal
    chassis.moveToPoint(-18, 18, 1100, {.forwards=false}, false);
    stopIntake();
    openGate();
    chassis.turnToPoint(pivot_x(-10.5), pivot_y(10.5), 400, {}, false);
    startOuttake();
    frontMotor.move(-25);
    middleMech.retract();
    chassis.moveToPoint(-10.5, 10.5, 1000, {.forwards=false, .maxSpeed = 70}, true);
    pros::delay(450);
    startMidScore();
    closeGate();
    pros::delay(1150); // middle goal score time
    stopMidScore();
    middleMech.extend();

    // Move towards long goal and score
    chassis.turnToPoint(-42, 49, 250, {}, false);
    chassis.moveToPoint(-42, 50, 1300, {});
    openGate();
    chassis.turnToPoint(-68, 49, 400, {}, false);
    /*chassis.moveToPoint(-27, 51, 1100, {.forwards=false, .maxSpeed=80}, false);
    startTopScore();
    pros::delay(1000);
    stopTopScore();*/

    // Refill at matchloader
    startIntake();
    openGate();
    chassis.moveToPoint(-68, 50, 900, {.maxSpeed=70, .minSpeed=50}, false);
    pros::delay(300);

    // Score again
    chassis.moveToPoint(-25, 48, 1100, {.forwards = false}, true);
    pros::delay(500);
    //closeGate();
    startTopScore(allianceColor);
    pros::delay(2300);
    stopTopScore();

    // Push
    closeGate();
    chassis.moveToPoint(-46, 58, 1200, {.minSpeed = 60, .earlyExitRange = 1}, false);
    chassis.turnToPoint(-16, 58, 500, {.forwards=false}, false);
    retractLeftArm();
    chassis.moveToPoint(-16, 58, 1200, {.forwards=false, .maxSpeed=60}, false);
    chassis.moveToPoint(-16, 58, 9999, {.forwards=false}, false);
    chassis.turnToHeading(-105, 700);
}
// Three balls -> Match loader -> Long goal -> Push
void leftControlRush() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.setPose(-48.75, 16.125, 90);
    RclMain.setRclPose(chassis.getPose());

    RclMain.updateBotPose();

    // Intake three balls
    startIntake();
    //topMotor.move_velocity(50);
    chassis.moveToPoint(-24, 24, 1500, {.minSpeed=50, .earlyExitRange=3}, true);
    pros::delay(450);
    openGate();

    // Head to matchloader
    chassis.turnToPoint(-52, 48, 600, {}, false);
    chassis.moveToPoint(-52, 46, 1100, {}, false);
    chassis.turnToPoint(-67, 48, 300, {}, false);
    chassis.moveToPoint(-67, 48, 700, {.maxSpeed=80}, false);
    pros::delay(300);
    //RclMain.updateBotPose(&right_rcl);

    // Move towards long goal sand score
    chassis.turnToPoint(pivot_x(-24), pivot_y(47), 200, {}, false);
    chassis.moveToPoint(-24, 47, 1300, {.forwards=false, .maxSpeed=100}, true);
    pros::delay(500);
    startTopScore(allianceColor);
    pros::delay(2650);
    stopTopScore();

    // Push
    closeGate();
    chassis.moveToPoint(-46, 61, 1200, {.minSpeed = 60, .earlyExitRange = 1}, false);
    chassis.turnToPoint(pivot_x(-16), pivot_y(61), 500, {}, false);
    retractLeftArm();
    chassis.moveToPoint(-16, 61, 1000, {.forwards=false, .maxSpeed=60}, false);
    chassis.moveToPoint(-16, 61, 9999, {.forwards=false}, false);

    //temporary
    //startTopScore();
}
// Match loader -> Long goal -> Push
void leftFastRush() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.setPose(-47, 13.25, 0);
    RclMain.setRclPose(chassis.getPose());

    RclMain.updateBotPose();

    // Head towards the matchloader and intake
    openGate();
    chassis.moveToPoint(-47, 39, 1200, {.minSpeed=60, .earlyExitRange=3}, false);
    chassis.turnToHeading(275, 400, {});
    startIntake();
    //pros::delay(200);
    chassis.moveToPoint(-63.4, 48, 900, {.maxSpeed=70}, false);
    pros::delay(400);

    // Score the long goal
    chassis.moveToPoint(-24, 48, 800, {.forwards=false}, true);
    pros::delay(350);
    startTopScore();
    closeGate();
    pros::delay(850);
    RclMain.updateBotPose(&right_rcl);
    pros::delay(500);

    // Push
    closeGate();
    chassis.moveToPoint(-46, 59, 1200, {.minSpeed = 60, .earlyExitRange = 1}, false);
    chassis.turnToPoint(pivot_x(-16), pivot_y(58), 500, {}, false);
    retractLeftArm();
    chassis.moveToPoint(-16, 58, 1000, {.forwards=false, .maxSpeed=60}, false);
    chassis.moveToPoint(-16, 58, 9999, {.forwards=false}, false);
}
// Three balls -> Match loader -> Long goal -> Push
void rightControlRush() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.setPose(-48.75, -16.125, 90);
    RclMain.setRclPose(chassis.getPose());

    RclMain.updateBotPose();

    // Intake three balls
    startIntake();
    //topMotor.move_velocity(50);
    chassis.moveToPoint(-24, -24, 1500, {.minSpeed=50, .earlyExitRange=3}, true);
    pros::delay(450);
    openGate();

    // Head to matchloader
    chassis.turnToPoint(-52, -48, 600, {}, false);
    chassis.moveToPoint(-52, -46, 1100, {}, false);
    chassis.turnToPoint(-67, -48, 300, {}, false);
    chassis.moveToPoint(-67, -48, 700, {.maxSpeed=60}, false);
    pros::delay(300);
    //RclMain.updateBotPose(&left_rcl);

    // Move towards long goal and score
    chassis.turnToPoint(pivot_x(-24), pivot_y(-47), 300, {}, false);
    chassis.moveToPoint(-24, -47, 1300, {.forwards=false, .maxSpeed=100}, true);
    pros::delay(500);
    startTopScore(allianceColor);
    pros::delay(2650);
    stopTopScore();

    // Push
    closeGate();
    chassis.moveToPoint(-46, -35.25, 1200, {.minSpeed=60, .earlyExitRange=1}, false);
    chassis.turnToPoint(-15, -35.25, 500, {.forwards=false}, false);
    retractLeftArm();
    chassis.moveToPoint(-15, -35.25, 1000, {.forwards=false, .maxSpeed=60}, false);
    chassis.moveToPoint(-15, -35.25, 9999, {.forwards=false}, false);
    chassis.turnToHeading(256, 700);
    //temporary
    //startTopScore();
}
// Match loader -> Long goal -> Push
void rightFastRush() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.setPose(-47, -13.25, 180);
    RclMain.setRclPose(chassis.getPose());

    RclMain.updateBotPose();

    // Head towards the matchloader and intake
    openGate();
    chassis.moveToPoint(-47, -39, 1200, {.minSpeed=60, .earlyExitRange=3}, false);
    chassis.turnToHeading(265, 400, {});
    startIntake();
    //pros::delay(200);
    chassis.moveToPoint(-63.4, -48, 900, {.maxSpeed=70}, false);
    pros::delay(400);

    // Score the long goal
    chassis.moveToPoint(-24, -48, 800, {.forwards=false}, true);
    pros::delay(350);
    startTopScore();
    closeGate();
    pros::delay(850);
    RclMain.updateBotPose(&left_rcl);
    pros::delay(500);

    // Push
    closeGate();
    chassis.moveToPoint(-46, -37, 1200, {.minSpeed = 60, .earlyExitRange = 1}, false);
    chassis.turnToPoint(pivot_x(-16), pivot_y(-58), 500, {}, false);
    retractLeftArm();
    chassis.moveToPoint(-16, -38, 1000, {.forwards=false, .maxSpeed=60}, false);
    chassis.moveToPoint(-16, -38, 9999, {.forwards=false}, false);
}
// Three balls -> Two balls -> Long goal -> Match loader -> Top mid -> Push
void leftv2() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.setPose(-48.75, 16.125, 90);
    RclMain.setRclPose(chassis.getPose());

    RclMain.updateBotPose();

    // Intake three balls
    stopIntake();
    startIntake();
    chassis.moveToPoint(-19, 24, 1000, {}, true);
    pros::delay(300);
    openGate();

    // Intake two balls
    chassis.turnToPoint(-6, 44, 350, {}, true);
    closeGate();
    chassis.moveToPoint(-6, 44, 1300, {.maxSpeed=70}, false);

    // Head to long goal
    chassis.moveToPose(-40, 44, 140, 2000, {.forwards=false, .minSpeed=100}, false);
    chassis.swingToHeading(270, lemlib::DriveSide::LEFT, 1500, {.direction = lemlib::AngularDirection::CW_CLOCKWISE, .minSpeed=127}, false);
    chassis.moveToPoint(-24, 48, 1200, {.forwards=false, .maxSpeed=100}, true);
    // Score
    pros::delay(500);
    startTopScore();
    closeGate();
    pros::delay(1050);
    RclMain.updateBotPose(&right_rcl);
    pros::delay(500);
    stopTopScore();

    // Refill at match loader
    startIntake();
    openGate();

    chassis.moveToPoint(-63, 47.5, 1400, {.maxSpeed=60, .minSpeed=50}, false);
    
    //closeGate();
    pros::delay(800);
    // Score top mid
    chassis.turnToPoint(-10, 10, 600, {.forwards=false}, false);
    chassis.moveToPoint(-10, 10, 400, {.forwards=false}, true);
    chassis.moveToPoint(-10, 10, 1200, {.forwards=false, .maxSpeed = 80}, true);
    startOuttake();
    topMotor.move(-80);
    frontMotor.move(-35);
    middleMech.retract();
    pros::delay(400);
    stopIntake();
    pros::delay(1000);
    startMidScore();
    closeGate();
    pros::delay(1200);
    stopMidScore();

    // Push
    chassis.moveToPoint(-36, 37, 1200, {}, false);
    chassis.turnToPoint(-10, 37, 700, {}, false);
    retractLeftArm();
    chassis.moveToPoint(-10, 37, 1300, {.maxSpeed=60}, false);
    chassis.moveToPoint(-10, 37, 9999, {}, false);
    chassis.turnToHeading(135, 700, {.minSpeed= 120});
}
void rightv2() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.setPose(-48.75, -16.125, 90);
    RclMain.setRclPose(chassis.getPose());

    RclMain.updateBotPose();

    // Intake three balls
    stopIntake();
    startIntake();
    chassis.moveToPoint(-19, -24, 1000, {}, true);
    pros::delay(300);
    openGate();

    // Intake two balls
    chassis.turnToPoint(-6, -44, 350, {}, true);
    closeGate();
    chassis.moveToPoint(-6, -44, 1300, {.maxSpeed=70}, false);

    // Head to long goal
    chassis.moveToPose(-40, -44, 40, 2000, {.forwards=false, .minSpeed=100}, false);
    chassis.swingToHeading(270, lemlib::DriveSide::RIGHT, 1500, {.direction = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .minSpeed=127}, false);
    chassis.moveToPoint(-24, -48, 1200, {.forwards=false, .maxSpeed=100}, true);
    // Score
    pros::delay(500);
    startTopScore();
    closeGate();
    pros::delay(1050);
    RclMain.updateBotPose(&left_rcl);
    pros::delay(500);
    stopTopScore();

    // Refill at match loader
    startIntake();
    openGate();

    chassis.moveToPoint(-63, -47.5, 1400, {.maxSpeed=60, .minSpeed=50}, false);
    
    //closeGate();
    pros::delay(800);
    // Score lower mid
    chassis.moveToPoint(-47.5, -47.5, 900, {.forwards=false, .minSpeed=60, .earlyExitRange=4}, true);
    pros::delay(600);
    closeGate();
    chassis.turnToPoint(-10, -10, 600, {}, false);
    chassis.moveToPoint(-10, -10, 400, {}, true);
    chassis.moveToPoint(-10, -10, 1200, {.maxSpeed = 80}, true);
    pros::delay(800);
    startOuttake();
    frontMotor.move(-25);
    pros::delay(1000);
    stopIntake();

    // Push
    chassis.moveToPoint(-36, -37, 1200, {.forwards=false}, false);
    chassis.turnToPoint(-10, -37, 700, {.forwards=false}, false);
    retractLeftArm();
    chassis.moveToPoint(-10, -37, 1300, {.forwards=false, .maxSpeed=60}, false);
    chassis.moveToPoint(-10, -37, 9999, {.forwards=false}, false);
    chassis.turnToHeading(225, 700, {.minSpeed= 120});
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
    //MclMain.set_pose(chassis.getPose().x, chassis.getPose().y, chassis.getPose().theta);
    //MclMain.startTracking();

    // Intake three balls
    stopIntake();
    startIntake();
    chassis.moveToPoint(-18, 26, 2000, {.maxSpeed=70}, true);
    pros::delay(1000);
    openGate();

    // Score two red
    chassis.turnToPoint(-9, 9, 800,{.forwards=false}, false);
    startOuttake();
    frontMotor.move(-25);
    middleMech.retract();
    chassis.moveToPose(-6, 6, 315, 3400, {.forwards=false}, true);
    pros::delay(1400);
    stopIntake();
    pros::delay(800);
    startMidScore();
    closeGate();
    pros::delay(1400); // middle goal score time
    stopMidScore();
    startOuttake();
    pros::delay(400);
    middleMech.extend();

    // Empty top-left loader
    chassis.moveToPoint(-47, 49.5, 2000, {}, false);
    openGate();
    chassis.turnToPoint(-63, 47, 600, {}, false);
    startIntake();
    chassis.moveToPoint(-70, 47, 3000, {.maxSpeed=50}, false);
    RclMain.updateBotPose(&right_rcl);
    stopIntake();
    closeGate();

    // Score at top-right long goal
    chassis.turnToHeading(225, 200, {}, false);
    chassis.moveToPose(-25, 61, 270, 1200, {.forwards=false, .lead=0.5, .minSpeed=50}, false);
    chassis.moveToPoint(38, 61, 1500, {.forwards=false}, false);
    chassis.turnToHeading(180, 600, {}, false);
    pros::delay(400);
    chassis.moveToPoint(38 , 47.5, 1200, {}, false);
    chassis.turnToPoint(28, 47.5, 600, {.forwards=false}, false);
    chassis.moveToPoint(28, 47.5, 1000, {.forwards=false, .maxSpeed=70}, false);
    startTopScore();
    pros::delay(3000);
    stopTopScore();

    // Refill at top-right loader then score again
    openGate();
    startIntake();
    chassis.moveToPoint(72, 46, 5000, {.maxSpeed=60}, false);
    chassis.moveToPoint(28, 47.5, 2000, {.forwards=false, .maxSpeed=80}, false);
    startTopScore();
    pros::delay(3000);
    stopTopScore();
    closeGate();

    // Clear park zone and get red balls
    chassis.moveToPoint(67, 28, 2000, {}, false);
    chassis.swingToPoint(180, 800, lemlib::DriveSide::RIGHT, false);
    odomLift.extend();
    startIntake();
    chassis.moveToPoint(67, -36, 3000, {.maxSpeed=80}, false);
    odomLift.retract();

    // Reset location
    chassis.turnToHeading(270, 600, {}, false);
    chassis.moveToPoint(chassis.getPose().x-7, chassis.getPose().y, 1200, {}, false);
    pros::delay(1000);
    RclMain.updateBotPose(&back_rcl);
    RclMain.updateBotPose(&left_rcl);

    // Get one more red ball then score everything in top mid
    chassis.turnToPoint(31.5, -21, 500, {}, false);
    chassis.moveToPoint(31.5, -21, 1200, {.maxSpeed=60}, false);
    chassis.turnToPoint(10.5, -10.5, 800, {.forwards=false}, false);
    openGate();
    startOuttake();
    frontMotor.move(-25);
    middleMech.retract();
    chassis.moveToPose(6, -6, 135, 3400, {.forwards=false}, true);
    pros::delay(1400);
    stopIntake();
    pros::delay(800);
    startMidScore();
    closeGate();
    pros::delay(1400); // middle goal score time
    stopMidScore();
    startOuttake();
    pros::delay(400);
    middleMech.extend();

    // Empty bottom-right loader
    chassis.moveToPoint(47, -47, 1500, {}, false);
    startIntake();
    openGate();
    chassis.turnToPoint(63, -47, 600, {}, false);
    chassis.moveToPoint(63, -47, 1200, {.maxSpeed=60}, false);

    // Score at bottom-left long goal
    chassis.turnToHeading(45, 200, {}, false);
    chassis.moveToPose(25, -61, 90, 1200, {.forwards=false, .lead=0.5, .minSpeed=50}, false);
    chassis.moveToPoint(-38, -61, 1500, {.forwards=false}, false);
    chassis.turnToHeading(0, 600, {}, false);
    pros::delay(400);
    chassis.moveToPoint(-38 , -47, 1200, {}, false);
    chassis.turnToPoint(-28, -47, 600, {.forwards=false}, false);
    chassis.moveToPoint(-28, -47, 1000, {.forwards=false, .maxSpeed=70}, false);
    startTopScore();
    pros::delay(3000);
    stopTopScore();

    // Refill at bottom-left loader then score again
    openGate();
    startIntake();
    chassis.moveToPoint(-66, -46, 3000, {.maxSpeed=60}, false);
    pros::delay(1000);
    chassis.moveToPoint(-28, -47.5, 2000, {.forwards=false, .maxSpeed=80}, false);
    startTopScore();
    pros::delay(3000);
    stopTopScore();
    closeGate();

    // Park
    chassis.moveToPoint(-60, -35, 2000, {}, false);
    chassis.turnToHeading(0, 800, {}, false);
    chassis.moveToPose(-67, -29, 180, 1400, {.lead=0.4}, false);
    odomLift.extend();
    startIntake();
    chassis.moveToPoint(-67, 0, 2000, {.maxSpeed=90}, true);
}