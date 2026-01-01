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
    chassis.setPose(-47, -13.25, 180);
    RclMain.setRclPose(chassis.getPose());

    RclMain.updateBotPose();

    // Head towards the matchloader and intake
    openGate();
    chassis.moveToPoint(-47, -39, 1200, {.minSpeed=60, .earlyExitRange=3}, false);
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
    chassis.moveToPose(-21, 38, 140, 2000, {.forwards=false, .minSpeed=50}, false);
    chassis.swingToHeading(270, lemlib::DriveSide::LEFT, 1000, {.minSpeed=60}, false);

    // Score
    startTopScore();
    closeGate();
    pros::delay(1200);
    RclMain.updateBotPose(&right_rcl);
    pros::delay(500);
    stopTopScore();

    // Refill at match loader
    startIntake();
    openGate();
    chassis.moveToPoint(-63, 50, 1400, {.maxSpeed=70, .minSpeed=50}, false);
    pros::delay(400);

    // Score top mid
    chassis.turnToPoint(-13, 11.5, 600, {.forwards=false}, false);
    chassis.moveToPoint(-13, 11.5, 1400, {.forwards=false}, true);
    startOuttake();
    frontMotor.move(-25);
    middleMech.retract();
    pros::delay(400);
    stopIntake();
    pros::delay(1000);
    startMidScore();
    closeGate();
    pros::delay(800);
    stopMidScore();

    // Push
    chassis.moveToPoint(-36, 36, 1200, {}, false);
    chassis.turnToPoint(-16, 37, 700, {}, false);
    retractLeftArm();
    chassis.moveToPoint(-16, 37, 1200, {.maxSpeed=60}, false);
    chassis.moveToPoint(-16, 37, 9999, {}, false);
    chassis.turnToHeading(105, 700);
}

void skills() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.setPose(-47, -13.25, 180);
    RclMain.setRclPose(chassis.getPose());

    RclMain.updateBotPose();

    // Head towards bottom-left match loader
    chassis.moveToPoint(-47, -46, 1800, {}, false);
    chassis.turnToHeading(270, 700, {});
    startIntake();
    openGate();
    chassis.moveToPoint(-68.5, -46, 1200, {.maxSpeed=60}, false);
    chassis.moveToPoint(-61.5, -46, 300, {.forwards=false, .maxSpeed=60}, false);
    chassis.moveToPoint(-68.5, -46, 1000, {.maxSpeed=60}, false);
    RclMain.updateBotPose(&left_rcl);
    pros::delay(1000);

    // Back off (#1)
    stopIntake();
    closeGate();
    //chassis.moveToPoint(-38, -48, 1000, {.forwards=false});
    //chassis.turnToHeading(180, 800, {}, false);
    chassis.moveToPoint(-49, -48,  1000, {.forwards=false}, false);
    chassis.turnToHeading(145, 800, {}, false);

    // Score bottom-right
    //chassis.moveToPoint(-38, -62, 1200, {}, false);
    //chassis.turnToHeading(90, 800, {});
    chassis.moveToPose(-15, -61, 90, 1500, {.lead=0.4, .minSpeed=55}, false);
    chassis.moveToPoint(20, -61, 2000, {.minSpeed=30, .earlyExitRange=12}, false);
     
    chassis.moveToPoint(40, -61, 1000, {.maxSpeed=60}, false);
    chassis.turnToHeading(0, 600, {});
    chassis.moveToPoint(38, -48, 1200, {}, true);
    RclMain.updateBotPose(&right_rcl);
    pros::delay(150);
    RclMain.updateBotPose(&back_rcl);
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

    // Score the long goal
    chassis.moveToPoint(24, -48, 1100, {.forwards=false, .maxSpeed=70}, false);
    
    startTopScore();
    closeGate();
    RclMain.updateBotPose(&right_rcl);
    pros::delay(3000);

    // Back off (#2)
    chassis.moveToPoint(47, -48, 1500, {}, false);
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
    chassis.turnToHeading(325, 700, {}, false);

    // Score the top-left goal
    //chassis.moveToPoint(38, 56, 1200, {}, false);
    //chassis.turnToHeading(270, 800, {});
    chassis.moveToPose(15, 62.5, 270, 1500, {.lead=0.4, .minSpeed=55}, false);
    chassis.moveToPoint(-20, 61.25, 2000, {.minSpeed=40, .earlyExitRange=12}, false);
    chassis.moveToPoint(-38, 61.25, 1000, {.maxSpeed=60}, false);
    chassis.turnToHeading(180, 800, {});
    RclMain.updateBotPose(&back_rcl);
    RclMain.updateBotPose(&right_rcl);
    chassis.moveToPoint(-38, 48, 1200, {}, false);
    chassis.turnToHeading(270, 800, {}, false);
    
    chassis.moveToPoint(-22, 48, 1100, {.forwards=false, .maxSpeed=70}, false);
    startTopScore();
    RclMain.updateBotPose(&right_rcl);
    pros::delay(3000);
    
    // Intake from top-left match loader
    stopTopScore();
    startIntake();
    openGate();
    chassis.moveToPoint(-68, 48, 1200, {.maxSpeed=60}, false);
    chassis.moveToPoint(-61.5, 47, 400, {.forwards=false, .maxSpeed=60}, false);
    chassis.moveToPoint(-78, 48, 1300, {.maxSpeed=60}, false);     
    pros::delay(2000);

    // Score the long goal
    chassis.moveToPoint(-20, 48, 1500, {.forwards=false, .maxSpeed=70}, false);
    startTopScore();
    closeGate();
    RclMain.updateBotPose(&right_rcl);
    pros::delay(3000);

    // Back off (#4)
    chassis.moveToPoint(-42, 48, 1000, {});
    stopTopScore();
    // Park
    chassis.moveToPose(-66, 18, 200, 2800, {.lead = .25}, false);
    //chassis.moveToPose(-64, 18, 180, 2500, {.maxSpeed = 100}, false);
    odomLift.extend();
    startOuttake();
    pros::delay(250);
    leftMotors.move(127);
    rightMotors.move(127);
    pros::delay(800);
    leftMotors.move(0);
    rightMotors.move(0);
}
// 90+ points
void skills_v2() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.setPose(-63, -18, 0);
    RclMain.setRclPose(chassis.getPose());

    // Back off to gain momentum
    chassis.moveToPoint(-63, -36, 1500, {.forwards=false}, false);
    RclMain.updateBotPose();

    // Clear red parking zone
    startIntake();
    odomLift.extend(); // Entering parking zone, lift up odom
    chassis.moveToPoint(-64, 0, 1400, {.minSpeed=70, .earlyExitRange=3}, false);
    chassis.moveToPoint(-64, 24, 600, {.maxSpeed=100, .minSpeed=60, .earlyExitRange=3}, true);
    pros::delay(600);
    openGate();
    pros::delay(300);
    RclMain.updateBotPose(&left_rcl); // Position correction
    chassis.turnToHeading(90, 600, {}, false);
    odomLift.retract(); // Leaving parking zone, resume odom
    closeGate();
    chassis.moveToPoint(chassis.getPose().x+20, chassis.getPose().y, 1200, {}, false);
    pros::delay(200);
    RclMain.updateBotPose(&left_rcl); // Position correction
    RclMain.updateBotPose(&back_rcl); // Position correction
    
    // Score Top Mid Goal
    chassis.turnToPoint(-29, 29, 600, {}, false);
    chassis.moveToPoint(-29, 29, 1300, {.maxSpeed=50}, false);
    chassis.turnToPoint(pivot_x(-11), pivot_y(11), 800, {}, false);
    stopIntake();
    chassis.moveToPoint(-11, 11, 2000, {.forwards=false, .maxSpeed=60}, true);
    openGate();
    pros::delay(600);
    startOuttake();
    frontMotor.move(-25);
    pros::delay(800);
    startMidScore();
    pros::delay(1600);
    stopMidScore();
    closeGate();

    // Intake from Top-Left Match Loader
    chassis.moveToPoint(-47, 47, 1300, {.minSpeed=35, .earlyExitRange=5}, false);
    openGate();
    chassis.turnToPoint(-64, 47, 600, {}, false);
    startIntake();
    chassis.moveToPoint(-64, 47, 1000, {.maxSpeed=80}, false);
    pros::delay(1200);
    RclMain.updateBotPose(&right_rcl); // Position correction

    // Back off and head toward Top-Right Long Goal
    chassis.moveToPoint(-49, 47, 1000, {.forwards=false}, false);
    closeGate();
    stopIntake();
    
    chassis.turnToHeading(25, 800, {}, false);
    chassis.moveToPose(-15, 59, 90, 1500, {.lead=0.4, .minSpeed=55}, false);
    chassis.moveToPoint(20, 59, 1200, {.minSpeed=30, .earlyExitRange=12}, false);
    chassis.moveToPoint(38, 59, 1000, {.maxSpeed=60}, false);
    
    chassis.turnToHeading(180, 800, {}, false);
    pros::delay(400);
    RclMain.updateBotPose(&left_rcl); // Position correction
    RclMain.updateBotPose(&back_rcl); // Position correction
    chassis.moveToPoint(38, 47, 700, {}, false);
    chassis.turnToPoint(pivot_x(24), pivot_y(47), 600, {}, false);
    chassis.moveToPoint(24, 47, 800, {.forwards=false, .minSpeed=40, .earlyExitRange=4}, false);
    startTopScore();
    pros::delay(1500);
    stopTopScore();

    // Refill at Top-Right Match Loader, then score Top-Right Long Goal again
    openGate();
    startIntake();
    chassis.moveToPoint(50, 47, 1000, {.minSpeed=40, .earlyExitRange=4}, false);
    chassis.moveToPoint(64, 47, 800, {.maxSpeed=60}, false);
    pros::delay(1200);
    RclMain.updateBotPose(&left_rcl); // Position correction

    chassis.moveToPoint(24, 47, 1200, {.forwards=false, .minSpeed=60, .earlyExitRange=6}, true);
    pros::delay(1000);
    startTopScore();
    pros::delay(1500);
    stopTopScore();
    closeGate();

    chassis.moveToPoint(38, 47, 800, {}, false);
    chassis.turnToPoint(38, -47, 600, {}, false);
    pros::delay(200);
    RclMain.updateBotPose(&back_rcl);
    chassis.moveToPoint(38, -47, 3000, {}, false);

    /*// Clear Blue Parking Zone
    chassis.moveToPoint(38, 47, 800, {}, false);
    chassis.turnToHeading(130, 400, {}, false);
    chassis.moveToPose(63, 30, 180, 2000, {}, false);

    startIntake();
    odomLift.extend(); // Entering parking zone, lift up odom
    chassis.moveToPoint(64, 0, 1400, {.minSpeed=70, .earlyExitRange=3}, false);
    chassis.moveToPoint(64, -24, 1200, {.maxSpeed=100, .minSpeed=60, .earlyExitRange=3}, true);
    pros::delay(600);
    openGate();
    pros::delay(600);
    RclMain.updateBotPose(&left_rcl); // Position correction
    odomLift.retract(); // Leaving parking zone, resume odom
    chassis.turnToHeading(270, 600, {}, false);
    chassis.moveToPoint(chassis.getPose().x-20, chassis.getPose().y, 1200, {}, false);
    pros::delay(200);
    RclMain.updateBotPose(&left_rcl); // Position correction
    RclMain.updateBotPose(&back_rcl); // Position correction
    closeGate();
    odomLift.retract(); // Leaving parking zone, resume odom
    chassis.turnToHeading(180, 800, {}, false);
    pros::delay(400);
    RclMain.updateBotPose(&left_rcl); // Position correction
    RclMain.updateBotPose(&back_rcl); // Position correction*/

    // Score at Bottom-Right Long Goal
    //chassis.moveToPoint(38, -35, 1200, {.minSpeed=60, .earlyExitRange=10}, false);
    //chassis.moveToPoint(38, -47, 800, {.maxSpeed=60}, false);
    chassis.turnToPoint(pivot_x(24), pivot_y(-47), 600, {}, false);
    chassis.moveToPoint(24, -47, 800, {.minSpeed=60, .earlyExitRange=6}, false);
    startTopScore();
    pros::delay(1500);
    stopTopScore();

    // Refill at Bottom-Right Match Loader
    openGate();
    startIntake();
    chassis.moveToPoint(50, -47, 1000, {.minSpeed=40, .earlyExitRange=4}, false);
    chassis.moveToPoint(64, -47, 800, {.maxSpeed=60}, false);
    pros::delay(1200);
    RclMain.updateBotPose(&right_rcl); // Position correction

    // Back off and head toward Bottom-Left Long Goal
    chassis.moveToPoint(49, -47, 1000, {.forwards=false}, false);
    closeGate();
    stopIntake();
    
    chassis.turnToHeading(215, 800, {}, false);
    chassis.moveToPose(15, -59, 270, 1500, {.lead=0.3, .minSpeed=55}, false);
    chassis.moveToPoint(-20, -59, 1200, {.minSpeed=30, .earlyExitRange=12}, false);
    chassis.moveToPoint(-38, -59, 1000, {.maxSpeed=60}, false);
    chassis.turnToHeading(0, 600, {}, false);
    pros::delay(400);
    RclMain.updateBotPose(&left_rcl); // Position correction
    RclMain.updateBotPose(&back_rcl); // Position correction
    chassis.moveToPoint(-38, -47, 700, {}, false);

    chassis.turnToPoint(pivot_x(-24), pivot_y(-47), 600, {}, false);
    chassis.moveToPoint(-24, -47, 800, {.forwards=false, .minSpeed=40, .earlyExitRange=4}, false);
    startTopScore();
    pros::delay(1500);
    stopTopScore();

    // Refill at Bottom-Left Match Loader, then score Bottom-Left Long Goal again
    openGate();
    startIntake();
    chassis.moveToPoint(-50, -47, 1000, {.minSpeed=40, .earlyExitRange=4}, false);
    chassis.moveToPoint(-64, -47, 800, {.maxSpeed=60}, false);
    pros::delay(1200);
    RclMain.updateBotPose(&left_rcl); // Position correction

    chassis.moveToPoint(-24, -47, 1200, {.forwards=false, .minSpeed=60, .earlyExitRange=6}, true);
    pros::delay(1000);
    startTopScore();
    pros::delay(1500);
    stopTopScore();
    closeGate();

    // Park
    chassis.moveToPoint(-49, -47, 1000, {}, false);
    chassis.turnToHeading(315, 600, {}, false);
    chassis.moveToPose(-63, -24, 0, 2000, { .lead=0.3, .minSpeed=50, .earlyExitRange=8}, false);
    chassis.moveToPoint(-63, 0, 1000, {}, false);
}