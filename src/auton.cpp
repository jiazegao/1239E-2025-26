#include "custom/auton.h"
#include "custom/configs.h"
#include "custom/util_funcs.h"
#include "lemlib/chassis/chassis.hpp"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include <cmath>

void soloAWP(){
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.setPose(-47, -13.25, 180);
    RclMain.setRclPose(chassis.getPose());

    // Head towards the matchloader and intake
    openGate();
    chassis.moveToPoint(-47, -38.5, 1200, {.minSpeed=60, .earlyExitRange=3}, false);
    chassis.turnToHeading(265, 800, {.minSpeed=60, .earlyExitRange=4});
    startIntake();
    //pros::delay(200);
    chassis.moveToPoint(-64, -47.5, 1150, {.maxSpeed=70}, false);
    pros::delay(50);

    // Score the long goal
    chassis.moveToPoint(-24, -47, 1000, {.forwards=false, .maxSpeed=90}, false);
    startTopScore();
    closeGate();
    pros::delay(1200);

    // Back off
    
   // chassis.moveToPoint(-40, -47, 600, {.minSpeed=60, .earlyExitRange=3});
    //chassis.turnToHeading(45, 800, {.minSpeed=60, .earlyExitRange=3});
    
    stopTopScore();
    // Intake 3 balls
    chassis.moveToPoint(-26, -24, 1000, {.maxSpeed=65, .minSpeed=30, .earlyExitRange=4}, true);
    // Intake 3 other balls
    pros::delay(900);
    openGate();
    startIntake();
    //chassis.turnToHeading(0, 300, {.minSpeed=60, .earlyExitRange=3});
 
 

 
// Intake 3 other balls

chassis.moveToPoint(-30, 24, 1500, {.minSpeed=50, .earlyExitRange=3}, true);
    pros::delay(40);
    closeGate();
    pros::delay(800);
    openGate();
// Score the mid goal

    pros::delay(500);
    stopIntake();
    //  chassis.turnToPoint(-21, 15,700,  {.forwards = false});    
    topMotor.move(127);
    frontMotor.move(-25);
    pros::delay(200);
    /*frontMotor.move(0);
    topMotor.move(0);*/
    //chassis.turnToHeading(315, 800, {}, false);
   
   // chassis.turnToHeading(330, 400, {}, false);
    //chassis.moveToPoint(-18, 9, 1000, {.forwards=false, .maxSpeed=70}, false);
    chassis.turnToPoint(pivot_x(-13), pivot_y(8), 500, {.minSpeed=60, .earlyExitRange=2}, false);
    chassis.moveToPoint(-14, 7.5, 1500, {.forwards=false, .maxSpeed=85});
    pros::delay(350);
    //startTopScore();
    startMidScore();
    chassis.waitUntilDone();
    pros::delay(450); // middle goal score time
    middleMech.extend();
    stopMidScore();
    stopTopScore();
    startIntake();
    //stopTopScore();
    
    // Move towards long goal and score
    
    chassis.turnToPoint(-47, 44, 200, {.minSpeed=60, .earlyExitRange=1});
    pros::delay(200);
    chassis.moveToPoint(-47, 44, 1900, {.minSpeed=60, .earlyExitRange=2});
    //closeGate();
    chassis.turnToHeading(270, 500, {.minSpeed=60, .earlyExitRange=1}, false);
    RclMain.updateBotPose();
    /*chassis.moveToPoint(-27, 51, 1100, {.forwards=false, .maxSpeed=80}, false);
    startTopScore();
    pros::delay(1000);
    stopTopScore();*/

    // Refill at matchloader
    startIntake();
    openGate();
    //RclMain.updateBotPose();
    chassis.moveToPoint(-76, 48, 1200, {.maxSpeed=80, .minSpeed=50}, false);
    pros::delay(350);

    // Score again
    chassis.moveToPoint(-25, 50, 900, {.forwards = false, .maxSpeed=100}, false);
    //closeGate();
    startTopScore();
}
  

void right(){
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.setPose(-48.75, -16.125, 90);
    RclMain.setRclPose(chassis.getPose());

    // Intake three balls
    stopIntake();
    startIntake();
    //topMotor.move_velocity(50);
    chassis.moveToPoint(-19, -28, 1500, {}, true);
    pros::delay(450);
    openGate();

    // Intake two balls
    chassis.turnToHeading(170, 800, {}, true);
    pros::delay(100);
    closeGate();
    chassis.moveToPoint(-8, -44, 2000, {.maxSpeed=70}, false);
    
    // Backoff and score middle goal
    chassis.moveToPoint(-18.5, -14.5, 1200, {.forwards=false}, false);
    /*stopIntake();
    openGate();
    chassis.turnToHeading(225, 800, {}, false);
    startOuttake();
    frontMotor.move(-25);
    middleMech.retract();
    chassis.moveToPose(-8, -6, 225, 2000, {.forwards=false, .minSpeed=80}, false);
    pros::delay(500);
    startMidScore();
    closeGate();
    pros::delay(1000);	 // middle goal score time
    stopMidScore();
    middleMech.extend();*/

    // Move towards long goal and score
    chassis.turnToPoint(-42, -48, 700, {.minSpeed=70, .earlyExitRange=3});
    chassis.moveToPoint(-42, -48, 2500, {});
    openGate();
    pros::delay(300);
    chassis.turnToHeading(270, 500, {.minSpeed=70, .earlyExitRange=3}, false);
    /*chassis.moveToPoint(-27, 51, 1100, {.forwards=false, .maxSpeed=80}, false);
    startTopScore();
    pros::delay(1000);
    stopTopScore();	*/

    // Refill at matchloader
    startIntake();
    openGate();
    chassis.moveToPoint(-68, -48, 1200, {.maxSpeed=70, .minSpeed=50}, false);
    pros::delay(600);

    // Score again
    chassis.moveToPoint(-25, -48, 1100, {.forwards=false, .maxSpeed=80}, false);
    //closeGate();
    startTopScore();
}
void left() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.setPose(-48.75, 16.125, 90);
    RclMain.setRclPose(chassis.getPose());

    // Intake three balls
    stopIntake();
    startIntake();
    //topMotor.move_velocity(50);
    chassis.moveToPoint(-19, 28, 1500, {}, true);
    pros::delay(450);
    openGate();

    // Intake two balls
    chassis.turnToHeading(10, 800, {}, true);
    pros::delay(100);
    closeGate();
    chassis.moveToPoint(-8, 44, 2000, {.maxSpeed=70}, false);
    // Backoff and score middle goal
    chassis.moveToPoint(-18.5, 14.5, 1200, {.forwards=false}, false);
    stopIntake();
    openGate();
    chassis.turnToHeading(315, 800, {}, false);
    startOuttake();
    frontMotor.move(-25);
    middleMech.retract();
    chassis.moveToPose(-8, 6, 315, 2000, {.forwards=false, .minSpeed = 80}, false);
    pros::delay(500);
    startMidScore();
    closeGate();
    pros::delay(1000); // middle goal score time
    stopMidScore();
    middleMech.extend();

    // Move towards long goal and score
    chassis.turnToPoint(-42, 48, 700, {.minSpeed = 70, .earlyExitRange = 3});
    chassis.moveToPoint(-42, 48, 2500, {});
    openGate();
    pros::delay(300);
    chassis.turnToHeading(270, 500, {.minSpeed = 70, .earlyExitRange = 3}, false);
    /*chassis.moveToPoint(-27, 51, 1100, {.forwards=false, .maxSpeed=80}, false);
    startTopScore();
    pros::delay(1000);
    stopTopScore();*/

    // Refill at matchloader
    startIntake();
    openGate();
    chassis.moveToPoint(-68, 49, 1200, {.maxSpeed=70, .minSpeed=50}, false);
    pros::delay(600);

    // Score again
    chassis.moveToPoint(-25, 49, 1100, {.forwards = false, .maxSpeed=80}, false);
    //closeGate();
    startTopScore();

   
}
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
    chassis.turnToPoint(-44, 49, 800, {.minSpeed = 40, .earlyExitRange = 3}, false);
    chassis.moveToPoint(-44, 49, 1500, {.minSpeed=30, .earlyExitRange=2}, false);
    chassis.turnToPoint(-67, 49, 300, {}, false);
    chassis.moveToPoint(-67, 49, 700, {}, false);
    pros::delay(700);

    // Move towards long goal and score
    chassis.moveToPoint(-24, 49, 1300, {.forwards=false}, false);
    startTopScore();
    pros::delay(2700);
    stopTopScore();

    // Push
    closeGate();
    chassis.moveToPoint(-44, 61, 1200, {.minSpeed = 60, .earlyExitRange = 1}, false);
    chassis.turnToPoint(-10, 61, 700, {.forwards=false}, false);
    retractLeftArm();
    chassis.moveToPoint(-15, 59, 1000, {.forwards=false, .maxSpeed=60}, false);
    chassis.moveToPoint(-15, 59, 9999, {.forwards=false}, false);

    //temporary
    //startTopScore();
}
void rightControlRush() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.setPose(-48.75, -16.125, 90);
    RclMain.setRclPose(chassis.getPose());

    RclMain.updateBotPose();

    // Intake three balls
    startIntake();
    //topMotor.move_velocity(50);
    chassis.moveToPoint(-24, -28, 1500, {.minSpeed=50, .earlyExitRange=3}, true);
    pros::delay(450);
    openGate();

    // Head to matchloader
    chassis.turnToPoint(-44, 49, 800, {.minSpeed=40, .earlyExitRange=3}, false);
    chassis.moveToPoint(-44, -49, 1500, {.minSpeed=30, .earlyExitRange=2}, false);
    chassis.turnToPoint(-67, 49, 300, {}, false);
    chassis.moveToPoint(-67, -49, 700, {}, false);
    pros::delay(400);
    RclMain.updateBotPose(&right_rcl);
    pros::delay(300);

    // Move towards long goal and score
    chassis.moveToPoint(0, -51, 1300, {.forwards=false}, false);
    startTopScore();
    pros::delay(2700);
    stopTopScore();

    // Push
    closeGate();
    chassis.moveToPoint(-44, -41, 1200, {.minSpeed=60, .earlyExitRange=1}, false);
    chassis.turnToPoint(-10, 41, 700, {.forwards=false}, false);
    retractLeftArm();
    chassis.moveToPoint(-13, -43, 5000, {.forwards=false, .maxSpeed=90}, false);

    //temporary
    //startTopScore();
}

void skills() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.setPose(-47, -13.25, 180);
    RclMain.setRclPose(chassis.getPose());

    // Head towards bottom-left match loader
    chassis.moveToPoint(-47, -46, 1800, {}, false);
    chassis.turnToHeading(270, 700, {});
    startIntake();
    openGate();
    chassis.moveToPoint(-68.5, -46, 2000, {.maxSpeed=60});
   
    RclMain.updateBotPose();
    pros::delay(2500);

    // Back off (#1)
    stopIntake();
    closeGate();
    //chassis.moveToPoint(-38, -48, 1000, {.forwards=false});
    //chassis.turnToHeading(180, 800, {}, false);
    chassis.moveToPoint(-49, -48,  1000, {.forwards=false}, false);
    chassis.turnToHeading(135, 800, {}, false);

    // Score bottom-right
    //chassis.moveToPoint(-38, -62, 1200, {}, false);
    //chassis.turnToHeading(90, 800, {});
    chassis.moveToPose(-15, -61, 90, 1500, {.lead=0.3, .minSpeed=55}, false);
    chassis.moveToPoint(20, -61, 2000, {.minSpeed=30, .earlyExitRange=12}, false);
     
    chassis.moveToPoint(40, -61, 1000, {.maxSpeed=60}, false);
    chassis.turnToHeading(0, 600, {});
    RclMain.updateBotPose();
    chassis.moveToPoint(38, -48, 1200, {}, false);
    chassis.turnToHeading(90, 600, {}, false);

    chassis.moveToPoint(24, -48, 1000, {.forwards=false, .maxSpeed=70}, false);
    startOuttake();
    pros::delay(25);
    startTopScore();
    pros::delay(2750);
    
    // Head towards bottom-right match loader
    startIntake();
    openGate();
    chassis.moveToPoint(68, -48, 1000, {.maxSpeed=80});
    chassis.moveToPoint(61, -48, 500, {.forwards = false, .maxSpeed=70});
    chassis.moveToPoint(68, -48, 500, {.maxSpeed=80}, false);

    pros::delay(1500);

    // Score the long goal
    chassis.moveToPoint(24, -48, 1100, {.forwards=false, .maxSpeed=70}, false);

    startTopScore();
    closeGate();
    RclMain.updateBotPose();
    pros::delay(4500);

   // Back off (#2)
    chassis.moveToPoint(47, -48, 1500, {});
    chassis.turnToHeading(0, 600, {}, false);
    stopTopScore();

    // Head towards top-right match loader
    chassis.moveToPoint(45, 47, 750, {.minSpeed=60});
    chassis.moveToPoint(45, 47, 2500, {.maxSpeed=75});
    pros::delay(300);
    chassis.turnToHeading(90, 500, {});
    pros::delay(200);
    startIntake();
    openGate();
    chassis.moveToPoint(68, 47, 1000, {.maxSpeed=60}); //
    chassis.moveToPoint(64, 47, 700, {.forwards=false, .maxSpeed=70});
    chassis.moveToPoint(69, 47, 500, {.maxSpeed=60}, false);

    RclMain.updateBotPose();
    pros::delay(1500);

    // Back off (#3)
    stopIntake();
    closeGate();
    //chassis.moveToPoint(45, 42, 1000, {.forwards=false});
    //chassis.turnToHeading(0, 800, {}, false);
    chassis.moveToPoint(49, 46, 1000, {.forwards=false}, false);
    chassis.turnToHeading(315, 700, {}, false);

    // Score the top-left goal
    //chassis.moveToPoint(38, 56, 1200, {}, false);
    //chassis.turnToHeading(270, 800, {});
    chassis.moveToPose(15, 62.5, 270, 1500, {.lead=0.3, .minSpeed=55}, false);
    chassis.moveToPoint(-20, 61.25, 2000, {.minSpeed=40, .earlyExitRange=12}, false);
    chassis.moveToPoint(-38, 61.25, 1000, {.maxSpeed=60}, false);
    chassis.turnToHeading(180, 800, {});
    chassis.moveToPoint(-38, 48, 1200, {}, false);
    chassis.turnToHeading(270, 800, {}, false);

    chassis.moveToPoint(-22, 48, 1100, {.forwards=false, .maxSpeed=70}, false);
    startTopScore();
    pros::delay(3000);
    
    // Intake from top-left match loader
    startIntake();
    openGate();
    chassis.moveToPoint(-65, 48, 2000, {.maxSpeed=70});

    pros::delay(2700);

    // Score the long goal
    chassis.moveToPoint(-20, 48, 2000, {.forwards=false, .maxSpeed=70}, false);
    startTopScore();
    closeGate();
    RclMain.updateBotPose();
    pros::delay(3000);

    // Back off (#4)
    chassis.moveToPoint(-42, 48, 1000, {});
    stopTopScore();
   // Park
   chassis.moveToPose(-66, 18, 200, 4000, {.lead = .25}, false);
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
    chassis.setPose(-46, 0, 270);
    RclMain.setRclPose(chassis.getPose());

    // Back off to gain momentum
    chassis.moveToPoint(-26, 0, 1500, {.forwards=false}, false);

    // Clear red parking zone
    startIntake();
    odomLift.extend(); // Entering parking zone, lift up odom
    chassis.moveToPoint(-70, 0, 800, {.earlyExitRange=6}, false);
    pros::delay(1200);
    chassis.moveToPoint(-46, 0, 800, {.forwards=false}, false);
    RclMain.updateBotPose(&left_rcl); // Position correction
    RclMain.updateBotPose(&right_rcl); // Position correction
    odomLift.retract(); // Leaving parking zone, resume odom
    chassis.turnToHeading(0, 600, {}, false);
    pros::delay(400);
    RclMain.updateBotPose(&left_rcl); // Position correction
    RclMain.updateBotPose(&back_rcl); // Position correction
    
    // Score Top Mid Goal
    chassis.turnToPoint(-18, 18, 600, {}, false);
    chassis.moveToPoint(-18, 18, 1300, {.maxSpeed=50}, false);
    chassis.turnToPoint(pivot_x(-11), pivot_y(11), 800, {}, false);
    stopIntake();
    chassis.moveToPoint(-11, 11, 1000, {.forwards=false, .maxSpeed=50}, true);
    pros::delay(600);
    startMidScore();
    pros::delay(1600);
    stopMidScore();

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
    
    chassis.turnToHeading(35, 800, {}, false);
    chassis.moveToPose(-15, 59, 90, 1500, {.lead=0.3, .minSpeed=55}, false);
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

    // Clear Blue Parking Zone
    chassis.moveToPoint(38, 47, 800, {}, false);
    chassis.turnToPoint(38, 0, 800, {}, false);
    pros::delay(400);
    RclMain.updateBotPose(&back_rcl); // Position correction
    chassis.moveToPoint(38, 15, 1200, {.minSpeed=70, .earlyExitRange=10}, false);
    chassis.moveToPoint(38, 0, 800, {.maxSpeed=50}, false);
    chassis.turnToPoint(63, 0, 600, {}, false);
    startIntake();
    odomLift.extend(); // Entering parking zone, lift up odom
    chassis.moveToPoint(63, 0, 1000, {.minSpeed=60, .earlyExitRange=6}, false);
    pros::delay(1200);

    // Score at Bottom-Right Long Goal
    chassis.moveToPoint(38, 0, 1200, {.forwards=false}, false);
    RclMain.updateBotPose(&left_rcl); // Position correction
    RclMain.updateBotPose(&right_rcl); // Position correction
    odomLift.retract(); // Leaving parking zone, resume odom
    chassis.turnToHeading(180, 800, {}, false);
    pros::delay(400);
    RclMain.updateBotPose(&left_rcl); // Position correction
    RclMain.updateBotPose(&back_rcl); // Position correction
    chassis.moveToPoint(38, -35, 1200, {.minSpeed=60, .earlyExitRange=10}, false);
    chassis.moveToPoint(38, -47, 800, {.maxSpeed=60}, false);
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
    chassis.moveToPose(-63, -21, 0, 1200, { .lead=0.3, .minSpeed=50, .earlyExitRange=8}, false);
    chassis.moveToPoint(-63, 0, 1000, {}, false);
}