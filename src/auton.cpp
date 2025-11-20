#include "custom/auton.h"
#include "custom/configs.h"
#include "custom/util_funcs.h"
#include "pros/motors.h"
#include "pros/rtos.hpp"

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
    chassis.turnToHeading(0, 300, {.minSpeed=60, .earlyExitRange=3});
 
 
// Intake 3 other balls

chassis.moveToPoint(-24.5, 29.5, 1500, {.maxSpeed=85, .minSpeed=50, .earlyExitRange=3}, true);
    pros::delay(40);
    closeGate();
    pros::delay(900);
    openGate();
// Score the mid goal

    pros::delay(600);
    stopIntake();
    //  chassis.turnToPoint(-21, 15,700,  {.forwards = false});    
    topMotor.move(127);
    frontMotor.move(-60);
    pros::delay(200);
    frontMotor.move(0);
    //topMotor.move(0);
    //chassis.turnToHeading(315, 800, {}, false);
   
   // chassis.turnToHeading(330, 400, {}, false);
    //chassis.moveToPoint(-18, 9, 1000, {.forwards=false, .maxSpeed=70}, false);
    chassis.moveToPose(-8.75, 9, 335, 2000, {.forwards=false, .minSpeed = 80});
    pros::delay(500);
    startTopScore();
    startMidScore();
    chassis.waitUntilDone();
    pros::delay(850);
    middleMech.extend();
    stopMidScore();
    stopTopScore();
    startIntake();
    //stopTopScore();
    
    // Move towards long goal and score
    
    chassis.turnToPoint(-45, 47, 200, {.minSpeed = 70});
     pros::delay(200);
    chassis.moveToPoint(-45, 47, 3500, {});
    //closeGate();
    pros::delay(150);
    chassis.turnToHeading(270, 500, {}, false);
    RclMain.updateBotPose();
    /*chassis.moveToPoint(-27, 51, 1100, {.forwards=false, .maxSpeed=80}, false);
    startTopScore();
    pros::delay(1000);
    stopTopScore();*/

    // Refill at matchloader
    startIntake();
    openGate();
    //RclMain.updateBotPose();
    chassis.moveToPoint(-70, 47, 1200, {.maxSpeed=80, .minSpeed=50}, false);
    pros::delay(200);

    // Score again
    chassis.moveToPoint(-25, 50, 1100, {.forwards = false, .maxSpeed=80}, false);
    //closeGate();
    startTopScore();
}
void right(){
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.setPose(48.75, -16.125, 270);
    RclMain.setRclPose(chassis.getPose());

    // Intake three balls
    stopIntake();
    startIntake();
    //topMotor.move_velocity(50);
    chassis.moveToPoint(17, -28, 1500, {}, true);
    pros::delay(450);
    openGate();

    // Intake two balls
    chassis.turnToHeading(190, 800, {}, true);
    pros::delay(100);
    closeGate();
    chassis.moveToPoint(7, -45, 2000, {.maxSpeed=70}, false);

    // Backoff and score middle goal
    chassis.moveToPoint(19, -18, 1200, {.forwards=false}, false);
    stopIntake();
    chassis.turnToHeading(135, 800, {}, false);
    middleMech.retract();
    chassis.moveToPoint(12, -12, 1500, {.forwards=false, .maxSpeed=70}, false);
    startTopScore();
    pros::delay(850);
    middleMech.extend();
    stopTopScore();

    // Move towards long goal and score
    chassis.moveToPoint(42, -50, 2500, {}, false);
    openGate();
    chassis.turnToHeading(90, 500, {}, false);
    /*chassis.moveToPoint(-27, 51, 1100, {.forwards=false, .maxSpeed=80}, false);
    startTopScore();
    pros::delay(1000);
    stopTopScore();		*/

    // Refill at matchloader
    startIntake();
    openGate();
    chassis.moveToPoint(68, -48, 1200, {.maxSpeed=80, .minSpeed=50}, false);
    pros::delay(100);

    // Score again
    chassis.moveToPoint(25, -48, 1100, {.forwards=false, .maxSpeed=80}, false);
    closeGate();
    startTopScore();
    pros::delay(2300);
    stopTopScore();
    chassis.moveToPoint(42, -48, 1500, {.minSpeed=60}, false);
    chassis.moveToPoint(27, -48, 750, {.forwards=false, .minSpeed=90}, false);
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
    chassis.moveToPoint(-17, 28, 1500, {}, true);
    pros::delay(450);
    openGate();

    // Intake two balls
    chassis.turnToHeading(10, 800, {}, true);
    pros::delay(100);
    closeGate();
    chassis.moveToPoint(-9.5, 45, 2000, {.maxSpeed=70}, false);

    // Backoff and score middle goal
    chassis.moveToPoint(-18.5, 17.5, 1200, {.forwards=false}, false);
    stopIntake();
    openGate();
    chassis.turnToHeading(315, 800, {}, false);
    startOuttake();
    frontMotor.move(-25);
    middleMech.retract();
    chassis.moveToPoint(-9.5, 12.25, 1500, {.forwards=false, .maxSpeed=70}, false);
    startMidScore();
    closeGate();
    pros::delay(950); // middle goal score time
    middleMech.extend();
    stopMidScore();

    // Move towards long goal and score
    chassis.turnToPoint(-42, 48, 700, {.minSpeed = 70, .earlyExitRange = 3});
    chassis.moveToPoint(-42, 48, 2500, {});
    openGate();
    pros::delay(300);
    chassis.turnToHeading(270, 500, {}, false);
    /*chassis.moveToPoint(-27, 51, 1100, {.forwards=false, .maxSpeed=80}, false);
    startTopScore();
    pros::delay(1000);
    stopTopScore();*/

    // Refill at matchloader
    startIntake();
    openGate();
    chassis.moveToPoint(-68, 50, 1200, {.maxSpeed=80, .minSpeed=50}, false);
    pros::delay(200);

    // Score again
    chassis.moveToPoint(-25, 50, 1100, {.forwards = false, .maxSpeed=80}, false);
    //closeGate();
    startTopScore();
    pros::delay(1000);
    stopTopScore();
    chassis.moveToPoint(-42, 52, 1500, {.minSpeed=60}, false);
    chassis.moveToPoint(-27, 52, 750, {.forwards=false, .minSpeed=90}, false);
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
    chassis.moveToPoint(-17, 28, 1500, {}, true);
    pros::delay(450);
    openGate();

    // Head to matchloader
    chassis.turnToPoint(-53, 47, 800, {}, false);
    chassis.moveToPoint(-53, 47, 1200, {}, false);
    chassis.turnToPoint(-65, 47, 300, {}, false);
    chassis.moveToPoint(-65, 47, 700, {}, false);
    pros::delay(1500);

    // Move towards long goal and score
    chassis.moveToPoint(-20, 47, 1800, {.forwards=false}, false);
    startTopScore();
    pros::delay(1200);
    stopTopScore();

    // Push
    chassis.moveToPoint(-42, 37, 1200, {.maxSpeed=60}, false);
    chassis.turnToPoint(pivot_x(-10), pivot_y(37), 400, {}, false);
    retractRightArm();
    chassis.moveToPoint(-10, 37, 1200, {.forwards=false}, false);
}
void rightControlRush() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.setPose(-48.75, -16.125, 90);
    RclMain.setRclPose(chassis.getPose());

    RclMain.updateBotPose();

    // Intake three balls
    startIntake();
    //topMotor.move_velocity(50);
    chassis.moveToPoint(-17, -28, 1500, {}, true);
    pros::delay(450);
    openGate();

    // Head to matchloader
    chassis.turnToPoint(-53, -47, 800, {}, false);
    chassis.moveToPoint(-53, -47, 1200, {}, false);
    chassis.turnToPoint(-65, -47, 300, {}, false);
    chassis.moveToPoint(-65, -47, 700, {}, false);
    pros::delay(1500);

    // Move towards long goal and score
    chassis.moveToPoint(-20, -47, 1800, {.forwards=false}, false);
    startTopScore();
    pros::delay(1200);
    stopTopScore();

    // Push
    chassis.moveToPoint(-42, -57, 1200, {.maxSpeed=60}, false);
    chassis.turnToPoint(pivot_x(-10), pivot_y(-57), 400, {}, false);
    retractRightArm();
    chassis.moveToPoint(-10, -57, 1200, {.forwards=false}, false);
}


void skills() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.setPose(-47, -13.25, 180);
    RclMain.setRclPose(chassis.getPose());

    // Head towards bottom-left match loader
    chassis.moveToPoint(-47, -46.5, 1800, {}, false);
    chassis.turnToHeading(270, 700, {});
    startIntake();
    openGate();
    chassis.moveToPoint(-64.5, -46.5, 1000, {.maxSpeed=70}, false);
    chassis.moveToPoint(-58, -46.5, 300, {.maxSpeed=70}, false);
    chassis.moveToPoint(-65, -46.5, 1000, {.maxSpeed=70}, false);

    RclMain.updateBotPose();
    pros::delay(1500);

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
    chassis.moveToPoint(65, -48, 2000, {.maxSpeed=70}, false);
    pros::delay(1200);

    // Score the long goal
    chassis.moveToPoint(24, -48, 1100, {.forwards=false, .maxSpeed=70}, false);

    startTopScore();
    closeGate();
    RclMain.updateBotPose();
    pros::delay(4500);

   // Back off (#2)
    chassis.moveToPoint(47, -46, 1500, {});
    chassis.turnToHeading(0, 600, {}, false);
    stopTopScore();

    // Head towards top-right match loader
    chassis.moveToPoint(45, 47, 750, {.minSpeed=60});
    chassis.moveToPoint(45, 47, 2500, {.maxSpeed=60}, false);
    pros::delay(300);
    chassis.turnToHeading(90, 400, {});
    pros::delay(200);
    startIntake();
    openGate();
    chassis.moveToPoint(66, 47.5, 1000, {.maxSpeed=70}, false);
    chassis.moveToPoint(62, 47.5, 700, {.maxSpeed=70}, false);
    chassis.moveToPoint(66, 47.5, 400, {.maxSpeed=70}, false);

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

    chassis.moveToPoint(-24, 48, 1100, {.forwards=false, .maxSpeed=70}, false);
    startTopScore();
    pros::delay(3000);
    
    // Intake from top-left match loader
    startIntake();
    openGate();
    chassis.moveToPoint(-64, 48, 1000, {.maxSpeed=70}, false);
    chassis.moveToPoint(-64, 48, 700, {.maxSpeed=70}, false);
    chassis.moveToPoint(-64, 48, 400, {.maxSpeed=70}, false);

    pros::delay(1000);

    // Score the long goal
    chassis.moveToPoint(-24, 48, 1100, {.forwards=false, .maxSpeed=70}, false);
    startTopScore();
    closeGate();
    RclMain.updateBotPose();
    pros::delay(3000);

    // Back off (#4)
    chassis.moveToPoint(-42, 47, 1000, {}, false);
    chassis.turnToHeading(180, 800, {}, false);
    stopTopScore();

   // Park
   chassis.moveToPose(-62, 20, 180, 2000, {.maxSpeed = 90});
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