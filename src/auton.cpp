#include "custom/auton.h"
#include "custom/configs.h"
#include "custom/util_funcs.h"
#include "pros/motors.h"

// 9 balls -> Top Middle Goal + Top Long Goal
void red_left() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.setPose(-48.75, 16.125, 90);

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
    chassis.moveToPoint(-10, 45, 2000, {.maxSpeed=70}, false);

    // Backoff and score middle goal
    chassis.moveToPoint(-19, 19, 1200, {.forwards=false}, false);
    stopIntake();
    chassis.turnToHeading(315, 800, {}, false);
    middleMech.retract();
    chassis.moveToPoint(-11.5, 10, 1500, {.forwards=false, .maxSpeed=70}, false);
    startMidScore();
    pros::delay(850);
    middleMech.extend();
    stopMidScore();

    // Move towards long goal and score
    chassis.moveToPoint(-42, 50, 2500, {}, false);
    openGate();
    chassis.turnToHeading(270, 500, {}, false);
    /*chassis.moveToPoint(-27, 51, 1100, {.forwards=false, .maxSpeed=80}, false);
    startTopScore();
    pros::delay(1000);
    stopTopScore();*/

    // Refill at matchloader
    startIntake();
    openGate();
    chassis.moveToPoint(-68, 50, 1200, {.maxSpeed=80, .minSpeed=50}, false);
    pros::delay(100);

    // Score again
    chassis.moveToPoint(-25, 50, 1100, {.forwards = false, .maxSpeed=80}, false);
    closeGate();
    startTopScore();
    pros::delay(2300);
    stopTopScore();
    chassis.moveToPoint(-42, 52, 1500, {.minSpeed=60}, false);
    chassis.moveToPoint(-27, 54, 750, {.forwards=false, .minSpeed=90}, false);
    startTopScore();
}
void blue_left() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.setPose(48.75, -16.125, 270);

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
    chassis.moveToPoint(10, -45, 2000, {.maxSpeed=70}, false);

    // Backoff and score middle goal
    chassis.moveToPoint(19, -19, 1200, {.forwards=false}, false);
    stopIntake();
    chassis.turnToHeading(135, 800, {}, false);
    middleMech.retract();
    chassis.moveToPoint(11.5, -10, 1500, {.forwards=false, .maxSpeed=70}, false);
    startMidScore();
    pros::delay(850);
    middleMech.extend();
    stopMidScore();

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
    chassis.moveToPoint(68, -50, 1200, {.maxSpeed=80, .minSpeed=50}, false);
    pros::delay(100);

    // Score again
    chassis.moveToPoint(25, -50, 1100, {.forwards=false, .maxSpeed=80}, false);
    closeGate();
    startTopScore();
    pros::delay(2300);
    stopTopScore();
    chassis.moveToPoint(42, -52, 1500, {.minSpeed=60}, false);
    chassis.moveToPoint(27, -54, 750, {.forwards=false, .minSpeed=90}, false);
    startTopScore();
}

// 9 balls -> Lower Middle Goal + Lower Long Goal
void red_right_scoreMid() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.setPose(-48.75, -16.125, 90);

    // Intake three balls
    stopIntake();
    startIntake();
    //topMotor.move_velocity(50);
    chassis.moveToPoint(-17, -28, 1500, {}, true);
    pros::delay(450);
    openGate();

    // Intake two balls
    chassis.turnToHeading(170, 600, {}, true);
    pros::delay(100);
    closeGate();
    chassis.moveToPoint(-9, -47, 1700, {.maxSpeed=70}, false);

    // Backoff and score middle goal
    chassis.moveToPoint(-18, -24, 1200, {.forwards=false}, false);
    stopIntake();
    chassis.turnToHeading(45, 800, {}, false);
    chassis.moveToPoint(-10, -16, 1500, { .maxSpeed=70}, true);
    startOuttake();
    pros::delay(850);
    stopOuttake();

    // Move towards long goal and score
    chassis.moveToPoint(-42, -48, 2500, {.forwards=false}, false);
    openGate();
    chassis.turnToHeading(270, 700, {}, false);
    /*chassis.moveToPoint(-27, 51, 1100, {.forwards=false, .maxSpeed=80}, false);
    startTopScore();
    pros::delay(1000);
    stopTopScore();	*/

    // Refill at matchloader
    startIntake();
    openGate();
    chassis.moveToPoint(-68, -48, 1200, {.maxSpeed=80, .minSpeed=50}, false);
    pros::delay(100);

    // Score again
    chassis.moveToPoint(-25, -48, 1100, {.forwards=false, .maxSpeed=80}, false);
    closeGate();
    startTopScore();
    pros::delay(2300);
    stopTopScore();
    chassis.moveToPoint(-42, -48, 1500, {.minSpeed=60}, false);
    chassis.moveToPoint(-27, -48, 750, {.forwards=false, .minSpeed=90}, false);
    startTopScore();
}
void blue_right_scoreMid() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.setPose(48.75, 16.125, 270);

    // Intake three balls
    stopIntake();
    startIntake();
    //topMotor.move_velocity(50);
    chassis.moveToPoint(17, 28, 1500, {}, true);
    pros::delay(450);
    openGate();

    // Intake two balls
    chassis.turnToHeading(350, 600, {}, true);
    pros::delay(100);
    closeGate();
    chassis.moveToPoint(9, 47, 1700, {.maxSpeed=70}, false);

    // Backoff and score middle goal
    chassis.moveToPoint(18, 24, 1200, {.forwards=false}, false);
    stopIntake();
    chassis.turnToHeading(225, 800, {}, false);
    chassis.moveToPoint(10, 16, 1500, {.maxSpeed=70}, true);
    startOuttake();
    pros::delay(850);
    stopOuttake();

    // Move towards long goal and score
    chassis.moveToPoint(42, 48, 2500, {.forwards=false}, false);
    openGate();
    chassis.turnToHeading(90, 700, {}, false);
    /*chassis.moveToPoint(-27, 51, 1100, {.forwards=false, .maxSpeed=80}, false);
    startTopScore();
    pros::delay(1000);
    stopTopScore();			*/

    // Refill at matchloader
    startIntake();
    openGate();
    chassis.moveToPoint(68, 48, 1200, {.maxSpeed=80, .minSpeed=50}, false);
    pros::delay(100);

    // Score again
    chassis.moveToPoint(25, 48, 1100, {.forwards=false, .maxSpeed=80}, false);
    closeGate();
    startTopScore();
    pros::delay(2300);
    stopTopScore();
    chassis.moveToPoint(42, 48, 1500, {.minSpeed=60}, false);
    chassis.moveToPoint(27, 48, 750, {.forwards=false, .minSpeed=90}, false);
    startTopScore();
}

// 9 balls -> Lower Long Goal
void red_right_noScoreMid() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.setPose(-48.75, -16.125, 90);

    // Intake three balls
    stopIntake();
    startIntake();
    //topMotor.move_velocity(50);
    chassis.moveToPoint(-17, -28, 1500, {}, true);
    pros::delay(450);
    openGate();

    // Intake two balls
    chassis.turnToHeading(170, 600, {}, true);
    pros::delay(100);
    closeGate();
    chassis.moveToPoint(-9, -47, 1700, {.maxSpeed=70}, false);

    // Backoff and score middle goal
    chassis.moveToPoint(-18, -24, 1200, {.forwards=false}, false);
    stopIntake();
    chassis.turnToHeading(45, 800, {}, false);
    chassis.moveToPoint(-10, -16, 1500, { .maxSpeed=70}, true);
    //startOuttake();
    pros::delay(850);
    //stopOuttake();

    // Move towards long goal and score
    chassis.moveToPoint(-42, -48, 2500, {.forwards=false}, false);
    openGate();
    chassis.turnToHeading(270, 700, {}, false);
    /*chassis.moveToPoint(-27, 51, 1100, {.forwards=false, .maxSpeed=80}, false);
    startTopScore();
    pros::delay(1000);
    stopTopScore();	*/

    // Refill at matchloader
    startIntake();
    openGate();
    chassis.moveToPoint(-68, -48, 1200, {.maxSpeed=80, .minSpeed=50}, false);
    pros::delay(100);

    // Score again
    chassis.moveToPoint(-25, -48, 1100, {.forwards=false, .maxSpeed=80}, false);
    closeGate();
    startTopScore();
    pros::delay(2300);
    stopTopScore();
    chassis.moveToPoint(-42, -48, 1500, {.minSpeed=60}, false);
    chassis.moveToPoint(-27, -48, 750, {.forwards=false, .minSpeed=90}, false);
    startTopScore();
}
void blue_right_noScoreMid() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.setPose(48.75, 16.125, 270);

    // Intake three balls
    stopIntake();
    startIntake();
    //topMotor.move_velocity(50);
    chassis.moveToPoint(17, 28, 1500, {}, true);
    pros::delay(450);
    openGate();

    // Intake two balls
    chassis.turnToHeading(350, 600, {}, true);
    pros::delay(100);
    closeGate();
    chassis.moveToPoint(9, 47, 1700, {.maxSpeed=70}, false);

    // Backoff and score middle goal
    chassis.moveToPoint(18, 24, 1200, {.forwards=false}, false);
    stopIntake();
    chassis.turnToHeading(225, 800, {}, false);
    chassis.moveToPoint(10, 16, 1500, {.maxSpeed=70}, true);
    //startOuttake();
    pros::delay(850);
    //stopOuttake();

    // Move towards long goal and score
    chassis.moveToPoint(42, 48, 2500, {.forwards=false}, false);
    openGate();
    chassis.turnToHeading(90, 700, {}, false);
    /*chassis.moveToPoint(-27, 51, 1100, {.forwards=false, .maxSpeed=80}, false);
    startTopScore();
    pros::delay(1000);
    stopTopScore();			*/

    // Refill at matchloader
    startIntake();
    openGate();
    chassis.moveToPoint(68, 48, 1200, {.maxSpeed=80, .minSpeed=50}, false);
    pros::delay(100);

    // Score again
    chassis.moveToPoint(25, 48, 1100, {.forwards=false, .maxSpeed=80}, false);
    closeGate();
    startTopScore();
    pros::delay(2300);
    stopTopScore();
    chassis.moveToPoint(42, 48, 1500, {.minSpeed=60}, false);
    chassis.moveToPoint(27, 48, 750, {.forwards=false, .minSpeed=90}, false);
    startTopScore();
}

// Solo AWP
void red_soloAWP() {
    chassis.setPose(-47, -13.25, 180);

    // Head towards the matchloader and intake
    openGate();
    chassis.moveToPoint(-47, -38.5, 1200, {.minSpeed=60, .earlyExitRange=3}, false);
    chassis.turnToHeading(265, 800, {.minSpeed=60, .earlyExitRange=4});
    startIntake();
    //pros::delay(200);
    chassis.moveToPoint(-64, -47.5, 1350, {.maxSpeed=70}, false);
    //pros::delay(800);

    // Score the long goal
    chassis.moveToPoint(-24, -47, 1000, {.forwards=false, .maxSpeed=90}, false);
    startTopScore();
    closeGate();
    pros::delay(1000);

    // Back off
    chassis.moveToPoint(-40, -47, 600, {.minSpeed=60, .earlyExitRange=3});
    chassis.turnToHeading(30, 800, {.minSpeed=60, .earlyExitRange=3});
    stopTopScore();
    
    // Intake 3 balls
    //chassis.moveToPoint(-22, -20, 1200, {.maxSpeed=70, .minSpeed=30, .earlyExitRange=3});

    // Score low goal
    chassis.moveToPoint(-15, -8.5, 1400, {.maxSpeed=90}, false);
    startOuttake();
    pros::delay(250);
    chassis.moveToPoint(-24, -24, 800, {.forwards=false, .minSpeed=70, .earlyExitRange=3});

    // Intake 3 other balls
    startIntake();
    chassis.turnToHeading(0, 300, {.minSpeed=60, .earlyExitRange=3});
    chassis.moveToPoint(-28, 22, 1500, {.maxSpeed=85, .minSpeed=50, .earlyExitRange=3}, true);
    pros::delay(50);
    closeGate();
    pros::delay(1000);
    openGate();

    // Score the mid goal
    /*chassis.turnToHeading(315, 400, {}, false);
    chassis.moveToPoint(-15, 14, 1000, {.forwards=false, .maxSpeed=70}, false);
    startIntake();
    startTopScore();
    pros::delay(800);
    stopIntake();
    stopTopScore();*/

    // Move towards the other matchloader and intake
    chassis.turnToPoint(-47, 46, 700, {.minSpeed=50, .earlyExitRange=3});
    chassis.moveToPoint(-47, 46, 1600, {.minSpeed=70, .earlyExitRange=3});
    chassis.turnToHeading(275, 500, {.minSpeed=60, .earlyExitRange=3});
    startIntake();
    chassis.moveToPoint(-67.5, 49, 1200, {.maxSpeed=80}, false);

    // Score the other long goal
    // chassis.turnToPoint(-33, 49, 800);
    chassis.moveToPoint(-30, 52, 1000, {.forwards=false, .maxSpeed=90, .minSpeed=50}, false);
    startTopScore();
    pros::delay(2250);
    chassis.moveToPoint(-42, 52, 1000, {.minSpeed=100});
}

void blue_soloAWP() {
    chassis.setPose(-47, -13.25, 180);

    // Head towards the matchloader and intake
    openGate();
    chassis.moveToPoint(-47, -38.5, 1200, {.minSpeed=60, .earlyExitRange=3}, false);
    chassis.turnToHeading(265, 800, {.minSpeed=60, .earlyExitRange=4});
    startIntake();
    //pros::delay(200);
    chassis.moveToPoint(-64, -47.5, 1350, {.maxSpeed=70}, false);
    //pros::delay(800);

    // Score the long goal
    chassis.moveToPoint(-24, -47, 1000, {.forwards=false, .maxSpeed=90}, false);
    startTopScore();
    closeGate();
    pros::delay(1000);

    // Back off
    chassis.moveToPoint(-40, -47, 600, {.minSpeed=60, .earlyExitRange=3});
    chassis.turnToHeading(30, 800, {.minSpeed=60, .earlyExitRange=3});
    stopTopScore();
    
    // Intake 3 balls
    //chassis.moveToPoint(-22, -20, 1200, {.maxSpeed=70, .minSpeed=30, .earlyExitRange=3});

    // Score low goal
    chassis.moveToPoint(-15, -8.5, 1400, {.maxSpeed=90}, false);
    startOuttake();
    pros::delay(250);
    chassis.moveToPoint(-24, -24, 800, {.forwards=false, .minSpeed=70, .earlyExitRange=3});

    // Intake 3 other balls
    startIntake();
    chassis.turnToHeading(0, 300, {.minSpeed=60, .earlyExitRange=3});
    chassis.moveToPoint(-28, 22, 1500, {.maxSpeed=85, .minSpeed=50, .earlyExitRange=3}, true);
    pros::delay(50);
    closeGate();
    pros::delay(1000);
    openGate();

    // Score the mid goal
    /*chassis.turnToHeading(315, 400, {}, false);
    chassis.moveToPoint(-15, 14, 1000, {.forwards=false, .maxSpeed=70}, false);
    startIntake();
    startTopScore();
    pros::delay(800);
    stopIntake();
    stopTopScore();*/

    // Move towards the other matchloader and intake
    chassis.turnToPoint(-47, 46, 700, {.minSpeed=50, .earlyExitRange=3});
    chassis.moveToPoint(-47, 46, 1600, {.minSpeed=70, .earlyExitRange=3});
    chassis.turnToHeading(275, 500, {.minSpeed=60, .earlyExitRange=3});
    startIntake();
    chassis.moveToPoint(-67.5, 49, 1200, {.maxSpeed=80}, false);

    // Score the other long goal
    // chassis.turnToPoint(-33, 49, 800);
    chassis.moveToPoint(-30, 52, 1000, {.forwards=false, .maxSpeed=90, .minSpeed=50}, false);
    startTopScore();
    pros::delay(2250);
    chassis.moveToPoint(-42, 52, 1000, {.minSpeed=100});
}

void skills() {
    chassis.setPose(-47, -13.25, 180);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);

    // Head towards bottom-left match loader
    chassis.moveToPoint(-47, -46, 1800, {}, false);
    chassis.turnToHeading(270, 700, {});
    startIntake();
    openGate();
    chassis.moveToPoint(-63, -46, 2000, {.maxSpeed=70}, false);
    pros::delay(1000);

    // Back off (#1)
    stopIntake();
    closeGate();
    //chassis.moveToPoint(-38, -48, 1000, {.forwards=false});
    //chassis.turnToHeading(180, 800, {}, false);
    chassis.moveToPoint(-49, -46,  1000, {.forwards=false}, false);
    chassis.turnToHeading(135, 800, {}, false);

    // Score bottom-right
    //chassis.moveToPoint(-38, -62, 1200, {}, false);
    //chassis.turnToHeading(90, 800, {});
    chassis.moveToPose(-15, -60, 90, 1500, {.lead=0.3, .minSpeed=55}, false);
    chassis.moveToPoint(20, -64, 2000, {.minSpeed=30, .earlyExitRange=12}, false);
    chassis.moveToPoint(38, -62, 1000, {.maxSpeed=60}, false);
    chassis.turnToHeading(0, 600, {});
    chassis.moveToPoint(38, -49, 1200, {}, false);
    chassis.turnToHeading(90, 600, {}, false);

    chassis.moveToPoint(24, -49, 1000, {.forwards=false, .maxSpeed=70}, false);
    startTopScore();
    pros::delay(1750);
    
    // Head towards bottom-right match loader
    startIntake();
    openGate();
    chassis.moveToPoint(63, -49, 2000, {.maxSpeed=70}, false);
    pros::delay(1000);

    // Score the long goal
    chassis.moveToPoint(24, -49, 1100, {.forwards=false, .maxSpeed=70}, false);
    startTopScore();
    closeGate();
    RclMain.updateBotPosition();
    pros::delay(3000);

    // Back off (#2)
    chassis.moveToPoint(47, -49, 800, {}, false);
    chassis.turnToHeading(0, 600, {}, false);
    stopTopScore();

    // Head towards top-right match loader
    chassis.moveToPoint(47, 25, 2500, {.minSpeed=60, .earlyExitRange=10}, false);
    chassis.moveToPoint(47, 45, 1500, {.maxSpeed=50}, false);
    chassis.turnToHeading(90, 700, {});
    startIntake();
    openGate();
    chassis.moveToPoint(65, 45, 2000, {.maxSpeed=70}, false);
    RclMain.updateBotPosition();
    pros::delay(1000);

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
    chassis.moveToPose(15, 60, 270, 1500, {.lead=0.3, .minSpeed=55}, false);
    chassis.moveToPoint(-20, 64, 2000, {.minSpeed=40, .earlyExitRange=12}, false);
    chassis.moveToPoint(-38, 62, 1000, {.maxSpeed=60}, false);
    chassis.turnToHeading(180, 800, {});
    chassis.moveToPoint(-38, 46, 1200, {}, false);
    chassis.turnToHeading(270, 800, {}, false);

    chassis.moveToPoint(-24, 47, 1100, {.forwards=false, .maxSpeed=70}, false);
    startTopScore();
    pros::delay(3000);
    
    // Intake from top-left match loader
    startIntake();
    openGate();
    chassis.moveToPoint(-63, 47, 2000, {.maxSpeed=70}, false);
    pros::delay(1000);

    // Score the long goal
    chassis.moveToPoint(-24, 47, 1100, {.forwards=false, .maxSpeed=70}, false);
    startTopScore();
    closeGate();
    pros::delay(3000);

    // Back off (#4)
    chassis.moveToPoint(-42, 47, 1000, {}, false);
    chassis.turnToHeading(180, 800, {}, false);
    stopTopScore();

    // Park
    chassis.moveToPoint(-42, 0, 2500, {}, false);
    chassis.turnToHeading(270, 800, {}, false);
    odomLift.extend();
    startTopScore();
    chassis.moveToPoint(-70, -8, 3000, {}, false);
    chassis.turnToHeading(180, 1000);

}