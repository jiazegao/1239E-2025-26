#include "custom/auton.h"
#include "custom/configs.h"
#include "custom/util_funcs.h"

// 9 balls -> Top Middle Goal + Top Long Goal
void red_left() {
    chassis.setPose(-50, 13.5, 90);

    // Intake three balls
    startIntake();
    chassis.moveToPose(-19, 23, 45, 1500, {}, true);
    pros::delay(1000);
    openGate();

    // Intake two balls
    chassis.turnToHeading(30, 800, {}, false);
    closeGate();
    chassis.moveToPoint(-6, 44, 2000, {.maxSpeed=70}, false);

    // Backoff and score middle goal
    chassis.moveToPoint(-21, 21, 1200, {.forwards=false}, false);
    stopIntake();
    chassis.turnToHeading(315, 800, {}, false);
    chassis.moveToPoint(-12, 12, 1500, {.forwards=false, .maxSpeed=70}, false);
    startTopScore();
    pros::delay(1000);
    stopTopScore();

    // Move towards long goal and score
    chassis.moveToPoint(-47, 47, 1800, {}, false);
    chassis.turnToHeading(270, 500, {}, false);
    chassis.moveToPoint(-31, 47, 1200, {.forwards=false, .maxSpeed=70}, false);
    startTopScore();
    pros::delay(1000);
    stopTopScore();

    // Refill at matchloader
    startIntake();
    openGate();
    chassis.moveToPoint(-63, 47, 1200, {.maxSpeed=100}, false);
    pros::delay(500);

    // Score again
    chassis.moveToPoint(-31, 47, 1200, {.maxSpeed=70}, false);
    closeGate();
    startTopScore();
}

// 9 balls -> Lower Middle Goal + Lower Long Goal
void red_right() {
    chassis.setPose(-50, -13.5, 90);

    // Intake three balls
    startIntake();
    chassis.moveToPose(-19, -23, 135, 1500, {}, true);
    pros::delay(1000);
    openGate();

    // Intake two balls
    chassis.turnToHeading(150, 800, {}, false);
    closeGate();
    chassis.moveToPoint(-6, -44, 2000, {.maxSpeed=70}, false);

    // Backoff and score middle goal
    chassis.moveToPoint(-21, -21, 1200, {.forwards=false}, false);
    stopIntake();
    chassis.turnToHeading(45, 800, {}, false);
    chassis.moveToPoint(-12, -12, 1500, {.maxSpeed=70}, false);
    startOuttake();
    pros::delay(1000);
    stopOuttake();

    // Move towards long goal and score
    chassis.moveToPoint(-47, -47, 1800, {.forwards=false}, false);
    chassis.turnToHeading(270, 800, {}, false);
    chassis.moveToPoint(-31, -47, 1200, {.forwards=false, .maxSpeed=70}, false);
    startTopScore();
    pros::delay(1000);
    stopTopScore();

    // Refill at matchloader
    startIntake();
    openGate();
    chassis.moveToPoint(-63, -47, 1200, {.maxSpeed=100}, false);
    pros::delay(500);

    // Score again
    chassis.moveToPoint(-31, -47, 1200, {.maxSpeed=70}, false);
    closeGate();
    startTopScore();
}

void red_soloAWP() {
    chassis.setPose(-47, -13.25, 180);

    // Head towards the matchloader and intake
    chassis.moveToPoint(-47, -42, 1200, {.minSpeed=60, .earlyExitRange=3}, false);
    chassis.turnToHeading(270, 800, {.minSpeed=60, .earlyExitRange=4});
    startIntake();
    openGate();
    //pros::delay(200);
    chassis.moveToPoint(-63, -47, 1150, {.maxSpeed=70}, false);
    //pros::delay(800);

    // Score the long goal
    chassis.moveToPoint(-24, -47, 1100, {.forwards=false, .maxSpeed=70}, false);
    startTopScore();
    closeGate();
    pros::delay(1000);

    // Back off
    chassis.moveToPoint(-38, -47, 600, {.minSpeed=60, .earlyExitRange=3});
    chassis.turnToHeading(30, 800, {}, false);
    stopTopScore();
    
    // Intake 3 balls
    //chassis.moveToPoint(-22, -20, 1200, {.maxSpeed=70, .minSpeed=30, .earlyExitRange=3});

    // Score low goal
    chassis.moveToPoint(-11, -8.5, 1750, {.maxSpeed=73}, false);
    startOuttake();
    pros::delay(50);
    chassis.moveToPoint(-24, -24, 800, {.forwards=false, .minSpeed=40}, false);

    // Intake 3 other balls
    startIntake();
    chassis.turnToHeading(0, 300, {}, false);
    chassis.moveToPoint(-25, 22, 1500, {.maxSpeed=70, .minSpeed=30, .earlyExitRange=3}, true);
    pros::delay(250);
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
    chassis.turnToPoint(-47, 49, 700, {.maxSpeed=90, .minSpeed=30, .earlyExitRange=3}, false);
    chassis.moveToPoint(-47, 49, 1600, {}, false);
    chassis.turnToHeading(270, 500, {}, false);
    startIntake();
    chassis.moveToPoint(-65, 49, 1200, {.maxSpeed=70}, false);

    // Score the other long goal
   // chassis.turnToPoint(-33, 49, 800);
    chassis.moveToPoint(-30, 50, 1200, {.forwards=false, .maxSpeed=70}, false);
    startTopScore();
    closeGate();
}