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
    chassis.setPose(-51, -14, 180);

    // Head towards the matchloader and intake
    chassis.moveToPoint(-51, -47, 1200, {}, false);
    chassis.turnToHeading(270, 800, {}, false);
    startIntake();
    openGate();
    chassis.moveToPoint(-63, -47, 1000, {.maxSpeed=70}, false);
    pros::delay(800);

    // Score the long goal
    chassis.moveToPoint(-31, -47, 1200, {.forwards=false, .maxSpeed=70}, false);
    startTopScore();
    closeGate();
    pros::delay(1000);
    stopTopScore();

    // Back off
    chassis.moveToPoint(-36, -47, 600, {}, false);
    chassis.turnToHeading(30, 800, {}, false);
    
    // Intake 3 balls
    startIntake();
    chassis.moveToPoint(-22, -22, 1200, {}, false);
    openGate();

    // Intake 3 other balls
    chassis.turnToHeading(0, 300, {}, false);
    chassis.moveToPoint(-22, 22, 1500, {}, true);
    pros::delay(500);
    closeGate();
    pros::delay(1000);
    openGate();

    // Score the mid goal
    chassis.turnToHeading(315, 400, {}, false);
    chassis.moveToPoint(-12, 12, 1000, {.forwards=false, .maxSpeed=70}, false);
    startOuttake();
    pros::delay(800);
    stopOuttake();

    // Move towads the other matchloader and intake
    chassis.moveToPoint(-47, 47, 1600, {}, false);
    chassis.turnToHeading(270, 500, {}, false);
    startIntake();
    chassis.moveToPoint(-63, 47, 1300, {.maxSpeed=70}, false);

    // Score the other long goal
    chassis.moveToPoint(-31, 47, 1200, {.forwards=false, .maxSpeed=70}, false);
    startOuttake();
    closeGate();
}