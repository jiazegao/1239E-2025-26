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
    openGate();
    chassis.moveToPoint(-47, -39.5, 1200, {.minSpeed=60, .earlyExitRange=3}, false);
    chassis.turnToHeading(260, 800, {.minSpeed=60, .earlyExitRange=4});
    startIntake();
    //pros::delay(200);
    chassis.moveToPoint(-64, -47.5, 1200, {.maxSpeed=70}, false);
    //pros::delay(800);

    // Score the long goal
    chassis.moveToPoint(-24, -47, 1000, {.forwards=false, .maxSpeed=90}, false);
    startTopScore();
    closeGate();
    pros::delay(1000);

    // Back off
    chassis.moveToPoint(-38, -47, 600, {.minSpeed=60, .earlyExitRange=3});
    chassis.turnToHeading(30, 800, {.minSpeed=60, .earlyExitRange=3});
    stopTopScore();
    
    // Intake 3 balls
    //chassis.moveToPoint(-22, -20, 1200, {.maxSpeed=70, .minSpeed=30, .earlyExitRange=3});

    // Score low goal
    chassis.moveToPoint(-11, -8.5, 1450, {.maxSpeed=90}, false);
    startOuttake();
    pros::delay(250);
    chassis.moveToPoint(-24, -24, 800, {.forwards=false, .minSpeed=70, .earlyExitRange=3});

    // Intake 3 other balls
    startIntake();
    chassis.turnToHeading(0, 300, {.minSpeed=60, .earlyExitRange=3});
    chassis.moveToPoint(-25, 22, 1500, {.maxSpeed=85, .minSpeed=50, .earlyExitRange=3}, true);
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
    chassis.turnToPoint(-47, 45.5, 700, {.minSpeed=50, .earlyExitRange=3});
    chassis.moveToPoint(-47, 45.5, 1600, {.minSpeed=70, .earlyExitRange=3});
    chassis.turnToHeading(275, 500, {.minSpeed=60, .earlyExitRange=3});
    startIntake();
    chassis.moveToPoint(-67.5, 49, 1200, {.maxSpeed=80}, false);

    // Score the other long goal
    // chassis.turnToPoint(-33, 49, 800);
    chassis.moveToPoint(-30, 50, 1000, {.forwards=false, .maxSpeed=90, .minSpeed=50}, false);
    startTopScore();
    pros::delay(2250);
    chassis.moveToPoint(-42, 52, 1000, {.minSpeed=100});
}

void skills() {
    chassis.setPose(-47, -13.25, 180);

    // Head towards bottom-left match loader
    chassis.moveToPoint(-47, -47, 2000, {}, false);
    chassis.turnToHeading(270, 800, {});
    startIntake();
    openGate();
    chassis.moveToPoint(-63, -47, 1150, {.maxSpeed=70}, false);
    pros::delay(1200);

    // Score the long goal
    chassis.moveToPoint(-24, -47, 1100, {.forwards=false, .maxSpeed=70}, false);
    startTopScore();
    closeGate();
    pros::delay(1000);

    // Back off (#1)
    chassis.moveToPoint(-38, -47, 1000, {});
    chassis.turnToHeading(180, 800, {}, false);
    stopTopScore();

    // Score bottom-right
    chassis.moveToPoint(-38, -60, 1200, {}, false);
    chassis.turnToHeading(90, 800, {});
    chassis.moveToPoint(38, -60, 3000, {}, false);
    chassis.turnToHeading(0, 800, {});
    chassis.moveToPoint(38, -47, 1200, {}, false);
    chassis.turnToHeading(90, 800, {}, false);

    chassis.moveToPoint(24, -47, 1100, {.forwards=false, .maxSpeed=70}, false);
    startTopScore();
    pros::delay(1000);
    
    // Head towards bottom-right match loader
    startIntake();
    openGate();
    chassis.moveToPoint(63, -47, 1600, {.maxSpeed=70}, false);
    pros::delay(1200);

    // Score the long goal
    chassis.moveToPoint(24, -47, 1100, {.forwards=false, .maxSpeed=70}, false);
    startTopScore();
    closeGate();
    pros::delay(2500);

    // Back off (#2)
    chassis.moveToPoint(42, -47, 1000, {}, false);
    chassis.turnToHeading(0, 800, {}, false);
    stopTopScore();

    // Head towards top-right match loader
    chassis.moveToPoint(42, 47, 3000, {}, false);
    chassis.turnToHeading(90, 800, {});
    startIntake();
    openGate();
    chassis.moveToPoint(63, 47, 1150, {.maxSpeed=70}, false);
    pros::delay(1200);

    // Score the long goal
    chassis.moveToPoint(24, 47, 1100, {.forwards=false, .maxSpeed=70}, false);
    startTopScore();
    closeGate();
    pros::delay(1000);

    // Back off (#3)
    chassis.moveToPoint(38, 47, 1000, {});
    chassis.turnToHeading(0, 800, {}, false);
    stopTopScore();

    // Score the top-left goal
    chassis.moveToPoint(38, 60, 1200, {}, false);
    chassis.turnToHeading(270, 800, {});
    chassis.moveToPoint(-38, 60, 3000, {}, false);
    chassis.turnToHeading(180, 800, {});
    chassis.moveToPoint(-38, 47, 1200, {}, false);
    chassis.turnToHeading(270, 800, {}, false);

    chassis.moveToPoint(-24, 47, 1100, {.forwards=false, .maxSpeed=70}, false);
    startTopScore();
    pros::delay(1000);
    
    // Intake from top-left match loader
    startIntake();
    openGate();
    chassis.moveToPoint(-63, 47, 1600, {.maxSpeed=70}, false);
    pros::delay(1200);

    // Score the long goal
    chassis.moveToPoint(-24, 47, 1100, {.forwards=false, .maxSpeed=70}, false);
    startTopScore();
    closeGate();
    pros::delay(2500);

    // Back off (#4)
    chassis.moveToPoint(-42, 47, 1000, {}, false);
    chassis.turnToHeading(180, 800, {}, false);
    stopTopScore();

    // Park
    chassis.moveToPoint(-42, 0, 2500, {}, false);
    chassis.turnToHeading(270, 800, {}, false);
    chassis.moveToPoint(-63, 0, 1200, {}, false);

}