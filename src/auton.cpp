#include "custom/auton.h"
#include "custom/configs.h"
#include "custom/util_funcs.h"
#include "pros/motors.h"

// Small helper: realign LemLib chassis to MCL estimate and print to brain
static inline void mclRealignAndLog(const char* tag = "") {
    lemlib::Pose est = MclMain.getPoseEstimate();
    chassis.setPose(est.x, est.y, est.theta);
    pros::lcd::print(4, "MCL[%s]  X=%.2f  Y=%.2f  Th=%.2f", tag, est.x, est.y, est.theta);
}

// 9 balls -> Top Middle Goal + Top Long Goal
void red_left() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.setPose(-48.75, 16.125, 90);
    MclMain.initParticles();                // (1) Init MCL at routine start
    mclRealignAndLog("start");

    // Intake three balls
    stopIntake();
    startIntake();
    chassis.moveToPoint(-17, 28, 1500, {}, true);
    pros::delay(450);
    openGate();

    // Intake two balls
    chassis.turnToHeading(10, 800, {}, true);
    pros::delay(100);
    closeGate();
    chassis.moveToPoint(-9.5, 45, 2000, {.maxSpeed=70}, false);

    // (3) Realign before precision score
    mclRealignAndLog("pre-mid");

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

    // Refill at matchloader
    startIntake();
    openGate();
    chassis.moveToPoint(-68, 50, 1200, {.maxSpeed=80, .minSpeed=50}, false);
    pros::delay(200);

    // (3) Realign before final long score
    mclRealignAndLog("pre-long");

    // Score again
    chassis.moveToPoint(-25, 50, 1100, {.forwards = false, .maxSpeed=80}, false);
    startTopScore();
    pros::delay(1000);
    stopTopScore();

    // Optional: show final MCL state
    mclRealignAndLog("end");
}

void blue_left() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.setPose(48.75, -16.125, 270);
    MclMain.initParticles();
    mclRealignAndLog("start");

    // Intake three balls
    stopIntake();
    startIntake();
    chassis.moveToPoint(17, -28, 1500, {}, true);
    pros::delay(450);
    openGate();

    // Intake two balls
    chassis.turnToHeading(190, 800, {}, true);
    pros::delay(100);
    closeGate();
    chassis.moveToPoint(10, -45, 2000, {.maxSpeed=70}, false);

    mclRealignAndLog("pre-mid");

    // Backoff and score middle goal
    chassis.moveToPoint(19, -19, 1200, {.forwards=false}, false);
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

    // Refill at matchloader
    startIntake();
    openGate();
    chassis.moveToPoint(68, -50, 1200, {.maxSpeed=80, .minSpeed=50}, false);
    pros::delay(100);

    mclRealignAndLog("pre-long");

    // Score again
    chassis.moveToPoint(25, -50, 1100, {.forwards=false, .maxSpeed=80}, false);
    closeGate();
    startTopScore();
    pros::delay(2300);
    stopTopScore();
    chassis.moveToPoint(42, -52, 1500, {.minSpeed=60}, false);
    chassis.moveToPoint(27, -54, 750, {.forwards=false, .minSpeed=90}, false);
    startTopScore();
    mclRealignAndLog("end");
}

// 9 balls -> Lower Middle Goal + Lower Long Goal
void red_right_scoreMid() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.setPose(-48.75, -16.125, 90);
    MclMain.initParticles();
    mclRealignAndLog("start");

    // Intake three balls
    stopIntake();
    startIntake();
    chassis.moveToPoint(-17, -28, 1500, {}, true);
    pros::delay(450);
    openGate();

    // Intake two balls
    chassis.turnToHeading(170, 600, {}, true);
    pros::delay(100);
    closeGate();
    chassis.moveToPoint(-9, -47, 1700, {.maxSpeed=70}, false);

    mclRealignAndLog("pre-mid");

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

    // Refill at matchloader
    startIntake();
    openGate();
    chassis.moveToPoint(-68, -48, 1200, {.maxSpeed=80, .minSpeed=50}, false);
    pros::delay(100);

    mclRealignAndLog("pre-long");

    // Score again
    chassis.moveToPoint(-25, -48, 1100, {.forwards=false, .maxSpeed=80}, false);
    closeGate();
    startTopScore();
    pros::delay(2300);
    stopTopScore();
    chassis.moveToPoint(-42, -48, 1500, {.minSpeed=60}, false);
    chassis.moveToPoint(-27, -48, 750, {.forwards=false, .minSpeed=90}, false);
    startTopScore();
    mclRealignAndLog("end");
}

void blue_right_scoreMid() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.setPose(48.75, 16.125, 270);
    MclMain.initParticles();
    mclRealignAndLog("start");

    // Intake three balls
    stopIntake();
    startIntake();
    chassis.moveToPoint(17, 28, 1500, {}, true);
    pros::delay(450);
    openGate();

    // Intake two balls
    chassis.turnToHeading(350, 600, {}, true);
    pros::delay(100);
    closeGate();
    chassis.moveToPoint(9, 47, 1700, {.maxSpeed=70}, false);

    mclRealignAndLog("pre-mid");

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

    // Refill at matchloader
    startIntake();
    openGate();
    chassis.moveToPoint(68, 48, 1200, {.maxSpeed=80, .minSpeed=50}, false);
    pros::delay(100);

    mclRealignAndLog("pre-long");

    // Score again
    chassis.moveToPoint(25, 48, 1100, {.forwards=false, .maxSpeed=80}, false);
    closeGate();
    startTopScore();
    pros::delay(2300);
    stopTopScore();
    chassis.moveToPoint(42, 48, 1500, {.minSpeed=60}, false);
    chassis.moveToPoint(27, 48, 750, {.forwards=false, .minSpeed=90}, false);
    startTopScore();
    mclRealignAndLog("end");
}

// 9 balls -> Lower Long Goal
void red_right_noScoreMid() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.setPose(-48.75, -16.125, 90);
    MclMain.initParticles();
    mclRealignAndLog("start");

    // Intake three balls
    stopIntake();
    startIntake();
    chassis.moveToPoint(-17, -28, 1500, {}, true);
    pros::delay(450);
    openGate();

    // Intake two balls
    chassis.turnToHeading(170, 600, {}, true);
    pros::delay(100);
    closeGate();
    chassis.moveToPoint(-9, -47, 1700, {.maxSpeed=70}, false);

    mclRealignAndLog("pre-mid");

    // Backoff and score middle goal
    chassis.moveToPoint(-18, -24, 1200, {.forwards=false}, false);
    stopIntake();
    chassis.turnToHeading(45, 800, {}, false);
    chassis.moveToPoint(-10, -16, 1500, { .maxSpeed=70}, true);
    pros::delay(850);

    // Move towards long goal and score
    chassis.moveToPoint(-42, -48, 2500, {.forwards=false}, false);
    openGate();
    chassis.turnToHeading(270, 700, {}, false);

    // Refill at matchloader
    startIntake();
    openGate();
    chassis.moveToPoint(-68, -48, 1200, {.maxSpeed=80, .minSpeed=50}, false);
    pros::delay(100);

    mclRealignAndLog("pre-long");

    // Score again
    chassis.moveToPoint(-25, -48, 1100, {.forwards=false, .maxSpeed=80}, false);
    closeGate();
    startTopScore();
    pros::delay(2300);
    stopTopScore();
    chassis.moveToPoint(-42, -48, 1500, {.minSpeed=60}, false);
    chassis.moveToPoint(-27, -48, 750, {.forwards=false, .minSpeed=90}, false);
    startTopScore();
    mclRealignAndLog("end");
}

void blue_right_noScoreMid() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.setPose(48.75, 16.125, 270);
    MclMain.initParticles();
    mclRealignAndLog("start");

    // Intake three balls
    stopIntake();
    startIntake();
    chassis.moveToPoint(17, 28, 1500, {}, true);
    pros::delay(450);
    openGate();

    // Intake two balls
    chassis.turnToHeading(350, 600, {}, true);
    pros::delay(100);
    closeGate();
    chassis.moveToPoint(9, 47, 1700, {.maxSpeed=70}, false);

    mclRealignAndLog("pre-mid");

    // Backoff and score middle goal
    chassis.moveToPoint(18, 24, 1200, {.forwards=false}, false);
    stopIntake();
    chassis.turnToHeading(225, 800, {}, false);
    chassis.moveToPoint(10, 16, 1500, {.maxSpeed=70}, true);
    pros::delay(850);

    // Move towards long goal and score
    chassis.moveToPoint(42, 48, 2500, {.forwards=false}, false);
    openGate();
    chassis.turnToHeading(90, 700, {}, false);

    // Refill at matchloader
    startIntake();
    openGate();
    chassis.moveToPoint(68, 48, 1200, {.maxSpeed=80, .minSpeed=50}, false);
    pros::delay(100);

    mclRealignAndLog("pre-long");

    // Score again
    chassis.moveToPoint(25, 48, 1100, {.forwards=false, .maxSpeed=80}, false);
    closeGate();
    startTopScore();
    pros::delay(2300);
    stopTopScore();
    chassis.moveToPoint(42, 48, 1500, {.minSpeed=60}, false);
    chassis.moveToPoint(27, 48, 750, {.forwards=false, .minSpeed=90}, false);
    startTopScore();
    mclRealignAndLog("end");
}

// Solo AWP
void red_soloAWP() {
    chassis.setPose(-47, -13.25, 180);
    MclMain.initParticles();
    mclRealignAndLog("start");

    // Head towards the matchloader and intake
    openGate();
    chassis.moveToPoint(-47, -38.5, 1200, {.minSpeed=60, .earlyExitRange=3}, false);
    chassis.turnToHeading(265, 800, {.minSpeed=60, .earlyExitRange=4});
    startIntake();
    chassis.moveToPoint(-64, -47.5, 1350, {.maxSpeed=70}, false);

    // Score the long goal
    chassis.moveToPoint(-24, -47, 1000, {.forwards=false, .maxSpeed=90}, false);
    startTopScore();
    closeGate();
    pros::delay(1000);

    // Back off
    chassis.moveToPoint(-40, -47, 600, {.minSpeed=60, .earlyExitRange=3});
    chassis.turnToHeading(30, 800, {.minSpeed=60, .earlyExitRange=3});
    stopTopScore();

    // Score low goal (with intake)
    startIntake();
    chassis.moveToPoint(-15, -8.5, 1400, {.maxSpeed=90}, true);
    pros::delay(950);
    startOuttake();
    pros::delay(650);
    chassis.moveToPoint(-24, -24, 800, {.forwards=false, .minSpeed=70, .earlyExitRange=3});

    // Intake 3 other balls
    startIntake();
    chassis.turnToHeading(0, 300, {.minSpeed=60, .earlyExitRange=3});
    chassis.moveToPoint(-28, 22, 1500, {.maxSpeed=85, .minSpeed=50, .earlyExitRange=3}, true);
    pros::delay(50);
    closeGate();
    pros::delay(1000);
    openGate();

    // Move towards the other matchloader and intake
    chassis.turnToPoint(-47.5, 46, 700, {.minSpeed=50, .earlyExitRange=3});
    chassis.moveToPoint(-47.5, 46, 1600, {.minSpeed=70, .earlyExitRange=3});
    chassis.turnToHeading(275, 500, {.minSpeed=60, .earlyExitRange=3});
    startIntake();
    chassis.moveToPoint(-67.5, 49, 1200, {.maxSpeed=80}, false);

    // Score the other long goal
    chassis.moveToPoint(-30, 50, 1000, {.forwards=false, .maxSpeed=90, .minSpeed=50}, false);
    startTopScore();
    pros::delay(2250);
    chassis.moveToPoint(-42, 50, 1000, {.minSpeed=100});

    mclRealignAndLog("end");
}

void blue_soloAWP() {
    chassis.setPose(-47, -13.25, 180);
    MclMain.initParticles();
    mclRealignAndLog("start");

    // Head towards the matchloader and intake
    openGate();
    chassis.moveToPoint(-47, -38.5, 1200, {.minSpeed=60, .earlyExitRange=3}, false);
    chassis.turnToHeading(265, 800, {.minSpeed=60, .earlyExitRange=4});
    startIntake();
    chassis.moveToPoint(-64, -47.5, 1350, {.maxSpeed=70}, false);

    // Score the long goal
    chassis.moveToPoint(-24, -47, 1000, {.forwards=false, .maxSpeed=90}, false);
    startTopScore();
    closeGate();
    pros::delay(1000);

    // Back off
    chassis.moveToPoint(-40, -47, 600, {.minSpeed=60, .earlyExitRange=3});
    chassis.turnToHeading(30, 800, {.minSpeed=60, .earlyExitRange=3});
    stopTopScore();

    // Score low goal (with intake)
    startIntake();
    chassis.moveToPoint(-15, -8, 1400, {.maxSpeed=90}, true);
    pros::delay(950);
    startOuttake();
    pros::delay(700);
    chassis.moveToPoint(-24, -24, 800, {.forwards=false, .minSpeed=70, .earlyExitRange=3});

    // Intake 3 other balls
    startIntake();
    chassis.turnToHeading(0, 300, {.minSpeed=60, .earlyExitRange=3});
    chassis.moveToPoint(-28, 22, 1500, {.maxSpeed=85, .minSpeed=50, .earlyExitRange=3}, true);
    pros::delay(50);
    closeGate();
    pros::delay(1000);
    openGate();

    // Move towards the other matchloader and intake
    chassis.turnToPoint(-47, 49, 700, {.minSpeed=50, .earlyExitRange=3});
    chassis.moveToPoint(-47, 49, 1600, {.minSpeed=70, .earlyExitRange=3});
    chassis.turnToHeading(275, 500, {.minSpeed=60, .earlyExitRange=3});
    startIntake();
    chassis.moveToPoint(-67.5, 49, 1200, {.maxSpeed=80}, false);

    // Score the other long goal
    chassis.moveToPoint(-30, 49, 1000, {.forwards=false, .maxSpeed=90, .minSpeed=50}, false);
    startTopScore();
    pros::delay(2250);
    chassis.moveToPoint(-42, 49, 1000, {.minSpeed=100});

    mclRealignAndLog("end");
}

void skills() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.setPose(-47, -13.25, 180);
    MclMain.initParticles();
    mclRealignAndLog("start");

    // Head towards bottom-left match loader
    chassis.moveToPoint(-47, -46, 1800, {}, false);
    chassis.turnToHeading(270, 700, {});
    startIntake();
    openGate();
    chassis.moveToPoint(-64, -46, 1000, {.maxSpeed=70}, false);
    chassis.moveToPoint(-57, -46, 700, {}, false);
    chassis.moveToPoint(-64, -46, 300, {.maxSpeed=70}, false);
    pros::delay(1500);

    // Back off (#1)
    stopIntake();
    closeGate();
    chassis.moveToPoint(-49, -48,  1000, {.forwards=false}, false);
    chassis.turnToHeading(135, 800, {}, false);

    // Score bottom-right
    chassis.moveToPose(-15, -59, 90, 1500, {.lead=0.3, .minSpeed=55}, false);
    chassis.moveToPoint(20, -59, 2000, {.minSpeed=30, .earlyExitRange=12}, false);
    chassis.moveToPoint(38, -58, 1000, {.maxSpeed=60}, false);
    chassis.turnToHeading(0, 600, {});
    chassis.moveToPoint(38, -48, 1200, {}, false);
    chassis.turnToHeading(90, 600, {}, false);

    // Score low/mid
    chassis.moveToPoint(24, -48, 1000, {.forwards=false, .maxSpeed=70}, false);
    startOuttake();
    pros::delay(25);
    startTopScore();
    pros::delay(2750);

    // Head towards bottom-right match loader
    startIntake();
    openGate();
    chassis.moveToPoint(64, -48, 2000, {.maxSpeed=70}, false);
    pros::delay(1200);

    // Score the long goal
    chassis.moveToPoint(24, -48, 1100, {.forwards=false, .maxSpeed=70}, false);
    startTopScore();
    closeGate();
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
    pros::delay(1500);

    // Back off (#3)
    stopIntake();
    closeGate();
    chassis.moveToPoint(49, 46, 1000, {.forwards=false}, false);
    chassis.turnToHeading(315, 700, {}, false);

    // Score the top-left goal
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
    chassis.moveToPoint(-63, 50, 1000, {.maxSpeed=70}, false);
    chassis.moveToPoint(-63, 50, 700, {.maxSpeed=70}, false);
    chassis.moveToPoint(-63, 50, 400, {.maxSpeed=70}, false);
    pros::delay(1000);

    // Score the long goal
    chassis.moveToPoint(-24, 50, 1100, {.forwards=false, .maxSpeed=70}, false);
    startTopScore();
    closeGate();
    pros::delay(3000);

    // Back off (#4)
    chassis.moveToPoint(-42, 47, 1000, {}, false);
    chassis.turnToHeading(180, 800, {}, false);
    stopTopScore();

    // Park
    chassis.moveToPoint(-42, 2, 2500, {}, false);
    chassis.turnToHeading(-270, 800, {}, false);
    odomLift.extend();
    startTopScore();
    chassis.moveToPoint(-70, 2, 3000, {.forwards = false}, false);

    mclRealignAndLog("end");
}
