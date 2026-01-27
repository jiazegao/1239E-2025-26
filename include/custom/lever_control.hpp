#pragma once

#include "configs.hpp"

// ----------------- LEVER MOTOR CONTROL -----------------

void initLeverControl();
/*
    - Must be called within void initialize()
    - Starts the pros::Task that manages lever motor control
*/
void score(int count = 1, bool slowScore = false);
/*
    - Scores a certain number of blocks in the intake
    - If count > balls in the intake, score what's in the intake
*/
void scoreColor(alliance_color color = allianceColor, bool slowScore = false);
/*
    - Scores all balls with the target color at the top of the intake
*/
void scoreAll(bool slowScore = false);
/*
    - Scores all balls in the intake
*/
void intakeFromMatchLoader(alliance_color color = allianceColor);
/*
    - Attempts to intake from matchload
    - Automatically spits out the wrong color balls
*/


// -------------------- Intake Control --------------------

void stopIntake();
/*
    - Deactivates front intake
*/
void startIntake();
/*
    - Activates front intake inward
*/
void startOuttake();
/*
    - Activates front intake outward
*/


// ----------------- SCORING LEVEL CONTROl -----------------

void extendLift();
/*
    - Extend lift piston -> Score long goal
*/
void retractLift();
/*
    - Retract lift piston -> Score top middle goal
*/