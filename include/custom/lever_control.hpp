#pragma once

#include "configs.hpp"

// ----------------- LEVER MOTOR CONTROL -----------------

// Scoring Presets (rpm)
const int FAST_TOP_SCORE = 100;
const int FAST_MID_SCORE = 60;
const int MODERATE_TOP_SCORE = 70;
const int MODERATE_MID_SCORE = 40;
const int SLOW_TOP_SCORE = 50;
const int SLOW_MID_SCORE = 30;

void initLeverControl();
/*
    - Must be called within void initialize()
    - Starts the pros::Task that manages lever motor control
*/
void score(int timeOut = 1000, int count = 1, int maxScoringSpeed = FAST_TOP_SCORE);
/*
    - Scores a certain number of blocks in the intake
    - If count > balls in the intake, score what's in the intake
*/
void scoreColor(int timeOut = 1000, int maxScoringSpeed = FAST_TOP_SCORE, alliance_color color = allianceColor);
/*
    - Scores all balls with the target color at the top of the intake
*/
void scoreAll(int timeOut = 1000, int maxScoringSpeed = FAST_TOP_SCORE);
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


// ----------------- MISCELLANEOUS --------------------------

void resetLever();

void setAutoReset(bool newConfig);

void startLeverTuningDisplay();