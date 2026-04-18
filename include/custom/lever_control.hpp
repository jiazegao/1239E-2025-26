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

enum LEVER_STAGE {INACTIVE, PRIMING, INTAKING, OUTTAKING, RAISING, LOWERING};
inline bool lever_antistuck_on = false;
inline bool fast_lever_score = false;
void startPriming();
void endPriming();
LEVER_STAGE getLeverStage();
inline bool intake_macro_lock = false;

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
void scoreReserve(int timeOut = 1000, int reserving = 0, int maxScoringSpeed = FAST_TOP_SCORE);

void scoreColor(int timeOut = 1000, int maxScoringSpeed = FAST_TOP_SCORE, alliance_color color = alliance_color::NONE);
/*
    - Scores all balls with the target color at the top of the intakes
*/
void scoreAll(int timeOut = 1000, int maxScoringSpeed = FAST_TOP_SCORE);
/*
    - Scores all balls in the intake
*/
void intakeFromMatchLoader(alliance_color color);
/*
    - Attempts to intake from matchload
    - Automatically spits out the wrong color balls
*/
void endIntakeMacro();
void hardResetLever();


// -------------------- Intake Control --------------------

void stopIntake();
/*
    - Deactivates front intake
*/
void startIntake();
/*
    - Activates front intake inward
*/
void startOuttake(int speed = 600);
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
void openHood();
void closeHood();
inline bool hoodLock = false;
inline bool fastOuttake = false;
inline bool antiStuckOn = true;

// ----------------- MISCELLANEOUS --------------------------

void resetLever();

void setAutoReset(bool newConfig);

void startLeverTuningDisplay();

void openGate();

void closeGate();

void toggleGate();

void intakeLiftLock(bool up);