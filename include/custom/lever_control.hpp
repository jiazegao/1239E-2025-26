#pragma once

#include "configs.hpp"

// ----------------- LEVER MOTOR CONTROL -----------------

void initLeverControl();
/*
    - Must be called within void initialize()
    - Starts the pros::Task that manages lever motor control
*/
void score(int count = 1);
/*
    - Scores a certain number of blocks in the intake
    - If count > balls in the intake, score what's in the intake
*/
void scoreColor(alliance_color color = allianceColor);
/*
    - Scores all balls with the target color at the top of the intake
*/
void scoreAll();
/*
    - Scores all balls in the intake
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
s*/


// ----------------- SCORING LEVEL CONTROl -----------------

void extendLift();
/*
    - Extend lift piston -> Score long goal
*/
void retractLift();
/*
    - Retract lift piston -> Score top middle goal
*/