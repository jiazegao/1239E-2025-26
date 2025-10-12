#pragma once

#include "custom/configs.h"

// Controls
void updateIntake();
void updatePneumatics();
void updateTankDrive();

// Auton functions
void startIntake();
void stopIntake();
void startOuttake();
void stopOuttake();
void startTopScore();
void stopTopScore();

// Pneumatics functions
void openGate();
void closeGate();
void openMid();
void closeMid();
void extendLeftArm();
void retractLeftArm();
void extendRightArm();
void retractRightArm();

// Display
inline bool controllerDiplsayStarted = false;
inline pros::Task controllerScreenTask([](){});
void startControllerDisplay();
inline bool brainDisplayStarted = false;
inline pros::Task brainScreenTask([](){});
void startBrainDisplay();