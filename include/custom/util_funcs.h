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
inline pros::Task* controllerScreenTask = nullptr;
void startControllerDisplay();
void stopControllerDisplay();
inline pros::Task* brainScreenTask = nullptr;
void startBrainDisplay();
void stopBrainDisplay();