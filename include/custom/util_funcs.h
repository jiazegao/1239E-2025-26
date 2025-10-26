#pragma once

#include "custom/configs.h"
#include "liblvgl/lvgl.h"

// Controls
void updateIntake();
void updatePneumatics();
void updateTankDrive();

// Auton functions
void startIntake();
void stopIntake();
void startOuttake();
void stopOuttake();
void startTopScore(int velocity = 127);
void stopTopScore();
void startMidScore();
void stopMidScore();

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
void stopControllerDisplay();
void startControllerDisplay();
void startControllerAutonSelectorDisplay();
void startControllerRclDisplay();

LV_IMAGE_DECLARE(FB_Logo);
inline lv_obj_t *image;
inline pros::Task* brainScreenTask = nullptr;
void stopBrainDisplay();
void startBrainDisplay();
void startBrainFBDisplay();