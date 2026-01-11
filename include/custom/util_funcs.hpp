#pragma once

#include "custom/configs.hpp"
#include "liblvgl/lvgl.h"

// Controls
void updateIntake();
void updatePneumatics();
void updateTankDrive();
void updateArcadeDrive();
void updateCurvatureDrive();
// Auton functions
inline pros::Task* colorOuttakeTask;
inline bool outtakeTaskRunning = false;
void startIntake();
void stopIntake();
void startOuttake();
void stopOuttake();
void startTopScore(int velocity = 127);
void startTopScore(alliance_color color);
void stopTopScore();
void startMidScore();
void stopMidScore();
double pivot(double curr_corrd, double pivot_coord);
double pivot_x(double pivot_coord);
double pivot_y(double pivot_coord);

// Pneumatics functions
void moveForward(double inches, int timeout, float maxSpeed=127, float minSpeed=1, bool async=true);
void jiggle(int repeats, int time, float forward=8.0, float backward=1.5);
void shake(int repeats, int time);
void openGate();
void closeGate();
void openMid();
void closeMid();
void extendMidDescore();
void retractMidDescore();
void extendLeftArm();
void retractLeftArm();

// Display
inline pros::Task* controllerScreenTask = nullptr;
void stopControllerDisplay();
void startControllerDisplay();
void startControllerAutonSelectorDisplay();
void startControllerRclDisplay();
void startControllerMatchDisplay();

LV_IMAGE_DECLARE(FB_Logo);
inline pros::Task* brainScreenTask = nullptr;
void stopBrainDisplay();
void startBrainDisplay();
void startBrainFBDisplay();

// Test Functions
void startControllerDistDataDisplay();
void startControllerOpticDisplay();
void startControllerRCLUpdate();

void startMclBenchmark();