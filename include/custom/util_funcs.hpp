#pragma once

#include "custom/configs.hpp"
#include "liblvgl/lvgl.h"

// Controls
void updateIntake();
void updatePneumatics();
void updateTankDrive();

// Auton functions
inline pros::Task* colorOuttakeTask = nullptr;
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
float pivot(float curr_corrd, float pivot_coord);
float pivot_x(float pivot_coord);
float pivot_y(float pivot_coord);

// Pneumatics functions
void moveForward(float inches, int timeout, float maxSpeed=127, float minSpeed=1, bool async=true);
void moveBackward(float inches, int timeout, float maxSpeed, float minSpeed,  bool async);
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
LV_IMAGE_DECLARE(FB_Logo);

inline pros::Task* brainDisplayTask = nullptr;
inline pros::Task* controllerDisplayTask = nullptr;
inline void (*brainDisplayFunc)() = [](){};
inline void (*controllerDisplayFunc)() = [](){};
inline int brainDisplayDelay = 50;
inline int controllerDisplayDelay = 100;

void initControllerDisplay();
void initBrainDisplay();

void startControllerCoordDisplay();
void startControllerAutonSelectorDisplay();
void startControllerRclCoordDisplay();

void startBrainCoordDisplay();
void startBrainFBDisplay();

// Test Functions
void startControllerDistDataDisplay();
void startControllerOpticDisplay();
void startControllerRCLInfoDisplay();

// MCL Functions
void startMclBenchmark(float x=0, float y=0, float theta=270, float autoReset = true);
void startMcl(float x, float y, float vexTheta, bool resetLeft, bool resetBack, bool resetRight);
void initLog();

// PID Tuner
void runPIDTuner();