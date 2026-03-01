#pragma once

#include "custom/configs.hpp"
#include "liblvgl/lvgl.h"

// Controls
void updatePneumatics();
void updateTankDrive();
void updateIntake();

// Pneumatics functions
void moveForward(float inches, int timeout, float maxSpeed=127, float minSpeed=1, bool async=true);
void moveBackward(float inches, int timeout, float maxSpeed, float minSpeed,  bool async);
void jiggle(int repeats, int time, float forward=8.0, float backward=1.5);
void shake(int repeats, int time);
void openGate();
void closeGate();
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

void startBrainCoordDisplay();
void startBrainFBDisplay();

// Test Functions
void startControllerOpticDisplay();
void startControllerRCLInfoDisplay();

// MCL Functions
void startMclBenchmark(float x=0, float y=0, float theta=270, float autoReset = true);
void startMcl(float x, float y, float vexTheta, bool resetLeft, bool resetBack, bool resetRight);
void initLog();