#pragma once

#include "custom/configs.h"

// Controls
void updateIntake();
void updatePneumatics();
void updateTankDrive();

// Display
extern bool controllerDiplsayStarted;
extern pros::Task controllerScreenTask;
void startControllerDisplay();
extern bool brainDisplayStarted;
extern pros::Task brainScreenTask;
void startBrainDisplay();