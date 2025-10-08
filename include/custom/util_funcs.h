#pragma once

#include "custom/configs.h"

// Controls
void updateIntake();
void updatePneumatics();
void updateTankDrive();

// Display
static bool controllerDiplsayStarted = false;
static pros::Task controllerScreenTask([](){});
void startControllerDisplay();
static bool brainDisplayStarted = false;
static pros::Task brainScreenTask([](){});
void startBrainDisplay();