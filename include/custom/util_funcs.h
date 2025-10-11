#pragma once

#include "custom/configs.h"

// Controls
void updateIntake();
void updatePneumatics();
void updateTankDrive();

// Display
inline bool controllerDiplsayStarted = false;
inline pros::Task controllerScreenTask([](){});
void startControllerDisplay();
inline bool brainDisplayStarted = false;
inline pros::Task brainScreenTask([](){});
void startBrainDisplay();