#pragma once

#include "custom/configs.h"

// Intake Controls
void indexerIn();
void indexerOut();
void stopIndexer();
void updateIndexer();

// Tank Drive Update
void updateTankDrive();

// Display
static bool controllerDiplsayStarted = false;
static pros::Task controllerScreenTask([](){});
void startControllerDisplay();
static bool brainDisplayStarted = false;
static pros::Task brainScreenTask([](){});
void startBrainDisplay();