#pragma once

#include "configs.h"

// Intake Controls
void indexerIn();
void indexerOut();
void stopIndexer();
void updateIndexer();

// Tank Drive Update
void updateTankDrive();

// Display
static bool controllerDiplsayStarted = false;
void startControllerDisplay();
static bool brainDisplayStarted = false;
void startBrainDisplay();