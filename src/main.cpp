#include "main.h"
#include "custom/configs.h"
#include "custom/auton.h"
#include "custom/util_funcs.h"
#include "custom/auton_selector.h" // IWYU pragma: keep
#include "custom/RclTracking.h"

void initialize() {
	pros::lcd::initialize();
	chassis.calibrate();
	chassis.setPose(0, 0, 0);
	//RclMain.startTracking();
	
	startBrainDisplay();
	startControllerDisplay();
}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol() {
	while (true) {
		// Indexer control
		updateIndexer();

		// Tank Drive
		updateTankDrive();

		pros::delay(20);
	}

}