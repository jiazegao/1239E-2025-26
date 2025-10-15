#include "main.h"
#include "custom/RclTracking.h"
#include "custom/configs.h"
#include "custom/auton.h"
#include "custom/util_funcs.h"
#include "custom/auton_selector.h" // IWYU pragma: keep

void initialize() {
    chassis.calibrate();
    chassis.setPose(0, 0, 0);

    //init_auton_selector();


    pros::lcd::initialize();
    startBrainDisplay();
    startControllerDisplay();


    //RclMain.startTracking();
}

void disabled() {}

void competition_initialize() {}

void autonomous() {

	//FOR TESTING***

	red_soloAWP();

	// Red Autons
	/*if (autonColor == autonColors::RED_AUTON) {
		if (autonType == autonTypes::LEFT) {
			red_left();
		} else if (autonType == autonTypes::RIGHT) {
			red_right();
		} else if (autonType == autonTypes::SOLO_AWP) {
			red_soloAWP();
		}
	}*/
}

void opcontrol() {

	while (true) {
		// Update Controls
		updateTankDrive();
		updateIntake();
		updatePneumatics();

		pros::delay(20);
	}

	
}