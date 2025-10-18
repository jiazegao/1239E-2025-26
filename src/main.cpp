#include "main.h"
#include "custom/RclTracking.h"
#include "custom/configs.h"
#include "custom/auton.h"
#include "custom/util_funcs.h"
#include "custom/auton_selector.h" // IWYU pragma: keep

void initialize() {
    chassis.calibrate();
    chassis.setPose(0, 0, 0);

    init_auton_selector();
	startControllerAutonSelectorDisplay();
	//startControllerDisplay();

    RclMain.startTracking();
}

void disabled() {}

void competition_initialize() {}

void autonomous() {

	//Ensure odom pod is down
	odomLift.retract();

	// Ensure descore arms are retracted
	extendLeftArm();
	extendRightArm();

	// Auton Selection
	if (runningSkills) {
		skills();
		return;
	}

	switch (allianceColor) {
		case alliance_color::RED:
			switch (autonType) {
				case autonTypes::LEFT:
					red_left();
					break;
				case autonTypes::RIGHT:
					red_right();
					break;
				case autonTypes::SOLO_AWP:
					red_soloAWP();
					break;
				default:
					red_left();
					break;
			}
			break;
		case alliance_color::BLUE:
			switch (autonType) {
				case autonTypes::LEFT:
					blue_left();
					break;
				case autonTypes::RIGHT:
					blue_right();
					break;
				case autonTypes::SOLO_AWP:
					blue_soloAWP();
					break;
				default:
					blue_left();
					break;
			}
			break;
		default:
			blue_soloAWP();
			break;
	}	


}

void opcontrol() {
	
	stopControllerDisplay();
	startControllerDisplay();

	//odomLift.extend();

	// Retract both descore arms
	extendLeftArm();
	extendRightArm();

	while (true) {
		// Update Controls
		updateTankDrive();
		updateIntake();
		updatePneumatics();

		pros::delay(20);
	}

	
}