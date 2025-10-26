#include "main.h"
#include "custom/RclTracking.h"
#include "custom/configs.h"
#include "custom/auton.h"
#include "custom/util_funcs.h"
#include "custom/auton_selector.h" // IWYU pragma: keep

void initialize() {
    chassis.calibrate();
    chassis.setPose(64, -62, 0);

    init_auton_selector();
//	startControllerAutonSelectorDisplay();
//startControllerRclDisplay();
startControllerDisplay();
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
		skills();

	// Auton Selection
	/*if (runningSkills) {
		skills();
		return;
	}

	switch (allianceColor) {
		case alliance_color::RED:
			switch (autonType) {
				case autonTypes::LEFT:
					red_left();
					return;
				case autonTypes::RIGHT_NOMID:
					red_right_noScoreMid();
					return;
				case autonTypes::RIGHT_WMID:
					red_right_scoreMid();
					return;
				case autonTypes::SOLO_AWP:
					red_soloAWP();
					return;
				default:
					red_left();
					return;
			}
			return;
		case alliance_color::BLUE:
			switch (autonType) {
				case autonTypes::LEFT:
					blue_left();
					return;
				case autonTypes::RIGHT_NOMID:
					blue_right_noScoreMid();
					return;
				case autonTypes::RIGHT_WMID:
					blue_right_scoreMid();
					return;
				case autonTypes::SOLO_AWP:
					blue_soloAWP();
					return;
				default:
					blue_left();
					return;
			}
			return;
		default:
			blue_soloAWP();
			return;
	}	*/


}

void opcontrol() {
	
	startControllerDisplay();

	odomLift.extend();

	// Retract both descore arms
	extendLeftArm();
	extendRightArm();

	// Display FB Logo
	pros::Task ([](){pros::delay(100); startBrainFBDisplay();});

	while (true) {
		// Update Controls
		updateTankDrive();
		updateIntake();
		updatePneumatics();

		pros::delay(20);
	}

	
}