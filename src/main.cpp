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

	//FOR SKILLS TESTING***

	chassis.setPose(-47, -13.25, 180);

    // Head towards the matchloader and intake
    openGate();
    chassis.moveToPoint(-47, -39.5, 1200, {.minSpeed=60, .earlyExitRange=3}, false);
    chassis.turnToHeading(260, 800, {.minSpeed=60, .earlyExitRange=4});
    startIntake();
    //pros::delay(200);
    chassis.moveToPoint(-67, -47.5, 2500, {.maxSpeed=70}, false);
    //pros::delay(800);

    // Move to other side of long goal and score
	chassis.moveToPoint(-48, -48, 1000, {.forwards=false}, false);
	closeGate();
	chassis.turnToHeading(180, 700, {}, false);
	chassis.moveToPoint(-48, -63, 1000, {}, false);
	chassis.turnToHeading(90, 700, {}, false);


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