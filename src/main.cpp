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


    //RclMain.startTracking();
}

void disabled() {}

void competition_initialize() {}

void autonomous() {

	//Ensure odom pod is down
	odomLift.retract();

	//FOR SKILLS TESTING***

	red_left();

}

void opcontrol() {

	while (true) {
		// Update Controls
		updateTankDrive();
		updateIntake();
		updatePneumatics();

		odomLift.extend();

		pros::delay(20);
	}

	
}