#include "main.h"
#include "custom/RclTracking.h"
#include "custom/configs.h"
#include "custom/auton.h"
#include "custom/util_funcs.h"
#include "custom/auton_selector.h" // IWYU pragma: keep
#include "pros/motors.h"
#include "pros/motors.h"

void initialize() {
    chassis.calibrate();
    chassis.setPose(0, 0, 0);

}

void disabled() {}

void competition_initialize() {}

void autonomous() {
	chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
	chassis.setPose(0, 0, 0);
	chassis.moveToPoint(0, 	170, 99999, {}); // Brief delay to ensure everything is initialized
	//chassis.turnToPoint(114, 114, 9999);
	chassis.waitUntilDone();
	chassis.turnToHeading(90, 600);
	chassis.moveToPoint(170, 170, 90999);
	chassis.waitUntilDone();
	chassis.turnToHeading(180, 400);
	chassis.moveToPoint(170, -170, 90999);
	chassis.waitUntilDone();
	chassis.turnToHeading(270, 400);
	chassis.moveToPoint(-170, -170, 99999);
}

void opcontrol() {
	
	startControllerDisplay();
	//startControllerRclDisplay();
	chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);

	odomLift.extend();

	// Retract both descore arms
	extendLeftArm();
	extendRightArm();
	stopTopScore();

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