#include "main.h"
#include "custom/RclTracking.h"
#include "custom/configs.h"
#include "custom/auton.h"
#include "custom/util_funcs.h"
#include "custom/auton_selector.h" // IWYU pragma: keep
#include "pros/motors.h"

void initialize() {
    chassis.calibrate();
    chassis.setPose(0, 0, 0);

}

void disabled() {}

void competition_initialize() {}

void autonomous() {
	startControllerDisplay();
	chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
	chassis.setPose(0, 0, 0);
	//1
	while (true) {
	chassis.moveToPoint(0, 	190, 99999, {}); // Brief delay to ensure everything is initialized
	//chassis.turnToPoint(114, 114, 9999);
	chassis.waitUntilDone();
	chassis.turnToHeading(90, 600, { .minSpeed=50}, false);
	chassis.moveToPoint(305, 195, 9999);
	chassis.waitUntilDone();
	chassis.turnToHeading(180, 400);
	//3
	chassis.moveToPoint(300, -130, 90999);
	chassis.waitUntilDone();
	//4
	chassis.turnToHeading(270, 400);
	chassis.moveToPoint(-0, -140, 99999);
	//5
	chassis.turnToHeading(0, 400);
	chassis.moveToPoint(0, 0, 99999);
	}
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