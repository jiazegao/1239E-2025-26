#include "main.h"
#include "custom/RclTracking.hpp"
#include "custom/configs.hpp"
#include "custom/auton.hpp"

#include "custom/util_funcs.hpp"
#include "custom/auton_selector.hpp" // IWYU pragma: keep
#include "liblvgl/llemu.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/motors.h"

#include "custom/lever_control.hpp"

void initialize() {
    chassis.calibrate();
    chassis.setPose(0, 0, 0);
	initControllerDisplay();

	initLeverControl();

	// Brain display (disabled)
	// initBrainDisplay();

	// Ensure odom pod is down
	odomLift.retract();

	// Auton Selection
	startControllerAutonSelectorDisplay();
	init_auton_selector();

	RclMain.startTracking();

	// Set Optical LED
	frontOptic.set_led_pwm(100);
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
	
	odomLift.retract();
	chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);

	startControllerRclCoordDisplay();

	// Ensure descore arms are retracted
	extendLeftArm();
	extendLeftArm();

	//soloAWP();
	runAuton();
}

void opcontrol() {
	
	//startControllerMatchDisplay(); 
	chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);

	odomLift.extend();

	// Retract both descore arms
	extendLeftArm();
	stopIntake();

	RclMain.setMaxSyncPerSec(0.001);
	startControllerCoordDisplay();

	// Display FB Logo
	startBrainFBDisplay();

	while (true) {
		// Update Controls
		updateTankDrive();
		updatePneumatics();

		pros::delay(20);
	}
}