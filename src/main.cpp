#include "main.h"
#include "custom/RclTracking.hpp"
#include "custom/configs.hpp"
#include "custom/auton.hpp"

#include "custom/util_funcs.hpp"
#include "custom/auton_selector.hpp" // IWYU pragma: keep
#include "liblvgl/llemu.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/motors.h"

#include "custom/lever_control.hpp"

void initialize() {
    chassis.calibrate();
    chassis.setPose(0, 0, 0);

	initLeverControl();

	pros::lcd::initialize();
	initBrainDisplay();
	initControllerDisplay();

	// Ensure odom pod is down
	odomLift.retract();

	// Auton Selection
	startControllerAutonSelectorDisplay();

	// Set Optical LED
	frontOptic.set_led_pwm(100);

	// Motor modes
	leverMotor.set_brake_mode(pros::MotorBrake::hold);
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
	
	odomLift.retract();
	chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
	
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

	// startControllerCoordDisplay();

	// Display FB Logo
	// startBrainFBDisplay();
	startBrainMotorInfoDisplay();

	while (true) {
		// Update Controls
		updateTankDrive();
		updatePneumatics();
		updateIntake();

		if (hoodLock) controller.rumble(".");

		pros::delay(20);
	}
}