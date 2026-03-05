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
	initLog();	// Critical; DO NOT REMOVE

	pros::lcd::initialize();
	// initBrainDisplay();
	// initControllerDisplay();

	// Set Optical LED
	frontOptic.set_led_pwm(100);

	// Motor modes
	leverMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
	
	chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
	
	// Ensure descore arms are retracted
	extendLeftArm();

	soloAWP();
}

void opcontrol() {
	
	//chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
	chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);

	// Retract descore arm
	extendLeftArm();
	stopIntake();
	
	while (true) {
		// Update Controls
		updateTankDrive();
		updatePneumatics();
		updateIntake();
		
		pros::delay(20);
	}
	

	//runPIDTuner();
}