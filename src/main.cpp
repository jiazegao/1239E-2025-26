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
	pros::lcd::initialize();

	// init_auton_selector();
	initControllerDisplay();
	initLeverControl();
	initBrainDisplay();
	initLog();	// Critical; DO NOT REMOVE

	// Set Optical LED
	frontOptic.set_led_pwm(100);

	lever_antistuck_on = false;

	// Motor modes
	leverMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	leverMotor.set_encoder_units(pros::MotorEncoderUnits::degrees);
    leverMotor.set_zero_position(0.0);
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
	
	chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
	lever_antistuck_on = false;
	
	// Ensure descore arms are retracted
	extendLeftArm();
	pros::Task([](){hardResetLever();});

	setMidScoreDelay(0);
	leftDescoreAuto();
}

void opcontrol() {
	
	//chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
	chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
	chassis.setPose(0, 0, 0);
	lever_antistuck_on = true;
	antiStuckOn = true;

	// Retract descore arm
	extendLeftArm();
	stopIntake();

	hardResetLever();

	// startBrainFBDisplay();
	startLeverTuningDisplay();
	
	while (true) {
		// Update Controls
		updateTankDrive();
		updatePneumatics();
		updateIntake();

		pros::lcd::print(0, "Drivetrain: %f", MclMain.getDTWheelDegrees());
		pros::lcd::print(1, "Unit: %d", drivetrain.leftMotors->get_encoder_units());

		pros::delay(20);
	}
}