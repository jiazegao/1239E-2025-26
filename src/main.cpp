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

	//Ensure odom pod is down
	odomLift.retract();

	// Auton Selection
	startControllerAutonSelectorDisplay();
	init_auton_selector();

    RclMain.startTracking();
	topOptic.set_led_pwm(100);
}

void disabled() {}

void competition_initialize() {}

void autonomous() {

	chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);

	startControllerRCLUpdate();

	// Ensure descore arms are retracted
	extendLeftArm();
	extendRightArm();
	
	//skills();
	runAuton();
}

void opcontrol() {
	
	//startControllerDisplay();
	startControllerRclDisplay();
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
		updateArcadeDrive();
		updateIntake();
		updatePneumatics();

		pros::delay(20);
	}

	
}