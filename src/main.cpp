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

	//FOR TESTING***

	red_soloAWP();

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

	pros::Task screenTask([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // log position telemetry
			//            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            // delay to save resources
            pros::delay(50);
        }
    });
	
}