
#pragma once 
#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "api.h" // IWYU pragma: keep 
#include "liblvgl/llemu.hpp"// IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp" 			// IWYU pragma: keep
#include "pros/abstract_motor.hpp" // IWYU pragma: keep
#include "pros/adi.hpp"// IWYU pragma: keep
#include "pros/misc.h"
#include "pros/motors.hpp" // IWYU pragma: keep
#include "pros/rtos.hpp"

#include "configs.h"
#include "auton.cpp"
#include "auton_selector.h"
#include "RclTracking.h"

void initialize() {
	pros::lcd::initialize();
	chassis.calibrate();
	chassis.setPose(0, 0, 0);

	RclMain.startTracking();
	pros::Task BrainScreenTask ([&]() {
		while (true) {
			pros::lcd::print(0, "X: %f", chassis.getPose().x);
			pros::lcd::print(1, "Y: %f", chassis.getPose().y);
			pros::lcd::print(2, "Theta: %f", chassis.getPose().theta);
			pros::delay(50);
		}
	});

	pros::Task ControllerScreenTask ([&](){
		while (true) {
			controller.clear();
			controller.print(0, 0, "X: %f", chassis.getPose().x);
			pros::delay(15);
			controller.print(1, 0, "Y: %f", chassis.getPose().y);
			pros::delay(15);
			controller.print(2, 0, "Heading: %f", chassis.getPose().theta);
			pros::delay(100);
		}
	});
}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol() {
	while (true) {
		// Indexer control
		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
			FrontIndexer.move_velocity(400);
			BackIndexer.move_velocity(400);
		} else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
			FrontIndexer.move_velocity(-400);
			BackIndexer.move_velocity(-400);
		} else {
			FrontIndexer.move_velocity(0);
			BackIndexer.move_velocity(0);
		}

		// Tank Drive
		int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
		int rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

		chassis.tank(leftY, rightY);
		pros::delay(20);
	}

}