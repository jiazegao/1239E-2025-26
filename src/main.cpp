
#pragma once 
#include "main.h"
#include "auton.h"
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
#include "util_funcs.h"
#include "auton_selector.h" // IWYU pragma: keep
#include "RclTracking.h"

void initialize() {
	pros::lcd::initialize();
	chassis.calibrate();
	chassis.setPose(0, 0, 0);
	RclMain.startTracking();
	
	startBrainDisplay();
	startControllerDisplay();
}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol() {
	while (true) {
		// Indexer control
		updateIndexer();

		// Tank Drive
		updateTankDrive();

		pros::delay(20);
	}

}