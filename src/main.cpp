
#include "main.h"
#include "auton.cpp"
#include "auton_selector.cpp"
#include "RclTracking.cpp"
#include "devices.h"
#include "liblvgl/llemu.hpp"

void lcd_display()  {

	int count = 0;

    while (true){

		count ++;

        pros::lcd::print(0, "X: %f", chassis.getPose().x);
        pros::lcd::print(1, "Y: %f", chassis.getPose().y);
        pros::lcd::print(2, "Heading: %f", chassis.getPose().theta);
		pros::lcd::print(4, "Running? %d", count);

        pros::delay(20);

    }
}

void initialize() {
	//init_auton_selector();
	pros::lcd::initialize();
	pros::Task([](){lcd_display();});
}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol() {
	while (true) {
		pros::delay(100);
	}

}