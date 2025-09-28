
#include "custom/util_funcs.h"
#include "custom/auton.h"
#include "pros/misc.h"

// Indexer control
void indexerIn() {
    FrontIndexer.move_velocity(-600);
	BackIndexer.move_velocity(-600);
}
void indexerOut() {
    FrontIndexer.move_velocity(600);
	BackIndexer.move_velocity(600);
}
void stopIndexer() {
    FrontIndexer.move_velocity(0);
	BackIndexer.move_velocity(0);
}
void updateIndexer() {
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) indexerOut();
    else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) indexerIn();
    else  stopIndexer();
}

// Tank drive
void updateTankDrive() { chassis.tank(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y), controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y)); }

// Display
void startControllerDisplay() {
    if (!controllerDiplsayStarted) {
        controllerDiplsayStarted = true;
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
}
void startBrainDisplay() {
    if (!brainDisplayStarted) {
        brainDisplayStarted = true;
        pros::Task BrainScreenTask ([&]() {
            while (true) {
                pros::lcd::print(0, "X: %f", chassis.getPose().x);
                pros::lcd::print(1, "Y: %f", chassis.getPose().y);
                pros::lcd::print(2, "Theta: %f", chassis.getPose().theta);
                pros::delay(50);
            }
        });
    }
}