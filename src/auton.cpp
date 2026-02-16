#include "custom/auton.hpp"
#include "custom/RclTracking.hpp"
#include "custom/configs.hpp"
#include "custom/util_funcs.hpp"
#include "custom/lever_control.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include <cmath>

void leftPush(double x_offset = 0, double y_offset = 0) {
    chassis.moveToPoint(-40+x_offset, 37.5+y_offset, 900, {}, false);
    chassis.turnToHeading(90, 600, {}, false);
    retractLeftArm();
    chassis.moveToPoint(-21+x_offset, 37.5+y_offset, 2000, {.minSpeed=127, .earlyExitRange=1}, false);
    chassis.turnToHeading(135, 2000, {.minSpeed=127}, false);
}
void rightPush(double x_offset = 0, double y_offset = 0) {
    chassis.moveToPoint(-40+x_offset, -37.5-y_offset, 900, {}, false);
    chassis.turnToHeading(270, 450, {}, false);
    retractLeftArm();
    chassis.moveToPoint(-24+x_offset, -37.5-y_offset, 2000, {.forwards=false, .minSpeed=127}, false);
}

// Push + 5 Long + 4 Mid + 5 Long
void soloAWP(){
    
}
// Three balls -> Match loader -> Long goal -> Push
// ---
void leftControlRush() {
    
}
// Match loader -> Long goal -> Push
// WORKS
void leftFastRush() {
    
}
// Three balls -> Match loader -> Long goal -> Push
// WORKS
void rightControlRush() {
    
}
// Match loader -> Long goal -> Push
// ---
void rightFastRush() {
    
}
// Three balls -> Two balls -> Long goal -> Match loader -> Top mid -> Push
// WORKS
void leftv2() {
    
}

void rightv2() {

}

void right() {

}

void NAAuto() {
    
}

void left() {

}

void skills() {
    
}
// 90+ points
void skills_v2() {

}

// 80 points
void skills_v3() {
}

// 119 points
void skills_119() {
    
}