#pragma once

#include "main.h" // IWYU pragma: keep

// Autonomous selection variables
enum class autonColors { AUTON_NONE, RED_AUTON, BLUE_AUTON };
enum class autonTypes { TYPE_NONE, TYPE1, TYPE2, TYPE3, TYPE4 };
extern autonColors autonColor;
extern autonTypes autonType;
extern bool runningSkills;
extern bool autonMoveToPose;

// GUI objects
extern lv_obj_t* label_color;
extern lv_obj_t* label_type;
extern lv_obj_t* label_skills;
extern lv_obj_t* btn_color;
extern lv_obj_t* btn_type;
extern lv_obj_t* btn_skills;

// Button event callback functions
void toggle_color(lv_event_t* e);

void toggle_type(lv_event_t* e);

void toggle_skills (lv_event_t* e);

// Initialize autonomous selector GUI
void init_auton_selector();
