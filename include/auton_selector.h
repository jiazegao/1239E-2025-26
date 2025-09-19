#pragma once

#include "main.h"
#include "liblvgl/lvgl.h"

// Autonomous selection variables
enum autonColors { AUTON_NONE, REDAUTON, BLUEAUTON };
enum autonTypes { TYPE_NONE, TYPE1, TYPE2, TYPE3, TYPE4 };
static autonColors autonColor = AUTON_NONE;
static autonTypes autonType = TYPE_NONE;
static bool runningSkills = false;
static bool autonMoveToPose = false;

// GUI objects
static lv_obj_t* label_color;
static lv_obj_t* label_type;
static lv_obj_t* label_skills;
static lv_obj_t* btn_color;
static lv_obj_t* btn_type;
static lv_obj_t* btn_skills;

// Button event callback functions
void toggle_color(lv_event_t* e);

void toggle_type(lv_event_t* e);

void toggle_skills (lv_event_t* e);

// Initialize autonomous selector GUI
void init_auton_selector();
