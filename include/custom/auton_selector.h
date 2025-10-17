#pragma once

#include "main.h" // IWYU pragma: keep
#include "configs.h"

// Autonomous selection variables
enum class autonTypes { TYPE_NONE, LEFT, RIGHT, SOLO_AWP };
inline autonTypes autonType = autonTypes::TYPE_NONE;
inline bool runningSkills = false;
inline bool autonMoveToPose = false;

// GUI objects
inline lv_obj_t* label_color;
inline lv_obj_t* label_type;
inline lv_obj_t* label_skills;
inline lv_obj_t* btn_color;
inline lv_obj_t* btn_type;
inline lv_obj_t* btn_skills;

// Button event callback functions
void toggle_color(lv_event_t* e);

void toggle_type(lv_event_t* e);

void toggle_skills (lv_event_t* e);

// Initialize autonomous selector GUI
void init_auton_selector();
