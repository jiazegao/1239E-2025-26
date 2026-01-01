#pragma once

#include "main.h" // IWYU pragma: keep

// Autonomous selection variables
enum class autonTypes { TYPE_NONE, LEFT, RIGHT, LEFT_RUSH, RIGHT_RUSH, LEFT_FAST, RIGHT_FAST, LEFT_V2, RIGHT_V2, SOLO_AWP };
inline autonTypes autonType = autonTypes::SOLO_AWP;
inline bool runningSkills = false;
inline bool autonMoveToPose = false;

// GUI objects
inline lv_obj_t* label_color;
inline lv_obj_t* label_type;
inline lv_obj_t* label_skills;
inline lv_obj_t* btn_color;
inline lv_obj_t* btn_type;
inline lv_obj_t* btn_skills;
inline lv_obj_t* btn_recalibrate;
inline lv_obj_t* label_recalibrate;

// Button event callback functions
void toggle_color(lv_event_t* e);

void toggle_type(lv_event_t* e);

void toggle_skills (lv_event_t* e);

void recalibrate(lv_event_t* e);

// Initialize autonomous selector GUI
void init_auton_selector();

void runAuton();