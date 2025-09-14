#pragma once

#include "main.h"
#include "liblvgl/lvgl.h"

// Autonomous selection variables
enum autonColors { AUTON_NONE, REDAUTON, BLUEAUTON };
enum autonTypes { TYPE_NONE, TYPE1, TYPE2, TYPE3, TYPE4 };
autonColors autonColor = AUTON_NONE;
autonTypes autonType = TYPE_NONE;
bool runningSkills = false;
bool autonMoveToPose = false;

// GUI objects
lv_obj_t* label_color;
lv_obj_t* label_type;
lv_obj_t* label_skills;
lv_obj_t* btn_color;
lv_obj_t* btn_type;
lv_obj_t* btn_skills;

// Button event callback functions
static void toggle_color(lv_event_t* e) {
    autonColor = (autonColor == REDAUTON) ? BLUEAUTON : REDAUTON;
    lv_label_set_text(label_color, (autonColor == REDAUTON) ? "Red" : "Blue");
    lv_obj_set_style_bg_color(btn_color, (autonColor == REDAUTON) ? lv_color_hex(0xFF0000) : lv_color_hex(0x0000FF), LV_PART_MAIN);
}

static void toggle_type(lv_event_t* e) {
    switch (autonType) {
        case TYPE1: autonType = TYPE2; lv_label_set_text(label_type, "TYPE 2"); break;
        case TYPE2: autonType = TYPE3; lv_label_set_text(label_type, "TYPE 3"); break;
        case TYPE3: autonType = TYPE4; lv_label_set_text(label_type, "TYPE 4"); break;
        default: autonType = TYPE1; lv_label_set_text(label_type, "TYPE 1"); break;
    }
}

static void toggle_skills (lv_event_t* e) {
    runningSkills = (runningSkills == true) ? false : true;
    lv_label_set_text(label_skills, (runningSkills == true) ? "SKILLS/Y" : "SKILLS/N");
    lv_obj_set_style_bg_color(btn_skills, (runningSkills == true) ? lv_color_hex(0x00FF00) : lv_color_hex(0x0000FF), LV_PART_MAIN);
}

// Initialize autonomous selector GUI
void init_auton_selector() {
    btn_color = lv_button_create(lv_screen_active());
    lv_obj_set_pos(btn_color, 30, 20);
    lv_obj_set_size(btn_color, 110, 200);
    lv_obj_add_event_cb(btn_color, toggle_color, LV_EVENT_CLICKED, NULL);
    lv_obj_set_style_bg_color(btn_color, lv_color_hex(0x808080), LV_PART_MAIN); // Default to gray
    label_color = lv_label_create(btn_color);
    lv_label_set_text(label_color, "Select Color");
    
    btn_type = lv_button_create(lv_screen_active());
    lv_obj_set_pos(btn_type, 180, 20);
    lv_obj_set_size(btn_type, 110, 200);
    lv_obj_add_event_cb(btn_type, toggle_type, LV_EVENT_CLICKED, NULL);
    label_type = lv_label_create(btn_type);
    lv_label_set_text(label_type, "Select Type");

    btn_skills = lv_button_create(lv_screen_active());
    lv_obj_set_pos(btn_skills, 340, 20);
    lv_obj_set_size(btn_skills, 110, 200);
    lv_obj_add_event_cb(btn_skills, toggle_skills, LV_EVENT_CLICKED, NULL);
    lv_obj_set_style_bg_color(btn_skills, lv_color_hex(0x808080), LV_PART_MAIN); // Default to gray
    label_skills = lv_label_create(btn_skills);
    lv_label_set_text(label_skills, "Skills?");
}