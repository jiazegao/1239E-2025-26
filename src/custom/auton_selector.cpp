
#include "custom/auton_selector.h"

// Button event callback functions
void toggle_color(lv_event_t* e) {
    allianceColor = (allianceColor == alliance_color::RED) ? alliance_color::BLUE : alliance_color::RED;
    lv_label_set_text(label_color, (allianceColor == alliance_color::RED) ? "Red" : "Blue");
    lv_obj_set_style_bg_color(btn_color, (allianceColor == alliance_color::RED) ? lv_color_hex(0xFF0000) : lv_color_hex(0x0000FF), LV_PART_MAIN);
}

void toggle_type(lv_event_t* e) {
    switch (autonType) {
        case autonTypes::LEFT: autonType = autonTypes::RIGHT_NOMID; lv_label_set_text(label_type, "RIGHT_NO_MID"); break;
        case autonTypes::RIGHT_NOMID: autonType = autonTypes::RIGHT_WMID; lv_label_set_text(label_type, "RIGHT_WITH_MID"); break;
        case autonTypes::RIGHT_WMID: autonType = autonTypes::SOLO_AWP; lv_label_set_text(label_type, "SOLO_AWP"); break;
        case autonTypes::SOLO_AWP: autonType = autonTypes::LEFT; lv_label_set_text(label_type, "LEFT"); break;
        default: autonType = autonTypes::LEFT; lv_label_set_text(label_type, "LEFT"); break;
    }
}

void toggle_skills (lv_event_t* e) {
    runningSkills = (runningSkills == true) ? false : true;
    lv_label_set_text(label_skills, (runningSkills == true) ? "SKILLS/Y" : "SKILLS/N");
    lv_obj_set_style_bg_color(btn_skills, (runningSkills == true) ? lv_color_hex(0x00FF00) : lv_color_hex(0xFF0000), LV_PART_MAIN);
}

void recalibrate(lv_event_t* e) {
    chassis.calibrate();
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
    lv_obj_set_pos(btn_type, 150, 20);
    lv_obj_set_size(btn_type, 110, 200);
    lv_obj_add_event_cb(btn_type, toggle_type, LV_EVENT_CLICKED, NULL);
    label_type = lv_label_create(btn_type);
    lv_label_set_text(label_type, "Select Type");

    btn_skills = lv_button_create(lv_screen_active());
    lv_obj_set_pos(btn_skills, 270, 20);
    lv_obj_set_size(btn_skills, 110, 200);
    lv_obj_add_event_cb(btn_skills, toggle_skills, LV_EVENT_CLICKED, NULL);
    lv_obj_set_style_bg_color(btn_skills, lv_color_hex(0x808080), LV_PART_MAIN); // Default to gray
    label_skills = lv_label_create(btn_skills);
    lv_label_set_text(label_skills, "Skills?");

    btn_recalibrate = lv_button_create(lv_screen_active());
    lv_obj_set_pos(btn_recalibrate, 390, 20);
    lv_obj_set_size(btn_recalibrate, 50, 200);
    lv_obj_add_event_cb(btn_recalibrate, recalibrate, LV_EVENT_CLICKED, NULL);
    label_recalibrate = lv_label_create(btn_recalibrate);
    lv_label_set_text(label_recalibrate, "Recal");
}

