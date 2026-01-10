
#include "custom/auton_selector.hpp"
#include "liblvgl/widgets/image/lv_image.h"
#include "custom/auton.hpp"

// Button event callback functions
void toggle_color(lv_event_t* e) {
    allianceColor = (allianceColor == alliance_color::RED) ? alliance_color::BLUE : alliance_color::RED;
    lv_label_set_text(label_color, (allianceColor == alliance_color::RED) ? "Red" : "Blue");
    lv_obj_set_style_bg_color(btn_color, (allianceColor == alliance_color::RED) ? lv_color_hex(0xFF0000) : lv_color_hex(0x0000FF), LV_PART_MAIN);
}

void toggle_type(lv_event_t* e) {
    switch (autonType) {
        case autonTypes::LEFT: autonType = autonTypes::LEFT_RUSH; lv_label_set_text(label_type, "LEFT_RUSH"); break;
        case autonTypes::LEFT_RUSH: autonType = autonTypes::LEFT_FAST; lv_label_set_text(label_type, "LEFT_FAST"); break;
        case autonTypes::LEFT_FAST: autonType = autonTypes::LEFT_V2; lv_label_set_text(label_type, "LEFT_V2"); break;
        case autonTypes::LEFT_V2: autonType = autonTypes::NAAUTO; lv_label_set_text(label_type, "NA_AUTO"); break;
        case autonTypes::NAAUTO: autonType = autonTypes::RIGHT; lv_label_set_text(label_type, "RIGHT"); break;
        case autonTypes::RIGHT: autonType = autonTypes::RIGHT_RUSH; lv_label_set_text(label_type, "RIGHT_RUSH"); break;
        case autonTypes::RIGHT_RUSH: autonType = autonTypes::RIGHT_FAST; lv_label_set_text(label_type, "RIGHT_FAST"); break;
        case autonTypes::RIGHT_FAST: autonType = autonTypes::RIGHT_V2; lv_label_set_text(label_type, "RIGHT_V2"); break;
        case autonTypes::RIGHT_V2: autonType = autonTypes::SOLO_AWP; lv_label_set_text(label_type, "SOLO_AWP"); break;
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
    lv_obj_set_pos(btn_color, 10, 20);
    lv_obj_set_size(btn_color, 110, 200);
    lv_obj_add_event_cb(btn_color, toggle_color, LV_EVENT_CLICKED, NULL);
    lv_obj_set_style_bg_color(btn_color, lv_color_hex(0x808080), LV_PART_MAIN); // Default to gray
    label_color = lv_label_create(btn_color);
    lv_label_set_text(label_color, "Select Color");
    
    btn_type = lv_button_create(lv_screen_active());
    lv_obj_set_pos(btn_type, 130, 20);
    lv_obj_set_size(btn_type, 110, 200);
    lv_obj_add_event_cb(btn_type, toggle_type, LV_EVENT_CLICKED, NULL);
    label_type = lv_label_create(btn_type);
    lv_label_set_text(label_type, "Select Type");

    btn_skills = lv_button_create(lv_screen_active());
    lv_obj_set_pos(btn_skills, 250, 20);
    lv_obj_set_size(btn_skills, 110, 200);
    lv_obj_add_event_cb(btn_skills, toggle_skills, LV_EVENT_CLICKED, NULL);
    lv_obj_set_style_bg_color(btn_skills, lv_color_hex(0x808080), LV_PART_MAIN); // Default to gray
    label_skills = lv_label_create(btn_skills);
    lv_label_set_text(label_skills, "Skills?");

    btn_recalibrate = lv_button_create(lv_screen_active());
    lv_obj_set_pos(btn_recalibrate, 370, 20);
    lv_obj_set_size(btn_recalibrate, 90, 200);
    lv_obj_add_event_cb(btn_recalibrate, recalibrate, LV_EVENT_CLICKED, NULL);
    label_recalibrate = lv_label_create(btn_recalibrate);
    lv_label_set_text(label_recalibrate, "Recal");
}

// Call the according autonomous
void runAuton() {
    // Auton Selection
	if (runningSkills) {
		skills_v2();
		return;
	}

    switch (autonType) {
        case autonTypes::LEFT:
            left();
            return;
        case autonTypes::LEFT_RUSH:
            leftControlRush();
            return;
        case autonTypes::LEFT_FAST:
            leftFastRush();
            return;
        case autonTypes::LEFT_V2:
            leftv2();
            return;
        case autonTypes::NAAUTO:
            NAAuto();
            return;
        case autonTypes::RIGHT:
            right();
            return;
        case autonTypes::RIGHT_RUSH:
            rightControlRush();
            return;
        case autonTypes::RIGHT_FAST:
            rightFastRush();
            return; 
        case autonTypes::RIGHT_V2:
            rightv2();
            return;
        case autonTypes::SOLO_AWP:
            soloAWP();
            return;
        default:
            soloAWP();
            return;
			
    }
}

