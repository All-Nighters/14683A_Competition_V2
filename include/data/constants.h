#pragma once
#include "main.h"


using namespace okapi;
namespace Constants {

	namespace CatmullRom {
		inline float INFLUENCE_RATIO[4][4] = {
			{0.0, -0.5,  1.0, -0.5},
			{1.0,  0.0, -2.5,  1.5},
			{0.0,  0.5,  2.0, -1.5},
			{0.0,  0.0, -0.5,  0.5}
		};
	}

	namespace PurePursuit {
		inline float LOOKAHEAD_RADIUS = 10;
		inline float DISPLACEMENT_P = 300;
		inline float DISPLACEMENT_I = 0.01;
		inline float DISPLACEMENT_D = 500;
		inline float ROTATION_P = 300;
		inline float ROTATION_I = 0.01;
		inline float ROTATION_D = 100;
	}
	namespace GraphicalInterface {
		inline int SCREEN_WIDTH  = 480;
		inline int SCREEN_HEIGHT = 272 - 30;

		// general styling
		inline lv_color_t CONTAINER_BACKGROUND       = LV_COLOR_HEX(0x1976D2); // old: 0x1E293B
		inline lv_color_t BUTTON_BACKGROUND_RELEASED = LV_COLOR_HEX(0x4791db); // old: 0x475569
		inline lv_color_t BUTTON_BACKGROUND_PRESSED  = LV_COLOR_HEX(0x3874B0);
		inline lv_color_t BUTTON_FOREGROUND_RELEASED = LV_COLOR_HEX(0xFFFFFF);
		inline lv_color_t BUTTON_FOREGROUND_PRESSED  = LV_COLOR_HEX(0xFFFFFF);
		inline lv_color_t LABEL_FOREGROUND           = LV_COLOR_HEX(0xFFFFFF);

		// home
		inline lv_color_t HOME_BACKGROUND            = LV_COLOR_HEX(0xF3F4F8);

		// selector styling
		inline lv_color_t SELECTOR_SIDEBAR_BACKGROUND                 = LV_COLOR_HEX(0x455770);
		inline lv_color_t SELECTOR_SIDEBAR_BUTTON_BACKGROUND_RELEASED = LV_COLOR_HEX(0x5A6B83);
		inline lv_color_t SELECTOR_SIDEBAR_BUTTON_BACKGROUND_PRESSED  = LV_COLOR_HEX(0x434F60);
		inline lv_color_t SELECTOR_BODY_BACKGROUND                    = LV_COLOR_HEX(0xF3F4F8);
		inline lv_color_t SELECTOR_BODY_LABEL_FOREGROUND              = LV_COLOR_HEX(0x000000);
		inline lv_color_t SELECTOR_BODY_BUTTON_BACKGROUND_RELEASED    = LV_COLOR_HEX(0x16A34A);
		inline lv_color_t SELECTOR_BODY_BUTTON_BACKGROUND_PRESSED     = LV_COLOR_HEX(0x15803D);
	}

    namespace Field {
        inline float FIELD_LENGTH = 3.6576;

        // high goal coordinates
        inline float RED_HIGH_GOAL_M[] = {0.45, 3.15, 0.76835};
        inline float BLUE_HIGH_GOAL_M[] = {3.15, 0.45, 0.76835};
        inline float RED_HIGH_GOAL_PCT[] = {14.54, 86, 20.99};
        inline float BLUE_HIGH_GOAL_PCT[] = {86, 14.54, 20.99};

        // disk
        inline QLength DISK_DIAMETER = 14_cm;
        inline QMass DISK_MASS = 0.06_kg;
        inline float DISK_HORIZONTAL_AREA = 0.015393804;
        inline float DISK_VERTICAL_AREA = 0.0028;
    }

	namespace Robot {
        inline QLength WHEEL_DIAMETER               = 2.75_in;
        inline QLength TRACK_LENGTH                 = 27_cm;
        inline QLength MIDDLE_ENCODER_DISTANCE      = 12_cm;
        inline QLength TRACKING_WHEEL_DIAMETER      = 2.75_in;
        inline float   EXTERNAL_GEAR_RATIO          = 1;
        inline AbstractMotor::gearset MOTOR_GEARSET = AbstractMotor::gearset::blue;
    }

}