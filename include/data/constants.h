#pragma once
#include "main.h"

namespace Constants {

	namespace CatmullRom {
		inline float INFLUENCE_RATIO[4][4] = {
			{0.0, -0.5,  1.0, -0.5},
			{1.0,  0.0, -2.5,  1.5},
			{0.0,  0.5,  2.0, -1.5},
			{0.0,  0.0, -0.5,  0.5}
		};
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

}