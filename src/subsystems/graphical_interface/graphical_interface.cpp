#include "graphical_interface.h"
#include "../../datas/constants.h"
#include "main.h"
#include <any>

#define WIDTH  Constants::GraphicalInterface::SCREEN_WIDTH
#define HEIGHT Constants::GraphicalInterface::SCREEN_HEIGHT

LV_IMG_DECLARE(allnighters_logo);

std::vector<GraphicalInterface::InterfaceComponent>            GraphicalInterface::interface_components;
std::vector<lv_style_t>                                        GraphicalInterface::interface_style;
GraphicalInterface::InterfaceStatus                            GraphicalInterface::interface_status;
int                                                            GraphicalInterface::interface_stage;
std::map<GraphicalInterface::InterfaceConfiguration, std::any> GraphicalInterface::interface_configuration;
std::vector<GraphicalInterface::InterfaceWindow>               GraphicalInterface::interface_history;

lv_res_t button_action_callback(lv_obj_t* button_object);

GraphicalInterface::GraphicalInterface() {
    // default values
    this->interface_status = GraphicalInterface::InterfaceStatus::HOME;
    this->interface_stage  = 0;
    this->interface_configuration = {
        {GraphicalInterface::InterfaceConfiguration::GAME_ROUND,    GraphicalInterface::InterfaceSelector::SELECTOR_ROUND_AUTONOMOUS},
        {GraphicalInterface::InterfaceConfiguration::GAME_TEAM,     GraphicalInterface::InterfaceSelector::SELECTOR_TEAM_RED},
        {GraphicalInterface::InterfaceConfiguration::GAME_MODE,     GraphicalInterface::InterfaceSelector::SELECTOR_MODE_SCORE},
        {GraphicalInterface::InterfaceConfiguration::GAME_POSITION, GraphicalInterface::InterfaceSelector::SELECTOR_POSITION_1}
    };
    // render
    this->interface_menu();
    this->interface_rerender();
}

void GraphicalInterface::interface_menu() {
    // root (hidden)
    lv_obj_t* root_container = this->container_initialize(lv_scr_act(), GraphicalInterface::InterfaceType::MENU_CONTAINER);
    lv_obj_t* root_button    = this->button_initialize(root_container, "",  GraphicalInterface::InterfaceType::MENU_BUTTON, GraphicalInterface::InterfaceAction::MENU_MENU);
    this->object_scale(root_container, 0, 0, 0, 0);
    this->object_scale(root_button,    0, 0, 0, 0);
    // header
    lv_obj_t* header_container = this->container_initialize(lv_scr_act(), GraphicalInterface::InterfaceType::MENU_CONTAINER);
    lv_obj_t* header_selector  = this->button_initialize(header_container, "Selector",  GraphicalInterface::InterfaceType::MENU_BUTTON, GraphicalInterface::InterfaceAction::MENU_SELECTOR);
    lv_obj_t* header_utilities = this->button_initialize(header_container, "Utilities", GraphicalInterface::InterfaceType::MENU_BUTTON, GraphicalInterface::InterfaceAction::MENU_UTILITIES);
    lv_obj_t* header_label     = this->label_initialize(header_container,  "  14683A",  GraphicalInterface::InterfaceType::MENU_LABEL);
    this->object_scale(header_container, 480, 50, 0,             0);
    this->object_scale(header_selector , 100, 50, 0,             0);
    this->object_scale(header_utilities, 100, 50, 100,           0);
    this->object_scale(header_label,     100, 16, (WIDTH - 100), (50 / 3));
    // footer
    lv_obj_t* footer_container = this->container_initialize(lv_scr_act(), GraphicalInterface::InterfaceType::MENU_CONTAINER);
    lv_obj_t* footer_return    = this->button_initialize(footer_container, "Return", GraphicalInterface::InterfaceType::MENU_BUTTON, GraphicalInterface::InterfaceAction::MENU_RETURN);
    lv_obj_t* footer_menu      = this->button_initialize(footer_container, "Menu"  , GraphicalInterface::InterfaceType::MENU_BUTTON, GraphicalInterface::InterfaceAction::MENU_MENU);
    this->object_scale(footer_container, 480, 50, 0,             (HEIGHT - 50));
    this->object_scale(footer_return   , 100, 50, 0,             0);
    this->object_scale(footer_menu     , 100, 50, (WIDTH - 100), 0);
    // home screen
    lv_obj_t* home_container = this->container_initialize(lv_scr_act(), GraphicalInterface::InterfaceType::HOME_CONTAINER);
    lv_obj_t* home_icon      = this->image_initialize(home_container, GraphicalInterface::InterfaceType::HOME_IMAGE);
    this->object_scale(home_container, WIDTH, (HEIGHT - 100), 0, 50);
    this->object_scale(home_icon,      480,   142,            0, 0);
    lv_img_set_src(home_icon, &allnighters_logo);
    // selector
    lv_obj_t* selector_container = this->container_initialize(lv_scr_act(), GraphicalInterface::InterfaceType::SELECTOR_CONTAINER);
    this->object_scale(selector_container, WIDTH, (HEIGHT - 100), 0, 50);
    // selector sidebar 1
    lv_obj_t* selector_sidebar            = this->container_initialize(selector_container, GraphicalInterface::InterfaceType::SELECTOR_SIDEBAR_CONTAINER);
    lv_obj_t* selector_sidebar_autonomous = this->button_initialize(   selector_sidebar,   "Autonomous",  GraphicalInterface::InterfaceType::SELECTOR_SIDEBAR_BUTTON_1, GraphicalInterface::InterfaceAction::SELECTOR_AUTONOMOUS);
    lv_obj_t* selector_sidebar_skill      = this->button_initialize(   selector_sidebar,   "Skill",       GraphicalInterface::InterfaceType::SELECTOR_SIDEBAR_BUTTON_1, GraphicalInterface::InterfaceAction::SELECTOR_SKILL);
    this->object_scale(selector_sidebar,            150, (HEIGHT - 100), 0, 0);
    this->object_scale(selector_sidebar_autonomous, 140, 50,             5, 5);
    this->object_scale(selector_sidebar_skill,      140, 50,             5, 60);
    // selector sidebar 2
    lv_obj_t* selector_sidebar_team     = this->button_initialize(selector_sidebar, "Team",     GraphicalInterface::InterfaceType::SELECTOR_SIDEBAR_BUTTON_2, GraphicalInterface::InterfaceAction::SELECTOR_TEAM);
    lv_obj_t* selector_sidebar_mode     = this->button_initialize(selector_sidebar, "Mode",     GraphicalInterface::InterfaceType::SELECTOR_SIDEBAR_BUTTON_2, GraphicalInterface::InterfaceAction::SELECTOR_MODE);
    lv_obj_t* selector_sidebar_position = this->button_initialize(selector_sidebar, "Position", GraphicalInterface::InterfaceType::SELECTOR_SIDEBAR_BUTTON_2, GraphicalInterface::InterfaceAction::SELECTOR_POSITION);
    this->object_scale(selector_sidebar_team,     140, (HEIGHT - 120) / 3, 5, 5);
    this->object_scale(selector_sidebar_mode,     140, (HEIGHT - 120) / 3, 5, 10 + (HEIGHT - 120) * (1.0f / 3.0f));
    this->object_scale(selector_sidebar_position, 140, (HEIGHT - 120) / 3, 5, 15 + (HEIGHT - 120) * (2.0f / 3.0f));
    // selector body
    lv_obj_t* selector_body          = this->container_initialize(selector_container, GraphicalInterface::InterfaceType::SELECTOR_BODY_CONTAINER);
    lv_obj_t* selector_body_label    = this->label_initialize(selector_body,  "Loading...", GraphicalInterface::InterfaceType::SELECTOR_BODY_LABEL);
    lv_obj_t* selector_body_execute  = this->button_initialize(selector_body, "Execute",    GraphicalInterface::InterfaceType::SELECTOR_BODY_BUTTON_2, GraphicalInterface::InterfaceAction::SELECTOR_EXECUTE);
    this->object_scale(selector_body,         (WIDTH - 150), (HEIGHT - 100),     150,           0);
    this->object_scale(selector_body_label,   (WIDTH - 150), 16,                 5,             5);
    this->object_scale(selector_body_execute, 140,           (HEIGHT - 120) / 3, (WIDTH - 295), 15 + (HEIGHT - 120) * (2.0f / 3.0f));
    GraphicalInterface::InterfaceComponent selector_renderer[19] = {
        {root_container,              NULL, GraphicalInterface::InterfaceType::MENU_CONTAINER},
        {root_button,                 NULL, GraphicalInterface::InterfaceType::MENU_BUTTON},
        {header_container,            NULL, GraphicalInterface::InterfaceType::MENU_CONTAINER},
        {header_selector,             NULL, GraphicalInterface::InterfaceType::MENU_BUTTON},
        {header_utilities,            NULL, GraphicalInterface::InterfaceType::MENU_BUTTON},
        {header_label,                NULL, GraphicalInterface::InterfaceType::MENU_LABEL},
        {footer_container,            NULL, GraphicalInterface::InterfaceType::MENU_CONTAINER},
        {footer_return,               NULL, GraphicalInterface::InterfaceType::MENU_BUTTON},
        {footer_menu,                 NULL, GraphicalInterface::InterfaceType::MENU_BUTTON},
        {home_container,              NULL, GraphicalInterface::InterfaceType::HOME_CONTAINER},
        {selector_sidebar,            NULL, GraphicalInterface::InterfaceType::SELECTOR_SIDEBAR_CONTAINER},
        {selector_sidebar_autonomous, NULL, GraphicalInterface::InterfaceType::SELECTOR_SIDEBAR_BUTTON_1},
        {selector_sidebar_skill,      NULL, GraphicalInterface::InterfaceType::SELECTOR_SIDEBAR_BUTTON_1},
        {selector_sidebar_team,       NULL, GraphicalInterface::InterfaceType::SELECTOR_SIDEBAR_BUTTON_2},
        {selector_sidebar_mode,       NULL, GraphicalInterface::InterfaceType::SELECTOR_SIDEBAR_BUTTON_2},
        {selector_sidebar_position,   NULL, GraphicalInterface::InterfaceType::SELECTOR_SIDEBAR_BUTTON_2},
        {selector_body,               NULL, GraphicalInterface::InterfaceType::SELECTOR_BODY_CONTAINER},
        {selector_body_label,         NULL, GraphicalInterface::InterfaceType::SELECTOR_BODY_LABEL},
        {selector_body_execute,       NULL, GraphicalInterface::InterfaceType::SELECTOR_BODY_BUTTON_2}
    };
    this->object_style(selector_renderer, sizeof(selector_renderer) / sizeof(selector_renderer[0]));
}

void GraphicalInterface::object_scale(lv_obj_t* object, lv_coord_t width, lv_coord_t height, lv_coord_t x_coordinates, lv_coord_t y_coordinates) {
    lv_obj_set_size(object, width, height);
	lv_obj_align(object, NULL, LV_ALIGN_IN_TOP_LEFT, x_coordinates, y_coordinates);
}

void GraphicalInterface::object_style(GraphicalInterface::InterfaceComponent objects[], int objects_size) {
    int object_style_offset = this->interface_style.size();
    for (int object_index = 0; object_index < objects_size; object_index++) {
        GraphicalInterface::InterfaceComponent object_loop = objects[object_index];
        int object_style_size = 0;
        // allocate new styles
        switch (object_loop.object_type) {
            case GraphicalInterface::InterfaceType::MENU_CONTAINER:
                object_style_size = 1;
                break;
            case GraphicalInterface::InterfaceType::MENU_BUTTON:
                object_style_size = 2;
                break;
            case GraphicalInterface::InterfaceType::MENU_LABEL:
                object_style_size = 1;
                break;
            case GraphicalInterface::InterfaceType::HOME_CONTAINER:
                object_style_size = 1;
                break;
            case GraphicalInterface::InterfaceType::SELECTOR_SIDEBAR_CONTAINER:
                object_style_size = 1;
                break;
            case GraphicalInterface::InterfaceType::SELECTOR_SIDEBAR_BUTTON_1:
                object_style_size = 2;
                break;
            case GraphicalInterface::InterfaceType::SELECTOR_SIDEBAR_BUTTON_2:
                object_style_size = 2;
                break;
            case GraphicalInterface::InterfaceType::SELECTOR_BODY_CONTAINER:
                object_style_size = 1;
                break;
            case GraphicalInterface::InterfaceType::SELECTOR_BODY_LABEL:
                object_style_size = 1;
                break;
            case GraphicalInterface::InterfaceType::SELECTOR_BODY_BUTTON_2:
                object_style_size = 2;
                break;
        }
        for (int style_index = 0; style_index < object_style_size; style_index++) {
            this->interface_style.push_back(lv_style_t());
            lv_style_copy(&this->interface_style[object_style_offset + style_index], &lv_style_pretty);
        }
        // styles by type
        switch (object_loop.object_type) {
            case GraphicalInterface::InterfaceType::MENU_CONTAINER:
                this->interface_style[object_style_offset + 0].body.main_color   = Constants::GraphicalInterface::CONTAINER_BACKGROUND;
                this->interface_style[object_style_offset + 0].body.grad_color   = Constants::GraphicalInterface::CONTAINER_BACKGROUND;
                this->interface_style[object_style_offset + 0].body.border.width = 1; // border line width
                this->interface_style[object_style_offset + 0].body.radius       = 0;
                lv_obj_set_style(object_loop.object_pointer, &this->interface_style[object_style_offset + 0]);
                break;
            case GraphicalInterface::InterfaceType::MENU_BUTTON:
                // 0=RELEASE 1=PRESS
                this->interface_style[object_style_offset + 0].body.main_color   = Constants::GraphicalInterface::BUTTON_BACKGROUND_RELEASED;
                this->interface_style[object_style_offset + 0].body.grad_color   = Constants::GraphicalInterface::BUTTON_BACKGROUND_RELEASED;
                this->interface_style[object_style_offset + 0].text.color        = Constants::GraphicalInterface::BUTTON_FOREGROUND_RELEASED;
                this->interface_style[object_style_offset + 0].body.border.width = 1; // border line width
                this->interface_style[object_style_offset + 0].body.radius       = 2;
                this->interface_style[object_style_offset + 1].body.main_color   = Constants::GraphicalInterface::BUTTON_BACKGROUND_PRESSED;
                this->interface_style[object_style_offset + 1].body.grad_color   = Constants::GraphicalInterface::BUTTON_BACKGROUND_PRESSED;
                this->interface_style[object_style_offset + 1].text.color        = Constants::GraphicalInterface::BUTTON_FOREGROUND_PRESSED;
                this->interface_style[object_style_offset + 1].body.border.width = 1; // border line width
                this->interface_style[object_style_offset + 1].body.radius       = 2;
                lv_btn_set_style(object_loop.object_pointer, LV_BTN_STYLE_REL, &this->interface_style[object_style_offset + 0]);
                lv_btn_set_style(object_loop.object_pointer, LV_BTN_STYLE_PR,  &this->interface_style[object_style_offset + 1]);
                break;
            case GraphicalInterface::InterfaceType::MENU_LABEL:
                this->interface_style[object_style_offset + 0].text.color        = Constants::GraphicalInterface::LABEL_FOREGROUND;
                lv_obj_set_style(object_loop.object_pointer, &this->interface_style[object_style_offset + 0]);
                break;
            case GraphicalInterface::InterfaceType::HOME_CONTAINER:
                this->interface_style[object_style_offset + 0].body.main_color   = Constants::GraphicalInterface::HOME_BACKGROUND;
                this->interface_style[object_style_offset + 0].body.grad_color   = Constants::GraphicalInterface::HOME_BACKGROUND;
                this->interface_style[object_style_offset + 0].body.border.width = 1; // border line width
                this->interface_style[object_style_offset + 0].body.radius       = 0;
                lv_obj_set_style(object_loop.object_pointer, &this->interface_style[object_style_offset + 0]);
                break;
            case GraphicalInterface::InterfaceType::SELECTOR_SIDEBAR_CONTAINER:
                this->interface_style[object_style_offset + 0].body.main_color   = Constants::GraphicalInterface::SELECTOR_SIDEBAR_BACKGROUND;
                this->interface_style[object_style_offset + 0].body.grad_color   = Constants::GraphicalInterface::SELECTOR_SIDEBAR_BACKGROUND;
                this->interface_style[object_style_offset + 0].body.border.width = 1; // border line width
                this->interface_style[object_style_offset + 0].body.radius       = 0;
                lv_obj_set_style(object_loop.object_pointer, &this->interface_style[object_style_offset + 0]);
                break;
            case GraphicalInterface::InterfaceType::SELECTOR_SIDEBAR_BUTTON_1:
                // 0=RELEASE 1=PRESS
                this->interface_style[object_style_offset + 0].body.main_color   = Constants::GraphicalInterface::SELECTOR_SIDEBAR_BUTTON_BACKGROUND_RELEASED;
                this->interface_style[object_style_offset + 0].body.grad_color   = Constants::GraphicalInterface::SELECTOR_SIDEBAR_BUTTON_BACKGROUND_RELEASED;
                this->interface_style[object_style_offset + 0].text.color        = Constants::GraphicalInterface::BUTTON_FOREGROUND_RELEASED;
                this->interface_style[object_style_offset + 0].body.border.width = 1; // border line width
                this->interface_style[object_style_offset + 0].body.radius       = 2;
                this->interface_style[object_style_offset + 1].body.main_color   = Constants::GraphicalInterface::SELECTOR_SIDEBAR_BUTTON_BACKGROUND_PRESSED;
                this->interface_style[object_style_offset + 1].body.grad_color   = Constants::GraphicalInterface::SELECTOR_SIDEBAR_BUTTON_BACKGROUND_PRESSED;
                this->interface_style[object_style_offset + 1].text.color        = Constants::GraphicalInterface::BUTTON_FOREGROUND_PRESSED;
                this->interface_style[object_style_offset + 1].body.border.width = 1; // border line width
                this->interface_style[object_style_offset + 1].body.radius       = 2;
                lv_btn_set_style(object_loop.object_pointer, LV_BTN_STYLE_REL, &this->interface_style[object_style_offset + 0]);
                lv_btn_set_style(object_loop.object_pointer, LV_BTN_STYLE_PR,  &this->interface_style[object_style_offset + 1]);
                break;
            case GraphicalInterface::InterfaceType::SELECTOR_SIDEBAR_BUTTON_2:
                // 0=RELEASE 1=PRESS
                this->interface_style[object_style_offset + 0].body.main_color   = Constants::GraphicalInterface::SELECTOR_SIDEBAR_BUTTON_BACKGROUND_RELEASED;
                this->interface_style[object_style_offset + 0].body.grad_color   = Constants::GraphicalInterface::SELECTOR_SIDEBAR_BUTTON_BACKGROUND_RELEASED;
                this->interface_style[object_style_offset + 0].text.color        = Constants::GraphicalInterface::BUTTON_FOREGROUND_RELEASED;
                this->interface_style[object_style_offset + 0].body.border.width = 1; // border line width
                this->interface_style[object_style_offset + 0].body.radius       = 2;
                this->interface_style[object_style_offset + 1].body.main_color   = Constants::GraphicalInterface::SELECTOR_SIDEBAR_BUTTON_BACKGROUND_PRESSED;
                this->interface_style[object_style_offset + 1].body.grad_color   = Constants::GraphicalInterface::SELECTOR_SIDEBAR_BUTTON_BACKGROUND_PRESSED;
                this->interface_style[object_style_offset + 1].text.color        = Constants::GraphicalInterface::BUTTON_FOREGROUND_PRESSED;
                this->interface_style[object_style_offset + 1].body.border.width = 1; // border line width
                this->interface_style[object_style_offset + 1].body.radius       = 2;
                lv_btn_set_style(object_loop.object_pointer, LV_BTN_STYLE_REL, &this->interface_style[object_style_offset + 0]);
                lv_btn_set_style(object_loop.object_pointer, LV_BTN_STYLE_PR,  &this->interface_style[object_style_offset + 1]);
                break;
            case GraphicalInterface::InterfaceType::SELECTOR_BODY_CONTAINER:
                this->interface_style[object_style_offset + 0].body.main_color   = Constants::GraphicalInterface::SELECTOR_BODY_BACKGROUND;
                this->interface_style[object_style_offset + 0].body.grad_color   = Constants::GraphicalInterface::SELECTOR_BODY_BACKGROUND;
                this->interface_style[object_style_offset + 0].body.border.width = 1; // border line width
                this->interface_style[object_style_offset + 0].body.radius       = 0;
                lv_obj_set_style(object_loop.object_pointer, &this->interface_style[object_style_offset + 0]);
                break;
            case GraphicalInterface::InterfaceType::SELECTOR_BODY_LABEL:
                this->interface_style[object_style_offset + 0].text.color = Constants::GraphicalInterface::SELECTOR_BODY_LABEL_FOREGROUND;
                lv_obj_set_style(object_loop.object_pointer, &this->interface_style[object_style_offset + 0]);
                break;
            case GraphicalInterface::InterfaceType::SELECTOR_BODY_BUTTON_2:
                // 0=RELEASE 1=PRESS
                this->interface_style[object_style_offset + 0].body.main_color   = Constants::GraphicalInterface::SELECTOR_BODY_BUTTON_BACKGROUND_RELEASED;
                this->interface_style[object_style_offset + 0].body.grad_color   = Constants::GraphicalInterface::SELECTOR_BODY_BUTTON_BACKGROUND_RELEASED;
                this->interface_style[object_style_offset + 0].text.color        = Constants::GraphicalInterface::BUTTON_FOREGROUND_RELEASED;
                this->interface_style[object_style_offset + 0].body.border.width = 1; // border line width
                this->interface_style[object_style_offset + 0].body.radius       = 2;
                this->interface_style[object_style_offset + 1].body.main_color   = Constants::GraphicalInterface::SELECTOR_BODY_BUTTON_BACKGROUND_PRESSED;
                this->interface_style[object_style_offset + 1].body.grad_color   = Constants::GraphicalInterface::SELECTOR_BODY_BUTTON_BACKGROUND_PRESSED;
                this->interface_style[object_style_offset + 1].text.color        = Constants::GraphicalInterface::BUTTON_FOREGROUND_PRESSED;
                this->interface_style[object_style_offset + 1].body.border.width = 1; // border line width
                this->interface_style[object_style_offset + 1].body.radius       = 2;
                lv_btn_set_style(object_loop.object_pointer, LV_BTN_STYLE_REL, &this->interface_style[object_style_offset + 0]);
                lv_btn_set_style(object_loop.object_pointer, LV_BTN_STYLE_PR,  &this->interface_style[object_style_offset + 1]);
                break;
        }
        // shift styles offset
        object_style_offset += object_style_size;
    }
}

std::vector<GraphicalInterface::InterfaceComponent> GraphicalInterface::get_children_by_object(lv_obj_t* object_parent) {
    std::vector<GraphicalInterface::InterfaceComponent> object_children;
    for (int component_index = 0; component_index < GraphicalInterface::interface_components.size(); component_index++) {
        GraphicalInterface::InterfaceComponent component_object = GraphicalInterface::interface_components[component_index];
        if (component_object.object_parent != object_parent) continue;
        object_children.push_back(component_object);
    }
    return object_children;
}

std::vector<GraphicalInterface::InterfaceComponent> GraphicalInterface::get_children_by_type(GraphicalInterface::InterfaceType object_type) {
    std::vector<GraphicalInterface::InterfaceComponent> object_filtered;
    for (int component_index = 0; component_index < GraphicalInterface::interface_components.size(); component_index++) {
        GraphicalInterface::InterfaceComponent component_object = GraphicalInterface::interface_components[component_index];
        if (component_object.object_type != object_type) continue;
        object_filtered.push_back(component_object);
    }
    return object_filtered;
}

lv_obj_t* GraphicalInterface::container_initialize(lv_obj_t* container_parent, GraphicalInterface::InterfaceType container_type) {
    lv_obj_t* container_object  = lv_cont_create(container_parent, NULL);
    this->interface_components.push_back({container_object, container_parent, container_type});
    return container_object;
}

lv_obj_t* GraphicalInterface::button_initialize(lv_obj_t* button_parent, std::string button_text, GraphicalInterface::InterfaceType button_type, GraphicalInterface::InterfaceAction button_action) {
    lv_obj_t* button_object  = lv_btn_create(button_parent, NULL);
    lv_obj_t* button_label = this->label_initialize(button_object, button_text, GraphicalInterface::InterfaceType::MENU_LABEL);
    lv_obj_set_free_num(button_object, button_action);
    lv_btn_set_action(button_object, LV_BTN_ACTION_CLICK, button_action_callback);
    this->interface_components.push_back({button_object, button_parent, button_type});
    return button_object;
}

lv_obj_t* GraphicalInterface::label_initialize(lv_obj_t* label_parent, std::string label_text, GraphicalInterface::InterfaceType label_type) {
    lv_obj_t* label_object = lv_label_create(label_parent, NULL);
    lv_label_set_text(label_object, label_text.c_str());
    this->interface_components.push_back({label_object, label_parent, label_type});
    return label_object;
}

lv_obj_t* GraphicalInterface::image_initialize(lv_obj_t* image_parent, GraphicalInterface::InterfaceType image_type) {
    lv_obj_t* image_object = lv_img_create(image_parent, NULL);
    this->interface_components.push_back({image_object, image_parent, image_type});
    return image_object;
}

void GraphicalInterface::interface_rerender() {
    std::map<GraphicalInterface::InterfaceType, bool> interface_visibility = {
        {GraphicalInterface::InterfaceType::HOME_CONTAINER,            (GraphicalInterface::interface_status == GraphicalInterface::InterfaceStatus::HOME)},
        {GraphicalInterface::InterfaceType::SELECTOR_CONTAINER,        (GraphicalInterface::interface_status == GraphicalInterface::InterfaceStatus::SELECTOR)},
        {GraphicalInterface::InterfaceType::SELECTOR_SIDEBAR_BUTTON_1, (GraphicalInterface::interface_status == GraphicalInterface::InterfaceStatus::SELECTOR && GraphicalInterface::interface_stage == 0)},
        {GraphicalInterface::InterfaceType::SELECTOR_SIDEBAR_BUTTON_2, (GraphicalInterface::interface_status == GraphicalInterface::InterfaceStatus::SELECTOR && GraphicalInterface::interface_stage == 1)},
        {GraphicalInterface::InterfaceType::SELECTOR_BODY_BUTTON_2,    (GraphicalInterface::interface_status == GraphicalInterface::InterfaceStatus::SELECTOR && GraphicalInterface::interface_stage == 1)}
    };
    for (int component_index = 0; component_index < GraphicalInterface::interface_components.size(); component_index++) {
        GraphicalInterface::InterfaceComponent loop_component = GraphicalInterface::interface_components[component_index];
        if (interface_visibility.find(loop_component.object_type) == interface_visibility.end()) continue;
        if (lv_obj_get_hidden(loop_component.object_pointer) == !interface_visibility[loop_component.object_type]) continue;
        lv_obj_set_hidden(loop_component.object_pointer, !interface_visibility[loop_component.object_type]);
    }
    if (GraphicalInterface::interface_status == GraphicalInterface::InterfaceStatus::SELECTOR) {
        std::vector<GraphicalInterface::InterfaceComponent> selector_description_objects = GraphicalInterface::get_children_by_type(GraphicalInterface::InterfaceType::SELECTOR_BODY_LABEL);
        static std::map<GraphicalInterface::InterfaceSelector, char*> configuration_display = {
            {GraphicalInterface::InterfaceSelector::SELECTOR_ROUND_AUTONOMOUS, "Autonomous"},
            {GraphicalInterface::InterfaceSelector::SELECTOR_ROUND_SKILL,      "Skill"},
            {GraphicalInterface::InterfaceSelector::SELECTOR_MODE_SCORE,       "Score"},
            {GraphicalInterface::InterfaceSelector::SELECTOR_MODE_SUPPORT,     "Support"},
            {GraphicalInterface::InterfaceSelector::SELECTOR_MODE_IDLE,        "Idle"},
            {GraphicalInterface::InterfaceSelector::SELECTOR_TEAM_RED,         "Red"},
            {GraphicalInterface::InterfaceSelector::SELECTOR_TEAM_BLUE,        "Blue"},
            {GraphicalInterface::InterfaceSelector::SELECTOR_POSITION_1,       "#1"},
            {GraphicalInterface::InterfaceSelector::SELECTOR_POSITION_2,       "#2"}
        };
        GraphicalInterface::InterfaceSelector selector_description_round    = std::any_cast<GraphicalInterface::InterfaceSelector>(GraphicalInterface::interface_configuration[GraphicalInterface::InterfaceConfiguration::GAME_ROUND]);
        GraphicalInterface::InterfaceSelector selector_description_team     = std::any_cast<GraphicalInterface::InterfaceSelector>(GraphicalInterface::interface_configuration[GraphicalInterface::InterfaceConfiguration::GAME_TEAM]);
        GraphicalInterface::InterfaceSelector selector_description_mode     = std::any_cast<GraphicalInterface::InterfaceSelector>(GraphicalInterface::interface_configuration[GraphicalInterface::InterfaceConfiguration::GAME_MODE]);
        GraphicalInterface::InterfaceSelector selector_description_position = std::any_cast<GraphicalInterface::InterfaceSelector>(GraphicalInterface::interface_configuration[GraphicalInterface::InterfaceConfiguration::GAME_POSITION]);
        std::string selector_description_text =
            "Round: "    + std::string(configuration_display[selector_description_round]) + "\n" +
            "Team: "     + std::string(configuration_display[selector_description_team])  + "\n" +
            "Mode: "     + std::string(configuration_display[selector_description_mode])  + "\n" +
            "Position: " + std::string(configuration_display[selector_description_position]);
        lv_label_set_text(selector_description_objects[0].object_pointer, selector_description_text.c_str());
    }
}

void GraphicalInterface::interface_shutdown() {
    lv_obj_del(lv_scr_act());
}

lv_res_t button_action_callback(lv_obj_t* button_object) {
    uint32_t button_id = lv_obj_get_free_num(button_object);
    static std::map<GraphicalInterface::InterfaceSelector, GraphicalInterface::InterfaceSelector> selector_configuration_toggle = {
        {GraphicalInterface::InterfaceSelector::SELECTOR_TEAM_RED,     GraphicalInterface::InterfaceSelector::SELECTOR_TEAM_BLUE},
        {GraphicalInterface::InterfaceSelector::SELECTOR_TEAM_BLUE,    GraphicalInterface::InterfaceSelector::SELECTOR_TEAM_RED},
        {GraphicalInterface::InterfaceSelector::SELECTOR_MODE_SCORE,   GraphicalInterface::InterfaceSelector::SELECTOR_MODE_SUPPORT},
        {GraphicalInterface::InterfaceSelector::SELECTOR_MODE_SUPPORT, GraphicalInterface::InterfaceSelector::SELECTOR_MODE_IDLE},
        {GraphicalInterface::InterfaceSelector::SELECTOR_MODE_IDLE,    GraphicalInterface::InterfaceSelector::SELECTOR_MODE_SCORE},
        {GraphicalInterface::InterfaceSelector::SELECTOR_POSITION_1,   GraphicalInterface::InterfaceSelector::SELECTOR_POSITION_2},
        {GraphicalInterface::InterfaceSelector::SELECTOR_POSITION_2,   GraphicalInterface::InterfaceSelector::SELECTOR_POSITION_1}
    };
    GraphicalInterface::InterfaceStatus interface_status_new = GraphicalInterface::interface_status;
    int                                 interface_stage_new  = GraphicalInterface::interface_stage;
    switch (button_id) {
        case GraphicalInterface::InterfaceAction::MENU_SELECTOR:
            interface_status_new = GraphicalInterface::InterfaceStatus::SELECTOR;
            interface_stage_new  = 0;
            break;
        case GraphicalInterface::InterfaceAction::SELECTOR_AUTONOMOUS:
            interface_status_new = GraphicalInterface::InterfaceStatus::SELECTOR;
            interface_stage_new  = 1;
            GraphicalInterface::interface_configuration[GraphicalInterface::InterfaceConfiguration::GAME_ROUND] = GraphicalInterface::InterfaceSelector::SELECTOR_ROUND_AUTONOMOUS;
            break;
        case GraphicalInterface::InterfaceAction::SELECTOR_SKILL:
            interface_status_new = GraphicalInterface::InterfaceStatus::SELECTOR;
            interface_stage_new  = 1;
            GraphicalInterface::interface_configuration[GraphicalInterface::InterfaceConfiguration::GAME_ROUND] = GraphicalInterface::InterfaceSelector::SELECTOR_ROUND_SKILL;
            break;
        case GraphicalInterface::InterfaceAction::SELECTOR_TEAM: {
            interface_status_new = GraphicalInterface::InterfaceStatus::SELECTOR;
            interface_stage_new  = 1;
            GraphicalInterface::InterfaceSelector previous_team = std::any_cast<GraphicalInterface::InterfaceSelector>(GraphicalInterface::interface_configuration[GraphicalInterface::InterfaceConfiguration::GAME_TEAM]);
            GraphicalInterface::interface_configuration[GraphicalInterface::InterfaceConfiguration::GAME_TEAM] = selector_configuration_toggle[previous_team];
            break; }
        case GraphicalInterface::InterfaceAction::SELECTOR_MODE: {
            interface_status_new = GraphicalInterface::InterfaceStatus::SELECTOR;
            interface_stage_new  = 1;
            GraphicalInterface::InterfaceSelector previous_mode = std::any_cast<GraphicalInterface::InterfaceSelector>(GraphicalInterface::interface_configuration[GraphicalInterface::InterfaceConfiguration::GAME_MODE]);
            GraphicalInterface::interface_configuration[GraphicalInterface::InterfaceConfiguration::GAME_MODE] = selector_configuration_toggle[previous_mode];
            break; }
        case GraphicalInterface::InterfaceAction::SELECTOR_POSITION: {
            interface_status_new = GraphicalInterface::InterfaceStatus::SELECTOR;
            interface_stage_new  = 1;
            GraphicalInterface::InterfaceSelector previous_position = std::any_cast<GraphicalInterface::InterfaceSelector>(GraphicalInterface::interface_configuration[GraphicalInterface::InterfaceConfiguration::GAME_POSITION]);
            GraphicalInterface::interface_configuration[GraphicalInterface::InterfaceConfiguration::GAME_POSITION] = selector_configuration_toggle[previous_position];
            break; }
        case GraphicalInterface::InterfaceAction::MENU_UTILITIES:
            interface_status_new = GraphicalInterface::InterfaceStatus::UTILITIES;
            interface_stage_new  = 0;
            break;
        case GraphicalInterface::InterfaceAction::MENU_RETURN: {
            if (GraphicalInterface::interface_history.size() <= 0) break;
            GraphicalInterface::InterfaceWindow previous_window = GraphicalInterface::interface_history[GraphicalInterface::interface_history.size() - 1];
            interface_status_new = previous_window.window_status;
            interface_stage_new  = previous_window.window_stage;
            break; }
        case GraphicalInterface::InterfaceAction::MENU_MENU:
            interface_status_new = GraphicalInterface::InterfaceStatus::HOME;
            interface_stage_new  = 0;
            break;
    }
    if (GraphicalInterface::interface_status != interface_status_new || GraphicalInterface::interface_stage != interface_stage_new) {
        if (button_id != GraphicalInterface::InterfaceAction::MENU_RETURN) GraphicalInterface::interface_history.push_back({GraphicalInterface::interface_status, GraphicalInterface::interface_stage});
        GraphicalInterface::interface_status = interface_status_new;
        GraphicalInterface::interface_stage  = interface_stage_new;
        if (button_id == GraphicalInterface::InterfaceAction::MENU_RETURN && GraphicalInterface::interface_history.size() > 0) GraphicalInterface::interface_history.pop_back();
        if (interface_status_new == GraphicalInterface::InterfaceStatus::HOME) GraphicalInterface::interface_history.clear();
    }
    GraphicalInterface::interface_rerender();
    return LV_RES_OK;
}

GraphicalInterface::InterfaceSelector GraphicalInterface::get_selector(GraphicalInterface::InterfaceConfiguration configuration_key) {
    if (GraphicalInterface::interface_configuration.find(configuration_key) == GraphicalInterface::interface_configuration.end()) return GraphicalInterface::SELECTOR_NULL;
    return std::any_cast<GraphicalInterface::InterfaceSelector>(GraphicalInterface::interface_configuration[configuration_key]);
}