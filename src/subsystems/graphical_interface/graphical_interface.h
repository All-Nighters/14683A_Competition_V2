#pragma once
#include <iostream>
#include <any>
#include "main.h"

class GraphicalInterface {
public:
    enum InterfaceType {
        MENU_CONTAINER,
        MENU_BUTTON,
        MENU_LABEL,
        HOME_CONTAINER,
        HOME_IMAGE,
        SELECTOR_CONTAINER,
        SELECTOR_SIDEBAR_CONTAINER,
        SELECTOR_SIDEBAR_BUTTON_1,
        SELECTOR_SIDEBAR_BUTTON_2,
        SELECTOR_BODY_CONTAINER,
        SELECTOR_BODY_LABEL,
        SELECTOR_BODY_BUTTON_2
    };
    enum InterfaceAction {
        MENU_SELECTOR,
        MENU_UTILITIES,
        MENU_RETURN,
        MENU_MENU,
        SELECTOR_AUTONOMOUS,
        SELECTOR_SKILL,
        SELECTOR_TEAM,
        SELECTOR_MODE,
        SELECTOR_POSITION,
        SELECTOR_EXECUTE
    };
    enum InterfaceStatus {
        HOME,
        SELECTOR,
        UTILITIES
    };
    enum InterfaceConfiguration {
        GAME_ROUND,
        GAME_TEAM,
        GAME_MODE,
        GAME_POSITION
    };
    enum InterfaceSelector {
        SELECTOR_NULL,
        SELECTOR_ROUND_AUTONOMOUS,
        SELECTOR_ROUND_SKILL,
        SELECTOR_TEAM_RED,
        SELECTOR_TEAM_BLUE,
        SELECTOR_MODE_SCORE,
        SELECTOR_MODE_SUPPORT,
        SELECTOR_MODE_IDLE,
        SELECTOR_POSITION_1,
        SELECTOR_POSITION_2
    };
    struct InterfaceComponent {
        lv_obj_t*                         object_pointer;
        lv_obj_t*                         object_parent;
        GraphicalInterface::InterfaceType object_type;
    };
    struct InterfaceWindow {
        GraphicalInterface::InterfaceStatus window_status;
        int                                 window_stage;
    };

    static GraphicalInterface::InterfaceStatus                            interface_status;
    static int                                                            interface_stage;
    static std::map<GraphicalInterface::InterfaceConfiguration, std::any> interface_configuration;
    static std::vector<GraphicalInterface::InterfaceWindow>               interface_history;

    GraphicalInterface();
    static void interface_rerender();
    static void interface_shutdown();

    static GraphicalInterface::InterfaceSelector get_selector(GraphicalInterface::InterfaceConfiguration configuration_key);

private:
    static std::vector<InterfaceComponent> interface_components;
    static std::vector<lv_style_t>         interface_style;

    void interface_menu();
    void object_scale(lv_obj_t* object, lv_coord_t width, lv_coord_t height, lv_coord_t x_coordinates, lv_coord_t y_coordinates);
    void object_style(GraphicalInterface::InterfaceComponent objects[], int objects_size);

    static std::vector<GraphicalInterface::InterfaceComponent> get_children_by_object(lv_obj_t* object_parent);
    static std::vector<GraphicalInterface::InterfaceComponent> get_children_by_type(GraphicalInterface::InterfaceType object_type);

    lv_obj_t* container_initialize(lv_obj_t* container_parent, GraphicalInterface::InterfaceType container_type);
    lv_obj_t* button_initialize(lv_obj_t* button_parent, std::string button_text, GraphicalInterface::InterfaceType button_type, GraphicalInterface::InterfaceAction button_action);
    lv_obj_t* label_initialize(lv_obj_t* label_parent, std::string label_text, GraphicalInterface::InterfaceType label_type);
    lv_obj_t* image_initialize(lv_obj_t* image_parent, GraphicalInterface::InterfaceType image_type);
};