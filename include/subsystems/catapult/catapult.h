class Catapult {
    public:
        Catapult(struct Core* core);
        ~Catapult();
        void reposition();
        void reset();
        void set_boost(bool use_boost);
        void set_voltage(int voltage);
        void fire(int fire_delay = 0);
        void wait_until_reloaded();
        bool is_reloaded();
        static bool continue_shooting;
        bool triggered;
    private:
        bool use_boost;
        struct Core* core;
        // std::unique_ptr<pros::Rotation> rotation_sensor;
        float voltage;
        int fire_delay;
        static void shooting_loop_trampoline(void* iparam);
        void shooting_loop();
        std::unique_ptr<pros::Task> shooting_task {nullptr};
};