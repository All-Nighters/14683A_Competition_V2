class Catapult {
    public:
        Catapult(struct Core* core);
        ~Catapult();
        void reposition();
        void reset();
        void set_boost(bool use_boost);
        void fire(int fire_delay = 0);
        void wait_until_reloaded();
        static bool continue_shooting;
    private:
        bool use_boost;
        struct Core* core;
        bool triggered;
        float voltage;
        int fire_delay;
        static void shooting_loop_trampoline(void* iparam);
        void shooting_loop();
        std::unique_ptr<pros::Task> shooting_task {nullptr};
};