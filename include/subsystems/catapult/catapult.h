class Catapult {
    public:
        Catapult(struct Core* core);
        ~Catapult();
        void reposition();
        void set_boost(bool use_boost);
        void fire();
    private:
        struct Core* core;
        bool triggered;
        float voltage;
        static void shooting_loop_trampoline(void* iparam);
        void shooting_loop();
        std::unique_ptr<pros::Task> shooting_task {nullptr};
};