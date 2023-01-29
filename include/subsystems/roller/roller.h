enum class RollerColor {RED, BLUE, NONE};
class Roller {
    public:
        Roller(struct Core* core);
        ~Roller();
        void rollto(RollerColor color);
    private:
        void roller_loop();
        static void roller_loop_trampoline(void* iparam);

        const int timeout = 10000;
        struct Core* core;
        RollerColor target_color;
        std::unique_ptr<pros::Task> roller_task {nullptr};
};