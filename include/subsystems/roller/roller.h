enum class RollerDirection {FORWARD, BACKWARD, NONE};
class Roller {
    public:
        Roller();
        Roller(struct Core* core);
        ~Roller();
        void rollto(RollerDirection color);
    private:
        void roller_loop();
        static void roller_loop_trampoline(void* iparam);

        struct Core* core;
        RollerDirection target_direction;
        std::unique_ptr<pros::Task> roller_task {nullptr};
};