enum class RollerDirection {FORWARD, BACKWARD, NONE};
class Roller {
    public:
        Roller();
        Roller(struct Core* core);
        ~Roller();
        void rollto(RollerDirection color);
        void stop();
    private:

        struct Core* core;
        RollerDirection target_direction;
};