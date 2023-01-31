class Intake {
    public:
        Intake();
        Intake(struct Core* core);
        ~Intake();
        void turn_on();
        void turn_on_rev();
        void turn_off();
    private:
        struct Core* core;
};