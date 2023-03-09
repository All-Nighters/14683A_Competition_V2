class Intake {
    public:
        Intake(struct Core* core);
        ~Intake();
        void turn_on();
        void turn_on_rev();
        void turn_off();
    private:
        struct Core* core;
        std::shared_ptr<Catapult> cata;
        bool is_enabled;
};