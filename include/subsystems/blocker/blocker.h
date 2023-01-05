class Blocker {
    public:
        Blocker(struct Core* core);
        ~Blocker();
        void deploy();
    private:
        struct Core* core;
};