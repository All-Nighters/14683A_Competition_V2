class Blocker {
    public:
        Blocker(struct Core* core);
        ~Blocker();
        void deploy();
        void close();
    private:
        struct Core* core;
};