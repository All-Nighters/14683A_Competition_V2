class Expansion {
    public:
        Expansion(struct Core* core);
        ~Expansion();
        void deploy();
    private:
        struct Core* core;
};