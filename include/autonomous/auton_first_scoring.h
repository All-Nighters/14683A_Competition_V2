#include "main.h"

class AutonFirstScoring {
    public:
        AutonFirstScoring(Chassis* chassis, Catapult* catapult, bool offset = false);
        ~AutonFirstScoring();
        void run();
    private:
        Chassis* chassis;
        Catapult* catapult;
        bool offset;
};