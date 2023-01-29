#include "main.h"

class AutonFirstScoring {
    public:
        AutonFirstScoring(Chassis* chassis, Catapult* catapult, Intake* intake, bool offset = false);
        ~AutonFirstScoring();
        void run();
    private:
        Chassis* chassis;
        Catapult* catapult;
        Intake* intake;
        bool offset;
};