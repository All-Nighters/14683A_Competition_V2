#include "main.h"

class AutonSecondScoring {
    public:
        AutonSecondScoring(Chassis* chassis, Catapult* catapult, Intake* intake, bool offset = false);
        ~AutonSecondScoring();
        void run();
    private:
        Chassis* chassis;
        Catapult* catapult;
        Intake* intake;
        bool offset;
};