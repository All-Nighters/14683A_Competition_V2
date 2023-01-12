#include "main.h"

class AutonFirstSupport {
    public:
        AutonFirstSupport(Chassis* chassis, Catapult* catapult, bool offset = false);
        ~AutonFirstSupport();
        void run();
    private:
        Chassis* chassis;
        Catapult* catapult;
        bool offset;
};