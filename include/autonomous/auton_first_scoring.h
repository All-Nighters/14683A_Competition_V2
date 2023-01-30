#include "main.h"

class AutonFirstScoring {
    public:
        AutonFirstScoring(struct Core* core, std::shared_ptr<Catapult> cata, bool offset);
        ~AutonFirstScoring();
        void run();
    private:
        std::shared_ptr<Odom> odometry;
        std::shared_ptr<Catapult> catapult_ptr;
        std::unique_ptr<Chassis> chassis_ptr;
        std::unique_ptr<Intake> intake_ptr;
        std::unique_ptr<Roller> roller_ptr;
        bool offset;
};