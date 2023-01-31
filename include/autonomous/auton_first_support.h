#include "main.h"

class AutonFirstSupport {
    public:
        AutonFirstSupport(struct Core* core, std::shared_ptr<Catapult> cata, bool offset);
        ~AutonFirstSupport();
        void run();
    private:
        std::shared_ptr<Odom> odometry;
        std::shared_ptr<Catapult> catapult_ptr;
        std::unique_ptr<Chassis> chassis_ptr;
        std::unique_ptr<Intake> intake_ptr;
        std::unique_ptr<Roller> roller_ptr;
        bool offset;
};