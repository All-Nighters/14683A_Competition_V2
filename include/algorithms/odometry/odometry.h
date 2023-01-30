typedef struct RobotPosition {
    float x_meter;      ///< x coordinate in meters 
    float x_pct;        ///< x coordinate in percents 
    float y_meter;      ///< y coordinate in meters 
    float y_pct;        ///< y coordinate in percents 
    float theta;        ///< rotation in degrees 
} RobotPosition;

enum class OdomMode {
    MOTOR_IMU,            ///< motor encoder and IMU odometry 
    MOTOR_FRONTTW_IMU,    ///< motor encoder, front tracking wheel (horizontal), and IMU odometry 
    LEFTTW_FRONTTW_IMU,   ///< left and front (horizontal) tracking wheels and IMU odometry 
    RIGHTTW_FRONTTW_IMU,  ///< right and front (horizontal) tracking wheels and IMU odometry 
    MOTOR_BACKTW_IMU,     ///< motor encoder, back tracking wheel (horizontal), and IMU odometry 
    LEFTTW_BACKTW_IMU,    ///< left and back (horizontal) tracking wheels and IMU odometry 
    RIGHTTW_BACKTW_IMU,   ///< right and back (horizontal) tracking wheels and IMU odometry 
    MIDDLETW_IMU,         ///< middle tracking wheel (vertical) and IMU odometry 
};

class Odom {
    public:
        Odom();
        Odom(struct Core* core, OdomMode mode);
        ~Odom();
        RobotPosition getState();
        void setState(QLength x, QLength y, QAngle angle);
        void setState(float x, float y, float angle);

    private:
        RobotPosition position;
        OdomMode odometry_mode;


        struct Core* core;

        float WHEEL_RADIUS;
        float THETA_START;
        //The starting x and y coordinates of the bot (meters)
        float X_START;
        float Y_START; 

        //Distances of tracking wheels from tracking center (meters)
        float LTrackRadius;
        float RTrackRadius;
        float STrackRadius;

        float LPos;
        float RPos;
        float SPos;
 
        float LPrevPos;
        float RPrevPos;
        float SPrevPos;
 
        float deltaDistL;
        float deltaDistR;
        float deltaDistS;
 
        float totalDeltaDistL;
        float totalDeltaDistR;
 
        float currentAbsoluteOrientation;
        float previousTheta;
 
        float deltaTheta;
        float avgThetaForArc;

        float deltaXLocal;
        float deltaYLocal;
 
        float deltaXGlobal;
        float deltaYGlobal;
 
        float xPosGlobal;
        float yPosGlobal;

        void reset_variables();
        void tare_sensors();
        static void start_odom(void* iparam);
        void position_tracking();
        std::unique_ptr<pros::Task> odom_task {nullptr};
};