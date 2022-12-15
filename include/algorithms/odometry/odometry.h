typedef struct RobotPosition {
    float x_meter;      ///< x coordinate in meters 
    float x_pct;        ///< x coordinate in percents 
    float y_meter;      ///< y coordinate in meters 
    float y_pct;        ///< y coordinate in percents 
    float theta;        ///< rotation in degrees 
} RobotPosition;

enum class OdomMode {
    MOTOR_IMU,          ///< motor encoder and IMU odometry 
    MOTOR_FRONTTW_IMU,    ///< motor encoder, front tracking wheel, and IMU
    LEFTTW_FRONTTW_IMU,   ///< left and front tracking wheels and IMU
    RIGHTTW_FRONTTW_IMU,  ///< right and front tracking wheels and IMU
    MOTOR_BACKTW_IMU,    ///< motor encoder, front tracking wheel, and IMU
    LEFTTW_BACKTW_IMU,   ///< left and front tracking wheels and IMU
    RIGHTTW_BACKTW_IMU,  ///< right and front tracking wheels and IMU
};

class Odom {
    public:
        Odom(struct Core* core, OdomMode mode);
        static RobotPosition getState();
        static void setState(QLength x, QLength y, QAngle angle);
        static void setState(float x, float y, float angle);

    private:
        static RobotPosition position;
        static OdomMode odometry_mode;


        // sensors
        static struct Core* core;

        static float WHEEL_RADIUS;
        static float THETA_START;
        //The starting x and y coordinates of the bot (meters)
        static float X_START;
        static float Y_START; 

        //Distances of tracking wheels from tracking center (meters)
        static float LTrackRadius;
        static float RTrackRadius;
        static float STrackRadius;

        static float LPos;
        static float RPos;
        static float SPos;
 
        static float LPrevPos;
        static float RPrevPos;
        static float SPrevPos;
 
        static float deltaDistL;
        static float deltaDistR;
        static float deltaDistS;
 
        static float totalDeltaDistL;
        static float totalDeltaDistR;
 
        static float currentAbsoluteOrientation;
        static float previousTheta;
 
        static float deltaTheta;
        static float avgThetaForArc;

        static float deltaXLocal;
        static float deltaYLocal;
 
        static float deltaXGlobal;
        static float deltaYGlobal;
 
        static float xPosGlobal;
        static float yPosGlobal;

        static void reset_variables();
        static void tare_sensors();
        static void position_tracking();
};