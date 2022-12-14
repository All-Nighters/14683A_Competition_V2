typedef struct RobotPosition {
    float x_meter;      ///< x coordinate in meters 
    float x_pct;        ///< x coordinate in percents 
    float y_meter;      ///< y coordinate in meters 
    float y_pct;        ///< y coordinate in percents 
    float theta;        ///< rotation in degrees 
} RobotPosition;

enum class OdomMode {
    MOTOR_IMU,          ///< motor encoder and IMU odometry 
    MOTOR_MIDTW_IMU,    ///< motor encoder, middle tracking wheel, and IMU
    LEFTTW_MIDTW_IMU,   ///< left and middle tracking wheels and IMU
    RIGHTTW_MIDTW_IMU,  ///< right and middle tracking wheels and IMU
};

class Odom {
    public:
        Odom();
        RobotPosition getState();
        void setState(QLength x, QLength y, QAngle angle);
        void setState(float x, float y, float angle);

    private:
        RobotPosition position;
        OdomMode odometry_mode;


        // sensors
        okapi::ADIEncoder* leftTW;
        okapi::ADIEncoder* rightTW;
        okapi::ADIEncoder* midTW;

        pros::IMU* imu1;
        pros::IMU* imu2;

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

        static void start_odom(void* iparam);
        void reset_variables();
        void position_tracking();
};