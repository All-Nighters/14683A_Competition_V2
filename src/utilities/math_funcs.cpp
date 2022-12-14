#include "main.h"

/**
 * @brief limit angle (degrees) to between -180 and 180
 * 
 * @param a angle in degrees
 * @returns formatted angle in degrees
 */
float formatAngle(float a) {
    int sign = a < 0 ? -1 : 1;
    float positive_a = abs(a);
    float mod = std::fmod(positive_a, 360);
    if (mod <= 180) {
        return sign * mod;
    } 
    else {
        return sign * (mod - 360);
    }
}

/**
 * @brief Convert meter coordinates to percentage coordinates
 * 
 * @param m length in meters
 * @returns length in percentage 
 */
float meterToPercentage(float m) {
    return m / Constants::FIELD::FIELD_LENGTH * 100.0;
}

/**
 * @brief Convert percentage coordinates to meter coordinates
 * 
 * @param m length in percentage
 * @returns length in meters 
 */
float percentageToMeter(float p) {
    return p / 100.0 * Constants::FIELD::FIELD_LENGTH;
}

/**
 * @brief limit a value within a certain limit
 * 
 * @param v the value to limit
 * @param lo the minimum boundary
 * @param hi the maximum boundary
 * @returns the limited value 
 */
float clamp(float v, float lo, float hi ) {
    return v < lo ? lo : hi < v ? hi : v;
}
