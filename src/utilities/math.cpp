#include <math.h>

namespace Math {
    float degree_to_radian(float degree) {
        return (degree / 360.0f) * (2 * M_PI);
    }
    float radian_to_degree(float radian) {
        return (radian / (2 * M_PI)) * 360;
    }
}