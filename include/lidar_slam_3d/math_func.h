#ifndef MATH_FUNC_H
#define MATH_FUNC_H

#include <math.h>

// Square
template <typename T>
T square(const T x)
{
    return x * x;
}

// Converts from degrees to radians.
inline double degToRad(double deg)
{
    return deg / 180.0 * M_PI;
}

// Converts form radians to degrees.
inline double radToDeg(double rad)
{
    return rad / M_PI * 180.0;
}

// Normalize angle into [-pi; pi].
inline double normalizeAngle(double angle)
{
    while(angle >= M_PI)
    {
        angle -= 2 * M_PI;
    }
    while(angle <= -M_PI)
    {
        angle += 2 * M_PI;
    }
    return angle;
}

#endif // MATH_FUNC_H
