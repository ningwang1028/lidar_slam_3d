#ifndef GEO_TRANSFORM_H
#define GEO_TRANSFORM_H

#include <math.h>

namespace lidar_slam_3d
{

static const double sm_a = 6378137.0;
static const double sm_b = 6356752.314;
static const double utm_scale_factor = 0.9996;

static double arcLengthOfMeridian(double phi)
{
    double alpha, beta, gamma, delta, epsilon, n;
    double result;

    /* Precalculate n */
    n = (sm_a - sm_b) / (sm_a + sm_b);

    /* Precalculate alpha */
    alpha = ((sm_a + sm_b) / 2.0) * (1.0 + (pow(n, 2.0) / 4.0) + (pow(n, 4.0) / 64.0));

    /* Precalculate beta */
    beta = (-3.0 * n / 2.0) + (9.0 * pow(n, 3.0) / 16.0) + (-3.0 * pow(n, 5.0) / 32.0);

    /* Precalculate gamma */
    gamma = (15.0 * pow(n, 2.0) / 16.0) + (-15.0 * pow(n, 4.0) / 32.0);

    /* Precalculate delta */
    delta = (-35.0 * pow(n, 3.0) / 48.0) + (105.0 * pow(n, 5.0) / 256.0);

    /* Precalculate epsilon */
    epsilon = (315.0 * pow(n, 4.0) / 512.0);

    /* Now calculate the sum of the series and return */
    result = alpha * (phi + (beta * sin(2.0 * phi)) + (gamma * sin(4.0 * phi)) +
                      (delta * sin(6.0 * phi)) + (epsilon * sin(8.0 * phi)));

    return result;
}

void WGS84ToUTM(double lat, double lon, double& x, double& y)
{
    double zone = floor((lon + 180.0) / 6) + 1;
    double cmeridian = (-183.0 + (zone * 6.0)) / 180.0 * M_PI;

    double N, nu2, ep2, t, t2, l;
    double l3coef, l4coef, l5coef, l6coef, l7coef, l8coef;
    double tmp;

    lat = lat / 180.0 * M_PI;
    lon = lon / 180.0 * M_PI;

    /* Precalculate ep2 */
    ep2 = (pow(sm_a, 2.0) - pow(sm_b, 2.0)) / pow(sm_b, 2.0);

    /* Precalculate nu2 */
    nu2 = ep2 * pow(cos(lat), 2.0);

    /* Precalculate N */
    N = pow(sm_a, 2.0) / (sm_b * sqrt(1 + nu2));

    /* Precalculate t */
    t = tan(lat);
    t2 = t * t;
    tmp = (t2 * t2 * t2) - pow(t, 6.0);

    /* Precalculate l */
    l = lon - cmeridian;

    /* Precalculate coefficients for l**n in the equations below
    so a normal human being can read the expressions for easting
    and northing
    -- l**1 and l**2 have coefficients of 1.0 */
    l3coef = 1.0 - t2 + nu2;


    l4coef = 5.0 - t2 + 9 * nu2 + 4.0 * (nu2 * nu2);

    l5coef = 5.0 - 18.0 * t2 + (t2 * t2) + 14.0 * nu2 - 58.0 * t2 * nu2;

    l6coef = 61.0 - 58.0 * t2 + (t2 * t2) + 270.0 * nu2	- 330.0 * t2 * nu2;

    l7coef = 61.0 - 479.0 * t2 + 179.0 * (t2 * t2) - (t2 * t2 * t2);

    l8coef = 1385.0 - 3111.0 * t2 + 543.0 * (t2 * t2) - (t2 * t2 * t2);

    /* Calculate easting (x) */
    x = N * cos(lat) * l + (N / 6.0 * pow(cos(lat), 3.0) * l3coef * pow(l, 3.0))
        + (N / 120.0 * pow(cos(lat), 5.0) * l5coef * pow(l, 5.0))
        + (N / 5040.0 * pow(cos(lat), 7.0) * l7coef * pow(l, 7.0));

    /* Calculate northing (y) */
    y = arcLengthOfMeridian(lat)
        + (t / 2.0 * N * pow(cos(lat), 2.0) * pow(l, 2.0))
        + (t / 24.0 * N * pow(cos(lat), 4.0) * l4coef * pow(l, 4.0))
        + (t / 720.0 * N * pow(cos(lat), 6.0) * l6coef * pow(l, 6.0))
        + (t / 40320.0 * N * pow(cos(lat), 8.0) * l8coef * pow(l, 8.0));

    /* Adjust easting and northing for UTM system. */
    x = x * utm_scale_factor + 500000.0;
    y = y * utm_scale_factor;
    if (y < 0.0)
    {
        y = y + 10000000.0;
    }
}

} // namespace lidar_slam_3d

#endif // GEO_TRANSFORM_H
