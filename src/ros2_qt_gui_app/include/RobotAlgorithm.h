#ifndef ROBOTAIGORITHM_H
#define ROBOTAIGORITHM_H
#include <math.h>
struct robotPose {
    double x{0};
    double y{0};
    double theta{0};
};

inline double deg2rad(double x){
    return M_PI * x / 180.0;
}

inline double rad2deg(double x){
    return 180.0 * x / M_PI;
}

#endif // ROBOTAIGORITHM_H
