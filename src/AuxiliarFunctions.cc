#include "AuxiliarFunctions.hh"


float asd_atan(float x, float y) {
        float theta = std::atan(y/x);
        if (x >= 0 and y >= 0)
                return theta;
        if (x >= 0 and y < 0)
                return theta + 2*M_PI;
        return theta + M_PI;
}
