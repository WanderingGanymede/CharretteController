#include "utils.h"

float fmap(float x, float in_min, float in_max, float out_min, float out_max, bool constrain) {
    if (in_min == in_max) {
        return (out_min + out_max) * 0.5f;
    }

    if (constrain) {
        if (x <= in_min) return out_min;
        if (x >= in_max) return out_max;
    }

    float mapped = out_min + (out_max - out_min) * (x - in_min) / (in_max - in_min);

    if (constrain) {
        if (mapped < out_min) mapped = out_min;
        if (mapped > out_max) mapped = out_max;
    }

    return mapped;
}
