#include "neblib/util/util.hpp"

int neblib::sign(int num)
{
    if (num > 0) return 1;
    if (num < 0) return -1;
    return 0;
}

int neblib::sign(float num)
{
    if (num > 0) return 1;
    if (num < 0) return -1;
    return 0;
}

int neblib::sign(double num)
{
    if (num > 0) return 1;
    if (num < 0) return -1;
    return 0;
}