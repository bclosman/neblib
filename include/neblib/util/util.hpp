#pragma once

#include "vex.h"

namespace neblib {

int sign(int num);
int sign(float num);
int sign(double num);

double restrain(double num, double min, double max);

template <class F>
vex::task launch_task(F&& function);

}