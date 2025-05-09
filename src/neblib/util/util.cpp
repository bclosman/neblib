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

double neblib::restrain(double num, double min, double max) {
  while (num > max) num -= (max - min);
  while (num < min) num += (max - min);
  return num;
}

double neblib::clamp(double num, double min, double max) {
  if (num > max) return max;
  else if (num < min) return min;
  else return num;
}
template <class F>
vex::task neblib::launch_task(F&& function) {
  //static_assert(std::is_invocable_r_v<void, F>);
  return vex::task([](void* parameters) {
    std::unique_ptr<std::function<void()>> ptr{static_cast<std::function<void()>*>(parameters)};
    (*ptr)();
    return 0;
  }, new std::function<void()>(std::forward<F>(function)));
}