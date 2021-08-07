#pragma once

#include "filter.h"
#include <math.h>
#define SIG(x) (x < 0 ? -1 : 1)

template <class T> class pid
{
  private:
    queue<T, 2> error; // e(k-1) = error[1] ; e(k-2) = error[0];
    float Kp, Ki, Kd;
    T target, control;
    uint32_t time_last;

  public:
    pid() : pid(0, 0, 0){}; // zero
    pid(float p, float i, float d) : Kp(p), Ki(i), Kd(d), control(0), time_last(0)
    {
        error.push_back(0);
        error.push_back(0);
    };
    void set_parameter(float p, float i, float d)
    {
        Kp = p, Ki = i, Kd = d;
    };
    void set_target(T tt)
    {
        target = tt;
    }
    T adjust_in_formate(T measure, int16_t dead_zone, int16_t max)
    {
        T err = target - measure;
        uint32_t time = time_us_32();
        float timestep = ((int64_t)time - (int64_t)time_last) / 1e6;
        control += Kp * (err - error[1]) + Ki * err * timestep + Kd * (err - 2 * error[1] + error[0]) / timestep;
        error.push_back(err);
        time_last = time;
        if (std::abs(control) > max)
            control = SIG(control) * max;
        if (std::abs(control) <= dead_zone)
            control = 0;
        return control;
    }
};