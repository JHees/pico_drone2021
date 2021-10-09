#pragma once
#include "hardware/pwm.h"
// #include
#include "pico/stdlib.h"
#include <stdio.h>
// #include "hardware/irq.h"
#include "hardware/clocks.h"
#include "hardware/timer.h"
#include <functional>

#include "filter.h"
#include "pid.h"
#define MOTOR_PINOUT(wheel)                                                                                            \
    motor::pinout(MOTOR_##wheel##_PWM, MOTOR_##wheel##_DIR, MOTOR_##wheel##_MEASURE_A, MOTOR_##wheel##_MEASURE_B)
#define MOTOR(wheel) MOTOR_PINOUT(wheel), PID(MOTOR_##wheel)
#define MOTORS MOTOR(LF), MOTOR(LB), MOTOR(RF), MOTOR(RB)
#define MOTION_PID PID(MOTION_X), PID(MOTION_Y), PID(MOTION_R)
#define ABS(x) (x < 0 ? -x : x)
#define SIG(x) (x < 0 ? -1 : 1)

#ifndef MOTOR_MAX_SPEED
#define MOTOR_MAX_SPEED 300
#endif

class motor;
class McCanum;

constexpr int max_nr_callback = 4;

motor *motor_registry[max_nr_callback];
void global_gpio_irq_callback(uint gpio, uint32_t event);
bool global_motion_callback(repeating_timer_t *rt);

class motor
{
    friend void global_gpio_irq_callback(uint gpio, uint32_t event);

  private:
    // template <int idx> static void speed_callback_registry(uint gpio, uint32_t event)
    // {
    //     return motor_registry[idx]->speed_callback(gpio, event);
    // }
    // constexpr static gpio_irq_callback_t speed_callback_pool[max_nr_callback] = {
    //     speed_callback_registry<0>,
    //     speed_callback_registry<1>,
    //     speed_callback_registry<2>,
    //     speed_callback_registry<3>,
    // };
    template <int idx> static bool pid_callback_registry(repeating_timer_t *rt)
    {
        return motor_registry[idx]->pid_callback(rt);
    }
    constexpr static repeating_timer_callback_t pid_callback_pool[max_nr_callback] = {
        pid_callback_registry<0>,
        pid_callback_registry<1>,
        pid_callback_registry<2>,
        pid_callback_registry<3>,
    };
    static int nr_used_callbacks;

  public:
    struct pinout
    {
        pinout(uint p, uint d, uint mA, uint mB) : pwm(p), direction(d), A(mA), B(mB){};
        uint pwm;
        uint direction;
        uint A, B;
    };

    pid<int16_t> pid_control;

  private:
    const uint pwm_gpio;
    const uint direction_gpio;
    const uint measureA_gpio, measureB_gpio;
    MiddleFilter<int16_t, 65> hall_queue;

    repeating_timer_t timer;
    volatile int16_t speed_measured;
    volatile bool ab_last;
    volatile bool edge_last;
    volatile uint32_t time_last;
    void speed_callback(uint gpio, uint32_t event)
    {
        // printf("%u, %lu\n", gpio, event);

        volatile bool ab = (gpio == measureA_gpio);
        volatile bool edge = (event & GPIO_IRQ_EDGE_RISE);
        volatile uint32_t time = time_us_32();
        // printf("%u\n", pwm_gpio);
        if (ab_last == ab)
        {
            gpio_acknowledge_irq(gpio, event);
            return;
        }
        hall_queue.set_flag();
        if (edge_last == edge)
        {
            hall_queue << ((ab_last < ab) ? (int64_t)time - (int64_t)time_last : (int64_t)time_last - (int64_t)time);
        }
        else
        {
            hall_queue << ((ab_last > ab) ? (int64_t)time - (int64_t)time_last : (int64_t)time_last - (int64_t)time);
        }
        ab_last = ab;
        edge_last = edge;
        time_last = time;

        gpio_acknowledge_irq(gpio, event);
    };
    bool pid_callback(repeating_timer_t *rt)
    {
        speed_measured = get_speed();
        control = pid_control.adjust_in_formate(speed_measured, MOTOR_DEADZONE, MOTOR_MAX_SPEED);
        set_speed(control);
        // set_speed(pid_control.adjust_in_formate(speed_measured, MOTOR_DEADZONE, MOTOR_MAX_SPEED));
        return true;
    }
    void set_speed(int16_t speed)
    {
        assert(speed >= -1000 && speed <= 1000);
        pwm_set_enabled(pwm_gpio_to_slice_num(pwm_gpio), true);
        bool dir = true;
        speed < 0 ? dir = false, speed = -speed : 0;
        pwm_set_gpio_level(pwm_gpio, speed);
        if (speed == 0)
            return;
        if (dir)
            gpio_put(direction_gpio, true);
        else
            gpio_put(direction_gpio, false);
    };
    int16_t get_speed()
    {
        if (hall_queue.get_flag())
        {
            hall_queue.clean();
            return 0;
        }
        auto res = hall_queue.reciprocal_result();
        int16_t res_speed = -int16_t(res * 12800 * 7.5);
        return ABS(res_speed) <= 1000 ? res_speed : SIG(res_speed) * 1000;
    }

  public:
    volatile int16_t control;

    motor() = delete;
    ~motor()
    {
        pwm_set_gpio_level(pwm_gpio, 0);
    };
    motor(pinout pin) : motor(pin.pwm, pin.direction, pin.A, pin.B){};
    motor(pinout pin, pid_parameter pid_p)
        : motor(pin.pwm, pin.direction, pin.A, pin.B, pid_p.Kp, pid_p.Ki, pid_p.Kd){};
    motor(uint pwm, uint dir, uint mA, uint mB) : motor(pwm, dir, mA, mB, 0.3f, 5, 0.001f){};
    motor(uint pwm, uint dir, uint mA, uint mB, float _Kp, float _Ki, float _Kd)
        : pwm_gpio(pwm), direction_gpio(dir), measureA_gpio(mA), measureB_gpio(mB), pid_control(_Kp, _Ki, _Kd)
    {
        auto idx = nr_used_callbacks++;
        motor_registry[idx] = this;
        // speed pwm init
        gpio_set_function(pwm_gpio, GPIO_FUNC_PWM);
        pwm_config cfg = pwm_get_default_config();
        pwm_config_set_clkdiv(&cfg, 125);
        pwm_config_set_wrap(&cfg, 999);
        pwm_init(pwm_gpio_to_slice_num(pwm_gpio), &cfg, false);

        // direction gpio init
        gpio_init(direction_gpio);
        gpio_set_dir(direction_gpio, GPIO_OUT);
        gpio_put(direction_gpio, 1);

        // measure GPIO init

        gpio_set_irq_enabled_with_callback(measureA_gpio, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true,
                                           global_gpio_irq_callback);

        gpio_set_irq_enabled_with_callback(measureB_gpio, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true,
                                           global_gpio_irq_callback);

        add_repeating_timer_ms(30, pid_callback_pool[idx], nullptr, &timer);
    };
    void set_speed_target(int16_t tt)
    {
        pid_control.set_target(tt);
    }
    int16_t speed() const
    {
        return speed_measured;
    }
};
int motor::nr_used_callbacks = 0;
class McCanum
{
    friend void global_gpio_irq_callback(uint gpio, uint32_t event);
    friend bool global_motion_pid_callback(repeating_timer_t *rt);

  public:
    motor LF, LB, RF, RB;
    const float coff_lw;
    // pid<int16_t> x_pid;
    // pid<int16_t> y_pid;
    // pid<int16_t> r_pid;
    uint32_t timestamp_last;
    repeating_timer_t timer;

  public:
    int16_t mX, mY, mR;
    int64_t integrator_x, integrator_y, integrator_r;
    McCanum(motor::pinout lf, pid_parameter lf_pid, motor::pinout lb, pid_parameter lb_pid, motor::pinout rf,
            pid_parameter rf_pid, motor::pinout rb, pid_parameter rb_pid)
        //  , pid_parameter x, pid_parameter y, pid_parameter r)
        : coff_lw(1), LF(lf, lf_pid), LB(lb, lb_pid), RF(rf, rf_pid), RB(rb, rb_pid)
    //, x_pid(x), y_pid(y), r_pid(r)
    {
        add_repeating_timer_ms(50, global_motion_callback, nullptr, &timer);
    };
    ~McCanum() = default;

  public:
    void set_motion(int16_t x_speed, int16_t y_speed, int16_t rotate)
    {
        int16_t speed[4] = {0};
        speed[0] = -x_speed - y_speed - rotate * coff_lw;
        speed[1] = x_speed - y_speed + rotate * coff_lw;
        speed[2] = -x_speed - y_speed + rotate * coff_lw;
        speed[3] = x_speed - y_speed - rotate * coff_lw;

        int16_t max = 0;
        max = max > ABS(speed[0]) ? max : ABS(speed[0]);
        max = max > ABS(speed[1]) ? max : ABS(speed[1]);
        max = max > ABS(speed[2]) ? max : ABS(speed[2]);
        max = max > ABS(speed[3]) ? max : ABS(speed[3]);
        if (max > 1000)
        {
            speed[0] *= 1000.0f / max;
            speed[1] *= 1000.0f / max;
            speed[2] *= 1000.0f / max;
            speed[3] *= 1000.0f / max;
        }
        RF.set_speed_target(speed[0]);
        LF.set_speed_target(speed[1]);
        LB.set_speed_target(speed[2]);
        RB.set_speed_target(speed[3]);
        // printf("%i, %i\n", speed[3], RB.control);we
    };
    void get_motion(int16_t &x_speed, int16_t &y_speed, int16_t &rotate)
    {
        int16_t speed[4] = {RF.speed(), LF.speed(), LB.speed(), RB.speed()};

        x_speed = (-speed[0] + speed[1] - speed[2] + speed[3]) / 4;
        y_speed = -(speed[0] + speed[1] + speed[2] + speed[3]) / 4;
        rotate = (-speed[0] + speed[1] + speed[2] - speed[3]) / 4 / coff_lw;
    };
    void get_motion()
    {
        uint32_t timestamp = time_us_32();
        get_motion(mX, mY, mR);
        if (!timestamp_last)
        {
            integrator_x += mX * int64_t(timestamp - timestamp_last) / 1e3;
            integrator_y += mY * int64_t(timestamp - timestamp_last) / 1e3;
            integrator_r += mR * int64_t(timestamp - timestamp_last) / 1e3;
        }
        timestamp_last = timestamp;
    }
    void reset_integrator()
    {
        integrator_x = 0;
        integrator_y = 0;
        integrator_r = 0;
        timestamp_last = 0;
    }
    // void set_motion_target(int16_t x_speed, int16_t y_speed, int16_t rotate)
    // {
    //     x_pid.set_target(x_speed);
    //     y_pid.set_target(y_speed);
    //     r_pid.set_target(rotate);
    // }
};