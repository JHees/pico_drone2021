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
#define MOTORS MOTOR_PINOUT(LF), MOTOR_PINOUT(LB), MOTOR_PINOUT(RF), MOTOR_PINOUT(RB)
#define ABS(x) (x < 0 ? -x : x)
#define SIG(x) (x < 0 ? -1 : 1)

#ifndef MOTOR_MAX_SPEED
#define MOTOR_MAX_SPEED 300
#endif
/*
 * motor
 *
 *    get_speed
 * 3. 编码器测速
 */
class motor;
constexpr int max_nr_callback = 4;

motor *registry[max_nr_callback];
class motor
{
  private:
    template <int idx> static void speed_callback_registry(uint gpio, uint32_t event)
    {
        registry[idx]->speed_callback(gpio, event);
    }
    constexpr static gpio_irq_callback_t speed_callback_pool[max_nr_callback] = {
        speed_callback_registry<0>,
        speed_callback_registry<1>,
        speed_callback_registry<2>,
        speed_callback_registry<3>,
    };
    template <int idx> static bool pid_callback_registry(repeating_timer_t *rt)
    {
        return registry[idx]->pid_callback(rt);
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
    MiddleFilter<int64_t, 65> hall_queue;

    void speed_callback(uint gpio, uint32_t event)
    {
        // printf("%u, %lu\n", gpio, event);
        volatile static bool ab_last;
        volatile static bool edge_last;
        volatile static uint32_t time_last;
        volatile bool ab = (gpio == measureA_gpio);
        volatile bool edge = (event & GPIO_IRQ_EDGE_RISE);
        volatile uint32_t time = time_us_32();
        if (ab_last == ab)
        {
            gpio_acknowledge_irq(gpio, event);
            return;
        }
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
        hall_queue.set_flag();
        gpio_acknowledge_irq(gpio, event);
    };
    bool pid_callback(repeating_timer_t *rt)
    {
        set_speed(pid_control.adjust_in_formate(get_speed(), 20, MOTOR_MAX_SPEED));
        return true;
    }

  public:
    motor() = delete;
    ~motor()
    {
        pwm_set_gpio_level(pwm_gpio, 0);
    };
    motor(pinout pin) : motor(pin.pwm, pin.direction, pin.A, pin.B){};
    motor(uint pwm, uint dir, uint mA, uint mB)
        : pwm_gpio(pwm), direction_gpio(dir), measureA_gpio(mA), measureB_gpio(mB), pid_control(0.3f, 5, 0.001f)
    {
        auto idx = nr_used_callbacks++;
        registry[idx] = this;
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
                                           speed_callback_pool[idx]);

        gpio_set_irq_enabled_with_callback(measureB_gpio, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true,
                                           speed_callback_pool[idx]);

        add_repeating_timer_ms(15, pid_callback_pool[idx], nullptr, nullptr);
    };
    void set_speed_target(int16_t tt)
    {
        pid_control.set_target(tt);
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
            return 0;
        }
        auto res = hall_queue.result();
        int16_t res_speed = -int16_t(1.0 / res * 12800 * 7.5);
        // printf("%lli, %i\n", res, res_speed);
        // hall_queue.clean();
        return ABS(res_speed) <= 1000 ? res_speed : SIG(res_speed) * 1000;
    }
};
int motor::nr_used_callbacks = 0;

class McCanum
{
  private:
    motor LF, LB, RF, RB;
    const float coff_lw;

  public:
    McCanum(motor::pinout lf, motor::pinout lb, motor::pinout rf, motor::pinout rb)
        : coff_lw(1), LF(lf), LB(lb), RF(rf), RB(rb){};
    ~McCanum() = default;

  public:
    void set_motion(int16_t x_speed, int16_t y_speed, int16_t rotate)
    {
        int16_t speed[4] = {0};
        speed[0] = -x_speed + y_speed - rotate * coff_lw;
        speed[1] = x_speed + y_speed + rotate * coff_lw;
        speed[2] = -x_speed + y_speed + rotate * coff_lw;
        speed[3] = x_speed + y_speed - rotate * coff_lw;

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
    };
    void get_motion(int16_t *x_speed, int16_t *y_speed, int16_t *rotate)
    {
    }
};