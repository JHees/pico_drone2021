#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
// #include "hardware/irq.h"
#include "hardware/clocks.h"

#define MOTOR_PINOUT(wheel) \
    motor::pinout(MOTOR_##wheel##_PWM, MOTOR_##wheel##_DIR, MOTOR_##wheel##_MEASURE_A, MOTOR_##wheel##_MEASURE_A)
#define MOTORS MOTOR_PINOUT(LF), MOTOR_PINOUT(LB), MOTOR_PINOUT(RF), MOTOR_PINOUT(RB)
#define ABS(x) (x < 0 ? -x : x)
/*
 * motor
 *
 *    get_speed
 * 3. 编码器测速
*/
class motor
{
private:
    const uint pwm_gpio;
    const uint direction_gpio;
    const uint measureA_gpio, measureB_gpio;

public:
    struct pinout
    {
        pinout(uint p, uint d, uint mA, uint mB)
            : pwm(p), direction(d), measureA(mA), measureB(mB){};
        uint pwm;
        uint direction;
        uint measureA, measureB;
    };

public:
    motor() = default;
    ~motor()
    {
        pwm_set_gpio_level(pwm_gpio, 0);
    };
    motor(pinout pin)
        : motor(pin.pwm, pin.direction, pin.measureA, pin.measureB){};
    motor(uint pwm, uint dir, uint mA, uint mB)
        : pwm_gpio(pwm), direction_gpio(dir), measureA_gpio(mA), measureB_gpio(mB)
    {
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

        // measure pwm init
        // gpio_set_function(measureA_gpio,GPIO_FUNC_PWM);
        // pwm_config cfg_measure = pwm_get_default_config();
        // pwm_config_set_clkdiv_mode(&cfg, PWM_DIV_B_RISING);
    };
    void set_speed(int16_t speed)
    {
        assert(speed >= -1000 & speed <= 1000);
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
};

class McCanum
{
private:
    motor LF, LB, RF, RB;
    const float coff_lw;

public:
    McCanum(uint8_t l, uint8_t w, motor::pinout lf, motor::pinout lb, motor::pinout rf, motor::pinout rb)
        : coff_lw(1), LF(lf), LB(lb), RF(rf), RB(rb){};
    ~McCanum() = default;

public:
    void set_motion(int16_t x_speed, int16_t y_speed, int16_t rotate)
    {
        int16_t speed[4] = {0};
        speed[0] = -x_speed + y_speed + rotate * coff_lw;
        speed[1] = x_speed + y_speed - rotate * coff_lw;
        speed[2] = -x_speed + y_speed - rotate * coff_lw;
        speed[3] = x_speed + y_speed + rotate * coff_lw;

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
        RF.set_speed(speed[0]);
        LF.set_speed(speed[1]);
        LB.set_speed(speed[2]);
        RB.set_speed(speed[3]);
    };
};
