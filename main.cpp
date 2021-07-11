#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "hardware/spi.h"

#include "motor.h"
#include "adc.h"
#include "ps2.h"

#define MOTOR_LF_PWM 0
#define MOTOR_LF_DIR 1
#define MOTOR_LF_MEASURE_A 2
#define MOTOR_LF_MEASURE_B 3
#define MOTOR_RF_PWM 4
#define MOTOR_RF_DIR 5
#define MOTOR_RF_MEASURE_A 6
#define MOTOR_RF_MEASURE_B 7
#define MOTOR_LB_PWM 8
#define MOTOR_LB_DIR 9
#define MOTOR_LB_MEASURE_A 10
#define MOTOR_LB_MEASURE_B 11
#define MOTOR_RB_PWM 12
#define MOTOR_RB_DIR 13
#define MOTOR_RB_MEASURE_A 14
#define MOTOR_RB_MEASURE_B 15

#define MOTOR_MAX_SPEED 300
// const uint MEASURE_PIN = 27;

int main()
{
    stdio_init_all();
    McCanum motion(126, 185, MOTORS);
    adc knob(26);
    ps2 ps(16, 17, 18, 19);

    // assert(pwm_gpio_to_channel(MEASURE_PIN) == PWM_CHAN_B);
    // uint slice_num_measurepin = pwm_gpio_to_slice_num(MEASURE_PIN);
    // gpio_set_function(MEASURE_PIN, GPIO_FUNC_PWM);

    // pwm_config pwm_cfg_measurepin = pwm_get_default_config();
    // pwm_config_set_clkdiv_mode(&pwm_cfg_measurepin, PWM_DIV_B_HIGH);
    // pwm_config_set_clkdiv(&pwm_cfg_measurepin, 250);
    // pwm_init(slice_num_measurepin, &pwm_cfg_measurepin, false);
    // pwm_set_gpio_level(MEASURE_PIN, 0);

    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    bool blink = 0;

    while (true)
    {
        if (ps.get_mode() != 0x73)
        {
            printf("error: The hand shank is not 0x73 mode or not ready\n");
            sleep_ms(1000);
            continue;
        }

        ps.fresh_data();
        int16_t speed_Y = ps.key_value_in_formate(ps2_key::LY, MOTOR_MAX_SPEED);
        int16_t speed_X = ps.key_value_in_formate(ps2_key::LX, MOTOR_MAX_SPEED);
        int16_t speed_rotate = ps.key_value_in_formate(ps2_key::RX, MOTOR_MAX_SPEED);
        motion.set_motion(speed_X, speed_Y, speed_rotate);

        uint16_t value = knob.get_value();
        // pwm_set_counter(slice_num_measurepin, 0);
        // pwm_set_enabled(slice_num_measurepin, true);
        // sleep_ms(10);
        // pwm_set_enabled(slice_num_measurepin, false);
        // float duty_cycle = pwm_get_counter(slice_num_measurepin) / (clock_get_hz(clk_sys) / 250 * 10 * 1e-3);
        // printf("ly: %u, speed: %i, duty: %f\n",
        //        ps.key_value(ps2_key::LY), speed, duty_cycle);
        printf("x: %i, y: %i, r: %i\n",
               speed_X, speed_Y, speed_rotate);
        gpio_put(PICO_DEFAULT_LED_PIN, blink);
        blink = !blink;
        sleep_ms(30);
    }

    return 0;
}
