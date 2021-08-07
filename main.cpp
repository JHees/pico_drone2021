#include "hardware/clocks.h"
#include "hardware/irq.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"
#include <stdio.h>

#include "config.h"
#include "adc.h"
#include "filter.h"
#include "motor.h"
#include "ps2.h"


// const uint MEASURE_PIN = 27;

int main()
{
    // stdio_init_all();
    stdio_uart_init_full(uart0, UART_BAUD_RATE, UART_TX, UART_RX);

    McCanum motion(MOTORS);
    volatile int16_t speed_Y = 0;
    volatile int16_t speed_X = 0;
    volatile int16_t speed_rotate = 0;

    adc knob(26);
    uint16_t value = 0;

    ps2 ps(PS_PINOUT);

    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    bool blink = 0;

    while (true)
    {
        ps.fresh_data();
        if (!ps.is_ready() || (ps.get_mode() != 0x73))
        {
            printf("warning: The hand shank is not 0x73 mode or not ready\n");
            motion.set_motion(0, 0, 0);

            sleep_ms(500);
            continue;
        }
        speed_Y = ps.key_value_in_formate(ps2_key::LY, MOTOR_MAX_SPEED);
        speed_X = ps.key_value_in_formate(ps2_key::LX, MOTOR_MAX_SPEED);
        speed_rotate = ps.key_value_in_formate(ps2_key::RX, MOTOR_MAX_SPEED);
        motion.set_motion(speed_X, speed_Y, speed_rotate);
        printf("ly: %u, lx: %u, rx: %u\n", ps.key_value(ps2_key::LX), ps.key_value(ps2_key::LY),
               ps.key_value(ps2_key::RX));
        value = knob.get_value();
        int16_t Kp = value / 4095.0 * MOTOR_MAX_SPEED;

        // printf("%i,%i,%i\n", Kp, speed, ad);
        // pwm_set_counter(slice_num_measurepin, 0);
        // pwm_set_enabled(slice_num_measurepin, true);
        // sleep_ms(10);
        // pwm_set_enabled(slice_num_measurepin, false);
        // float duty_cycle = pwm_get_counter(slice_num_measurepin) / (clock_get_hz(clk_sys) / 250 * 10 * 1e-3);
        // printf("ly: %u, speed: %i, duty: %f\n",
        //        ps.key_value(ps2_key::LY), speed, duty_cycle);
        gpio_put(PICO_DEFAULT_LED_PIN, blink);
        blink = !blink;
        sleep_ms(10);
    }

    return 0;
}
