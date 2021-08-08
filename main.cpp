#include "hardware/clocks.h"
#include "hardware/irq.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"
#include <stdio.h>

#include "adc.h"
#include "config.h"
#include "filter.h"
#include "motor.h"
#include "ps2.h"

// const uint MEASURE_PIN = 27;
McCanum motion(MOTORS);

void global_gpio_irq_callback(uint gpio, uint32_t event)
{
    switch (gpio)
    {
    case MOTOR_LF_MEASURE_A:
    case MOTOR_LF_MEASURE_B:
        return motion.LF.speed_callback(gpio, event);
    case MOTOR_RF_MEASURE_A:
    case MOTOR_RF_MEASURE_B:
        return motion.RF.speed_callback(gpio, event);
    case MOTOR_LB_MEASURE_A:
    case MOTOR_LB_MEASURE_B:
        return motion.LB.speed_callback(gpio, event);
    case MOTOR_RB_MEASURE_A:
    case MOTOR_RB_MEASURE_B:
        return motion.RB.speed_callback(gpio, event);
    }
}
// bool global_motion_pid_callback(repeating_timer_t *rt)
// {
//     motion.get_motion(motion.mX, motion.mY, motion.mR);
//     motion.set_motion(motion.x_pid.adjust_in_formate(motion.mX, 5, MOTION_MAX_SPEED),
//                       motion.y_pid.adjust_in_formate(motion.mY, 5, MOTION_MAX_SPEED),
//                       motion.r_pid.adjust_in_formate(motion.mR, 5, MOTION_MAX_SPEED));
//     return true;
// }
int main()
{
    // stdio_init_all();
    stdio_uart_init_full(uart0, UART_BAUD_RATE, UART_TX, UART_RX);
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
        speed_Y = ps.key_value_in_formate(ps2_key::LY, MOTION_MAX_SPEED);
        speed_X = ps.key_value_in_formate(ps2_key::LX, MOTION_MAX_SPEED);
        speed_rotate = ps.key_value_in_formate(ps2_key::RX, MOTION_MAX_SPEED);

        value = knob.get_value();
        float Kp = (value / 4095.0 - 0.5) * 2 * 10;

        motion.set_motion(speed_X, speed_Y, speed_rotate);
        // int16_t mX, mY, mR;
        // motion.get_motion(mX, &mY, &mR);

        printf("X_target=%i, X_speed=%i,Y_target=%i, Y_speed=%i,R_target=%i,R_speed=%i\n", speed_X, motion.mX, speed_Y,
               motion.mY, speed_rotate, motion.mR);
 
        gpio_put(PICO_DEFAULT_LED_PIN, blink);
        blink = !blink;
        sleep_ms(30);
    }

    return 0;
}
