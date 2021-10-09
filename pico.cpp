#include "hardware/clocks.h"
#include "hardware/irq.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"
#include <stdio.h>
#include <string>

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
bool global_motion_callback(repeating_timer_t *rt)
{
    motion.get_motion();
    return true;
}
char uart_read_char()
{
    if (uart_is_readable_within_us(uart0, 1000))
    {
        return uart_getc(uart0);
    }
    else
    {
        return 0;
    }
}
int main()
{
    // stdio_init_all();
    stdio_uart_init_full(uart0, UART_BAUD_RATE, UART_TX, UART_RX);
    volatile int16_t speed_Y = 0;
    volatile int16_t speed_X = 0;
    volatile int16_t speed_rotate = 0;

    adc knob(26);
    ps2 ps(PS_PINOUT);

    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    bool blink = 0;
    // pid<int16_t> x_pid(PID(POS_X));
    // pid<int16_t> y_pid(PID(POS_Y));
    // pid<int16_t> r_pid(PID(POS_R));
    // x_pid.set_target(0);
    // y_pid.set_target(0);
    // r_pid.set_target(0);
    int pos_x = 0, pos_y = 0, pos_r = 0;
    char buffer[256];
    while (true)
    {
        uint32_t timestemp = time_us_32();

        if (knob.get_value() > 4095 / 2)
        {
            motion.set_motion(0, 0, 0);
            sleep_ms(500);
            continue;
        }
        ps.fresh_data();
        if (ps.is_ready() && (ps.get_mode() == 0x73))
        {
            speed_Y = ps.key_value_in_formate(ps2_key::LY, MOTION_MAX_SPEED);
            speed_X = ps.key_value_in_formate(ps2_key::LX, MOTION_MAX_SPEED);
            speed_rotate = ps.key_value_in_formate(ps2_key::RX, MOTION_MAX_SPEED);
        }
        else
        {
            // printf("warning: hand shank is not ready, set control as auto\n");
            int bufpos_x = 0, bufpos_y = 0, bufpos_r = 0;
            bool is_neg = false; // is it a negative num
            int flag = 0;

            size_t idx = 0;
            while (1)
            {
                auto buf = uart_read_char();
                if (buf == '\n' || idx >= 256)
                {
                    break;
                }
                if (buf == 0)
                {
                    flag = -1;
                    break;
                }

                buffer[idx++] = buf;
            }
            if (flag == 0)
                for (size_t i = 0; i < idx; ++i)
                {
                    switch (buffer[i])
                    {
                    // ++i; // skip space
                    case ',':
                        is_neg = 0;
                        ++flag;
                        continue;
                    case ' ':
                        continue;
                    case '-':
                        is_neg = true;
                        continue;
                    }

                    switch (flag)
                    {
                    case 0:
                        bufpos_x = bufpos_x * 10 + (is_neg ? -1 : 1) * (buffer[i] - '0');
                        break;
                    case 1:
                        bufpos_y = bufpos_y * 10 + (is_neg ? -1 : 1) * (buffer[i] - '0');
                        break;
                    case 2:
                        bufpos_r = bufpos_r * 10 + (is_neg ? -1 : 1) * (buffer[i] - '0');
                        break;
                    default:
                        flag = -1;
                        goto break1;
                    }
                }
        break1:
            if (flag != -1)
            {
                speed_X = bufpos_x;
                speed_Y = bufpos_y;
                speed_rotate = bufpos_r;
            }
            // else
            // {
            //     // pos_x += motion.integrator_x;
            //     // pos_y += motion.integrator_y;
            //     // pos_r += motion.integrator_r;
            //     pos_x = 0;
            //     pos_y = 0;
            //     pos_r = 0;
            // }

            // speed_X = x_pid.adjust_in_formate(pos_x, MOTION_DEADZONE, MOTION_MAX_SPEED);
            // speed_Y = y_pid.adjust_in_formate(pos_y, MOTION_DEADZONE, MOTION_MAX_SPEED);
            // speed_rotate = r_pid.adjust_in_formate(pos_r, MOTION_DEADZONE, MOTION_MAX_SPEED);
        }
        printf("xyr: %i, %i, %i\n", speed_X, speed_Y, speed_rotate);
        // if (ps.is_key_pressed(ps2_key::UP))
        // {
        //     motion.reset_integrator();
        //     motion.set_motion(0, 300, 0);
        //     sleep_ms(500);
        //     motion.set_motion(0, 0, 0);
        //     printf("%i\n", motion.mY);
        // }
        // motion.set_motion(speed_X, speed_Y, speed_rotate);
        motion.LF.set_speed_target(speed_X);
        printf("speed_X=%i, measure=%i, control=%i\n", speed_X, motion.LF.speed(), motion.LF.control);
        motion.reset_integrator();

        gpio_put(PICO_DEFAULT_LED_PIN, blink);
        blink = !blink;
        sleep_ms(30);
    }

    return 0;
}
