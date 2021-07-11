#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"

class adc
{
private:
    uint adc_gpio;

public:
    adc() = default;
    ~adc() = default;
    adc(uint adc) : adc_gpio(adc)
    {
        adc_init();
        adc_gpio_init(adc_gpio);
    };
    uint16_t get_value()
    {
        adc_select_input(adc_gpio - 26);
        return adc_read();
    };
    // void set_value_format();
};
