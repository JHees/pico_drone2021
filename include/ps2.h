#include "pico/stdio.h"
#include "hardware/spi.h"
// #include "pico/binary_info.h"
#define PS_PINOUT PS_CLK, PS_ATT, PS_COM, PS_DAT
#define TICK_CLK 10
class ps2
{
private:
    const uint spi_do;
    const uint spi_di;
    const uint spi_cs;
    const uint spi_clk;
    uint8_t head[2] = {0x01, 0x42};
    struct
    {
        uint8_t mode;
        uint8_t ready;
        uint8_t data[6];
    } ps2_data;
    enum class key_mask : uint16_t
    {
        SELECT = (0x00 << 8 | 0x01 << 0),
        L = (0x00 << 8 | 0x01 << 1),
        R = (0x00 << 8 | 0x01 << 2),
        START = (0x00 << 8 | 0x01 << 3),
        UP = (0x00 << 8 | 0x01 << 4),
        RIGHT = (0x00 << 8 | 0x01 << 5),
        DOWN = (0x00 << 8 | 0x01 << 6),
        LEFT = (0x00 << 8 | 0x01 << 7),
        LT = (0x01 << 8 | 0x01 << 0),
        RT = (0x01 << 8 | 0x01 << 1),
        LB = (0x01 << 8 | 0x01 << 2),
        RB = (0x01 << 8 | 0x01 << 3),
        Y = (0x01 << 8 | 0x01 << 4),
        B = (0x01 << 8 | 0x01 << 5),
        A = (0x01 << 8 | 0x01 << 6),
        X = (0x01 << 8 | 0x01 << 7),
        RX = (0x02 << 8 | 0xff),
        RY = (0x03 << 8 | 0xff),
        LX = (0x04 << 8 | 0xff),
        LY = (0x05 << 8 | 0xff),

    };

public:
    typedef key_mask ps2_key;
    ps2() = default;
    ~ps2() = default;
    ps2(uint CLK, uint CS, uint DO, uint DI)
        : spi_do(DO), spi_di(DI), spi_cs(CS), spi_clk(CLK)
    {
        gpio_init(spi_clk);
        gpio_set_dir(spi_clk, GPIO_OUT);
        gpio_put(spi_clk, 1);

        gpio_init(spi_do);
        gpio_set_dir(spi_do, GPIO_OUT);
        gpio_put(spi_do, 0);

        gpio_init(spi_di);
        gpio_set_dir(spi_di, GPIO_IN);

        gpio_init(spi_cs);
        gpio_set_dir(spi_cs, GPIO_OUT);
        gpio_put(spi_cs, 1);
    };

private:
    inline void cs_select()
    {
        asm volatile("nop \n nop \n nop");
        gpio_put(spi_cs, 0);
        asm volatile("nop \n nop \n nop");
    };
    inline void cs_deselect()
    {
        asm volatile("nop \n nop \n nop");
        gpio_put(spi_cs, 1);
        asm volatile("nop \n nop \n nop");
    };
    inline void tick()
    {
        gpio_put(spi_clk, 1);
        sleep_us(TICK_CLK);
        gpio_put(spi_clk, 0);
        sleep_us(TICK_CLK);
        gpio_put(spi_clk, 1);
        sleep_us(TICK_CLK);
    };
    uint8_t write_read_8bits(uint8_t data = 0x0)
    {
        volatile uint16_t ref = 0x0001;
        volatile uint8_t buf = 0x0;
        for (ref = 0x1; ref < 0x100; ref <<= 1)
        {
            gpio_put(spi_do, ref & data);
            buf = (gpio_get(spi_di) ? ref : 0) | buf;
            tick();
        }
        return buf;
    };
    void write(uint8_t *data, uint8_t len)
    {
        volatile uint8_t n = (len + 7) / 8, i = 0;
        switch (len % 8)
        {
        case 0:
            do
            {
                write_read_8bits(*(data + (i++)));
            case 7:
                write_read_8bits(*(data + (i++)));
            case 6:
                write_read_8bits(*(data + (i++)));
            case 5:
                write_read_8bits(*(data + (i++)));
            case 4:
                write_read_8bits(*(data + (i++)));
            case 3:
                write_read_8bits(*(data + (i++)));
            case 2:
                write_read_8bits(*(data + (i++)));
            case 1:
                write_read_8bits(*(data + (i++)));
            } while (--n > 0);
        };
        // for (size_t i = 0; i < len; ++i)
        // {
        //     write_read_8bits(*(data + i));
        // }
    };
    void read(uint8_t *dst, uint8_t len)
    {
        volatile uint8_t n = (len + 7) / 8, i = 0;
        switch (len % 8)
        {
        case 0:
            do
            {
                *(dst + (i++)) = write_read_8bits();
            case 7:
                *(dst + (i++)) = write_read_8bits();
            case 6:
                *(dst + (i++)) = write_read_8bits();
            case 5:
                *(dst + (i++)) = write_read_8bits();
            case 4:
                *(dst + (i++)) = write_read_8bits();
            case 3:
                *(dst + (i++)) = write_read_8bits();
            case 2:
                *(dst + (i++)) = write_read_8bits();
            case 1:
                *(dst + (i++)) = write_read_8bits();
            } while (--n > 0);
        };
        // for (size_t i = 0; i < len; ++i)
        // {
        //     *(data + i)) = write_read_8bits();
        // }
    }

public:
    bool fresh_data()
    {
        cs_select();
        write(head, 1);
        ps2_data.mode = write_read_8bits(head[1]);
        ps2_data.ready = write_read_8bits();
        if (ps2_data.ready != 0x5A)
        {
            cs_deselect();
            return false;
        }
        read(ps2_data.data, count_of(ps2_data.data));
        cs_deselect();
        return true;
    };
    uint8_t get_mode() const
    {
        return ps2_data.mode;
    };
    bool is_ready() const
    {
        return (ps2_data.ready == 0x5A);
    };
    bool is_key_pressed(ps2_key key) const
    {
        assert(key != key_mask::LX && key != key_mask::LY && key != key_mask::RX && key != key_mask::RY);
        if (ps2_data.ready == 0x5A)
            return !bool(ps2_data.data[(uint16_t)key >> 8] & (uint16_t)key & 0x00ff);
        else
            return 0;
    };

    uint8_t key_value(ps2_key key) const
    {
        if (ps2_data.ready != 0x5A)
            return 0;
        if (key == key_mask::LX || key == key_mask::LY || key == key_mask::RX || key == key_mask::RY)
            return uint8_t(ps2_data.data[(uint16_t)key >> 8] & (uint16_t)key & 0x00ff);
        else
            return !bool(ps2_data.data[(uint16_t)key >> 8] & (uint16_t)key & 0x00ff);
    };

    int16_t key_value_in_formate(ps2_key key, uint16_t range) const
    {
        int16_t value = key_value(key);

        if (key == key_mask::LY || key == key_mask::RY)
        {
            value -= 127;
        }
        else if (key == key_mask::LX || key == key_mask::RX)
        {
            value = 128 - value;
        }
        else
            return value ? range : 0;
        return value > 0 ? value / 128.0f * range : value / 127.0f * range;
    }

    void test_all_key() const 
    {
#define PRINT_KEY(key) is_key_pressed(ps2_key::key) ? printf(#key " ") : 0
#define PRINT_KEY_VALUE(key) printf(#key " %u ", key_value(ps2_key::key))
        PRINT_KEY(LT);
        PRINT_KEY(LB);
        PRINT_KEY(UP);
        PRINT_KEY(DOWN);
        PRINT_KEY(LEFT);
        PRINT_KEY(RIGHT);
        PRINT_KEY(SELECT);
        PRINT_KEY(X);
        PRINT_KEY(Y);
        PRINT_KEY(A);
        PRINT_KEY(B);
        PRINT_KEY(RB);
        PRINT_KEY(RT);
        if (ps2_data.mode == 0x73)
        {
            PRINT_KEY_VALUE(LX);
            PRINT_KEY_VALUE(LY);
            PRINT_KEY_VALUE(RX);
            PRINT_KEY_VALUE(RY);
        }
        else
            printf("mode: 0x41");
        printf("\n");
    }
};

typedef ps2::ps2_key ps2_key;