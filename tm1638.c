/*
 * Copyright 2013-2023 Chris Rhodin <chris@notav8.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "project.h"

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include "pinmap.h"
#include "tm1638.h"

#define TM1638_DELAY_US         (2)

// TM1638 commands
#define TM1638_CMD_SET_DATA     0x40
#define TM1638_CMD_SET_ADDR     0xC0
#define TM1638_CMD_SET_DISPLAY  0x80

// TM1638 data settings (use bitwise OR to contruct complete command)
#define TM1638_SET_DATA_WRITE   0x00 // write data to the display register
#define TM1638_SET_DATA_READ    0x02 // read the key scan data
#define TM1638_SET_DATA_A_ADDR  0x00 // automatic address increment
#define TM1638_SET_DATA_F_ADDR  0x04 // fixed address
#define TM1638_SET_DATA_M_NORM  0x00 // normal mode
#define TM1638_SET_DATA_M_TEST  0x10 // test mode

// TM1638 display control command set (use bitwise OR to consruct complete command)
#define TM1638_SET_DISPLAY_OFF  0x00 // off
#define TM1638_SET_DISPLAY_ON   0x08 // on

/*
 * Display raw segments at position (0x00..0x07)
 *
 *      bits:
 *        -0 (a)-
 *       |       |
 *     5 (f)   1 (b)
 *       |       |
 *        -6 (g)-
 *       |       |
 *     4 (e)   2 (c)
 *       |       |
 *        -3 (d)- *7 (dp)
 *
 * Example segment configurations:
 * - for character 'H', segments=0b01110110
 * - for character '-', segments=0b01000000
 * - etc.
 */
static const uint8_t PROGMEM _digit_segments[] =
{
    0x3F, // 0
    0x06, // 1
    0x5B, // 2
    0x4F, // 3
    0x66, // 4
    0x6D, // 5
    0x7D, // 6
    0x07, // 7
    0x7F, // 8
    0x6F, // 9
    0x77, // A
    0x7C, // b
    0x39, // C
    0x5E, // d
    0x79, // E
    0x71, // F
};

/*
 * segment buffer for LED display
 */
static uint16_t segment_buffer[8];

/*
 * default to display on at maximum brightness
 */
static uint8_t _config = TM1638_SET_DISPLAY_ON | TM1638_MAX_BRIGHTNESS;

/*
 * read a long (32 bits) from the TM1638
 */
static uint32_t
TM1638_read_long(void)
{
    uint32_t value = 0;

    TM1638_DIO_INPUT();
    TM1638_DIO_HIGH();

    for (uint8_t i = 0; i < 32; i++)
    {
        TM1638_CLK_LOW();
        _delay_us(TM1638_DELAY_US);

        if (TM1638_DIO_READ())
        {
            value |= 0x80000000;
        }
        value >>= 1;

        TM1638_CLK_HIGH();
        _delay_us(TM1638_DELAY_US);
    }

    TM1638_DIO_OUTPUT();

    return value;
}

/*
 * write bytes to the TM1638
 */
static void
TM1638_write_bytes(uint8_t const * bytes, uint8_t num)
{
    for (uint8_t n = 0; n < num; n++)
    {
        uint8_t byte = *bytes++;

        for (uint8_t i = 0; i < 8; ++i)
        {
            TM1638_CLK_LOW();
            if (byte & 0x01)
            {
                TM1638_DIO_HIGH();
            }
            else
            {
                TM1638_DIO_LOW();
            }
            _delay_us(TM1638_DELAY_US);

            TM1638_CLK_HIGH();
            _delay_us(TM1638_DELAY_US);

            byte >>= 1;
        }
    }
}

static void
TM1638_write_byte(uint8_t byte)
{
    TM1638_write_bytes(&byte, 1);
}

static void
TM1638_send_command(const uint8_t command)
{
    TM1638_STB_LOW();
    _delay_us(TM1638_DELAY_US);

    TM1638_write_bytes(&command, 1);

    TM1638_STB_HIGH();
    _delay_us(TM1638_DELAY_US);
}

void
TM1638_send_config(const uint8_t enable, const uint8_t brightness)
{
    _config = (enable ? TM1638_SET_DISPLAY_ON : TM1638_SET_DISPLAY_OFF)
            | (brightness > TM1638_MAX_BRIGHTNESS ? TM1638_MAX_BRIGHTNESS : brightness);

    TM1638_send_command(TM1638_CMD_SET_DISPLAY | _config);
}

void
TM1638_enable(const uint8_t enable)
{
    TM1638_send_config(enable,
                       _config & TM1638_MAX_BRIGHTNESS);
}

void
TM1638_brightness(const uint8_t brightness)
{
    TM1638_send_config(_config & TM1638_SET_DISPLAY_ON,
                       brightness);
}

void
TM1638_write_segments(void)
{
    TM1638_send_command(TM1638_CMD_SET_DATA);

    TM1638_STB_LOW();
    _delay_us(TM1638_DELAY_US);

    TM1638_write_byte(TM1638_CMD_SET_ADDR);

    for (uint8_t i = 0; i < ARRAY_SIZE(segment_buffer); i++)
    {
        TM1638_write_bytes((uint8_t *) segment_buffer, 16);
    }

    TM1638_STB_HIGH();
    _delay_us(TM1638_DELAY_US);
}

void
TM1638_set_digit(uint8_t const digit, int8_t const value)
{
    if (digit <= TM1638_MAX_DIGIT)
    {
        uint16_t const digit_mask = 0x0001 << digit;
        uint8_t segments;

        if ((value < 0) || (value > TM1638_MAX_VALUE))
        {
            segments = 0x00;
        }
        else
        {
            segments = pgm_read_word(&_digit_segments[value]);
        }

        for (uint8_t i = 0; i < ARRAY_SIZE(segment_buffer); i++)
        {
            if (segments & 0x01)
            {
                segment_buffer[i] |= digit_mask;
            }
            else
            {
                segment_buffer[i] &= ~digit_mask;
            }

            segments >>= 1;
        }
    }
}

uint32_t
TM1638_scan_keys(void)
{
    uint32_t keys;

    TM1638_STB_LOW();
    _delay_us(TM1638_DELAY_US);

    TM1638_write_byte(TM1638_CMD_SET_DATA | TM1638_SET_DATA_READ);

    keys = TM1638_read_long();

    TM1638_STB_HIGH();
    _delay_us(TM1638_DELAY_US);

    return keys;
}

void
TM1638_init(const uint8_t enable, const uint8_t brightness)
{
    pinmap_set(TM1638_DIO | TM1638_CLK | TM1638_STB);
    pinmap_set_ddr(TM1638_DIO | TM1638_CLK | TM1638_STB);
    _delay_us(TM1638_DELAY_US);

    TM1638_send_config(enable, brightness);

    for (uint8_t i = 0; i < ARRAY_SIZE(segment_buffer); i++)
    {
        segment_buffer[i] = 0x0000;
    }

    TM1638_write_segments();
}

