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
#ifndef _TM1638_H_
#define _TM1638_H_

#include <stdint.h>

// Main Settings
#define TM1638_MAX_BRIGHTNESS   7
#define TM1638_MAX_DIGIT        9
#define TM1638_MAX_VALUE        15

/*
 * Initialize TM1638 LED controller
 */
extern void TM1638_init(const uint8_t enable, const uint8_t brightness);

/*
 * Configure TM1638
 */
extern void TM1638_send_config(const uint8_t enable, const uint8_t brightness);

/*
 * Turn display on/off.
 * value: 1 - on, 0 - off
 */
extern void TM1638_enable(const uint8_t enable);

/*
 * Set display brightness.
 * Min value: 0
 * Max value: 7
 */
extern void TM1638_brightness(const uint8_t brightness);

/*
 * Scan keys
 */
extern uint32_t TM1638_scan_keys(void);

/*
 * Display digit ('0'..'F')
 */
extern void TM1638_set_digit(uint8_t const position, int8_t const value);

/*
 * Write segment buffer to TM1638
 */
extern void TM1638_write_segments(void);

#endif /* !_TM1638_H_ */

