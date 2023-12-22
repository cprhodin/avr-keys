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

#include "timer.h"
#include "tick.h"
#include "tm1638.h"

#include <stdio.h>
#include <stdlib.h>
#include <util/atomic.h>

#include <avr/pgmspace.h>

uint8_t volatile events = 0;

uint16_t volatile count = 0;

static int8_t scan_key_handler(struct timer_event * this_timer_event)
{
    count++;

    /* set event to enable key scan */
    events |= 0x01;

    /* advance this timer 5 milliseconds */
    this_timer_event->tbtick += TBTICKS_FROM_MS(5);

    /* reschedule this timer */
    return 1;
}

static struct timer_event scan_key_event = {
    .next = &scan_key_event,
    .handler = scan_key_handler,
};


void main(void)
{
    uint32_t last_keys;

    // initialize
    ATOMIC_BLOCK(ATOMIC_FORCEON)
    {
        tbtick_init();
        tick_init();
    }
    // interrupts are enabled

    TM1638_init(1/*enable*/, 3/*brighness*/);

    last_keys = TM1638_scan_keys();

    scan_key_event.tbtick = TBTICKS_FROM_MS(5);
    schedule_timer_event(&scan_key_event, NULL);

    for (;;)
    {
        uint32_t keys;

        if (events & 0x01)
        {
            events &= ~0x01;

            keys = TM1638_scan_keys();

            if (keys != last_keys)
            {
                for (uint8_t i = 0; i < 8; i++)
                {
                    TM1638_set_digit(i, 0xf & (keys >> (i * 4)));
                }

                last_keys = keys;

                TM1638_write_segments();
            }
        }
    }
}

