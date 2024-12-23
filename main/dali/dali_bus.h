#ifndef DALI_BUS_H
#define DALI_BUS_H

/*
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301  USA
*/

#include "driver/gptimer.h"

// Conversion of Dali Arduino library from C++ to C and ESP-IDF:
// https://github.com/hubsif/arduino-dali

/** some enum */
typedef enum _dali_ret_t {
    DALI_RX_EMPTY = -1,
    DALI_RX_ERROR = -2,
    DALI_SENT = -3,
    DALI_INVALID_PARAMETER = -4,
    DALI_BUSY = -5,
    DALI_READY_TIMEOUT = -6,
    DALI_SEND_TIMEOUT = -7,
    DALI_COLLISION = -8,
} dali_ret_t;

typedef enum _dali_bus_state_t {
    TX_START_1ST,
    TX_START_2ND,
    TX_BIT_1ST,
    TX_BIT_2ND,
    TX_STOP_1ST,
    TX_STOP,
    IDLE,
    SHORT,
    WAIT_RX,
    RX_START,
    RX_BIT,
    RX_STOP
} dali_bus_state_t;

typedef struct _dali_bus {
    uint8_t tx_pin, rx_pin;
    uint8_t active_low;
    uint8_t tx_message[3];
    uint8_t tx_length;

    volatile uint8_t bus_idle_count;

    volatile dali_bus_state_t state;
    volatile uint8_t tx_pos;
    volatile uint8_t tx_bus_level;
    volatile uint8_t tx_collision;

    volatile unsigned long rx_last_change;
    volatile uint8_t rx_message;
    volatile uint8_t rx_length;
    volatile uint8_t rx_error;

    volatile uint32_t last_level;
    volatile uint32_t last_delta;
    volatile uint32_t bus_changes;

    volatile uint64_t isr_us;
    volatile uint64_t isr_count;

    gptimer_handle_t gptimer;
    volatile bool rx_paused;
} dali_bus;

void dali_bus_free(dali_bus *bus);
dali_bus *dali_bus_begin(uint8_t tx_pin, uint8_t rx_pin, bool active_low);
dali_ret_t dali_bus_send_raw(dali_bus *bus, const uint8_t *message, uint8_t length);
bool dali_bus_is_idle(dali_bus *bus);
int dali_bus_get_last_response(dali_bus *bus);
uint32_t dali_bus_get_bus_level(dali_bus *bus);
void dali_bus_set_bus_level(dali_bus *bus, uint32_t level);
bool dali_bus_is_delta_within_te(unsigned long delta);
bool dali_bus_is_delta_within_2te(unsigned long delta);

#endif