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

// Conversion of Dali Arduino library from C++ to C and ESP-IDF:
// https://github.com/hubsif/arduino-dali

#include "esp_timer.h"
#include "esp_log.h"
#include "dali/dali.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/*
    0- 31  arc power control commands
   32-143  configuration commands
  144-223  query commands
  224-255  application extended commands
  256-271  special commands
  272-287  extended special commands

  or:
  256-271  0b10100001
  272-287  0b11000001

  no address: >= 256
  repeat: 32-128(-143), 258, 259,
*/

dali *dali_begin(uint8_t tx_pin, uint8_t rx_pin, bool active_low /* = true*/)
{
    dali_bus *bus = NULL;
    dali *dali_ = NULL;

    bus = dali_bus_begin(tx_pin, rx_pin, active_low);
    if (bus == NULL) {
        return NULL;
    }

    dali_ = (dali *)calloc(1, sizeof(dali));
    if (dali_ == NULL) {

        return NULL;
    }
    dali_->bus = bus;

    return dali_;
}

int dali_send_raw_wait(dali *dali_, const uint8_t *message, uint8_t length, uint8_t timeout)
{
    int64_t time = esp_timer_get_time();

    int result;

    // ESP_LOGI("dali_send_raw_wait", "time: %llu, timeout: %d", time, (timeout == 0 ? 50 : timeout) * 1000);
    while (!dali_bus_is_idle(dali_->bus)) {
        int64_t time_now = esp_timer_get_time();
        if ((time_now - time) > ((timeout == 0 ? 50 : timeout) * 1000)) {
            return DALI_READY_TIMEOUT;
        }
    }
    result = dali_bus_send_raw(dali_->bus, message, length);

    while (!dali_bus_is_idle(dali_->bus)) {
        int64_t time_now = esp_timer_get_time();
        if ((time_now - time) > ((timeout == 0 ? 50 : timeout) * 1000)) {
            return DALI_READY_TIMEOUT;
        }
    }
    // ESP_LOGI("dali_send_raw_wait", "Result: %d, RX error: %u, last delta: %lu, isr: %llu, bus changes: %lu, length: %u", result, dali_->bus->rx_error, dali_->bus->last_delta, dali_->bus->isr_us, dali_->bus->bus_changes, dali_->bus->rx_length);
    return (result != DALI_SENT) ? result : dali_bus_get_last_response(dali_->bus);
}

uint8_t *dali_prepare_cmd(uint8_t *message, uint8_t address, uint8_t command, uint8_t type, uint8_t selector)
{
    message[0] = type << 7;
    message[0] |= address << 1;
    message[0] |= selector;

    message[1] = command;
    return message;
}

dali_ret_t dali_send_arc(dali *dali_, uint8_t address, uint8_t value, dali_address_type_t addr_type)
{
    uint8_t message[2];
    return dali_bus_send_raw(dali_->bus, dali_prepare_cmd(message, address, value, addr_type, 0), 2);
}

dali_ret_t dali_send_arc_wait(dali *dali_, uint8_t address, uint8_t value, dali_address_type_t addr_type, uint8_t timeout)
{
    uint8_t message[2];
    return dali_send_raw_wait(dali_, dali_prepare_cmd(message, address, value, addr_type, 0), 2, timeout);
}

dali_ret_t dali_send_cmd(dali *dali_, uint8_t address, dali_cmd_t command, dali_address_type_t addr_type)
{
    uint8_t message[2];
    return dali_send_raw(dali_, dali_prepare_cmd(message, address, command, addr_type, 1), 2);
}

int dali_send_cmd_wait(dali *dali_, uint8_t address, dali_cmd_t command, dali_address_type_t addr_type, uint8_t timeout)
{
    uint8_t send_count = (command > 32 && command < 143) ? 2 : 1; // config commands need to be sent twice

    uint8_t message[2];
    int result = 0;

    while (send_count) {
        result = dali_send_raw_wait(dali_, dali_prepare_cmd(message, address, command, addr_type, 1), 2, timeout);
        if (result != DALI_RX_EMPTY) {
            return result;
        }
        send_count--;
    }

    return result;
}

uint8_t *dali_prepare_special_cmd(uint8_t *message, uint16_t command, uint8_t value)
{
    message[0] = ((uint8_t) command + 16) << 1; // convert command number
    message[0] |= 0b10000001;                // to special command uint8_t

    message[1] = value;
    return message;
}

dali_ret_t dali_send_special_cmd(dali *dali_, dali_special_cmd_t command, uint8_t value)
{
    if (command < 256 || command > 287) {
        return 1;
    }
    uint8_t message[2];
    return dali_bus_send_raw(dali_->bus, dali_prepare_special_cmd(message, command, value), 2);
}

int dali_send_special_cmd_wait(dali *dali_, dali_special_cmd_t command, uint8_t value, uint8_t timeout)
{
    uint8_t message[2];
    return dali_send_raw_wait(dali_, dali_prepare_special_cmd(message, command, value), 2, timeout);
}

void dali_commission(dali *dali_, uint8_t start_address, bool only_new)
{
    dali_->next_short_address = start_address;
    dali_->commission_only_new = only_new;

    // start commissioning
    dali_->commission_state = DALI_COMMISSION_INIT;
}

void dali_commission_tick(dali *dali_)
{
    // TODO: set timeout for commissioning?
    // TODO: also clear group addresses?

    static uint8_t search_iterations;
    static unsigned long current_search_address;

    if (dali_bus_is_idle(dali_->bus)) { // wait until bus is idle
        switch (dali_->commission_state) {
        case DALI_COMMISSION_INIT:
            dali_send_special_cmd(dali_, DALI_CMD_INITIALISE, (dali_->commission_only_new ? 255 : 0));
            dali_->commission_state = DALI_COMMISSION_INIT2;
            break;
        case DALI_COMMISSION_INIT2:
            dali_send_special_cmd(dali_, DALI_CMD_INITIALISE, (dali_->commission_only_new ? 255 : 0));
            dali_->commission_state = (dali_->commission_only_new ? DALI_COMMISSION_RANDOM : DALI_COMMISSION_WRITE_DTR);
            break;
        case DALI_COMMISSION_WRITE_DTR:
            dali_send_special_cmd(dali_, DALI_CMD_SET_DTR, 255);
            dali_->commission_state = DALI_COMMISSION_REMOVE_SHORT;
            break;
        case DALI_COMMISSION_REMOVE_SHORT:
            dali_send_cmd(dali_, 63, DALI_CMD_DTR_AS_SHORT, DALI_GROUP_ADDRESS);
            dali_->commission_state = DALI_COMMISSION_REMOVE_SHORT2;
            break;
        case DALI_COMMISSION_REMOVE_SHORT2:
            dali_send_cmd(dali_, 63, DALI_CMD_DTR_AS_SHORT, DALI_GROUP_ADDRESS);
            dali_->commission_state = DALI_COMMISSION_RANDOM;
            break;
        case DALI_COMMISSION_RANDOM:
            dali_send_special_cmd(dali_, DALI_CMD_RANDOMISE, 0);
            dali_->commission_state = DALI_COMMISSION_RANDOM2;
            break;
        case DALI_COMMISSION_RANDOM2:
            dali_send_special_cmd(dali_, DALI_CMD_RANDOMISE, 0);
            dali_->commission_state = DALI_COMMISSION_RANDOMWAIT;
            break;
        case DALI_COMMISSION_RANDOMWAIT:  // wait 100ms for random address to generate
            if (dali_->bus->bus_idle_count >= 255) {
                dali_->commission_state = DALI_COMMISSION_STARTSEARCH;
            }
            break;
        case DALI_COMMISSION_STARTSEARCH:
        case DALI_COMMISSION_SEARCHHIGH:
            if (dali_->commission_state == DALI_COMMISSION_STARTSEARCH) {
                search_iterations = 0;
                current_search_address = 0xFFFFFF;
            }
            dali_send_special_cmd(dali_, DALI_CMD_SEARCHADDRH, (current_search_address >> 16) & 0xFF);
            dali_->commission_state = DALI_COMMISSION_SEARCHMID;
            break;
        case DALI_COMMISSION_SEARCHMID:
            dali_send_special_cmd(dali_, DALI_CMD_SEARCHADDRM, (current_search_address >> 8) & 0xFF);
            dali_->commission_state = DALI_COMMISSION_SEARCHLOW;
            break;
        case DALI_COMMISSION_SEARCHLOW:
            dali_send_special_cmd(dali_, DALI_CMD_SEARCHADDRL, (current_search_address) & 0xFF);
            dali_->commission_state = DALI_COMMISSION_COMPARE;
            break;
        case DALI_COMMISSION_COMPARE:
            dali_send_special_cmd(dali_, DALI_CMD_COMPARE, 0);
            dali_->commission_state = DALI_COMMISSION_CHECKFOUND;
            break;
        case DALI_COMMISSION_CHECKFOUND: {
            // create scope for response variable
            int response = dali_bus_get_last_response(dali_->bus);
            if (response != DALI_RX_EMPTY)
                if (search_iterations >= 24) { // ballast found
                    dali_->commission_state = DALI_COMMISSION_PROGRAMSHORT;
                } else {
                    current_search_address -= (0x800000 >> search_iterations);
                    dali_->commission_state = DALI_COMMISSION_SEARCHHIGH;
                } else if (search_iterations == 0 || search_iterations > 24) { // no device at all responded or error
                dali_->commission_state = DALI_COMMISSION_TERMINATE;
            } else if (search_iterations == 24) { // device responded before, but didn't now, so address is one higher
                current_search_address++;           // and for the device to act at upcoming commands, we need to send the actual address
                dali_->commission_state = DALI_COMMISSION_SEARCHHIGH;
            } else { // there's a device that didn't respond anymore, increase address
                current_search_address += (0x800000 >> search_iterations);
                dali_->commission_state = DALI_COMMISSION_SEARCHHIGH;
            }
            search_iterations++;
            break;
        }
        case DALI_COMMISSION_PROGRAMSHORT:
            dali_send_special_cmd(dali_, DALI_CMD_PROGRAMSHORT, (dali_->next_short_address << 1) | 1);
            dali_->commission_state = DALI_COMMISSION_VERIFYSHORT;
            break;
        case DALI_COMMISSION_VERIFYSHORT:
            dali_send_special_cmd(dali_, DALI_CMD_VERIFYSHORT, (dali_->next_short_address << 1) | 1);
            dali_->commission_state = DALI_COMMISSION_VERIFYSHORTRESPONSE;
            break;
        case DALI_COMMISSION_VERIFYSHORTRESPONSE:
            if (dali_bus_get_last_response(dali_->bus) == 0xFF) {
                dali_->next_short_address++;
                dali_->commission_state = DALI_COMMISSION_WITHDRAW;
            } else
                // error, stop commissioning
            {
                dali_->commission_state = DALI_COMMISSION_TERMINATE;
            }
            break;
        case DALI_COMMISSION_WITHDRAW:
            dali_send_special_cmd(dali_, DALI_CMD_WITHDRAW, 0);
            dali_->commission_state = DALI_COMMISSION_STARTSEARCH;
            break;
        case DALI_COMMISSION_TERMINATE:
            dali_send_special_cmd(dali_, DALI_CMD_TERMINATE, 0);
            dali_->commission_state = DALI_COMMISSION_OFF;
            break;
        default:
            break;
        }
    }
}
