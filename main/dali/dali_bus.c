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

#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_attr.h"
#include "driver/gpio.h"

#include "dali/dali_bus.h"

#define TAG  "Dali Bus"
#define LOW  0
#define HIGH 1

const int DALI_BAUD = 1200;
const unsigned long DALI_SCALING = 1;
const unsigned long DALI_TE = 417;
const unsigned long DALI_TE_MIN = (80 * DALI_TE) / 100;                  // 333us
const unsigned long DALI_TE_MAX = (120 * DALI_TE) / 100;                 // 500us

// wrapper for interrupt handler
static void dali_bus_pinchange_isr(dali_bus *bus);
static void IRAM_ATTR dali_bus_hook_pinchange_isr(void *arg)
{
    dali_bus *bus;

    bus = (dali_bus *)arg;
    dali_bus_pinchange_isr(bus);
}

static void dali_bus_timer_isr(dali_bus *bus);
static bool IRAM_ATTR dali_bus_hook_timer_isr(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *arg)
{
    dali_bus *bus;
    bus = (dali_bus *)arg;
    dali_bus_timer_isr(bus);
    return true;
}

void dali_bus_free(dali_bus *bus)
{
    assert(bus != NULL);
    free(bus);
}

dali_bus *dali_bus_begin(uint8_t tx_pin, uint8_t rx_pin, bool active_low)
{
    dali_bus *bus = NULL;
    esp_err_t err;

    bus = (dali_bus *)calloc(1, sizeof(dali_bus));
    assert(bus != NULL);

    bus->tx_pin = tx_pin;
    bus->rx_pin = rx_pin;
    bus->active_low = active_low;

    // init bus state
    bus->state = IDLE;

    // TX pin setup
    gpio_config_t tx_io_conf = {};
    tx_io_conf.intr_type = GPIO_INTR_DISABLE;
    tx_io_conf.mode = GPIO_MODE_OUTPUT;
    tx_io_conf.pin_bit_mask = (1ULL << tx_pin);
    tx_io_conf.pull_down_en = 0;
    tx_io_conf.pull_up_en = 0;
    err = gpio_config(&tx_io_conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize TX pin %d", tx_pin);
        free(bus);
        return NULL;
    }

    dali_bus_set_bus_level(bus, HIGH);

    // RX pin setup
    gpio_config_t rx_io_conf = {};
    rx_io_conf.intr_type = GPIO_INTR_ANYEDGE;
    rx_io_conf.mode = GPIO_MODE_INPUT;
    rx_io_conf.pin_bit_mask = (1ULL << rx_pin);
    rx_io_conf.pull_down_en = 0;
    rx_io_conf.pull_up_en = 0;
    err = gpio_config(&rx_io_conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize RX pin %d", rx_pin);
        free(bus);
        return NULL;
    }

    // Setup timer interrupt
    gptimer_config_t timer_config = {
        .clk_src       = GPTIMER_CLK_SRC_DEFAULT,
        .direction     = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000, // 1MHz, 1 tick=1us
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &bus->gptimer));

    gptimer_alarm_config_t alarm_config = {
        .reload_count = 0,
        .alarm_count  = DALI_TE, // period  in us
        .flags.auto_reload_on_alarm = true,
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(bus->gptimer, &alarm_config));

    gptimer_event_callbacks_t cbs = {
        .on_alarm = dali_bus_hook_timer_isr,
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(bus->gptimer, &cbs, (void *)bus));

    ESP_ERROR_CHECK(gptimer_enable(bus->gptimer));
    ESP_ERROR_CHECK(gptimer_start(bus->gptimer));

    // Get last level to simulate pin state change
    bus->last_level = dali_bus_get_bus_level(bus);

    // Setup ISR
    gpio_install_isr_service(0);
    gpio_isr_handler_add(rx_pin, dali_bus_hook_pinchange_isr, (void *)bus);


    return bus;
}

dali_ret_t dali_bus_send_raw(dali_bus *bus, const uint8_t *message, uint8_t length)
{
    if (length > 3) {
        return DALI_INVALID_PARAMETER;
    }
    if (bus->state != IDLE) {
        return DALI_BUSY;
    }

    // prepare variables for sending
    for (uint8_t i = 0; i < length; i++) {
        bus->tx_message[i] = message[i];
        bus->tx_length = length * 8;
        bus->tx_collision = 0;
        bus->rx_message = DALI_RX_EMPTY;
        bus->rx_length = 0;

        // initiate transmission
        bus->state = TX_START_1ST;
    }
    return DALI_SENT;
}

bool dali_bus_is_idle(dali_bus *bus)
{
    return (bus->state == IDLE);
}

int dali_bus_get_last_response(dali_bus *bus)
{
    int response;
    switch (bus->rx_length) {
    case 16:
        response = bus->rx_message;
        break;
    case 0:
        response = DALI_RX_EMPTY;
        break;
    default:
        // ESP_LOGE(TAG, "Bus received error, rx_length: %d", bus->rx_length);
        response = DALI_RX_ERROR;
    }
    bus->rx_length = 0;
    return response;
}

static void IRAM_ATTR dali_bus_timer_isr(dali_bus *bus)
{
    bus->isr_count++;

    if (bus->bus_idle_count < 0xff) { // increment idle counter avoiding overflow
        bus->bus_idle_count++;
    }

    if (bus->bus_idle_count == (4 * DALI_SCALING) && dali_bus_get_bus_level(bus) == LOW) { // bus is low idle for more than 2 TE, something's pulling down for too long
        bus->state = SHORT;
        dali_bus_set_bus_level(bus, HIGH);
        // TODO: log error?
    }

    // timer state machine
    switch (bus->state) {
    case TX_START_1ST: // initiate transmission by setting bus low (1st half)
        if (bus->bus_idle_count >= (26 * DALI_SCALING)) { // wait at least 9.17ms (22 TE) settling time before sending (little more for TCI compatibility)
            dali_bus_set_bus_level(bus, LOW);
            bus->state = TX_START_2ND;
        }
        break;
    case TX_START_2ND: // send start bit (2nd half)
        dali_bus_set_bus_level(bus, HIGH);
        bus->tx_pos = 0;
        bus->state = TX_BIT_1ST;
        break;
    case TX_BIT_1ST: // prepare bus for bit (1st half)
        if (bus->tx_message[bus->tx_pos >> 3] & 1 << (7 - (bus->tx_pos & 0x7))) {
            dali_bus_set_bus_level(bus, LOW);
        } else {
            dali_bus_set_bus_level(bus, HIGH);
        }
        bus->state = TX_BIT_2ND;
        break;
    case TX_BIT_2ND: // send bit (2nd half)
        if (bus->tx_message[bus->tx_pos >> 3] & 1 << (7 - (bus->tx_pos & 0x7))) {
            dali_bus_set_bus_level(bus, HIGH);
        } else {
            dali_bus_set_bus_level(bus, LOW);
        }
        bus->tx_pos++;
        if (bus->tx_pos < bus->tx_length) {
            bus->state = TX_BIT_1ST;
        } else {
            bus->state = TX_STOP_1ST;
        }
        break;
    case TX_STOP_1ST: // 1st stop bit (1st half)
        dali_bus_set_bus_level(bus, HIGH);
        bus->state = TX_STOP;
        break;
    case TX_STOP: // remaining stop half-bits
        if (bus->bus_idle_count >= (4 * DALI_SCALING)) {
            bus->state = WAIT_RX;
            bus->bus_idle_count = 0;
        }
        break;
    case WAIT_RX: // wait 9.17ms (22 TE) for a response
        if (bus->bus_idle_count > (23 * DALI_SCALING)) {
            bus->rx_error = 4;
            bus->state = IDLE; // response timed out
        }
        break;
    case RX_STOP:
        if (bus->bus_idle_count > (4 * DALI_SCALING)) {
            // rx message incl stop bits finished.
            bus->state = IDLE;
        }
        break;
    case RX_START:
    case RX_BIT:
        if (bus->bus_idle_count > (3 * DALI_SCALING)) { // bus has been inactive for too long
            bus->state = IDLE;    // rx has been interrupted, bus is idle
        }
        break;
    case IDLE:
    case SHORT:
        break;
    }
}

static void IRAM_ATTR dali_bus_pinchange_isr(dali_bus *bus)
{
    uint32_t bus_level = dali_bus_get_bus_level(bus); // TODO: do we have to check if level actually changed?
    if (bus_level == bus->last_level) {
        return;
    }
    bus->last_level = bus_level;

    bus->bus_idle_count = 0;           // reset idle counter so timer knows that something's happening

    if (bus->state <= TX_STOP) {          // check if we are transmitting
        if (bus_level != bus->tx_bus_level) { // check for collision
            bus->tx_collision = 1;             // signal collision
            bus->state = IDLE;                 // stop transmission
        }
        return;                        // no collision, ignore pin change
    }

    // logical bus level changed -> store timings
    int64_t tmp_ts = esp_timer_get_time() / 1000;
    bus->isr_us = esp_timer_get_time();
    uint32_t delta = (uint32_t)(tmp_ts - bus->rx_last_change); // store delta since last change
    bus->last_delta = delta;
    bus->rx_last_change = (uint32_t)tmp_ts;                       // store timestamp

    // rx state machine
    switch (bus->state) {
    case WAIT_RX:
        if (bus_level == LOW) { // start of rx frame
            bus->state = RX_START;
        } else {
            bus->state = IDLE;    // bus can't actually be high, reset
        }
        // TODO: log error?
        break;
    case RX_START:
        if (bus_level == HIGH && dali_bus_is_delta_within_te(delta)) { // validate start bit
            bus->rx_length = 0; // clear old rx message
            bus->rx_message = 0;
            bus->state = RX_BIT;
        } else {
            if (dali_bus_is_delta_within_te(delta)) {
                bus->rx_error = 3;
            } else {
                bus->rx_error = 1;
            }
            bus->rx_length = DALI_RX_ERROR;
            bus->state = RX_STOP;
        }
        break;
    case RX_BIT:
        if (dali_bus_is_delta_within_te(delta)) {              // check if change is within time of a half-bit
            if (bus->rx_length % 2) {                      // if rxLength is odd (= actual bit change)
                bus->rx_message = bus->rx_message << 1 | bus_level;    // shift in received bit
            }
            bus->rx_length++;
        } else if (dali_bus_is_delta_within_2te(delta)) {       // check if change is within time of two half-bits
            bus->rx_message = bus->rx_message << 1 | bus_level;   // shift in received bit
            bus->rx_length += 2;
        } else {
            bus->rx_error = 2;
            bus->rx_length = DALI_RX_ERROR;
            bus->state = RX_STOP; // timing error -> reset state
        }
        if (bus->rx_length == 16) { // check if all 8 bits have been received
            bus->state = RX_STOP;
        }
        break;
    case SHORT:
        if (bus_level == HIGH) {
            bus->state = IDLE;    // recover from bus error
        }
        break;
    default:
        break;  // ignore
    }
    bus->isr_us = esp_timer_get_time() - bus->isr_us;
}

bool dali_bus_is_delta_within_te(unsigned long delta)
{
    return (DALI_TE_MIN <= delta && delta <= DALI_TE_MAX);
}

bool dali_bus_is_delta_within_2te(unsigned long delta)
{
    return (2 * DALI_TE_MIN <= delta && delta <= 2 * DALI_TE_MAX);
}

uint32_t dali_bus_get_bus_level(dali_bus *bus)
{
    return (bus->active_low ? !gpio_get_level(bus->rx_pin) : gpio_get_level(bus->rx_pin));
}

void dali_bus_set_bus_level(dali_bus *bus, uint32_t level)
{
    gpio_set_level(bus->tx_pin, (bus->active_low ? !level : level));
    bus->tx_bus_level = level;
}
