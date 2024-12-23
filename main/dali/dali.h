/***********************************************************************
 * This library is free software; you can redistribute it and/or       *
 * modify it under the terms of the GNU Lesser General Public          *
 * License as published by the Free Software Foundation; either        *
 * version 2.1 of the License, or (at your option) any later version.  *
 *                                                                     *
 * This library is distributed in the hope that it will be useful,     *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of      *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU   *
 * Lesser General Public License for more details.                     *
 *                                                                     *
 * You should have received a copy of the GNU Lesser General Public    *
 * License along with this library; if not, write to the Free Software *
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,          *
 * MA 02110-1301  USA                                                  *
 ***********************************************************************/

/**
 * @mainpage
 * This library allows you to create a DALI controller/gateway with an Arduino device.
 * It supports sending commands, receiving responses and commissioning devices.
 * It requires the TimerOne library for transmission and the PinChangeInterrupt library
 * for reception. These libraries need to be installed with the Library Manager in the
 * Arduino IDE.
 *
 * @par Simple example
 * @include dali_blink.ino
 *
 * @par Changelog
 * - v0.0.3 (2022-09-19) Make commissioning state public
 * - v0.0.2 (2019-05-14) Initial release
 *
 * @author hubsif <hubsif@gmx.de>
 * @example dali_blink.ino
 *
 * @file Dali.h
 * @brief Main header file of the DALI library for Arduino
 */


#ifndef DALI_H
#define DALI_H

#include "dali_bus.h"

/** DALI commands */
typedef enum _dali_cmd_t {
    DALI_CMD_OFF = 0, DALI_CMD_UP = 1, DALI_CMD_DOWN = 2, DALI_CMD_STEP_UP = 3, DALI_CMD_STEP_DOWN = 4,
    DALI_CMD_RECALL_MAX = 5, DALI_CMD_RECALL_MIN = 6, DALI_CMD_STEP_DOWN_AND_OFF = 7, DALI_CMD_ON_AND_STEP_UP = 8,
    DALI_CMD_GO_TO_LAST = 10, // DALI-2
    DALI_CMD_GO_TO_SCENE = 16,
    DALI_CMD_RESET = 32, DALI_CMD_ARC_TO_DTR = 33,
    DALI_CMD_SAVE_VARS = 34, DALI_CMD_SET_OPMODE = 35, DALI_CMD_RESET_MEM = 36, DALI_CMD_IDENTIFY = 37, // DALI-2
    DALI_CMD_DTR_AS_MAX = 42, DALI_CMD_DTR_AS_MIN = 43, DALI_CMD_DTR_AS_FAIL = 44, DALI_CMD_DTR_AS_POWER_ON = 45, DALI_CMD_DTR_AS_FADE_TIME = 46, DALI_CMD_DTR_AS_FADE_RATE = 47,
    DALI_CMD_DTR_AS_EXT_FADE_TIME = 48, // DALI-2
    DALI_CMD_DTR_AS_SCENE = 64, DALI_CMD_REMOVE_FROM_SCENE = 80,
    DALI_CMD_ADD_TO_GROUP = 96, DALI_CMD_REMOVE_FROM_GROUP = 112,
    DALI_CMD_DTR_AS_SHORT = 128,
    DALI_CMD_QUERY_STATUS = 144, DALI_CMD_QUERY_BALLAST = 145, DALI_CMD_QUERY_LAMP_FAILURE = 146, DALI_CMD_QUERY_LAMP_POWER_ON = 147, DALI_CMD_QUERY_LIMIT_ERROR = 148,
    DALI_CMD_QUERY_RESET_STATE = 149, DALI_CMD_QUERY_MISSING_SHORT = 150, DALI_CMD_QUERY_VERSION = 151, DALI_CMD_QUERY_DTR = 152, DALI_CMD_QUERY_DEVICE_TYPE = 153,
    DALI_CMD_QUERY_PHYS_MIN = 154, DALI_CMD_QUERY_POWER_FAILURE = 155,
    DALI_CMD_QUERY_OPMODE = 158, DALI_CMD_QUERY_LIGHTTYPE = 159, // DALI-2
    DALI_CMD_QUERY_ACTUAL_LEVEL = 160, DALI_CMD_QUERY_MAX_LEVEL = 161, DALI_CMD_QUERY_MIN_LEVEL = 162, DALI_CMD_QUERY_POWER_ON_LEVEL = 163, DALI_CMD_QUERY_FAIL_LEVEL = 164, DALI_CMD_QUERY_FADE_SPEEDS = 165,
    DALI_CMD_QUERY_SPECMODE = 166, DALI_CMD_QUERY_NEXT_DEVTYPE = 167, DALI_CMD_QUERY_EXT_FADE_TIME = 168, DALI_CMD_QUERY_CTRL_GEAR_FAIL = 169, // DALI-2
    DALI_CMD_QUERY_SCENE_LEVEL = 176,
    DALI_CMD_QUERY_GROUPS_0_7 = 192, DALI_CMD_QUERY_GROUPS_8_15 = 193,
    DALI_CMD_QUERY_ADDRH = 194, DALI_CMD_QUERY_ADDRM = 195, DALI_CMD_QUERY_ADDRL = 196
} dali_cmd_t;

typedef enum _dali_special_cmd_t {
    DALI_CMD_TERMINATE = 256, DALI_CMD_SET_DTR = 257,
    DALI_CMD_INITIALISE = 258, DALI_CMD_RANDOMISE = 259, DALI_CMD_COMPARE = 260, DALI_CMD_WITHDRAW = 261,
    DALI_CMD_SEARCHADDRH = 264, DALI_CMD_SEARCHADDRM = 265, DALI_CMD_SEARCHADDRL = 266,
    DALI_CMD_PROGRAMSHORT = 267, DALI_CMD_VERIFYSHORT = 268, DALI_CMD_QUERY_SHORT = 269, DALI_CMD_PHYS_SEL = 270,
    DALI_CMD_ENABLE_DT = 272, DALI_CMD_LOAD_DTR1 = 273, DALI_CMD_LOAD_DTR2 = 274, DALI_CMD_WRITE_MEM_LOC = 275,
    DALI_CMD_WRITE_MEM_LOC_NOREPLY = 276 // DALI-2
} dali_special_cmd_t;

typedef enum _dali_dev_type_t {
    DALI_FLUORESCENT_LAMP,
    DALI_EMERGENCY_LIGHT,
    DALI_DISCHARGE_LAMP,
    DALI_HALOGEN_LAMP,
    DALI_INCANDESCENT_LAMP,
    DALI_DC_CONVERTER,
    DALI_LED_MODULE,
    DALI_SWITCH,
    DALI_COLOUR_CTRL,
    DALI_SEQUENCER,
    DALI_OPTICAL_CTRL
} dali_dev_type_t;

/** Type of address (short/group) */
typedef enum _dali_address_type_t {
    DALI_SHORT_ADDRESS = 0,
    DALI_GROUP_ADDRESS = 1
} dali_address_type_t;

typedef enum _dali_commission_state_t {
    DALI_COMMISSION_OFF, DALI_COMMISSION_INIT, DALI_COMMISSION_INIT2, DALI_COMMISSION_WRITE_DTR, DALI_COMMISSION_REMOVE_SHORT, DALI_COMMISSION_REMOVE_SHORT2, DALI_COMMISSION_RANDOM, DALI_COMMISSION_RANDOM2, DALI_COMMISSION_RANDOMWAIT,
    DALI_COMMISSION_STARTSEARCH, DALI_COMMISSION_SEARCHHIGH, DALI_COMMISSION_SEARCHMID, DALI_COMMISSION_SEARCHLOW,
    DALI_COMMISSION_COMPARE, DALI_COMMISSION_CHECKFOUND, DALI_COMMISSION_PROGRAMSHORT,
    DALI_COMMISSION_VERIFYSHORT, DALI_COMMISSION_VERIFYSHORTRESPONSE, DALI_COMMISSION_QUERYDEVICETYPE, DALI_COMMISSION_QUERYDEVICETYPERESPONSE,
    DALI_COMMISSION_WITHDRAW, DALI_COMMISSION_TERMINATE
} dali_commission_state_t;

typedef struct _dali {
    dali_bus *bus;
    /** next address to program on commissioning. When commissioning finished, reflects number of ballasts found. */
    uint8_t next_short_address;

    /** When true, only ballasts without short address set are commissioned. */
    bool commission_only_new;

    /** commissioning state machine states */
    dali_commission_state_t commission_state; /**< current state of commissioning state machine */
} dali;

/** Start the DALI bus
 * @param tx_pin       Pin to use for transmission
 * @param rx_pin       Pin to use for reception. Must support Pin Change Interrupt.
 * @param active_low  set to false if bus is active low
 *
 * Initialize the hardware for DALI usage (i.e. set pin modes, timer and interrupts). By default the bus is
 * driven active-low, meaning with the µC tx pin being low the DALI bus will be high (idle). For transmission
 * the µC pin will be set high, which will pull the DALI voltage low. This behaviour
 * is used by most DALI hardware interfaces. The same logic applies to the rx pin. */

dali *dali_begin(uint8_t tx_pin, uint8_t rx_pin, bool active_low/* = true*/);

/** Send a direct arc level command
 * @param  address    destination address
 * @param  value      arc level
 * @param  addr_type  address type (short/group)
 * @return ::daliReturnValue
 *
 * This methods sends a "direct arc power control command" to the bus
 * It doesn't check if the bus is ready and returns immediately, stating if transmission could be
 * initiated through its response value ::daliReturnValue. */
dali_ret_t dali_send_arc(dali *dali_, uint8_t address, uint8_t value, dali_address_type_t addr_type);

/** Send a direct arc level command and wait for its completion
 * @param  address    destination address
 * @param  value      arc level
 * @param  addr_type  address type (short/group)
 * @return ::daliReturnValue
 *
 * This methods sends a "direct arc power control command" to the bus
 * It uses sendRawWait(), so it waits for the bus to become idle before and after transmission. */
dali_ret_t dali_send_arc_wait(dali *dali_, uint8_t address, uint8_t value, dali_address_type_t addr_type/* = DALI_SHORT_ADDRESS*/, uint8_t timeout/* = 50*/);

/** Send a DALI command
 * @param  address    destination address
 * @param  command    DALI command
 * @param  addr_type  address type (short/group)
 * @return ::daliReturnValue
 * This method sends a DALI Command to the bus (Commands from 0 to 255).
 * It doesn't check if the bus is ready and returns immediately, stating if transmission could be
 * initiated through its response value ::daliReturnValue.
 * Note that some of the special commands need to be sent twice (258 - INITIALISE, 259 - RANDOMISE), which
 * this method doesn't do by itself. */
dali_ret_t dali_send_cmd(dali *dali_, uint8_t address, dali_cmd_t command, dali_address_type_t addr_type/*= DALI_SHORT_ADDRESS*/);

/** Send a DALI command, wait for its completion and return the response if available
 * @param  address    destination address
 * @param  command    DALI command
 * @param  addr_type  address type (short/group)
 * @param  timeout    time in ms to wait for action to complete
 * @return returns either the response, DALI_RX_EMPTY or any of ::daliReturnValue on error  */
int dali_send_cmd_wait(dali *dali_, uint8_t address, dali_cmd_t command, dali_address_type_t addr_type/* = DALI_SHORT_ADDRESS*/, uint8_t timeout/* = 50*/);

/** Send a DALI special command
 * @param  command  DALI special command
 * @param  value    Value (2nd byte)
 * @return ::daliReturnValue
 *
 * This method sends a DALI Special Command to the bus (Special Commands are commands from 256 to 287).
 * It doesn't check if the bus is ready and returns immediately, stating if transmission could be
 * initiated through its response value ::daliReturnValue.
 * Note that some of the special commands need to be sent twice (258 - INITIALISE, 259 - RANDOMISE), which
 * this method doesn't do by itself. */
dali_ret_t dali_send_special_cmd(dali *dali_, dali_special_cmd_t command, uint8_t value/* = 0*/);

/** Send a DALI special command, wait for its completion and return the response if available
 * @param  command  DALI special command
 * @param  value    Value (2nd byte)
 * @param  timeout  time in ms to wait for action to complete
 * @return returns either the response, DALI_RX_EMPTY or any of ::daliReturnValue on error
 *
 * This method sends a DALI Special Command to the bus (Special Commands are commands from 256 to 287).
 * It uses sendRawWait(), so it waits for the bus to become idle before and after transmission.
 * It returns either the received response, DALI_RX_EMPTY if no response has been received or any of
 * ::daliReturnValue if an error has occurred.
 * Note that some of the special commands need to be sent twice (258 - INITIALISE, 259 - RANDOMISE), which
 * this method doesn't do by itself. */
int dali_send_special_cmd_wait(dali *dali_, dali_special_cmd_t command, uint8_t value/* = 0*/, uint8_t timeout/*= 50*/);

/** Send raw values to the DALI bus
 * @param message  byte array to send
 * @param length   length of the byte array
 * @param timeout  time in ms to wait for action to complete
 *
 * This method sends a raw byte array of @p length to the bus. The array can be three bytes max.
 * It waits for the bus to become idle before and after transmission. It returns either the received response,
 * DALI_RX_EMPTY if no response has been received or any of ::daliReturnValue if an error has occurred. */
int dali_send_raw(dali *dali_, const uint8_t *message, uint8_t length);
int dali_send_raw_wait(dali *dali_, const uint8_t *message, uint8_t length, uint8_t timeout/* = 50*/);

/** Initiate commissioning of all DALI ballasts
 * @param startAddress  address starting short address assignment from
 * @param onlyNew       commission only ballasts without short address
 *
 * This method starts the DALI commissioning process. During commissioning the method commission_tick()
 * needs to be called repeatedly until commissioning has finished. By default commissioning is done for
 * all ballasts on the bus (@p onlyNew = false). With this, at first current short addresses from
 * all ballasts are removed. Then all found ballasts are assigned a new short address, starting
 * from @p startAddress. Commissioning has finished when @p commissionState is set back to DALI_COMMISSION_OFF.
 * The number of ballasts found can be determined from #nextShortAddress.
 * With @p onlyNew = true ballasts with a short address assigned are ignored. The caller is responsible
 * for setting an appropriate value to @p startAddress. */
void dali_commission(dali *dali_, uint8_t start_address/* = 0*/, bool only_new/* = false*/);

/** State machine ticker for commissioning. See commission(). */
void dali_commission_tick(dali *dali_);

uint8_t *dali_prepare_cmd(uint8_t *message, uint8_t address, uint8_t command, uint8_t type, uint8_t selector);

/** Prepares a byte array for sending DALI Special Commands */
uint8_t *dali_prepare_special_cmd(uint8_t *message, uint16_t command, uint8_t value);

#endif // DALI_H