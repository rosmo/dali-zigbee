# DALI2 Zigbee example

Zigbee code for ESP32-C6 to control 5 DALI dimmers with effects. Has a multistate cluster to
switch between different effects. Also has a LED emulator for testing new effects.

DALI control implement using a C++ to C port of the Arduino DALI library: [github.com/hubsif/arduino-dali](https://github.com/hubsif/arduino-dali).

Needs a big flash (8 MB).

## Troubleshooting

`Stack BDB Device Start failure with ESP_FAIL status, steering`: try erasing the flash with
`idf.py erase-flash`.


