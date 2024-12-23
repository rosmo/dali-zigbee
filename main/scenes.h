#ifndef _SCENES_H
#define _SCENES_H

#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <math.h>

#ifdef ESP_IDF_VERSION
#include "esp_random.h"
#endif

#define TOTAL_LIGHTS   5
#define MAX_BRIGHTNESS 254
#define MIN_BRIGHTNESS 0
#define TOTAL_EFFECTS  4

typedef struct _light_effect light_effect;
typedef void (*render_effect)(light_effect *effect, uint32_t frame);

typedef struct _light_state {
    bool    on;
    uint8_t level;
} light_state;

typedef struct _light_effect {
    uint8_t       fps;
    uint8_t       total_lights;
    uint8_t       max_brightness;
    uint8_t       min_brightness;
    uint32_t      speed;
    render_effect render;
    light_state   light_state[TOTAL_LIGHTS];
    uint32_t      effect_config;
    void          *user_data;
} light_effect;

void fade_in_out(light_effect *effect, uint32_t frame);
void fade_in_sync(light_effect *effect, uint32_t frame);
void light_rail(light_effect *effect, uint32_t frame);
void twinkle_stars(light_effect *effect, uint32_t frame);

extern light_effect light_effects[TOTAL_EFFECTS];

#ifdef ESP_IDF_VERSION
#define RANDOM_UINT8() (uint8_t)(esp_random()>>24)
#else
#define RANDOM_UINT8() (uint8_t)(rand() % 256)
#endif

#endif