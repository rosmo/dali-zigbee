#include "scenes.h"
#include <stddef.h>
#include <stdlib.h>
#include <stdio.h>

#ifdef ESP_PLATFORM
#include "esp_log.h"
#define TAG "scenes"
#endif

light_effect light_effects[TOTAL_EFFECTS] = {
    {
        .fps = 20,
        .render = (void *)fade, // fade in
        .total_lights = TOTAL_LIGHTS,
        .max_brightness = MAX_BRIGHTNESS,
        .min_brightness = MIN_BRIGHTNESS,
        .effect_config = 1,
        .free_user_data = true,
        .user_data = NULL,
    },
    {
        .fps = 20,
        .render = (void *)fade, // fade out
        .total_lights = TOTAL_LIGHTS,
        .max_brightness = MAX_BRIGHTNESS,
        .min_brightness = MIN_BRIGHTNESS,
        .effect_config = 2,
        .free_user_data = true,
        .user_data = NULL,
    },
    {
        .fps = 5,
        .render = (void *)fade_in_out,
        .total_lights = TOTAL_LIGHTS,
        .max_brightness = MAX_BRIGHTNESS,
        .min_brightness = MIN_BRIGHTNESS,
        .free_user_data = true,
        .user_data = NULL,
    },
    {
        .fps = 5,
        .render = (void *)fade_in_sync,
        .total_lights = TOTAL_LIGHTS,
        .max_brightness = MAX_BRIGHTNESS,
        .min_brightness = MIN_BRIGHTNESS,
        .free_user_data = true,
        .user_data = NULL,
    },
    {
        .fps = 10,
        .render = (void *)light_rail,
        .total_lights = TOTAL_LIGHTS,
        .max_brightness = MAX_BRIGHTNESS,
        .min_brightness = MIN_BRIGHTNESS,
        .speed = 10000,
        .free_user_data = true,
        .user_data = NULL,
    },
    {
        .fps = 20,
        .render = (void *)twinkle_stars,
        .total_lights = TOTAL_LIGHTS,
        .max_brightness = MAX_BRIGHTNESS,
        .min_brightness = MIN_BRIGHTNESS,
        .speed = 4000,
        .free_user_data = true,
        .user_data = NULL,
    },
    {
        .fps = 20,
        .render = (void *)pulsing_swoosh,
        .total_lights = TOTAL_LIGHTS,
        .max_brightness = MAX_BRIGHTNESS,
        .min_brightness = MIN_BRIGHTNESS,
        .speed = 2000,
        .free_user_data = true,
        .user_data = NULL,
    },
};

void increase_clamp(light_effect *effect, uint8_t light, uint32_t increase) 
{
    if (increase == 0) increase = 1;
    if ((effect->light_state[light].level + increase) > effect->max_brightness) {
        effect->light_state[light].level = effect->max_brightness;
    } else {
        effect->light_state[light].level += increase;
    }
}

void decrease_clamp(light_effect *effect, uint8_t light, uint32_t decrease) 
{
    if (decrease == 0) decrease = 1;
    if ((effect->light_state[light].level + decrease) < effect->min_brightness) {
        effect->light_state[light].level = effect->min_brightness;
    } else {
        effect->light_state[light].level -= decrease;
    }
}

bool fade_off(light_effect *effect, uint32_t frame)
{
    bool all_minimum = true;
    for (int i = 0; i < effect->total_lights; i++) {
        if (effect->light_state[i].level > effect->min_brightness) {
            decrease_clamp(effect, i, effect->speed);
            all_minimum = false;
        }
    }
    return all_minimum;
}

void fade(light_effect *effect, uint32_t frame)
{
    uint8_t *dir = NULL;
    if (effect->user_data == NULL) {
        effect->user_data = (void *)calloc(effect->total_lights, sizeof(uint8_t));
        assert(effect->user_data != NULL);
        dir = (uint8_t *)effect->user_data;
        for (int i = 0; i < effect->total_lights; i++) {
            if (effect->effect_config > 0) {
                dir[i] = effect->effect_config;
            } else {
                dir[i] = 0;
            }
        }
        ESP_LOGI(TAG, "Direction set to %d, effect config: %lu", dir[0], effect->effect_config);
    } else {
        dir = (uint8_t *)effect->user_data;
    }

    for (int i = 0; i < effect->total_lights; i++) {
        switch (dir[i]) {
        case 1:
            increase_clamp(effect, i, effect->speed);
            if (effect->effect_config == 1 && effect->light_state[i].level >= effect->max_brightness)
            {
                dir[i] = 0;
            }
            break;
        case 2:
            decrease_clamp(effect, i, effect->speed);
            if (effect->effect_config == 2 && effect->light_state[i].level <= effect->min_brightness)
            {
                dir[i] = 0;
            }
            break;
        }
    }
}


void fade_in_out(light_effect *effect, uint32_t frame)
{
    uint8_t *dir = NULL;
    if (effect->user_data == NULL) {
        effect->user_data = (void *)calloc(effect->total_lights, sizeof(uint8_t));
        assert(effect->user_data != NULL);
        dir = (uint8_t *)effect->user_data;
        for (int i = 0; i < effect->total_lights; i++) {
            if (effect->effect_config > 0) {
                dir[i] = effect->effect_config;
            } else {
                dir[i] = 1;
            }
        }
    } else {
        dir = (uint8_t *)effect->user_data;
    }

    for (int i = 0; i < effect->total_lights; i++) {
        switch (dir[i]) {
        case 1:
            increase_clamp(effect, i, effect->speed);
            if (effect->light_state[i].level >= effect->max_brightness) {
                dir[i] = 2;
            }
            break;
        case 2:
            decrease_clamp(effect, i, effect->speed);
            if (effect->light_state[i].level <= effect->min_brightness) {
                dir[i] = 1;
            }
            break;
        }
    }
}

void fade_in_sync(light_effect *effect, uint32_t frame)
{
    uint8_t *dir = NULL;
    if (effect->user_data == NULL) {
        bool all_minimum = fade_off(effect, frame);
        if (all_minimum) {
            effect->user_data = (void *)calloc(1, sizeof(uint8_t));
            assert(effect->user_data != NULL);
            *(uint8_t *)effect->user_data = 1;
        }
    }
    if (effect->user_data != NULL) {
        dir = (uint8_t *)effect->user_data;
        bool change_dir = false;
        for (int i = 0; i < effect->total_lights; i++) {
            if (*dir == 1) {
                increase_clamp(effect, i, effect->speed);
                if (effect->light_state[i].level >= effect->max_brightness) {
                    change_dir = true;
                }
            } else {
                decrease_clamp(effect, i, effect->speed);
                if (effect->light_state[i].level <= effect->min_brightness) {
                    change_dir = true;
                }
            }
            if (change_dir) {
                *dir = (*dir == 1 ? 2 : 1);
            }
        }
    }
}

typedef struct  {
    int   dir;
    int   current;
    float level;
    float step;
} light_rail_data;

void light_rail(light_effect *effect, uint32_t frame)
{
    light_rail_data *d = NULL;
    if (effect->user_data == NULL) {
        bool all_minimum = fade_off(effect, frame);
        if (all_minimum) {
            effect->user_data = (void *)calloc(1, sizeof(light_rail_data));
            assert(effect->user_data != NULL);

            // Speed 10000ms / 200 ms per frame (5 fps) = increase every 50 ms
            // Max brightness - min brightness = 254
            // Brightness ramp 254 / 50

            d = (light_rail_data *)effect->user_data;
            d->dir = 1;
            d->level = 0;
            float step_every_ms = (float)(effect->speed / (1000 / effect->fps)); // == 50 ms
            d->step = (float)(effect->max_brightness - effect->min_brightness) / step_every_ms; // = 5.08 per step
        }
    } else {
        d = (light_rail_data *)effect->user_data;
    }

    if (effect->user_data != NULL) {
        if (d->dir == 1) {
            d->level += d->step;
        }
        for (int i = d->current; i >= 0; i--) {
            double level = 0.0;
            level = floor(d->level / (d->current - i + 1));
            effect->light_state[i].level = (uint8_t)level;
        }
        for (int i = d->current + 1; i < effect->total_lights; i++) {
            effect->light_state[i].level = 0;
        }
        if (d->level > (float)effect->max_brightness) {
            d->current += 1;
            d->level = 0;
            if (d->current == effect->total_lights) {
                d->current = 0;
            }
        }
    }
}

typedef struct  {
    uint8_t   target;
    uint8_t   level;
} twinkle_stars_data;

void twinkle_stars(light_effect *effect, uint32_t frame)
{
    twinkle_stars_data *d = NULL;
    if (effect->user_data == NULL) {
        bool all_minimum = fade_off(effect, frame);
        if (all_minimum) {
            effect->user_data = (void *)calloc(effect->total_lights, sizeof(twinkle_stars_data));
            assert(effect->user_data != NULL);

            d = (twinkle_stars_data *)effect->user_data;
            for (int i = 0; i < effect->total_lights; i++) {
                d[i].target = RANDOM_UINT8();
            }
        }
    } else {
        d = (twinkle_stars_data *)effect->user_data;
    }

    if (d != NULL) {
        for (int i = 0; i < effect->total_lights; i++) {
            if (d[i].level < d[i].target) {
                d[i].level++;
            } else if (d[i].level > d[i].target) {
                d[i].level--;
            } else {
                d[i].target = RANDOM_UINT8();
            }
            effect->light_state[i].level = d[i].level;
        }
    }
}

typedef struct  {
    uint8_t   light;
    float     level;
    uint8_t   direction;
    float     step;
    uint8_t   switch_after_pulses;
    uint8_t   pulses;
} pulsing_swoosh_data;

void pulsing_swoosh(light_effect *effect, uint32_t frame)
{
    pulsing_swoosh_data *d = NULL;
    if (effect->user_data == NULL) {
        effect->user_data = (void *)calloc(1, sizeof(pulsing_swoosh_data));
        assert(effect->user_data != NULL);

        d = (pulsing_swoosh_data *)effect->user_data;
        d->light = 2;
        d->level = 0.0;
        d->switch_after_pulses = 10;

        float step_every_ms = (float)(effect->speed / (1000 / effect->fps)); // == 50 ms
        d->step = (float)(effect->max_brightness - effect->min_brightness) / step_every_ms; // = 5.08 per step
    } else {
        d = (pulsing_swoosh_data *)effect->user_data;
    }

    if (d != NULL) {
        if (d->direction == 0) {
            d->level += d->step;
        } else {
            d->level -= d->step;
        }
        if (d->level < effect->min_brightness || d->level > effect->max_brightness) {
            d->direction = !d->direction;
            if (d->level < effect->min_brightness) {
                d->level = effect->min_brightness;
                d->pulses++;
                if (d->pulses > d->switch_after_pulses) {
                    d->pulses = 0;
                    d->light = RANDOM_UINT8() % effect->total_lights;
                }
            }
            if (d->level > effect->max_brightness) {
                d->level = effect->max_brightness;
            }
        }

        effect->light_state[d->light].level = (uint8_t)d->level;

        // Bulbs to the left...
        float lf = d->level / effect->total_lights;
        for (int i = d->light - 1; i >= 0; i--) {
            effect->light_state[i].level = (uint8_t)(lf * (i + 1));
        }
        // Bulbs to the right...
        for (int i = d->light + 1, o = effect->total_lights - d->light - 1; i < effect->total_lights; i++, o--) {
            effect->light_state[i].level = (uint8_t)(lf * o);
        }
    }
}