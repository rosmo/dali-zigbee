#include <stdint.h>
#include <time.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "scenes.h"

static light_state last_light_state[TOTAL_LIGHTS];

uint64_t esp_timer_get_time()
{
    struct timespec ts;
    uint64_t bigtime;

    if (clock_gettime(CLOCK_REALTIME, &ts) != 0) {
        abort();
    }

    return ts.tv_sec * UINT64_C(1000000000) + ts.tv_nsec;
}

uint32_t esp_random()
{

}

// Main Dali task
void main()
{
    uint8_t counter;
    int     last_effect = -1, effect = 4;
    uint32_t fps_sleep = 0, frame = 0;
    uint64_t frame_start_time = 0, frame_end_time = 0;
    uint64_t effect_start_time = 0;
    int64_t  frame_diff = 0;

    light_effect *light_effect = NULL;
    while (true) {
        frame_start_time = esp_timer_get_time() / 1000;

        if (last_effect != effect) {
            last_effect = effect;
            light_effect = &light_effects[effect];
            fps_sleep = 1000 / light_effect->fps;
            effect_start_time = esp_timer_get_time() / 1000000;
            printf("Effect %d active, FPS sleep: %u\n", effect, fps_sleep);
        }
        light_effect->render((void *)light_effect, frame);
        frame++;

        if (frame % 15 == 0) {
            char light_status[256] = { '\0' };
            snprintf(light_status, sizeof(light_status), "Frame %-8u ", frame);
            for (int i = 0; i < light_effect->total_lights; i++) {
                char buf[32] = { '\0' };
                snprintf(buf, sizeof(buf), "| %-6d", light_effect->light_state[i].level);
                strcat(light_status, buf);
            }
            strcat(light_status, "|");
            printf("%s\n", light_status);
        }

        frame_end_time = esp_timer_get_time() / 1000;

        frame_diff = frame_end_time - frame_start_time;
        if (frame_diff < fps_sleep) {
            usleep(fps_sleep - frame_diff);
        }

        // Store last state
        memcpy(last_light_state, light_effect->light_state, sizeof(light_state) * light_effect->total_lights);
    }
}

