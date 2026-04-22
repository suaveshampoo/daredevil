#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "pico/stdlib.h"
#include "lidar_scan.h"
#include "spin.h"

// One lidar revolution per platform position keeps each pass fast enough to
// refine over multiple sweeps instead of waiting for one giant capture.
#define PACKETS_PER_SWEEP 90
#define MAX_AZIMUTH_DEG 180.0f
#define MAX_AZIMUTH_STEPS ((int)(RING_GEAR_REVOLUTION / 2.0f))
#define STEP_DELAY_US 3700
#define STEP_SETTLE_MS 10
#define SWEEP_TIMEOUT_MS 4000
#define BYTE_TIMEOUT_MS 20

static int current_steps = 0;

static void wait_for_usb_serial(void) {
    for (int i = 0; i < 50; i++) {
        if (stdio_usb_connected()) {
            break;
        }
        sleep_ms(100);
    }
}

static bool collect_sweep(void) {
    uint8_t packet[PACKET_LEN];
    int packets_collected = 0;
    absolute_time_t deadline = make_timeout_time_ms(SWEEP_TIMEOUT_MS);

    while (packets_collected < PACKETS_PER_SWEEP && !time_reached(deadline)) {
        if (!read_lidar_packet(packet, BYTE_TIMEOUT_MS)) {
            continue;
        }

        ScanPoint scan[4];
        float rpm = 0.0f;
        extract_data(packet, scan, &rpm);

        for (int i = 0; i < 4; i++) {
            printf("P,%d,%d,%u,%u\n",
                   current_steps,
                   scan[i].angle,
                   (uint16_t)scan[i].distance,
                   scan[i].quality);
        }

        packets_collected++;
    }

    return packets_collected > 0;
}

static void emit_headers(void) {
    printf("MODE,continuous\n");
    printf("META,%.2f,%d\n",
           MAX_AZIMUTH_DEG,
           MAX_AZIMUTH_STEPS);
    printf("CFG,%d,%d,%d,%d,%d\n",
           PACKETS_PER_SWEEP,
           STEP_DELAY_US,
           STEP_SETTLE_MS,
           SWEEP_TIMEOUT_MS,
           BYTE_TIMEOUT_MS);
}

static void run_pass(int pass_index, bool forward) {
    const char *direction_name = forward ? "forward" : "reverse";
    const int end_step = forward ? MAX_AZIMUTH_STEPS : 0;

    printf("FRAME_START,%d,%s,%d,%d\n",
           pass_index,
           direction_name,
           current_steps,
           end_step);

    while (true) {
        float azimuth_deg = spin_get_degree(current_steps);
        if (!collect_sweep()) {
            printf("Sweep timeout at azimuth %.2f\n", azimuth_deg);
        }

        if (current_steps == end_step) {
            break;
        }

        spin_step_once(forward, STEP_DELAY_US, &current_steps);
        sleep_ms(STEP_SETTLE_MS);
    }

    printf("FRAME_DONE,%d,%s,%d\n",
           pass_index,
           direction_name,
           current_steps);
}

int main(void) {
    stdio_init_all();
    wait_for_usb_serial();

    spin_init();
    lidar_uart_init();

    emit_headers();

    int pass_index = 0;
    while (true) {
        run_pass(pass_index++, true);
        run_pass(pass_index++, false);
    }

    return 0;
}
