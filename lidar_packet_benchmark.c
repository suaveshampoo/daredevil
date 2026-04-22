#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "pico/stdlib.h"
#include "lidar_scan.h"

#define BYTE_TIMEOUT_MS 20
#define REPORT_INTERVAL_MS 5000

static void wait_for_usb_serial(void) {
    for (int i = 0; i < 50; i++) {
        if (stdio_usb_connected()) {
            break;
        }
        sleep_ms(100);
    }
}

static const char *stability_label(float rpm_min, float rpm_max) {
    float span = rpm_max - rpm_min;
    if (span <= 1.5f) {
        return "very stable";
    }
    if (span <= 4.0f) {
        return "stable";
    }
    if (span <= 8.0f) {
        return "shaky";
    }
    return "unstable";
}

int main(void) {
    stdio_init_all();
    wait_for_usb_serial();
    lidar_uart_init();

    printf("Lidar packet benchmark started\n");
    printf("Adjust voltage and watch packet rate plus RPM stability.\n");
    printf("Higher packets/sec is better. Smaller RPM spread is better.\n");
    printf("\n");

    uint8_t packet[PACKET_LEN];
    absolute_time_t window_start = get_absolute_time();
    uint32_t packets_in_window = 0;
    float rpm_sum = 0.0f;
    float rpm_min = 0.0f;
    float rpm_max = 0.0f;
    float best_packets_per_sec = 0.0f;

    while (true) {
        if (read_lidar_packet(packet, BYTE_TIMEOUT_MS)) {
            ScanPoint scan[4];
            float rpm = 0.0f;
            extract_data(packet, scan, &rpm);

            if (packets_in_window == 0) {
                rpm_min = rpm;
                rpm_max = rpm;
            } else {
                if (rpm < rpm_min) {
                    rpm_min = rpm;
                }
                if (rpm > rpm_max) {
                    rpm_max = rpm;
                }
            }

            rpm_sum += rpm;
            packets_in_window++;
        }

        int64_t elapsed_us = absolute_time_diff_us(window_start, get_absolute_time());
        if (elapsed_us < (int64_t)REPORT_INTERVAL_MS * 1000) {
            continue;
        }

        float elapsed_s = (float)elapsed_us / 1000000.0f;
        float packets_per_sec = packets_in_window / elapsed_s;
        float points_per_sec = (packets_in_window * 4.0f) / elapsed_s;
        float avg_rpm = packets_in_window > 0 ? (rpm_sum / packets_in_window) : 0.0f;

        if (packets_per_sec > best_packets_per_sec) {
            best_packets_per_sec = packets_per_sec;
        }

        printf("Window: %.2f s\n", elapsed_s);
        printf("Packets: %lu total | %.2f packets/sec\n",
               (unsigned long)packets_in_window,
               packets_per_sec);
        printf("Points:  %lu total | %.2f points/sec\n",
               (unsigned long)(packets_in_window * 4),
               points_per_sec);
        printf("RPM:     avg %.2f | min %.2f | max %.2f | spread %.2f (%s)\n",
               avg_rpm,
               rpm_min,
               rpm_max,
               rpm_max - rpm_min,
               stability_label(rpm_min, rpm_max));
        printf("Best packet rate so far: %.2f packets/sec\n", best_packets_per_sec);
        printf("RAW,%.2f,%lu,%lu,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
               elapsed_s,
               (unsigned long)packets_in_window,
               (unsigned long)(packets_in_window * 4),
               packets_per_sec,
               points_per_sec,
               avg_rpm,
               rpm_min,
               rpm_max,
               best_packets_per_sec);
        printf("\n");

        window_start = get_absolute_time();
        packets_in_window = 0;
        rpm_sum = 0.0f;
        rpm_min = 0.0f;
        rpm_max = 0.0f;
    }

    return 0;
}
