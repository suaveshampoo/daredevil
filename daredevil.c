#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"

#define START_BYTE 0xFA
#define PACKET_LEN 22

#define LDS_UART uart0
#define LDS_BAUD 115200
#define LDS_TX_PIN 0
#define LDS_RX_PIN 1

typedef struct {
    int angle;
    int distance;
    int quality;
} ScanPoint;

bool checksum_verify(uint8_t packet[PACKET_LEN]) {
    uint16_t checksum = packet[20] | (packet[21] << 8);

    uint32_t chksum_packet = 0;
    for (int i = 0; i < 10; i++) {
        uint16_t data = packet[i * 2] | (packet[i * 2 + 1] << 8);
        chksum_packet = (chksum_packet << 1) + data;
    }

    chksum_packet = (chksum_packet & 0x7FFF) + (chksum_packet >> 15);
    chksum_packet &= 0x7FFF;

    return checksum == chksum_packet;
}

void extract_data(uint8_t packet[PACKET_LEN], ScanPoint scan[4], float *rpm) {
    *rpm = (packet[2] | (packet[3] << 8)) / 64.0f;

    for (int i = 0; i < 4; i++) {
        int base_angle = (packet[1] - 0xA0) * 4;
        int angle = base_angle + i;
        int raw_dist = packet[4 + i * 4] | (packet[5 + i * 4] << 8);
        int distance = raw_dist & 0x3FFF;
        int quality = packet[6 + i * 4] | (packet[7 + i * 4] << 8);

        scan[i].angle = angle;
        scan[i].distance = distance;
        scan[i].quality = quality;
    }
}

int main(void) {
    stdio_init_all();
    sleep_ms(2000);

    uart_init(LDS_UART, LDS_BAUD);
    gpio_set_function(LDS_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(LDS_RX_PIN, GPIO_FUNC_UART);

    uint8_t packet[PACKET_LEN];
    int packet_pos = 0;

    while (1) {
        if (!uart_is_readable(LDS_UART)) {
            tight_loop_contents();
            continue;
        }

        uint8_t b = uart_getc(LDS_UART);

        if (packet_pos == 0) {
            if (b != START_BYTE) {
                continue;
            }
            packet[packet_pos++] = b;
            continue;
        }

        packet[packet_pos++] = b;

        if (packet_pos < PACKET_LEN) {
            continue;
        }

        packet_pos = 0;

        uint8_t idx = packet[1];
        if (!(0xA0 <= idx && idx <= 0xF9)) {
            continue;
        }

        if (!checksum_verify(packet)) {
            continue;
        }

        ScanPoint scan[4];
        float rpm;
        extract_data(packet, scan, &rpm);

        printf("RPM: %.2f\n", rpm);
        for (int i = 0; i < 4; i++) {
            printf("Angle: %d, Distance: %d, Quality: %d\n",
                   scan[i].angle,
                   scan[i].distance,
                   scan[i].quality);
        }
    }

    return 0;
}