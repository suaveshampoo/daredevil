#include <stdio.h>

#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "lidar_scan.h"


void lidar_uart_init(void) {
    uart_init(LDS_UART, LDS_BAUD);
    gpio_set_function(LDS_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(LDS_RX_PIN, GPIO_FUNC_UART);
    uart_set_format(LDS_UART, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(LDS_UART, true);
    uart_set_hw_flow(LDS_UART, false, false);
}


bool checksum_verify(const uint8_t packet[PACKET_LEN]) {
    uint16_t checksum = packet[20] | (packet[21] << 8);
    uint32_t chksum_packet = 0;

    for (int i = 0; i < 10; i++) {
        uint16_t data = packet[i * 2] | (packet[i * 2 + 1] << 8);
        chksum_packet = (chksum_packet << 1) + data;
    }

    chksum_packet = (chksum_packet & 0x7FFF) + (chksum_packet >> 15);
    chksum_packet &= 0x7FFF;

    return checksum == (uint16_t)chksum_packet;
}


bool read_lidar_packet(uint8_t packet[PACKET_LEN], uint32_t byte_timeout_ms) {
    while (uart_is_readable(LDS_UART)) {
        if (uart_getc(LDS_UART) != START_BYTE) {
            continue;
        }

        packet[0] = START_BYTE;

        for (int i = 1; i < PACKET_LEN; i++) {
            absolute_time_t deadline = make_timeout_time_ms(byte_timeout_ms);
            while (!uart_is_readable(LDS_UART) && !time_reached(deadline)) {
                tight_loop_contents();
            }

            if (!uart_is_readable(LDS_UART)) {
                return false;
            }

            packet[i] = uart_getc(LDS_UART);
        }

        if (packet[1] < 0xA0 || packet[1] > 0xF9) {
            return false;
        }

        if (!checksum_verify(packet)) {
            return false;
        }

        return true;
    }

    return false;
}


void extract_data(const uint8_t packet[PACKET_LEN], ScanPoint scan[4], float *rpm) {
    int base_angle = (packet[1] - 0xA0) * 4;
    *rpm = (packet[2] | (packet[3] << 8)) / 64.0f;

    for (int i = 0; i < 4; i++) {
        int raw_dist = packet[4 + i * 4] | (packet[5 + i * 4] << 8);

        scan[i].angle = base_angle + i;
        scan[i].distance = raw_dist & 0x3FFF;
        scan[i].quality = packet[6 + i * 4] | (packet[7 + i * 4] << 8);
    }
}


#ifndef LIDAR_SCAN_NO_MAIN
int main(void) {
    stdio_init_all();

    for (int i = 0; i < 50; i++) {
        if (stdio_usb_connected()) {
            break;
        }
        sleep_ms(100);
    }

    lidar_uart_init();

    while (true) {
        while (uart_is_readable(LDS_UART)) {
            uint8_t byte = uart_getc(LDS_UART);
            putchar_raw(byte);
        }

        tight_loop_contents();
    }

    return 0;
}
#endif
