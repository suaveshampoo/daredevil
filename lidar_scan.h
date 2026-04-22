#ifndef LIDAR_SCAN_H
#define LIDAR_SCAN_H

#include <stdbool.h>
#include <stdint.h>
#include "hardware/uart.h"

#define START_BYTE 0xFA
#define PACKET_LEN 22

#define LDS_UART uart0
#define LDS_BAUD 115200
#define LDS_TX_PIN 16
#define LDS_RX_PIN 17

typedef struct {
    int angle;
    int distance;
    int quality;
} ScanPoint;

void lidar_uart_init(void);
bool checksum_verify(const uint8_t packet[PACKET_LEN]);
bool read_lidar_packet(uint8_t packet[PACKET_LEN], uint32_t byte_timeout_ms);
void extract_data(const uint8_t packet[PACKET_LEN], ScanPoint scan[4], float *rpm);

#endif
