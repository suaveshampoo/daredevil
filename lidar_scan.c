#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"

#define LDS_UART uart0
#define LDS_BAUD 115200
#define LDS_TX_PIN 16
#define LDS_RX_PIN 17

int main(void){
    stdio_init_all();

    for (int i = 0; i < 50; i++) {
        if (stdio_usb_connected()) {
            break;
        }
        sleep_ms(100);
    }

    uart_init(LDS_UART, LDS_BAUD);

    // Pico TX -> LiDAR RX, Pico RX -> LiDAR TX.
    gpio_set_function(LDS_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(LDS_RX_PIN, GPIO_FUNC_UART);

    uart_set_format(LDS_UART, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(LDS_UART, true);
    uart_set_hw_flow(LDS_UART, false, false);

    while (true){
        while (uart_is_readable(LDS_UART)){
            uint8_t byte = uart_getc(LDS_UART);
            putchar_raw(byte);
        }

        tight_loop_contents();
    }

    return 0;
}
