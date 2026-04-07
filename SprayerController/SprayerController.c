// Capstone I16
// Written by Finn Rush
// PROTOTYPE

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/pio.h"

// LED pin
#define LED 25

// UART pins
#define UART_ID uart0
#define BAUD_RATE 115200
#define UART_TX 12
#define UART_RX 13

// On pins
#define ON_HI 6
#define ON_PIN 7

// Motor control pins
#define MTR_1 21
#define MTR_2 20

// Pump control pins
#define PMP_1 19
#define PMP_2 18

// Limit switch pins
#define LIM_1 11
#define LIM_2 10
#define LIM_HI 9

int main() {
    stdio_init_all();

    // Set LED
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    gpio_put(LED, 1);

    // Init UART
    uart_init(UART_ID, BAUD_RATE);

    // Set the TX and RX pins
    gpio_set_function(UART_TX, GPIO_FUNC_UART);
    gpio_set_function(UART_RX, GPIO_FUNC_UART);

    // Configure format do i need this
    // uart_set_format(UART_ID, 8, 1, UART_PARITY_NONE);

    // Enable FIFO
    uart_set_fifo_enabled(UART_ID, true);

    // Send a string
    uart_puts(UART_ID, "UART ready\n");

    // Set the on switch pins
    gpio_init(ON_HI);
    gpio_set_dir(ON_HI, GPIO_OUT);
    gpio_pull_up(ON_HI);
    gpio_put(ON_HI, 1);

    gpio_init(ON_PIN);
    gpio_set_dir(ON_PIN, GPIO_IN);
    gpio_pull_down(ON_PIN);

    // Set the motor pins
    gpio_init(MTR_1);
    gpio_set_dir(MTR_1, GPIO_OUT);
    gpio_pull_down(MTR_1);
    gpio_put(MTR_1, 0);

    gpio_init(MTR_2);
    gpio_set_dir(MTR_2, GPIO_OUT);
    gpio_pull_down(MTR_2);
    gpio_put(MTR_2, 0);

    // Set the pump pins
    gpio_init(PMP_1);
    gpio_set_dir(PMP_1, GPIO_OUT);
    gpio_pull_down(PMP_1);
    gpio_put(PMP_1, 0);

    gpio_init(PMP_2);
    gpio_set_dir(PMP_2, GPIO_OUT);
    gpio_pull_down(PMP_2);
    gpio_put(PMP_2, 0);

    // Set the limit switch pins
    gpio_init(LIM_1);
    gpio_set_dir(LIM_1, GPIO_IN);
    gpio_pull_down(LIM_1);

    gpio_init(LIM_2);
    gpio_set_dir(LIM_2, GPIO_IN);
    gpio_pull_down(LIM_2);

    gpio_init(LIM_HI);
    gpio_set_dir(LIM_HI, GPIO_OUT);
    gpio_pull_up(LIM_HI);
    gpio_put(LIM_HI, 1);

    const float THRESHOLD = 90; //yellow temp? min red temp? green temp?
    char buf[64];
    int i = 0;
    bool cool = false;
    bool onL = false;
    bool lim1L = true;
    bool lim2L = false;

    while (true) {
        // Is it on?
        if (ON_PIN) {
            onL = true;
        } else {
            onL = false;
        }

        // UART logic
        // If a byte is available, read it in from Truck01: 
        if (uart_is_readable(UART_ID)) {
            char c = uart_getc(UART_ID);

            // Echo it back
            uart_putc_raw(UART_ID, c);

            if (c == '\n' || c == '\r') {
                buf[i] = 0;

                char label[16];
                float temp;

                if (sscanf(buf, "%f", &temp) == 1) {
                    if (temp > THRESHOLD) {
                        cool = true;
                    } else if (temp < THRESHOLD) {
                        cool = false;
                    }
                }
                i = 0;
            } else {
                if (i < sizeof(buf) - 1) {
                    buf[i++] = c;
                }
            }
        }

        if (LIM_1) {
            lim1L = true;
            lim2L = false;
        } else if (LIM_2) {
            lim1L = false;
            lim2L = true;
        }

        if (onL && cool) {
            gpio_put(PMP_1, 1);
            gpio_put(PMP_2, 0);

            gpio_put(MTR_1, lim1L);
            gpio_put(MTR_2, lim2L);
        }
    }

    return 0;

}
