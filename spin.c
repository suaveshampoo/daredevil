#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"

#define STEP_PIN 0
#define DIR_PIN 1

#define FULL__STEP_REVOLUTION 200
#define RING_GEAR_RATIO (81.0f / 20.0f)
#define RING_GEAR_REVOLUTION 810


float get_degree(int current_steps){
    return current_steps * (360.0 / RING_GEAR_REVOLUTION);
}

// For making one step
void step(int delay_us){
    gpio_put(STEP_PIN, 1);
    sleep_us(delay_us);
    gpio_put(STEP_PIN, 0);
    sleep_us(delay_us);
}

// Ramp up to speed
void ramp_up(int delay_us){
    for (int d = 3000; d >= delay_us; d-= 200){
        for (int i = 0; i < 50; i++){
            step(d);
        }
    }
}

// Move degrees in specified direction
void move_degrees(float degree, int direction, int delay_us, int *current_steps){
    float motor_steps = (degree / 360.0) * RING_GEAR_REVOLUTION;
    gpio_put(DIR_PIN, direction);

    for (int i = 0; i < motor_steps; i++){
        step(delay_us);

        if (direction == 1){
            (*current_steps)++;
        } 
        else{
            (*current_steps)--;
        }

        float angle = get_degree(*current_steps);
        printf("Angle: %.2f degrees\n", angle);
    }
}

// Sweep in specified degrees 
void sweep_in_degrees(float degree, int delay_us, int *current_steps){
    while (true){
        move_degrees(degree, 1, delay_us, current_steps);
        sleep_us(10);
        move_degrees(degree, 0, delay_us, current_steps);
        sleep_us(10);
    }
}


int main(void) {
    stdio_init_all();
    gpio_init(STEP_PIN);
    gpio_set_dir(STEP_PIN, GPIO_OUT);

    gpio_init(DIR_PIN);
    gpio_set_dir(DIR_PIN, GPIO_OUT);

    int current_steps = 0;

    // Calculate steps for 180 degrees in 3 seconds with no ramp
    // So 3700[us]
    sweep_in_degrees(180, 3700, &current_steps);
}

