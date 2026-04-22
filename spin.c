#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "spin.h"

void spin_init(void) {
    gpio_init(STEP_PIN);
    gpio_set_dir(STEP_PIN, GPIO_OUT);
    gpio_put(STEP_PIN, 0);

    gpio_init(DIR_PIN);
    gpio_set_dir(DIR_PIN, GPIO_OUT);
    gpio_put(DIR_PIN, 1);
}

float spin_get_degree(int current_steps){
    return current_steps * (360.0f / RING_GEAR_REVOLUTION);
}

void spin_step_pulse(int delay_us){
    gpio_put(STEP_PIN, 1);
    sleep_us(delay_us);
    gpio_put(STEP_PIN, 0);
    sleep_us(delay_us);
}

void ramp_up(int delay_us){
    for (int d = 3000; d >= delay_us; d-= 200){
        for (int i = 0; i < 50; i++){
            spin_step_pulse(d);
        }
    }
}

void spin_step_once(bool forward, int delay_us, int *current_steps) {
    gpio_put(DIR_PIN, forward ? 1 : 0);
    spin_step_pulse(delay_us);
    *current_steps += forward ? 1 : -1;
}

void move_degrees(float degree, int direction, int delay_us, int *current_steps){
    float motor_steps = (degree / 360.0f) * RING_GEAR_REVOLUTION;
    gpio_put(DIR_PIN, direction);

    for (int i = 0; i < motor_steps; i++){
        spin_step_pulse(delay_us);

        if (direction == 1){
            (*current_steps)++;
        } 
        else{
            (*current_steps)--;
        }

        float angle = spin_get_degree(*current_steps);
        printf("Angle: %.2f degrees\n", angle);
    }
}

void sweep_in_degrees(float degree, int delay_us, int *current_steps){
    while (true){
        move_degrees(degree, 1, delay_us, current_steps);
        sleep_us(10);
        move_degrees(degree, 0, delay_us, current_steps);
        sleep_us(10);
    }
}

#ifndef SPIN_NO_MAIN
int main(void) {
    stdio_init_all();
    spin_init();

    int current_steps = 0;

    sweep_in_degrees(180, 3700, &current_steps);
    return 0;
}
#endif
