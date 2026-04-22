#ifndef SPIN_H
#define SPIN_H

#include <stdbool.h>

#define STEP_PIN 0
#define DIR_PIN 1
#define RING_GEAR_REVOLUTION 810.0f

void spin_init(void);
float spin_get_degree(int current_steps);
void spin_step_pulse(int delay_us);
void spin_step_once(bool forward, int delay_us, int *current_steps);
void ramp_up(int delay_us);
void move_degrees(float degree, int direction, int delay_us, int *current_steps);
void sweep_in_degrees(float degree, int delay_us, int *current_steps);

#endif
