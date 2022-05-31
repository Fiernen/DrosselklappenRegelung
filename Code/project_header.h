// #pragma once
// #ifndef PROJECT_HEADER_H_
// #define PROJECT_HEADER_H_



#define DEBUG 1
#define MEASURE_IMPULSE_RESP 0
#define PULSE_WIDTH_SAMPLES 2



#define	F_CPU 3686400



#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>


struct controller_params {
	int16_t kP_position; // Gain
	int16_t kP_speed; // Gain
	int16_t TN_speed; // Integrator time constant
	int16_t position_setpoint;
};

struct filter_params {
	#define filter_size 4
	int16_t stack[filter_size];
	uint8_t increment;
	uint8_t last_increment;
	int16_t sum;
};

