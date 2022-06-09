// #pragma once
// #ifndef PROJECT_HEADER_H_
// #define PROJECT_HEADER_H_



#define DEBUG 1
#define TEST 1
#define MEASURE_IMPULSE_RESP 0
#define PULSE_WIDTH_SAMPLES 2
#define MANUAL_CONTROL 0
#define FILTER_SIZE 5
#define WIRE_TOLERANCE 10

#define	F_CPU 3686400
#define LOW_ADC2 94
#define HIGH_ADC2 970
#define LOW_ADC5 926
#define HIGH_ADC5 47
#define ANGLE_RANGE (LOW_ADC5 - HIGH_ADC5 + HIGH_ADC2-LOW_ADC2) / 2

#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>

uint8_t wire_damage;

struct controller_params {
	int16_t kP_position; // Gain
	int16_t kP_speed; // Gain
	int16_t TN_speed; // Integrator time constant
	int16_t position_setpoint;
};

struct filter_params {
	int16_t stack[FILTER_SIZE];
	uint8_t increment;
	uint8_t last_increment;
	int16_t sum;
};


