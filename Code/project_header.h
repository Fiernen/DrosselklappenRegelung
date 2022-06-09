// #pragma once
// #ifndef PROJECT_HEADER_H_
// #define PROJECT_HEADER_H_



#define DEBUG 1
#define TEST 1
#define MEASURE_IMPULSE_RESP 0
#define PULSE_WIDTH_SAMPLES 2
#define MANUAL_CONTROL 0
#define FILTER_SIZE 5


#define	F_CPU 3686400
#define ANGLE_RANGE 863
#define LOW_ADC2 172+1-74-2
#define HIGH_ADC2 960
#define LOW_ADC5 843+3+76+1
#define HIGH_ADC5 053

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


