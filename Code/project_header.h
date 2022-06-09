#pragma once
// #ifndef PROJECT_HEADER_H_
// #define PROJECT_HEADER_H_



#define DEBUG 1
#define TEST 0
#define MEASURE_IMPULSE_RESP 0
#define PULSE_WIDTH_SAMPLES 2
#define MANUAL_CONTROL 0
#define FILTER_SIZE 32


#define	F_CPU 3686400
#define ANGLE_RANGE 863
#define LOW_ADC2 97
#define HIGH_ADC2 960
#define LOW_ADC5 923
#define HIGH_ADC5 053
#define MEAN_AD (HIGH_ADC2+HIGH_ADC5+1)/2 // Smart rounding
 #define WIRE_TOLERANCE 10
	 
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
	uint16_t stack[FILTER_SIZE];
	uint8_t increment;
	uint16_t sum;
};


