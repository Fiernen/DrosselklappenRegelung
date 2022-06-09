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


uint16_t USART_send_1;
uint16_t USART_send_2;
uint16_t USART_send_3;
uint16_t USART_send_4;
uint16_t USART_send_5;
uint16_t USART_send_6;

