// #pragma once
// #ifndef PROJECT_HEADER_H_
// #define PROJECT_HEADER_H_



#define DEBUG 1
#define TEST 1
#define MEASURE_IMPULSE_RESP 0
#define PULSE_WIDTH_SAMPLES 2
#define MANUAL_CONTROL 0
#define FILTER_SIZE 32
#define WIRE_TOLERANCE 10

#define	F_CPU 3686400
#define LOW_ADC2 94
#define HIGH_ADC2 970
#define LOW_ADC5 926
#define HIGH_ADC5 47
#define ANGLE_RANGE (LOW_ADC5 - HIGH_ADC5 + HIGH_ADC2-LOW_ADC2) / 2

#include <stdint.h>
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
uint16_t USART_send_7;
uint16_t USART_send_8;


uint8_t debug_var;