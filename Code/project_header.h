// #pragma once
// #ifndef PROJECT_HEADER_H_
// #define PROJECT_HEADER_H_


#define default_kP_position 50
#define default_kP_speed 70
#define default_kP_TN_speed 150

#define STARTUP_MODE 0
#define DEBUG 1
#define TEST 1
#define MEASURE_IMPULSE_RESP 0
#define PULSE_WIDTH_SAMPLES 2
#define MANUAL_CONTROL 0
#define FILTER_SIZE 24
#define WIRE_TOLERANCE 10

#define	F_CPU 3686400
#define LOW_ADC2 94
#define HIGH_ADC2 970
#define LOW_ADC5 926
#define HIGH_ADC5 47
#define ANGLE_RANGE 877

#include <stdint.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>

uint8_t wire_damage;


uint16_t USART_send_position;
uint16_t USART_send_position_setpoint;
uint16_t USART_send_duty_cycle_scaled;
uint16_t USART_send_speed_setpoint;
uint16_t USART_send_speed;
uint16_t USART_send_speed_P_term;
uint16_t USART_send_duty_cycle;
uint16_t USART_send_speed_I_term;


uint8_t debug_var;