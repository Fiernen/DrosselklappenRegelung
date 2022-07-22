// #pragma once
#ifndef PROJECT_HEADER_H_
#define PROJECT_HEADER_H_

// Default controller parameters:
#define default_kP_position 50
#define default_kP_speed 70
#define default_kP_TN_speed 150

// Enables some outputs which can be used to measure timings etc. for debug purposes
#define DEBUG 1

// Filter size
#define FILTER_SIZE 24

// Tolerance value which characterizes the allowed deviation of the redundant vale angle sensors
#define WIRE_TOLERANCE 10

#define	F_CPU 3686400

// Limits and Range of ADC-Values of potis, which measure the valves angle
#define LOW_ADC2 94
#define HIGH_ADC2 970
#define LOW_ADC5 926
#define HIGH_ADC5 47
#define ANGLE_RANGE 877

// Used libraries 
#include <stdint.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>

// Global variable used to track wire damage
uint8_t wire_damage;

// Global variables used for USART interface
uint16_t USART_send_position;
uint16_t USART_send_position_setpoint;
uint16_t USART_send_duty_cycle_scaled;
uint16_t USART_send_speed_setpoint;
uint16_t USART_send_speed;
uint16_t USART_send_speed_P_term;
uint16_t USART_send_duty_cycle;
uint16_t USART_send_speed_I_term;
uint8_t USART_send_complete;
uint8_t USART_send_kP_position;
uint8_t USART_send_kP_speed;
uint8_t USART_send_TN_speed;

#endif