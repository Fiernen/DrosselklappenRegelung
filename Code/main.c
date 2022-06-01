/* DrosselklappenRegelung is a Project for Controlling a throttle valve

	by Valerian Weber and Beat Fürst
	
Features:
	- Measure position of valve
	- Position control
	- Displaying current set point and actual value on LCD-display
	- Pushing form a computer new control parameters
	
		
*/

#include "project_header.h"
#include "lcd_functions.h"
#include "USART_functions.h"
#include "controller_functions.h"

struct filter_params filter = {.increment=0, .last_increment=FILTER_SIZE, .stack[FILTER_SIZE] = {0}, .sum=0};
struct controller_params Motor_ctrl_params = {.kP_position=1, .kP_speed=1, .TN_speed=64, .position_setpoint=200};

uint16_t position;
int16_t dduty_cycle;



void TEST_Motor_controller(struct controller_params *params)
{
	uint8_t posi;
	
	for (uint16_t ii=0; ii<0xFFF; ii++)
	{
		posi = ii;
		Motor_controller(posi, params);
	}
}



void TEST_FIR_filter(struct filter_params *params)
{
	uint8_t posi;
	
	for (uint16_t ii=0; ii<0xFFF; ii++)
	{
		posi = ii;
		FIR_filter(posi, params);
	}
}



/* measure_impulse_response() implements an alternative to the normel mode of operation, to measure an impulse response
	Call this function inside TIMER0_OVF_vect with controller disabled.
*/
void measure_impulse_response()
{
	static uint8_t sample_counter = 0;
	uint16_t posi;
	
	if (sample_counter <= PULSE_WIDTH_SAMPLES)
	{
		OCR1A = ICR1*3/4;
	}
	else
	{
		posi = position_measure();
		USART_send_16(posi);
	}
	sample_counter++;
}



/* manual_control() measuers a potentiometer for dutycycle control
	Call this function instead of normal control-loop in the TIMER0_OVF_vect interrupt for manual control.
*/
void manual_control()
{	
	// ADC?
	ADMUX &= ~0b1111;
	ADMUX |= 0b0010; // PC?
	// Measure
	ADCSRA |= 1<<ADSC; // Start Conversion
	while(ADCSRA&(1<<ADSC)); // Wait for completed conversion (ADSC switches back to 0)
	OCR1A = ADC;
}


/* main() initalises all components and handles not real time relevent tasks
	
*/
int main(void)
{
	#if DEBUG
	DDRB |= 1<<DDB0;
	#endif
	
	TEST_Motor_controller(&Motor_ctrl_params);
	TEST_FIR_filter(&filter);
	
	char lcd_str[16];

	// Initialization:
	lcd_init();
	USART_init();
	TimerPWM_init();
	TimerController_init();
	ADConverter_init();
	sei();
	
	

	// Main Loop:
	while(1)
	{
		// Write to LCD-display:
		lcd_cmd(0xC1);
		lcd_zahl_16(position,lcd_str);
		lcd_text(lcd_str);
		
		lcd_cmd(0xC7);
		lcd_zahl_s16(dduty_cycle,lcd_str);
		lcd_text(lcd_str);

// 		lcd_cmd(0x80);
// 		lcd_zahl_s16(speed_P_term,lcd_str);
// 		lcd_text(lcd_str);
// 
// 		lcd_cmd(0x87);
// 		lcd_zahl_s16(speed_I_term,lcd_str);
// 		lcd_text(lcd_str);

		// Send over USART:
// 		USART_send_16(position);

		// Catch new controller parameters:

	}
	return 0;
}



/* ISR(TIMER0_OVF_vect) is a interrupt which is triggered by a timer 0 overflow.

*/
ISR(TIMER0_OVF_vect)
{

	#if DEBUG
	PORTB |= 1<<0; // Time measure 
	#endif
	
	#if MEASURE_IMPULSE_RESP // Compile as impulse response measure (disabled control-loop)
	measure_impulse_response();
	#elif MANUAL_CONTROL // Compile manual control (disabled control-loop)
	manual_control();
	#else // Compile normal operation control-loop
	position = position_measure();
// 	position = FIR_filter(position, &filter);
	dduty_cycle = Motor_controller(position, &Motor_ctrl_params);
	OCR1A = dduty_cycle;
	#endif
	
	#if DEBUG
	PORTB &= ~(1<<0);
	#endif

}