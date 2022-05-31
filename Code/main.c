/* DrosselklappenRegelung is a Project for Controlling a throttle valve

	by Valerian Weber and Beat Fürst
	
Features:
	? Measure position of valve
	? Position control
	? Displaying current set point and actual value on LCD-display
	? Pushing form a computer new control parameters
	
		
*/


#include "project_header.h"
#include "lcd_functions.h"
#include "USART_functions.h"
#include "controller_functions.h"

struct filter_params filter;
struct controller_params Motor_ctrl_params = {.kP_position = 1, .kP_speed=1,.TN_speed=64,.position_setpoint=200};

uint16_t position;
int16_t dduty_cycle;

void TEST_Motor_controller(struct controller_params *params)
{
	for (uint16_t ii=0; ii<0xFFF; ii++)
	{
		position = ii;
		Motor_controller(position, params);
	}
}



void TEST_FIR_filter(struct filter_params *params)
{
	for (uint16_t ii=0; ii<0xFFF; ii++)
	{
		position = ii;
		FIR_filter(position, params);
	}
}

/* measure_impulse_response() implements an alternative to the normel mode of operation, to measure an impulse response
*/
void measure_impulse_response()
{
	static uint8_t sample_counter = 0;
	
	if (sample_counter <= PULSE_WIDTH_SAMPLES)
	{
		OCR1A = ICR1*3/4;
	}
	else
	{
		position = position_measure();
		USART_send_16(position);
	}
	sample_counter++;
}

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
	OCR1A = ICR1*0.50; // Update PWM timer compare value
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
	
// 	#if MEASURE_IMPULSE_RESP // Implement impulse response measure
// 	measure_impulse_response();
// 
// 	
// 	#else // Normal Operation
// 	#endif

	position = position_measure();
// 	position = FIR_filter(position, &filter);
	dduty_cycle = Motor_controller(position, &Motor_ctrl_params);
	OCR1A = ICR1/2 + dduty_cycle;

	
	#if DEBUG
	PORTB &= ~(1<<0);
	#endif

}