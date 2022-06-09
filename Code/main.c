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

// struct filter_params filter = {.increment=0, .last_increment=FILTER_SIZE, .stack = {0}, .sum=0};
struct controller_params Motor_ctrl_params = {.kP_position=10, .kP_speed=1000, .TN_speed=10, .position_setpoint=800};
struct controller_params *params = &Motor_ctrl_params;



uint16_t position;
int16_t dduty_cycle;

int16_t speed;
int16_t position_error;
int16_t speed_setpoint;
int16_t speed_error;
int16_t speed_P_term;
int16_t speed_I_term;
int32_t duty_cycle;
uint16_t duty_cycle_scaled;
	


/* setpoint_measure() measuers a potentiometer at PC0 to change setpoint
	
*/
uint16_t setpoint_measure()
{	
	// ADC0
	ADMUX &= ~0b1111;
	ADMUX |= 0b0000; // PC0
	// Measure
	ADCSRA |= 1<<ADSC; // Start Conversion
	while(ADCSRA&(1<<ADSC)); // Wait for completed conversion (ADSC switches back to 0)
	uint16_t new_setpoint = ADC;
	// Limit to positions which can be reached by the system:
	if (new_setpoint > ANGLE_RANGE)
	{
		new_setpoint = ANGLE_RANGE;
	}
	return new_setpoint;

}


/* main() initalises all components and handles not real time relevent tasks
	
*/
int main(void)
{
	char lcd_str[16];
	wire_damage = 0;

	#if DEBUG
		DDRB |= 1<<DDB0;
	#endif
	
	// Initialization:
	DDRB |= 1<<PB5; // Enable signal port
	PORTB |= 1<<PB5; // Disable power electronics
	lcd_init();
	USART_init();
	TimerPWM_init();
	TimerController_init();
	ADConverter_init();

	

	
	sei(); // Enable interrupts
	PORTB &= ~(1<<PB5); // Enable power electronics
	

	// Main Loop:
	while(1)
	{		
		// Send to PC via USART:
		USART_send_package();

		// Write to LCD-display:
		if (wire_damage)
		{
			lcd_cmd(0x01);
			lcd_cmd(0x80);
			lcd_text("Broken Wire!");
			lcd_cmd(0xC0);
			lcd_text("Is shutdown!");
			while (wire_damage);
			lcd_cmd(0x01);
		}
		lcd_cmd(0x80);
		lcd_angle(Motor_ctrl_params.position_setpoint, lcd_str);
		lcd_text(lcd_str);		
		
		lcd_cmd(0x87);
		lcd_angle(position,lcd_str);
		lcd_text(lcd_str);

		lcd_cmd(0xC0);
		lcd_zahl_s16(speed_setpoint, lcd_str);
		lcd_text(lcd_str);

		lcd_cmd(0xC7);
		lcd_zahl_s16(speed_P_term,lcd_str);
		lcd_text(lcd_str);
		
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

	position = position_measure();
	USART_send_4 = position;
	position = FIR_filter(position);
// 	dduty_cycle = Motor_controller(position, &Motor_ctrl_params);
	

	static int16_t prev_sample_position = 0;
	static int32_t speed_error_integral = 0;

	// D-Term-speed;
	speed = (position-prev_sample_position); // derivative
	prev_sample_position = position;

	// P-term-position, with overflow protection:
	position_error = limit_int16((int32_t) params->position_setpoint - position, INT16_MIN, INT16_MAX);
	speed_setpoint = limit_int16((int32_t) 45 * position_error, INT16_MIN, INT16_MAX);
	
	// P-term-speed, with overflow protection:
	speed_error = limit_int16((int32_t) speed_setpoint - speed, INT16_MIN, INT16_MAX);
	speed_P_term = limit_int16((int32_t) speed_error / (10), INT16_MIN, INT16_MAX);
	
	// I-term-speed, with limits/overflow protection:
	speed_error_integral = limit_integral((int32_t) speed_error_integral + speed_P_term, INT16_MIN, INT16_MAX);
	speed_I_term = limit_int16((int32_t) speed_error_integral / 1, INT16_MIN, INT16_MAX);
	
	// Controller output P+I, with limits/overflow protection:
	duty_cycle = limit_int16((int32_t) speed_P_term, INT16_MIN, INT16_MAX);
	duty_cycle = (duty_cycle + 32767);
	duty_cycle = duty_cycle*ICR1;
	duty_cycle = duty_cycle/UINT16_MAX;
	
	// Controller output scaling:
	duty_cycle_scaled = (uint16_t) duty_cycle;
	
	OCR1A = duty_cycle_scaled;
	
	USART_send_1 = position;
	USART_send_2 = params->position_setpoint;
	USART_send_3 = duty_cycle_scaled;
	
	USART_send_4 = speed_setpoint;
	USART_send_5 = speed;
	USART_send_6 = speed_P_term;
	USART_send_8 = speed_I_term;
	
	uint16_t new_position_setpoint = setpoint_measure();
	params->position_setpoint = FIR_filter2(new_position_setpoint);
	
	#if DEBUG
		PORTB &= ~(1<<0);
	#endif

}