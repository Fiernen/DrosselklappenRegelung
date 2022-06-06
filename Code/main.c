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
struct controller_params Motor_ctrl_params = {.kP_position=1, .kP_speed=1, .TN_speed=10, .position_setpoint=800};
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
	


void TEST_Motor_controller(struct controller_params *params)
{
	uint8_t posi;
	uint16_t dc;
	
	for (uint16_t ii=0; ii<0xFFF; ii++)
	{
		posi = ii;
		dc = Motor_controller(posi, params);
		posi = ii + ii;
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
	uint32_t angle =  800;
	lcd_angle(angle, lcd_str);

	#if DEBUG
		DDRB |= 1<<DDB0;
	#endif
	
	#if TEST
		TEST_Motor_controller(&Motor_ctrl_params);
		
	// 	TEST_FIR_filter(&filter);
	#endif
	
	

	// Initialization:
	lcd_init();
	USART_init();
	TimerPWM_init();
	TimerController_init();
	ADConverter_init();
	
	sei(); // Enable interrupts
	
	

	// Main Loop:
	while(1)
	{
		// get new set point:
		Motor_ctrl_params.position_setpoint = setpoint_measure();
		
		// Write to LCD-display:
		lcd_cmd(0x80);
		lcd_angle(Motor_ctrl_params.position_setpoint, lcd_str);
		lcd_text(lcd_str);		
		
		lcd_cmd(0x87);
		lcd_angle(position,lcd_str);
		lcd_text(lcd_str);

		lcd_cmd(0xC0);
		lcd_zahl_16(Motor_ctrl_params.position_setpoint, lcd_str);
		lcd_text(lcd_str);

		lcd_cmd(0xC7);
		lcd_zahl_16(position,lcd_str);
		lcd_text(lcd_str);
		
// 		lcd_cmd(0x87);
// 		lcd_zahl_s16(speed_P_term,lcd_str);
// 		lcd_text(lcd_str);
// 
// 		lcd_cmd(0xC0);
// 		lcd_zahl_16(duty_cycle_scaled,lcd_str);
// 		lcd_text(lcd_str);
// 
// 		lcd_cmd(0xC7);
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

	#else // Compile normal operation control-loop
		position = position_measure();
	// 	position = FIR_filter(position, &filter);
	// 	dduty_cycle = Motor_controller(position, &Motor_ctrl_params);
	
		// Limits for overflow protection
		int16_t MAX_position_error = INT16_MAX/params->kP_position;
		int16_t MIN_position_error = -INT16_MAX/params->kP_position;
		int16_t MAX_speed_error = INT16_MAX/params->kP_speed;
		int16_t MIN_speed_error = -INT16_MAX/params->kP_speed;
		// 	int32_t MAX_speed_error_integral = INT16_MAX/(params->TN_speed*10); // Wierd behavior line 203 changes value in unknown way, Problem not found
		int32_t MIN_speed_error_integral = (int32_t) INT16_MIN/params->TN_speed*10;
		#define MAX_PWM_duty_cycle INT16_MAX
		#define MIN_PWM_duty_cycle INT16_MIN // must be symmetrical for scaling
	
		static int16_t prev_sample_position = 0;
		static int32_t speed_error_integral = 0;

		// D-Term-speed;
		speed = (position-prev_sample_position); // derivative
		prev_sample_position = position;

		// P-term-position, with overflow protection:
		position_error = limit_int16((int32_t) params->position_setpoint - position, MIN_position_error, MAX_position_error);
		speed_setpoint = params->kP_position * position_error;
	
		// P-term-speed, with overflow protection:
		speed_error = limit_int16((int32_t) speed_setpoint - speed, MIN_speed_error, MAX_speed_error);
		speed_P_term = params->kP_speed * speed_error;
	
		// I-term-speed, with limits/overflow protection:
		speed_error_integral = limit_integral(speed_error_integral + speed_P_term, MIN_speed_error_integral, -MIN_speed_error_integral-1);
		speed_I_term = speed_error_integral*params->TN_speed/10;
	
		// Controller output P+I, with limits/overflow protection:
		duty_cycle = limit_int16((int32_t) speed_P_term + speed_I_term, MIN_PWM_duty_cycle, MAX_PWM_duty_cycle);
		duty_cycle = (duty_cycle + 32767);
		duty_cycle = duty_cycle*ICR1;
		duty_cycle = duty_cycle/UINT16_MAX;
	
		// Controller output scaling:
		duty_cycle_scaled = (uint16_t) duty_cycle;
	
		OCR1A = duty_cycle_scaled;
	#endif
	
	#if DEBUG
		PORTB &= ~(1<<0);
	#endif

}