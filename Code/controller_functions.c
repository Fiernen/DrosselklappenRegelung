/* controller_functions.c is collection of functions to initalise and operate the main tasks of the project
	measuring, filtering, controlling position and setting a PWM signal.

*/

#include "project_header.h"
// #include "controller_functions.h"


/* TimerController_init() initialises the Timer/Counter 1 for controller interrupt
	Timer 0 for measuring and controller calculation:
*/
void TimerController_init()
{
	TCCR0 = (1<<CS01)|(1<<CS00);	// clk_IO/64 --> sample rate 4.42 ms / sample freq 225 Hz
	TIMSK |= (1<<TOIE0);			// Enable interrupt for Timer overflow		
}



/* ADConverter_init() does the setup for the AD-Converter

*/
void ADConverter_init()
{
	ADMUX = (0<<REFS1) | (1<<REFS0); // Internal voltage reference
	ADMUX |= 0<<ADLAR; // write right sided
	ADCSRA |= 1<<ADEN; // Enable
	ADCSRA |=  (1<<ADPS2) | (1<<ADPS1) | (0<<ADPS0); // Prescaler = 64
	
}



/* Timer1_init configures the timer and PWM signal for motor control
	The pulse width can be set anywhere in the code after initialization via OCR1A (16bit) which must be smaller than ICR1.
*/
void TimerPWM_init(void)
{
	DDRB |= (1<<DDB1); // OC1A = PortB1
	
	TCCR1A = (1<<COM1A1)|(0<<COM1A0); // Clear OC1A on Compare Match, set OC1A at BOTTOM, (non-inverting mode)
	
	// Operation Mode: 14 Fast PWM with compare value at OCR1A
	TCCR1A |= (1<<WGM11)|(0<<WGM10);
	TCCR1B  = (1<<WGM13)|(1<<WGM12);
	
	/* Set Frequency:
		The Frequency of the PWM signal is 225 Hz. F_PWM = F_CPU/(N*(TOP+1))
		Generally aim for low prescaler N and high TOP-value for better accuracy */
	ICR1 = 0x07FF; // TOP-value 11 bit
	TCCR1B |= (0<<CS12)|(0<<CS11)|(1<<CS10); // Prescaler N = 8
	
	OCR1A = ICR1/2; // Set default duty cycle to 50%
}



check_wire_integrety(uint16_t AD2, uint16_t AD5)
{
	if ((AD2 > (HIGH_ADC2+10)) || (AD2 < (LOW_ADC2-10)) || (AD5 > (HIGH_ADC5+10)) || (AD5 < (LOW_ADC5-10)))
	{
		wire_damage = 1;
		PORTB |= 1<<PB5; // Disable power electronics
	}
}


/* measure() measueres the AD-values at C2 and C5 and means the values

*/
int16_t position_measure(void)
{
	// ADC2
	ADMUX &= ~0b1111;
	ADMUX |= 0b0010; // PC2
	// Measure
	ADCSRA |= 1<<ADSC; // Start Conversion
	while(ADCSRA&(1<<ADSC)); // Wait for completed conversion (ADSC switches back to 0)
	uint16_t AD_value_2 = ADC;
	
	// ADC5
	ADMUX &= ~0b1111;
	ADMUX |= 0b0101; // PC5
	// Measure
	ADCSRA |= 1<<ADSC; // Start Conversion
	while(ADCSRA&(1<<ADSC)); // Wait for completed conversion (ADSC switches back to 0)
	uint16_t AD_value_5 = ADC;
	
	// Open-circuit detection of the potentiometers:
	/* Compare AD values. When the Difference is to high --> Open-circuit
			--> Warning. Shutdown? or switch to operation with just one potentiometer*/
	check_wire_integrety(AD_value_2, AD_value_5);	

	AD_value_2 = AD_value_2 - LOW_ADC2;
	AD_value_5 = -AD_value_5 + LOW_ADC5;	

	// Mean:
	return (AD_value_2 + AD_value_5 + 1)/2; // Ultra smart rounding
}



/* FIR_filter(new_value, *params) implements a FIR-filter
	a function call replaces the oldest value in the stack and calculates the new mean.
*/
int16_t FIR_filter(int16_t new_value, struct filter_params *params)
{
	params->stack[params->increment] = new_value; // Replace oldest field with new value
	params->sum = params->sum - params->stack[params->last_increment] + new_value; // Correct sum
	
	// Increment to next container field:
	params->last_increment = params->increment;
	params->increment++;
	if (params->increment > FILTER_SIZE)
	{
		params->increment = 0;
	}
	
	return params->sum/FILTER_SIZE; // Return mean value
}

/* limit_int16(var, MAX, MIN) Limits the argument var between INT16_MIN and INT16_MAX 

*/
int16_t check_int16_overunderflow(int32_t var)
{
	if (var > INT16_MAX)
	{
		return INT16_MAX;
	}
	else if (var < INT16_MIN)
	{
		return INT16_MIN;
	}
	return var;
}



/* limit_int16(var, MAX, MIN) Limits the argument var between MIN and MAX

*/
int16_t limit_int16(int32_t var, int16_t MIN, int16_t MAX)
{
	if (var > MAX)
	{
		return MAX;
	}
	else if (var < MIN)
	{
		return MIN;
	}
	return var;
}



/* limit_integral(var, MAX, MIN) Limits the argument var between MIN and MAX
	This function does not register overflows.
*/
int32_t limit_integral(int32_t var, int32_t MIN, int32_t MAX)
{
	if (var > MAX)
	{
		return MAX;
	}
	else if (var < MIN)
	{
		return MIN;
	}
	return var;
}



/* Motor_controller() implements a cascading P-position-controller into PI-speed-controller

*/
int16_t Motor_controller(uint16_t position, struct controller_params *params)
{

	int16_t speed;
	int16_t position_error;
	int16_t speed_setpoint;
	int16_t speed_error;
	int16_t speed_P_term;
	int16_t speed_I_term;
	int16_t duty_cycle;
	int32_t duty_cycle_scaled;
	
	int16_t MAX_position_error = INT16_MAX/params->kP_position;
	int16_t MIN_position_error = -INT16_MAX/params->kP_position;
	int16_t MAX_speed_error = INT16_MAX/params->kP_speed;
	int16_t MIN_speed_error = -INT16_MAX/params->kP_speed;
// 	int32_t MAX_speed_error_integral = INT16_MAX/(params->TN_speed*10); // Wierd behavior line 203 changes value in unknown way, Problem not found
	int32_t MIN_speed_error_integral = INT16_MIN/(params->TN_speed*10);
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

	// Controller output scaling:
	duty_cycle_scaled = ((duty_cycle + INT16_MAX)*ICR1)/UINT16_MAX; // int32 because the biggest number is 134148098)
	
	return duty_cycle_scaled;
}


