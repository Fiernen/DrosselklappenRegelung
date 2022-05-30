#include "project_header.h"


int16_t kP_position = 1; // Gain
int16_t kP_speed = 1; // Gain
int16_t TN_speed = 5; // Integrator time constant
uint16_t position;	// Mean AD value
int16_t position_setpoint = 863;


int16_t duty_cycle;

#define DEBUG 1


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
	TCCR1B |= (0<<CS12)|(1<<CS11)|(0<<CS10); // Prescaler N = 8
}



/* measure() measueres the AD-values at C2 and C5 and means the values

*/
int16_t position_measure(void)
{
	
	#define LOW_ADC2 172+1-74-2
	#define HIGH_ADC2 960
	#define LOW_ADC5 843+3+76+1
	#define HIGH_ADC5 053

	// ADC2
	ADMUX &= ~0b1111;
	ADMUX |= 0b0010; // PC2
	// Measure
	ADCSRA |= 1<<ADSC; // Start Conversion
	while(ADCSRA&(1<<ADSC)); // Wait for completed conversion (ADSC switches back to 0)
	uint16_t AD_value_2 = ADC;
	AD_value_2 -= LOW_ADC2;
	
	// ADC5
	ADMUX &= ~0b1111;
	ADMUX |= 0b0101; // PC5
	// Measure
	ADCSRA |= 1<<ADSC; // Start Conversion
	while(ADCSRA&(1<<ADSC)); // Wait for completed conversion (ADSC switches back to 0)
	uint16_t AD_value_5 = ADC;
	AD_value_5 = -AD_value_5 + LOW_ADC5;
	
	// Open-circuit detection of the potentiometers:
	/* Compare AD values. When the Difference is to high --> Open-circuit
			--> Warning. Shutdown? or switch to operation with just one potentiometer*/
	
	// Mean:
	return (AD_value_2 + AD_value_5 + 1)/2; // Ultra smart rounding
}



struct filter_params {
	#define filter_size 5
	int16_t container[filter_size];
	uint8_t increment;
	uint8_t last_increment;
	int16_t sum;
};

struct filter_params filter;

int16_t FIR_filter(int16_t new_value, struct filter_params *params)
{
	params->container[params->increment] = new_value; // Replace oldest field with new value
	params->sum = params->sum - params->container[params->last_increment] + new_value; // Correct sum
	
	// Increment to next container field:
	params->last_increment = params->increment;
	params->increment++;
	if (params->increment > filter_size)
	{
		params->increment = 0;
	}
	
	return params->sum/filter_size; // Return mean value
}



int16_t prev_time_step_position = 0;
int16_t speed_error_integral = 0;

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

/* Motor_controller() implements a cascading P-position-controller into PI-speed-controller
			
*/
int16_t Motor_controller()
{

	
	// Limits for overflow protection
	int16_t MAX_position_error = INT16_MAX/kP_position;
	int16_t MIN_position_error = -INT16_MAX/kP_position;
	int16_t MAX_speed_error = INT16_MAX/kP_speed;
	int16_t MIN_speed_error = -INT16_MAX/kP_speed;
	int16_t MAX_speed_error_integral = 0x6FFF/5;
	int16_t MIN_speed_error_integral = -0x6FFF/5;
	#define MAX_PWM_duty_cycle INT16_MAX
	#define MIN_PWM_duty_cycle -INT16_MAX // must be symetrical for scaling
	

	
	// D-Term-speed;
	int16_t speed = (position-prev_time_step_position); // derivative
	prev_time_step_position = position;

	// P-term-position, with overflow protection:
	int32_t position_error = position_setpoint - position;
	if (position_error > MAX_position_error)
	{
		position_error = MAX_position_error;
	}
	else if (position_error < MIN_position_error)
	{
		position_error = MIN_position_error;
	}
	int16_t speed_setpoint = kP_position * position_error;
	
	// P-term-speed, with overflow protection:
	int32_t speed_error = speed_setpoint - speed;
	if (speed_error > MAX_speed_error)
	{
		speed_error = MAX_speed_error;
	} 
	else if (speed_error < MIN_speed_error)
	{
		speed_error = MIN_speed_error;
	}
	int16_t speed_P_term = kP_speed * speed_error;
	
	// I-term-speed, with limits/overflow protection:
	int32_t temp_integral = speed_error_integral + speed_P_term;
	if (temp_integral > MAX_speed_error_integral)
	{
		speed_error_integral = MAX_speed_error_integral;
	} 
	else if(temp_integral < MIN_speed_error_integral)
	{
		speed_error_integral = MIN_speed_error_integral;
	}
	else
	{
		speed_error_integral = speed_error_integral + speed_P_term;
		
	}
	int16_t speed_I_term = speed_error_integral/TN_speed;
	
	// Control value sum, with limits/overflow protection:
	int16_t duty_cycle;
	int32_t temp_PWM_duty_cycle = speed_P_term + speed_I_term;
	if (temp_PWM_duty_cycle > MAX_PWM_duty_cycle)
	{
		duty_cycle = MAX_PWM_duty_cycle;
	}
	else if(temp_PWM_duty_cycle < MIN_PWM_duty_cycle)
	{
		duty_cycle = MIN_PWM_duty_cycle;
	}
	else
	{
		duty_cycle = speed_P_term + speed_I_term;
	}	
	
	// PWM scaling:
	duty_cycle /= MAX_PWM_duty_cycle/0x7FF;
	return duty_cycle;
}



/* TimerController_init() initialises the Timer/Counter 1 for controller interrupt
	Timer 0 for measuring and PID Calculation:
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
	int32_t temp;
	int16_t res;
	int32_t res2;
	int32_t min = INT16_MIN;
		
int main(void)
{
	#if DEBUG
	DDRB |= 1<<DDB0;
	#endif
		
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
		lcd_zahl_16(OCR1A,lcd_str);
		lcd_text(lcd_str);
		
		lcd_cmd(0xC7);
		lcd_zahl_s16(duty_cycle,lcd_str);
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

// Test check_int16_overunderflow()
// 		temp = (int32_t) INT16_MAX + 1;
// 		res = check_int16_overunderflow(temp);
// 		res2 = res;
// 		_delay_ms(2);
// 		temp = INT16_MIN - 1;
// 		res = check_int16_overunderflow(temp);
// 		_delay_ms(1);
// 		temp = 0+1;
// 		res = check_int16_overunderflow(temp);
// 		_delay_ms(1);
// 		temp = 0-1;
		
		
		
		// Catch new controller parameters:

	}
	
}


/* ISR(TIMER0_OVF_vect) is a interrupt which is triggered by a timer 0 overflow.


*/
ISR(TIMER0_OVF_vect)
{

	#if DEBUG
	PORTB |= 1<<0; // Time measure 
	#endif
	

	
	position = position_measure();
	
	position = FIR_filter(position, &filter);

	duty_cycle = Motor_controller();
	
	OCR1A = ICR1/2 + duty_cycle;
	
	
	
	#if DEBUG
	PORTB &= ~(1<<0);
	#endif

}