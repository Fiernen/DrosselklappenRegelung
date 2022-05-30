#define	F_CPU			3686400
#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>

#define FILTER_ORDER 5
uint16_t filter_container[FILTER_ORDER];

uint16_t AD_value_2; 
uint16_t AD_value_5;
uint16_t position;	// Mean AD value

int16_t speed;
int16_t speed_setpoint;
int16_t speed_P_term; // Result for P-term
int16_t speed_I_term;
int16_t prev_time_step_position = 0;
// Vars for control loop:

// P-position-controller:
int16_t position_setpoint = 863;
int16_t position_error;
int16_t kP_position = 1; // Gain



// D-Term:
	
// PI-speed-controller:

int16_t speed_error;
	
// P-term:
int16_t kP_speed = 1; // Gain

// I-term:
int16_t TN_speed = 5; // Integrator time constant
int16_t speed_error_integral = 0;
int32_t temp_integral;

// Result
int32_t temp_PWM_duty_cycle; // temp value to check for overflow
int16_t PWM_duty_cycle;



#define	RS				0b00000100
#define	ENABLE			0b00001000
#define OHB				0b11110000
#define RS232BITS		0b00000011
#define LCD_RESET		0b00110000
#define LCD_INTERFACE	0b00100000

#define DEBUG 1

/* lcd_zahl converts a 8 bit number into a 3 digit char vector

*/
void lcd_zahl(uint8_t zahl,char* text)
{
	char ziff1;								// Hunderterstelle
	char ziff2;								// Zehnerstelle
	
	ziff1 = zahl/100;						// Hunderterstelle ermitteln
	zahl -= ziff1 * 100;					// Hunderter abziehen
	
	ziff2 = zahl/10;						// Zehnerstelle ermitteln
	zahl -= ziff2 * 10;						// Zehner abziehen
	
	text[0] = ziff1 + 0x30;					// ASCII-Code f?r Hunderter
	text[1] = ziff2 + 0x30;					// ASCII-Code f?r Zehner
	text[2] = zahl + 0x30;					// ASCII-Code f?r Einer
	text[3] = 0x00;							// Endekennung
	return;
}

/* lcd_zahl_16 converts a 16 bit number into a 5 digit char vector
	
*/
void lcd_zahl_16(uint16_t num, char* written)
{
	uint16_t devisor;
	uint8_t digit;
	uint8_t i = 0;
	
	for (devisor=10000; devisor != 0; devisor /= 10)
	{
		digit = num/devisor;
		written[i] = digit + 0x30;
		num -= digit*devisor;
		i++;
	}
	written[i] = 0x00; // End marker
	return;
}

/* lcd_zahl_16 converts a 16 bit number into a 5 digit char vector
	
*/
void lcd_zahl_s16(int16_t num, char* written)
{
	uint16_t devisor;
	uint8_t digit;
	uint8_t i = 0;
	
	if (num>>15)
	{
		written[i] = '-';
		num = ~num+1;
	}
	else if (num == 0)
	{
		written[i] = ' ';
	}
	else
	{
		written[i] = '+';
	}
	i++;
	
	for (devisor=10000; devisor != 0; devisor /= 10)
	{
		digit = num/devisor;
		written[i] = digit + 0x30;
		num -= digit*devisor;
		i++;
	}
	written[i] = 0x00; // End marker
	return;
}

/* lcd_text writes the contents of a char vector to the the LCD display

*/
void lcd_text(char* ztext)
{
	uint8_t j = 0;							// Z?hlvariable initialisieren
	
	while(ztext[j] != 0)
	{
		PORTD &= (~OHB);					// OHB=0 setzen
		PORTD |= RS;						// RS=1 setzen
		PORTD |= ENABLE;					// E=1 setzen
		PORTD |= (ztext[j] & OHB);			// Einsen aus OHB von ztext ?bernehmen
		PORTD &= ~ENABLE;					// E=0 setzen
		PORTD |= ENABLE;					// E=1 setzen
		PORTD &= ~OHB;						// OHB in PORTD l?schen
		PORTD |= (ztext[j]<<4);				// UHB von ztext in OHB von PORTD schreiben
		PORTD &= ~ENABLE;					// E=0 setzen
		j++;								// j inkrementieren
		_delay_ms(1);						// Befehlsausf?hrung
	}
	return;
}

/* lcd_cmd sends a command to the LCD display

*/
void lcd_cmd(unsigned char cmd)
{
	PORTD &= (ENABLE + RS232BITS);		// OHB=0 und RS=0 setzen
	PORTD |= ENABLE;					// E=1 setzen
	PORTD |= (cmd & OHB);				// Einsen aus OHB von cmd ?bernehmen
	PORTD &= ~ENABLE;					// E=0 setzen
	PORTD |= ENABLE;					// E=1 setzen
	PORTD &= ~OHB;						// OHB in PORTD l?schen
	PORTD |= (cmd<<4);					// UHB von cmd in OHB von PORTD schreiben
	PORTD &= ~ENABLE;					// E=0 setzen
	_delay_ms(1);						// Befehlsausf?hrung
	return;
}


/* lcd_init() initializes the LCD display

*/
void lcd_init()
{
	/* Check "Displaytech Ltd LCD MODULE 162C SERIES PRODUCT SPECIFICATIONS"
		-	page 6 for a flowchart after which this function is written

	*/
	DDRD |= 0xFC;	// Set outputs for pins 2-7 of D, which connect to the LCD board and are used for the communication
	_delay_ms(16);	// Wait for LCD boot duration after receiving power (It's expected that this function will run at the start of main())
	
	// Shift LCD board into initialize mode by soft reseting three times:
	PORTD &= ~0xF0; // Disable data bits
	PORTD |= 0x08; // Enable
	PORTD |= 0x30;	// Soft Reset
	PORTD &= ~0x08;	// Disable
	_delay_ms(5); // Individual command length, can be different for the following commands

	PORTD |= 0x08;
	PORTD &= ~0x08;
	_delay_ms(1);
	
	PORTD |= 0x08;	
	PORTD &= ~0x08;
	_delay_ms(1);	// undocumented delay after the third soft reset is needed!
	
	// LCD board is now in initalise mode:
	// Configure display settings:
	PORTD &= ~0xF0; // Disable all LCD pins D2-7
	PORTD |= 0x08;
	PORTD |= 0x20;	// Setup interface to 4-bit mode so lcd_cmd can be used 
	PORTD &= ~0x08;
	_delay_ms(1);
	
	lcd_cmd(0x28);	// Set Function: 	4 Bit Interface, 2 Lines, 5 X 7 Dots

	lcd_cmd(0x06);	

	lcd_cmd(0x0F);	

	lcd_cmd(0x01);
	_delay_ms(2);
}

/* USART_init() initializes the USART interface

*/
void USART_init(void)
{
	UBRRL = 23; // Baud rate 9600Bd
	UCSRA = 0;
	UCSRB = (1<<RXEN)|(1<<TXEN); // Enable receive and send
	UCSRC = (1<<URSEL)|(1<<UCSZ1)|(1<<UCSZ0); // 8 data bits
}

/* USART_send sends a 8 bit massage over UART

*/
void USART_send(uint8_t msg)
{
	// Send USART
	while (!(UCSRA & (1<<UDRE))); // Check for empty data buffer
	UDR = msg;

}

/* UART_send sends a 16 bit massage over UART

*/
void USART_send_16(uint16_t msg)
{
	// Low must be send before high, otherwise are spikes in the resulting graph in the terminal app when the lower bite "overflows"
	while (!(UCSRA & (1<<UDRE))); // Check for empty data buffer
	UDR = msg & 0xFF; // Low 8 bits
	
	while (!(UCSRA & (1<<UDRE)));
	UDR = msg>>8; // High 8 bits
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
	TCCR1B |= (0<<CS12)|(1<<CS11)|(0<<CS10); // Prescaler N = 8
}



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
	
	OCR1A = 0x07FF*0.50; // Update PWM timer compare value
	
	// Init Timer 0 for measuring and PID Calculation:
	TCCR0 = (1<<CS01)|(1<<CS00);	// clk_IO/64 --> sample rate 4.42 ms
	TIMSK |= (1<<TOIE0);			// Enable interrupt for Timer overflow	
	
    // Init ADC:
	ADMUX = (0<<REFS1) | (1<<REFS0); // Internal voltage reference
	ADMUX |= 0<<ADLAR; // write right sided
	ADCSRA |= 1<<ADEN; // Enable
	ADCSRA |=  (1<<ADPS2) | (1<<ADPS1) | (0<<ADPS0); // Prescaler = 64
	
	
// 	struct filter_param{
// 		final uint8_t order 5
// 		
// 		};
	
	sei();
	
	while(1)
	{
		lcd_cmd(0xC1);
		lcd_zahl_16(OCR1A,lcd_str);
		lcd_text(lcd_str);
		USART_send_16(position);
		
		lcd_cmd(0xC7);
		lcd_zahl_s16(PWM_duty_cycle,lcd_str);
		lcd_text(lcd_str);

		lcd_cmd(0x80);
		lcd_zahl_s16(speed_P_term,lcd_str);
		lcd_text(lcd_str);

		lcd_cmd(0x87);
		lcd_zahl_s16(speed_I_term,lcd_str);
		lcd_text(lcd_str);

	}
	
}



/* measure() measueres the AD-values at C2 and C5 and means the values

*/
int16_t position_measure(void)
{
	// Minima and maxima for both 
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
	AD_value_2 = ADC;
	AD_value_2 -= LOW_ADC2;
	
	// ADC5
	ADMUX &= ~0b1111;
	ADMUX |= 0b0101; // PC5
	// Measure
	ADCSRA |= 1<<ADSC; // Start Conversion
	while(ADCSRA&(1<<ADSC)); // Wait for completed conversion (ADSC switches back to 0)
	AD_value_5 = ADC;
	AD_value_5 = -AD_value_5 + LOW_ADC5;
	
	// Open-circuit detection of the potentiometers:
	/* Compare AD values. When the Difference is to high --> Open-circuit
			--> Warning. Shutdown? or switch to operation with just one potentiometer*/
	
	// Mean:
	return (AD_value_2 + AD_value_5 + 1)/2; // Ultra smart rounding
}


struct filter_params {
	#define filter_size 5
	int16_t container[filter_size] = {0};
	uint8_t increment = 0;
	uint8_t last_increment = filter_size;
	int16_t sum = 0;
};

// filter = 

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

int16_t motor_control(int16_t position, int)
{
	
}

ISR(TIMER0_OVF_vect) // Last runtime measure = 0.483 us
{
	/* ISR(TIMER0_OVF_vect) is a interrupt which is triggered by a timer 0 overflow.
		
	
	*/
	#if DEBUG
	PORTB |= 1<<0; // Time measure 
	#endif
	

	
	position = position_measure();


	// Motor control:
	/* Cascading P-position-controller into PI-speed-controller
	
	Needed Components:
		Filter		--> revolving vector
		Derivative	--> slope from last sample point
		
	 */
	
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
	speed = (position-prev_time_step_position); // derivative
	prev_time_step_position = position;

	// P-term-position, with overflow protection:
	position_error = position_setpoint - position;
	if (position_error > MAX_position_error)
	{
		position_error = MAX_position_error;
	}
	else if (position_error < MIN_position_error)
	{
		position_error = MIN_position_error;
	}
	speed_setpoint = kP_position * position_error;
	
	// P-term-speed, with overflow protection:
	speed_error = speed_setpoint - speed;
	if (speed_error > MAX_speed_error)
	{
		speed_error = MAX_speed_error;
	} 
	else if (speed_error < MIN_speed_error)
	{
		speed_error = MIN_speed_error;
	}
	speed_P_term = kP_speed * speed_error;
	
	// I-term-speed, with limits/overflow protection:
	temp_integral = speed_error_integral + speed_P_term;
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
		speed_error_integral = speed_error_integral + speed_error;
		
	}
	speed_I_term = speed_error_integral/TN_speed;
	
	// Control value sum, with limits/overflow protection:
	temp_PWM_duty_cycle = speed_P_term + speed_I_term;
	if (temp_PWM_duty_cycle > MAX_PWM_duty_cycle)
	{
		PWM_duty_cycle = MAX_PWM_duty_cycle;
	}
	else if(temp_PWM_duty_cycle < MIN_PWM_duty_cycle)
	{
		PWM_duty_cycle = MIN_PWM_duty_cycle;
	}
	else
	{
		PWM_duty_cycle = speed_P_term + speed_I_term;
	}	
	
	// PWM scaling:
	PWM_duty_cycle /= MAX_PWM_duty_cycle/0x7FF;
	OCR1A = 0x7FF/2 + PWM_duty_cycle;
	
	
	
	#if DEBUG
	PORTB &= ~(1<<0);
	#endif

}