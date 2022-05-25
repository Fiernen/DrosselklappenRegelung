#define	F_CPU			3686400
#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>

uint16_t AD_value_2; 
uint16_t AD_value_5;
uint16_t AD_mean;	// Mean AD value

#define	RS				0b00000100
#define	ENABLE			0b00001000
#define OHB				0b11110000
#define RS232BITS		0b00000011
#define LCD_RESET		0b00110000
#define LCD_INTERFACE	0b00100000

#define DEBUG 1

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

void lcd_init()
{
	/* Check "Displaytech Ltd LCD MODULE 162C SERIES PRODUCT SPECIFICATIONS"
		-	page 6 for a flowchart after which this function is written

	*/
	DDRD |= 0xFC;	// Set outputs for pins 2-7 of D, which connect to the LCD board and are used for the communication
	_delay_ms(16);	// Wait for LCD boot duration after receiving power (It's expected that this function will run at the start of main())
	
	// Shift LCD board into initialise mode by soft reseting three times:
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


void USART_init(void)
{
	// USART configuration:
	UBRRL = 23; // Baud rate 9600Bd
	UCSRA = 0;
	UCSRB = (1<<RXEN)|(1<<TXEN); // Receive & send enable
	UCSRC = (1<<URSEL)|(1<<UCSZ1)|(1<<UCSZ0); // 8 data bits
}

/* UART_send sends a 8 bit massage over UART

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
	while (!(UCSRA & (1<<UDRE))); // Check for empty data buffer
	UDR = msg & 0xFF; // Low 8 bits
	
	while (!(UCSRA & (1<<UDRE)));
	UDR = msg>>8; // High 8 bits
}

int main(void)
{
	#if DEBUG
	DDRB = (1<<DDB0);
	#endif
	
	char lcd_str[16];
	
	// Initialisation
	lcd_init();
	USART_init();
	
	// Init Timer 0 for measuring and PID Calculation:
	TCCR0 = (1<<CS00)|(1<<CS02);	// clk_IO/1024 (From prescaler)
	TIMSK |= (1<<TOIE0);			// Enable interrupt for Timer overflow	
	
    // Init ADC:
	ADMUX = (0<<REFS1) | (1<<REFS0); // Internal voltage reference
	ADMUX |= 0<<ADLAR; // write right sided
	ADCSRA |= 1<<ADEN; // Enable
	ADCSRA |=  (1<<ADPS2) | (1<<ADPS1) | (0<<ADPS0); // Prescaler = 64
	
	
	
	sei();
	
	uint16_t kk = 0;
	
	while(1)
	{
		lcd_cmd(0x80);
		lcd_zahl_16(AD_value_2,lcd_str);
		lcd_text(lcd_str);
		lcd_cmd(0xC0);
		lcd_zahl_16(AD_value_5,lcd_str);
		lcd_text(lcd_str);
// 		lcd_cmd(0x84);
// 		lcd_zahl(AD_mean,lcd_str);
// 		lcd_text(lcd_str);
// 		USART_send(AD_mean);
		lcd_cmd(0x87);
		lcd_zahl_16(AD_mean,lcd_str);
		lcd_text(lcd_str);
		USART_send_16(AD_mean);

	}
	
}

#define LOW_ADC2 172+1-74-2
#define HIGH_ADC2 960
#define LOW_ADC5 843+3+76+1
#define HIGH_ADC5 053

ISR(TIMER0_OVF_vect) //Last runtime measure = 0.48 ms
{
	/* ISR(TIMER0_OVF_vect) is a interrupt which is triggered by a timer 0 overflow.
		
	
	*/
	
	
	#if DEBUG
	PORTB |= 1<<0; // Time measure 
	#endif
	
	// ADC2 043 ... 240
	ADMUX &= ~0b1111;
	ADMUX |= 0b0010; // PC2
	// Measure
	ADCSRA |= 1<<ADSC; // Start Conversion
	while(ADCSRA&(1<<ADSC)); // Wait for completed conversion (ADSC switches back to 0)
	AD_value_2 = ADC;
	AD_value_2 -= LOW_ADC2;
	
	// ADC5 211 ... 013/014
	ADMUX &= ~0b1111;
	ADMUX |= 0b0101; // PC5
	// Measure
	ADCSRA |= 1<<ADSC; // Start Conversion
	while(ADCSRA&(1<<ADSC)); // Wait for completed conversion (ADSC switches back to 0)
	AD_value_5 = ADC;
	AD_value_5 = -AD_value_5 + LOW_ADC5;
	
	// Mean:
	AD_mean = (AD_value_2 + AD_value_5 + 1)/2; // Ultra smart rounding
	

	
	
	
	
	#if DEBUG
	PORTB &= ~(1<<0);
	#endif

}