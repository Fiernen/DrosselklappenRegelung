#include "project_header.h"



#define	RS				0b00000100
#define	ENABLE			0b00001000
#define OHB				0b11110000
#define RS232BITS		0b00000011
#define LCD_RESET		0b00110000
#define LCD_INTERFACE	0b00100000



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

/* lcd_angle converts a 16 bit number into a 5 digit char vector with decimal comma
	
*/
void lcd_angle(uint16_t num, char* written)
{
	num = num*10*90/ANGLE_RANGE; // scale
	uint16_t devisor;
	uint8_t digit;
	uint8_t i = 0;
	
	for (devisor=100; devisor != 0; devisor /= 10)
	{
		if (i == 2) // Add a comma
		{
			written[i] = 44;
			i++;
		}
		digit = num/devisor;
		written[i] = digit + 0x30;
		num -= digit*devisor;
		i++;
	}
	written[i] = 0xDF;
	i++;
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


