/* Collection of functions to operate a LCD board

	by Beat Fürst 31.03.2022
*/

#include <avr/io.h>
#include <util/delay.h>
#include "lcd_functions.h"

/* lcd_cmd(cmd) write comand to lcd board
	Parameter:
    	cmd: Kommando, Typ: uint8_t
	
	Please note, that the lcd diplay should be initialised first with lcd_init().
*/
void lcd_cmd(uint8_t cmd)
{
	/* Check "Displaytech Ltd LCD MODULE 162C SERIES PRODUCT SPECIFICATIONS"
		-	pages 3 and 4 for details regarding timings
	*/
	PORTD &= 0x0B;			// Disable RS, data bits/payload and E

	PORTD |= 0x08;			// Enable
	PORTD |= (cmd & 0xF0);	// transfer top four bits of payload
	PORTD &= ~0x08;			// Disable

	PORTD &= ~0xF0;			// Disable data bits

	PORTD |= 0x08;			// Enable
	PORTD |= cmd<<4;		// transfer lower four bits of payload
	PORTD &= ~0x08;			// Disable
}

/* lcd_init() initialises LCD board 

	This Function needs to run bevore it is possible to write to the LCD Display.
*/
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


//////////////////////////////////////////////7

#define	RS				0b00000100
#define	ENABLE			0b00001000
#define OHB				0b11110000
#define RS232BITS		0b00000011
#define LCD_RESET		0b00110000
#define LCD_INTERFACE	0b00100000


void convert_num2char(uint8_t zahl,char* text)
{
	char ziff1;								// Hunderterstelle
	char ziff2;								// Zehnerstelle
	
	ziff1 = zahl/100;						// Hunderterstelle ermitteln
	zahl -= ziff1 * 100;					// Hunderter abziehen
	
	ziff2 = zahl/10;						// Zehnerstelle ermitteln
	zahl -= ziff2 * 10;						// Zehner abziehen
	
	text[0] = ziff1 + 0x30;					// ASCII-Code f�r Hunderter
	text[1] = ziff2 + 0x30;					// ASCII-Code f�r Zehner
	text[2] = zahl + 0x30;					// ASCII-Code f�r Einer
	text[3] = 0x00;							// Endekennung
	return;
}



void lcd_text(char* ztext)
{
	uint8_t j = 0;							// Z�hlvariable initialisieren
	
	while(ztext[j] != 0)
	{
		PORTD &= (~OHB);					// OHB=0 setzen
		PORTD |= RS;						// RS=1 setzen
		PORTD |= ENABLE;					// E=1 setzen
		PORTD |= (ztext[j] & OHB);			// Einsen aus OHB von ztext �bernehmen
		PORTD &= ~ENABLE;					// E=0 setzen
		PORTD |= ENABLE;					// E=1 setzen
		PORTD &= ~OHB;						// OHB in PORTD l�schen
		PORTD |= (ztext[j]<<4);				// UHB von ztext in OHB von PORTD schreiben
		PORTD &= ~ENABLE;					// E=0 setzen
		j++;								// j inkrementieren
		_delay_ms(1);						// Befehlsausf�hrung
	}
	return;
}

// /* lcd_set_cursor_pos(row, column) sets LCD cursor position
// 	Parameter:
// 		- row		Type: unit8_t
// 		- column	Type: unit8_t
//  */
// int lcd_shift_cursor(uint8_t row, uint8_t column)
// {
// 	
// }
// 
// int lcd_shift_display(uint8_t row, uint8_t column)
// {
// 
// }

