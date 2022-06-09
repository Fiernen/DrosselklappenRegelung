#include "project_header.h"



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



/* USART_send_set_is(uint16_t set, uint16_t is) sends set point and current value via USART
	
*/
void USART_send_set_is(uint16_t set, uint16_t is)
{
	
	// Header:
//  	USART_send_16((uint16_t) 0xFFFF); // 0xFFFF should not be a possible value of set point or current value
	
	// Data:
	USART_send(255);
	USART_send(0);
	USART_send(100);
	USART_send(10);
	USART_send(110);
	USART_send_16(set);
	USART_send_16(is);
	USART_send_16(OCR1A);
}


