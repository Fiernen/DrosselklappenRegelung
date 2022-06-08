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
void USART_send_set_is()
{
	
	// Header:
//  	USART_send_16((uint16_t) 0xFFFF); // 0xFFFF should not be a possible value of set point or current value
	
	// Data:
	USART_send(255);
	USART_send(0);
	USART_send(100);
	USART_send(10);
	USART_send(110);
	USART_send_16(USART_send_1);
	USART_send_16(USART_send_2);
	USART_send_16(USART_send_3);
	USART_send_16(USART_send_4);
	USART_send_16(USART_send_5);
	USART_send_16(USART_send_6);
	
}



// void USART_receive(void)
// {
// 	uint8_t dummy;
// 	static uint8_t USART_Recieve_counter = 0;
// 	
// 	if (UCSRA & (1<<RXC))
// 	{
// 		if (UDR == 0xFF) // Check if header is send (parameters are not allowed to be 255)
// 		{
// 			USART_Recieve_counter = 1; // Set counter for trailing parameters
// 			dummy = UDR; // flush latest msg
// 
// 		}
// 		else
// 		{
// 			switch (USART_Recieve_counter) // Set parameter according to the trailing positions after the header
// 			{
// 				case 1: kP_position = UDR; USART_Recieve_counter++; break;
// 				case 2: kP_speed = UDR; USART_Recieve_counter++; break;
// 				case 3: TN_speed = UDR; USART_Recieve_counter = 0; break;
// 				default: dummy = UDR; //flush last msg
// 			}
// 		}
// 	}
// }



void USART_flush_receive(void)
{
	uint8_t dummy = 0;
	while (UCSRA & (1<<RXC))  // Flush any msg in USART receive buffer
	{
		dummy = UDR;
	}
}
