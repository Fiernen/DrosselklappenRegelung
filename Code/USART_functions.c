#include "project_header.h"



/* USART_init() initializes the USART interface

*/
void USART_init(void)
{
	UBRRL = 23; // Baud rate 9600Bd
	UCSRA = 0;
	UCSRB = (1<<RXEN)|(1<<TXEN); // Enable receive and send
	UCSRC = (1<<URSEL)|(1<<UCSZ1)|(1<<UCSZ0); // 8 data bits
	
	USART_send_1 = 0;
	USART_send_2 = 0;
	USART_send_3 = 0;
	USART_send_4 = 0;
	USART_send_5 = 0;
	USART_send_6 = 0;
	USART_send_7 = 0;
	USART_send_8 = 0;
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



/* USART_send_debug() 
	
*/
void USART_send_package(void)
{
	
	// Header:
	USART_send(255);
	USART_send(0);
	USART_send(100);
	USART_send(10);
	USART_send(110);
	
	// Data:
	USART_send_16(USART_send_1);
	USART_send_16(USART_send_2);
	USART_send_16(USART_send_3);
	USART_send_16(USART_send_4);
	USART_send_16(USART_send_5);
	USART_send_16(USART_send_6);
	USART_send_16(USART_send_7);
	USART_send_16(USART_send_8);
	
}



void USART_receive(uint8_t* kP_position, uint8_t* kP_speed, uint8_t* TN_speed)
{
	uint8_t dummy;
	static uint8_t USART_Recieve_counter = 1;

// 	while (UCSRA & (1<<RXC))
// 	{
// 		dummy = UDR;
// 		if (dummy == 0xFF) // Check if header is send (parameters are not allowed to be 255)
// 		{
// 			USART_Recieve_counter = 1; // Set counter for trailing parameters
// 
// 		}
// 		else
// 		{
// 			switch (USART_Recieve_counter) // Set parameter according to the trailing positions after the header
// 			{
// 				case 1: *kP_position = dummy; USART_Recieve_counter++; break;
// 				case 2: *kP_speed = dummy; USART_Recieve_counter++; break;
// 				case 3: *TN_speed = dummy; USART_Recieve_counter = 1; break;
// 			}
// 		}
// 	}
// 	}
	if (UCSRA & (1<<RXC))
	{
		*kP_position = UDR;
		*kP_speed = UDR;
		*TN_speed = UDR;
	}
}



void USART_flush_receive(void)
{
	_delay_ms(500);
	uint8_t dummy = 0;
	while (UCSRA & (1<<RXC))  // Flush any msg in USART receive buffer
	{
		dummy = UDR;
	}
	return;
}