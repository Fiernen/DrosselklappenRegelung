#include "project_header.h"
#include "USART_functions.h"
#include "EEPROM_function.h"


/* USART_init() initializes the USART interface

*/
void USART_init(void)
{
	UBRRL = 23; // Baud rate 9600Bd
	UCSRA = 0;
	UCSRB |= (1<<RXCIE); // RX Complete Interrupt Enable
	UCSRB |= (1<<RXEN)|(1<<TXEN); // Enable receive and send
	UCSRC = (1<<URSEL)|(1<<UCSZ1)|(1<<UCSZ0); // 8 data bits
	
	USART_send_position = 0;
	USART_send_position_setpoint = 0;
	USART_send_duty_cycle_scaled = 0;
	USART_send_speed_setpoint = 0;
	USART_send_speed = 0;
	USART_send_speed_P_term = 0;
	USART_send_duty_cycle = 0;
	USART_send_speed_I_term = 0;
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
	UDR = msg >> 8; // High 8 bits
}



/* USART_send_debug() 
	
*/
void USART_send_package(void)
{
	
	// Header:
	USART_send(255);
	USART_send(1);
	USART_send(101);
	
// 	/* Send Data: */
// 	USART_send_16(USART_send_position);
// 	USART_send_16(USART_send_position_setpoint);
// 	USART_send_16(USART_send_duty_cycle);
	
	/* Send unsigned int */
	USART_send_16(USART_send_position);
	USART_send_16(USART_send_position_setpoint);
	USART_send_16(USART_send_duty_cycle_scaled);
	
	/* Send signed int */
	USART_send_16(USART_send_speed);
	USART_send_16(USART_send_speed_setpoint);
	USART_send_16(USART_send_speed_P_term);
	USART_send_16(USART_send_speed_I_term);
	USART_send_16(USART_send_duty_cycle);
}



void USART_receive(uint8_t* kP_position, uint8_t* kP_speed, uint8_t* TN_speed)
{
	static uint8_t msg_incoming = 0;
	if (UCSRA & (1<<RXC))
	{
		if (msg_incoming)
		{
			*kP_position = UDR;
			*kP_speed = UDR;
			*TN_speed = UDR;
			save_ctrl_params2EEPROM(*kP_position, *kP_speed, *TN_speed);
			msg_incoming = 0;
		}
		else
		{
			uint8_t header0 = UDR;
			uint8_t header1 = UDR;
			uint8_t header2 = UDR;

			if (header0 == 255 && header1 == 50 && header2 == 100)
			{
				msg_incoming = 1;
			}
			else
			{
				USART_flush_receive();
			}
		}
	}
}



void USART_flush_receive(void)
{
	uint8_t dummy = 0;
	while (UCSRA & (1<<RXC))  // Flush any msg in USART receive buffer
	{
		dummy = UDR;
	}
	return;
}



void USART_receive_ISR(uint8_t* kP_position, uint8_t* kP_speed, uint8_t* TN_speed)
{
	static uint8_t header_finder[3] = {0};
	static uint8_t finder_counter = 0;
	static uint8_t header_found = 0;
	static uint8_t header[3] = {255,50,100};
	#define header_length 3
	
	
	
	while (UCSRA & (1<<RXC))
	{
		if (!header_found)
		{
			header_finder[finder_counter] = UDR;
			if (finder_counter == (header_length-1))
			{
				finder_counter = 0;
			}
			else
			{
				finder_counter++;
			}
			
			
			// Match header_finder to header
			for (uint8_t idx = 0; idx < header_length; idx++)
			{
				// uint8_t subidx = 0;
				uint8_t temp_array[3] = {0};
				uint8_t k = idx;
				
				for (uint8_t ii = 0; ii < 3; ii++)
				{
					temp_array[ii] = header_finder[k];
					if (k == (header_length-1))
					{
						k = 0;
					}
					else
					{
						k++;
					}
				}
				
				if (temp_array[0] == header[0] && temp_array[1] == header[1] && temp_array[2] == header[2])
				{
					header_found = 1;
				}
			}
		}
		else
		{
			static uint8_t msg_counter = 0;
			switch (msg_counter)
			{
				case 0 : *kP_position = UDR; msg_counter++;	break;
				case 1 : *kP_speed = UDR; msg_counter++; break;
				case 2 : *TN_speed = UDR; msg_counter = 0;
					header_found = 0;
					for (uint8_t jj = 0; jj < header_length; jj++)
					{
						header_finder[jj] = 0;
					}
					break;
			}
		}
	}
}
