/* DrosselklappenRegelung is a Project for Controlling a throttle valve

	by Valerian Weber and Beat Fürst
	
Features:
	- Measure position of valve
	- Position control
	- Displaying current set point and actual value on LCD-display
	- Pushing form a computer new control parameters
	
		
*/

#include "project_header.h"
#include "lcd_functions.h"
#include "USART_functions.h"
#include "controller_functions.h"



uint8_t kP_position=10;
uint8_t kP_speed=1;
uint8_t TN_speed=20;
uint16_t position_setpoint = 0;
uint16_t position;



/* main() initalises all components and handles not real time relevent tasks
	
*/
int main(void)
{
	char lcd_str[16]; // Character array used by a number of LCD functions
	wire_damage = 0;

	#if DEBUG
		DDRB |= 1<<DDB0;
	#endif

	// Initialization:
	DDRB |= 1<<PB5; // Enable signal port
	PORTB |= 1<<PB5; // Disable power electronics
	
	lcd_init();
	USART_init();
	TimerPWM_init();
	TimerController_init();
	ADConverter_init();
	USART_flush_receive();

	
	sei(); // Enable interrupts
	PORTB &= ~(1<<PB5); // Enable power electronics
	
	uint8_t USART_Recieve_counter = 0;

	// Main Loop:
	while(1)
	{

		// get new set point:
		position_setpoint = setpoint_measure();
		
		// Send to PC via USART:
		USART_send_set_is();

		// Write to LCD-display:
		if (wire_damage)
		{
			lcd_cmd(0x01);
			lcd_cmd(0x80);
			lcd_text("Broken Wire!");
			lcd_cmd(0xC0);
			lcd_text("Is shutdown!");
			_delay_ms(250);
			lcd_cmd(0x01);
		}
		else
		{
		lcd_cmd(0x80);
		lcd_angle(position_setpoint, lcd_str);
		lcd_text(lcd_str);		
		
		lcd_cmd(0x87);
		lcd_angle(position,lcd_str);
		lcd_text(lcd_str);

		lcd_cmd(0xC0);
		lcd_zahl_16(position_setpoint, lcd_str);
		lcd_text(lcd_str);

		lcd_cmd(0xC4);
		lcd_zahl(kP_speed,lcd_str);
		lcd_text(lcd_str);
		
		lcd_cmd(0xC8);
		lcd_zahl(TN_speed,lcd_str);
		lcd_text(lcd_str);
		
		lcd_cmd(0xCC);
		lcd_zahl(000,lcd_str);
		lcd_text(lcd_str);
		
		}
	}
	return 0;
}



/* ISR(TIMER0_OVF_vect) is a interrupt which is triggered by a timer 0 overflow.

*/
ISR(TIMER0_OVF_vect)
{
	
	#if DEBUG
		PORTB |= 1<<0; // Time measure 
	#endif
	
	
		position = position_measure();
	 	position = FIR_filter(position);
		OCR1A = Motor_controller(position, position_setpoint, kP_position, kP_speed, TN_speed);

	
	#if DEBUG
		PORTB &= ~(1<<0);
	#endif

}
