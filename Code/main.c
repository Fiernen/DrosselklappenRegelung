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
#include "EEPROM_function.h"


uint16_t position;
uint16_t position_setpoint;
uint8_t kP_position = 45;
uint8_t kP_speed = 10;
uint8_t TN_speed = 1;



/* main() initalises all components and handles not real time relevent tasks
	
*/
int main(void)
{
	char lcd_str[16];
	wire_damage = 0;

	#if DEBUG
		DDRB |= 1<<DDB0;
	#endif
	
	// Initialization:
	DDRB |= 1<<PB5; // Enable signal port
	PORTB |= 1<<PB5; // Disable power electronics
	lcd_init();
	USART_init();
	USART_flush_receive();
	TimerPWM_init();
	TimerController_init();
	ADConverter_init();
	read_ctrl_params_from_EEPROM(&kP_position, &kP_speed, &TN_speed);
	

	
	sei(); // Enable interrupts
	PORTB &= ~(1<<PB5); // Enable power electronics
	

	// Main Loop:
	while(1)
	{	
		// Catch new parameters:
		USART_receive(&kP_position, &kP_speed, &TN_speed);
		save_ctrl_params2EEPROM(kP_position, kP_speed, TN_speed);
		
		// Send to PC via USART:
		USART_send_package();

		// Write to LCD-display:
		if (wire_damage)
		{
			lcd_cmd(0x01);
			lcd_cmd(0x80);
			lcd_text("Broken Wire!");
			lcd_cmd(0xC0);
			lcd_text("Is shutdown!");
			while (wire_damage);
			lcd_cmd(0x01);
		}
		lcd_cmd(0x80);
		lcd_angle(position_setpoint, lcd_str);
		lcd_text(lcd_str);		
		
		lcd_cmd(0x87);
		lcd_angle(position,lcd_str);
		lcd_text(lcd_str);
		
// 		if (UCSRA & (1<<RXC))
// 		{
// 			lcd_cmd(0xC0);
// 			lcd_zahl(UDR,lcd_str);
// 			lcd_text(lcd_str);		
// 		}
		lcd_cmd(0xC0);
 		lcd_zahl(kP_position,lcd_str);
		lcd_text(lcd_str);
		
		lcd_cmd(0xC4);
		lcd_zahl(kP_speed,lcd_str);
		lcd_text(lcd_str);
		
		lcd_cmd(0xC8);
		lcd_zahl(TN_speed,lcd_str);
		lcd_text(lcd_str);

// 		lcd_cmd(0xC0);
// 		lcd_zahl_s16(USART_send_4, lcd_str);
// 		lcd_text(lcd_str);
// 
// 		lcd_cmd(0xC7);
// 		lcd_zahl_s16(USART_send_7,lcd_str);
// 		lcd_text(lcd_str);
		
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
	USART_send_4 = position;
	position = FIR_filter(position);
	
	OCR1A = Motor_controller(position, position_setpoint, kP_position, kP_speed, TN_speed);

	uint16_t new_position_setpoint = setpoint_measure();
	position_setpoint = FIR_filter2(new_position_setpoint);
	
	#if DEBUG
		PORTB &= ~(1<<0);
	#endif

}