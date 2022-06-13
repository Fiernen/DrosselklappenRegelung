/* DrosselklappenRegelung is a Project for Controlling a throttle valve

	by Valerian Weber and Beat F�rst
	
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
uint16_t position_setpoint = 0;
uint8_t kP_position = 45;
uint8_t kP_speed = 15;
uint8_t TN_speed = 1;




/* main() initalises all components and handles not real time relevent tasks
	
*/
int main(void)
{
	char lcd_str[16];
	wire_damage = 0;

	#if DEBUG
		DDRB |= 1<<DDB0;
		DDRB |= 1<<DDB2;
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
			while (wire_damage)
			{
				// Catch new parameters:
				USART_receive(&kP_position, &kP_speed, &TN_speed);
				save_ctrl_params2EEPROM(kP_position, kP_speed, TN_speed);
		
				// Send to PC via USART:
				USART_send_package();
			}
			lcd_cmd(0x01);
		}

		
		
		/* Display angles */
		lcd_cmd(0x80);
// 		lcd_text('Soll: ');
		lcd_angle(position_setpoint, lcd_str);
		lcd_text(lcd_str);
		
		lcd_cmd(0xC0);
// 		lcd_text(' Ist: ');
		lcd_angle(position,lcd_str);
		lcd_text(lcd_str);
		
		/* Display controller parameters: */
// 		lcd_cmd(0x80);
// 		lcd_text('kP_p: ');
// 		lcd_zahl(kP_position,lcd_str);
// 		lcd_text(lcd_str);
// 		
// 		lcd_cmd(0xC0);
// 		lcd_text('kP_s: ');
// 		lcd_zahl(kP_speed,lcd_str);
// 		lcd_text(lcd_str);
// 		
// 		lcd_cmd(0xC8);
// 		lcd_text('kP_p: ');
// 		lcd_zahl(TN_speed,lcd_str);
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

	static uint8_t setpoint_preset_number = 0;
	static uint8_t counter_startup_freq = 225;
	static uint8_t startup_mode_active = 1;
	static uint16_t startup_setpoints[4] = {200,400,600,800};
	
	if (startup_mode_active)
	{
		if (counter_startup_freq == 225)
		{
			position_setpoint = startup_setpoints[setpoint_preset_number];			
			
			// count up
			setpoint_preset_number++;
			if (setpoint_preset_number > 4)
			{
// 				setpoint_preset_number = 0;
				startup_mode_active = 0;
			}
			counter_startup_freq = 0;
		}
		else
		{
			counter_startup_freq++;
		}
	}
	else 
	{
		uint16_t new_position_setpoint = setpoint_measure();
		position_setpoint = FIR_filter2(new_position_setpoint);
	}
	
	
	
	
	#if DEBUG
		PORTB &= ~(1<<0);
	#endif

}


