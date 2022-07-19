/* DrosselklappenRegelung is a Project for Controlling a throttle valve

	by Valerian Weber and Beat Fürst
	
Features:
	- Measure position of valve
	- Position control
	- Displaying current set point and actual value angular position on LCD-display
	- Pushing form a computer new control parameters
	- Sending current state to computer
	
		
*/



#include "project_header.h"
#include "lcd_functions.h"
#include "USART_functions.h"
#include "controller_functions.h"
#include "EEPROM_function.h"



// Global variables:
uint16_t position;
uint16_t position_setpoint = 0;
uint8_t kP_position;
uint8_t kP_speed;
uint8_t TN_speed;



/* main() initalises all components and handles not real time relevent tasks
	
*/
int main(void)
{
	char lcd_str[16];
	wire_damage = 0;

	#if DEBUG
	DDRB |= 1<<DDB0;
	DDRB |= 1<<DDB2;
	DDRB |= 1<<DDB3;
	DDRB |= 1<<DDB4;
	#endif
	
	// Initialization: 
	DDRB |= 1<<PB5; // Enable signal port
	PORTB |= 1<<PB5; // Disable power electronics
	
	lcd_init();
	TimerPWM_init();
	TimerController_init();
	ADConverter_init();
	read_ctrl_params_from_EEPROM(&kP_position, &kP_speed, &TN_speed);
	USART_init();
	
	// Print static text to lcd display
	lcd_cmd(0x80);
	lcd_text("Sollwert:");
	lcd_cmd(0xC0);
	lcd_text("Istwert :");	
	
	sei(); // Enable interrupts
	PORTB &= ~(1<<PB5); // Enable power electronics
	

	// Main Loop:
	while(1)
	{	
		// Catch new parameters:
//  		USART_receive(&kP_position, &kP_speed, &TN_speed); // deprecated, solved with interrupt
		
		// Send to PC via USART:
		#if DEBUG
		PORTB |= 1<<3; // Time measure
		#endif
		USART_send_package();
		#if DEBUG
		PORTB &= ~(1<<3);
		#endif

		// Write to LCD-display:
		if (wire_damage)
		{
			lcd_cmd(0x01);
			lcd_cmd(0x80);
			lcd_text("Broken Wire!");
			lcd_cmd(0xC0);
			lcd_text("Is shutdown!");
		}
		else
		{
			// Display angles:
			lcd_cmd(0x8A);
			lcd_angle(position_setpoint, lcd_str);
			lcd_text(lcd_str);
		
			lcd_cmd(0xCA);
			lcd_angle(position,lcd_str);
			lcd_text(lcd_str);
		
		// Other stuff which can be displayed on the lcd display, for debug purpose
			/* 
			// Display controller parameters:
			lcd_cmd(0x80);
			lcd_text("kPp:");
			lcd_zahl(kP_position,lcd_str); // kP_position
			lcd_text(lcd_str);
		
			lcd_cmd(0xC0);
			lcd_text("kPs:");
			lcd_zahl(kP_speed,lcd_str); // kP_speed
			lcd_text(lcd_str);
		
			lcd_cmd(0xC8);
			lcd_text("TNs:");
			lcd_zahl(TN_speed,lcd_str); // TN_speed
			lcd_text(lcd_str);
			
			// Display angles and ctrl paras
			lcd_cmd(0x80);
			lcd_angle(position_setpoint,lcd_str);
			lcd_text(lcd_str);
		
			lcd_cmd(0x89);
			lcd_angle(position,lcd_str);
			lcd_text(lcd_str);
		
			lcd_cmd(0xC0);
			lcd_zahl(kP_position,lcd_str);
			lcd_text(lcd_str);
		
			lcd_cmd(0xC4);
			lcd_zahl(kP_speed,lcd_str);
			lcd_text(lcd_str);
		
			lcd_cmd(0xC8);
			lcd_zahl(TN_speed,lcd_str);
			lcd_text(lcd_str);
		
			// Display controller terms:
			lcd_cmd(0x80);
			lcd_zahl_s16(USART_send_speed_setpoint,lcd_str); // P-Position-Term
			lcd_text(lcd_str);
				
			lcd_cmd(0xC0);
			lcd_zahl_s16(USART_send_speed_P_term,lcd_str); // P-Speed-Term
			lcd_text(lcd_str);
				
			lcd_cmd(0xC8);
			lcd_zahl_s16(USART_send_speed_I_term,lcd_str); // I-Speed-Term
			lcd_text(lcd_str);
			*/
		}
	}
	return 0;
}



/* ISR(TIMER2_COMP_vect) is a interrupt which is triggered by a timer 0 overflow and handles the control

*/
ISR(TIMER2_COMP_vect)
{
	#if DEBUG
	PORTB |= 1<<0; // Time measure
	#endif
	
	
	
	// Stati:
	static uint8_t powered = 0;
	static uint8_t startup_mode_active = 0;
	
	
	
	position = position_measure();
	position = FIR_filter(position);
	if (USART_send_complete)
	{
		USART_send_speed_setpoint = position;
	}
	
	if (powered)
	{
		OCR1A = Motor_controller(position, position_setpoint, kP_position, kP_speed, TN_speed);
	}
	else // Wait for power
	{
		OCR1A = 1230; // ca. 60%
		if (position >= 100) // Checks with constant duty cycle, if the valve moves.
		{
			// When valve moved, it is assumed that the power electronics are turned on
			powered = 1; // turns controller on
			startup_mode_active = 1; // switch into startup mode
		}
	}
	
	/* Startup mode:
	Is unique to startup of the controller
	Sets a number of different set points before returning to default operation mode, with reads the set point from a poti.*/
	static uint8_t setpoint_preset_number = 0;
	#define controller_sample_frequency 900 // Frequency of interrupt
	static uint16_t sample_counter = 0; // counts samples till next set point change in startup mode
	uint16_t startup_setpoints[9] = {400,600,800,350,300,250,200,150,100};
	uint16_t new_position_setpoint;
	
	if (startup_mode_active)
	{
		new_position_setpoint = startup_setpoints[setpoint_preset_number];
		
		if (sample_counter == controller_sample_frequency)
		{
			setpoint_preset_number++; // Go to next preset set point
			sample_counter = 0; // Reset sample counter
		}
		
		if (setpoint_preset_number >= 9)
		{
			startup_mode_active = 0;
		}
		
		sample_counter++; // count up
	}
	else // default operating mode (poti measure for set point and filter)
	{
		new_position_setpoint = setpoint_measure();
	}
	
	position_setpoint = FIR_filter2(new_position_setpoint);
	
	if (USART_send_complete)
	{
		USART_send_position_setpoint = position_setpoint;
	}
	
	
	
	USART_send_complete = 0;
	
	
	
	#if DEBUG
	PORTB &= ~(1<<0);
	#endif
}



/* ISR(USART_RXC_vect) is triggered when the controller is receiving

*/
ISR(USART_RXC_vect)
{
	#if DEBUG
	PORTB |= 1<<4; // Time measure
	#endif
	USART_receive_ISR(&kP_position, &kP_speed, &TN_speed);
	#if DEBUG
	PORTB &= ~(1<<4);
	#endif
}


