#ifndef USART_FUNCTIONS_H_
#define USART_FUNCTIONS_H_

void USART_init(void);
void USART_send(uint8_t msg);
void USART_send_16(uint16_t msg);
void USART_send_set_is( uint16_t position_setpoint, uint16_t position, int16_t speed_setpoint, int16_t speed, int16_t speed_P_term, int16_t speed_I_term);

#endif /* USART_FUNCTIONS_H_ */