#ifndef USART_FUNCTIONS_H_
#define USART_FUNCTIONS_H_

void USART_init(void);
void USART_send(uint8_t msg);
void USART_send_16(uint16_t msg);
void USART_send_package(void);
void USART_receive(uint8_t* kP_position, uint8_t* kP_speed, uint8_t* TN_speed);
void USART_flush_receive(void);
void USART_receive_ISR(uint8_t* kP_position, uint8_t* kP_speed, uint8_t* TN_speed)

#endif /* USART_FUNCTIONS_H_ */