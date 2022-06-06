#ifndef USART_FUNCTIONS_H_
#define USART_FUNCTIONS_H_

void USART_init(void);
void USART_send(uint8_t msg);
void USART_send_16(uint16_t msg);
void USART_send_set_is(uint16_t set, uint16_t is);

#endif /* USART_FUNCTIONS_H_ */