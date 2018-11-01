#ifndef __usart_H
#define __usart_H

#include "system.h"
#include <stdbool.h>
#include <stdio.h>

#define usart3_data_max 1024

extern uint8_t usart3_recieve_data[usart3_data_max];
extern uint32_t usart3_recieve_len;
extern bool usart3_recieve_all;

int fputc(int ch, FILE *p);

void usart3_write(uint8_t *buf, uint16_t cnt);
void USART3_Init(uint32_t baud);

#endif
