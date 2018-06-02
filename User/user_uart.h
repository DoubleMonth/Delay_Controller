#ifndef __UART_H
#define __UART_H

#include <stdio.h>
#include "stm8s.h"
void uart2Init();
void uart2SendByte(uint8_t data);
void uart2SendString(uint8_t* Data,uint16_t len);
uint8_t uart2ReceiveByte(void);
#endif