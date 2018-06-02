#ifndef __USER_GPIO_H
#define __USER_GPIO_H
#include "stm8s.h"
#include "stm8s_gpio.h"
#define KEY1_PORT GPIOC 
#define KEY1_PIN GPIO_PIN_1
#define KEY2_PORT GPIOC 
#define KEY2_PIN GPIO_PIN_2
#define KEY3_PORT GPIOC 
#define KEY3_PIN GPIO_PIN_3
#define REALY_PORT GPIOB 
#define REALY_PIN GPIO_PIN_1
#define INT1_GPIO GPIOB
#define INT1_PIN GPIO_PIN_3
#define INT2_GPIO GPIOB
#define INT2_PIN GPIO_PIN_2

void gpioInit();
#endif