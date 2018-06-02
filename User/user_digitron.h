#ifndef __USER_DIGITRON_H
#define __USER_DIGITRON_H
#include "stm8s.h"
//#include "stdbool.h"
//#include <stdlib.h>
//uint8_t DUAN_TABLE[] = {0xFC, 0x60, 0xDA, 0xF2, 0x66, 0xB6, 0xBE, 0xE0, 0xFE, 0xF6};
void digitronInit(void);
void showNum(uint8_t num);
void showPositionNum(uint8_t firstbit, uint8_t number);
#endif
