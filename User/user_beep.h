#ifndef __USER_BEEP_H
#define __USER_BEEP_H
#include "stm8s.h"
struct BEEP_INFOR
{
    uint8_t on_off;//蜂鸣器开关
    uint8_t mode;//响动方式，短响、长响
    uint8_t time;//响动时间
};
enum BEEP_MODE
{
    SHORT_,
    LONG_,
};
void beepInit();
void beepCtrl();
#endif
