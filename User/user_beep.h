#ifndef __USER_BEEP_H
#define __USER_BEEP_H
#include "stm8s.h"
struct BEEP_INFOR
{
    uint8_t on_off;//����������
    uint8_t mode;//�춯��ʽ�����졢����
    uint8_t time;//�춯ʱ��
};
enum BEEP_MODE
{
    SHORT_,
    LONG_,
};
void beepInit();
void beepCtrl();
#endif
