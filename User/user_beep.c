#include "user_beep.h"
#include "stm8s_beep.h"
#include "user_ctrl.h"
void beepInit()
{
    BEEP_LSICalibrationConfig(128000);
    BEEP_Init(BEEP_FREQUENCY_2KHZ);
    CLK_LSICmd(ENABLE); 
}
struct BEEP_INFOR beep_infor;
void beepCtrl()
{
    if(beep_infor.time==0)
    BEEP_Cmd(DISABLE); 
}