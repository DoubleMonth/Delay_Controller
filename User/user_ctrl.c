#include "user_ctrl.h"
#include <stdlib.h>
#include "user_uart.h"
#include "user_digitron.h"
#include "user_beep.h"
#include "user_key.h"
#include "user_gpio.h"
uint16_t task_monitor = 0;
extern volatile uint8_t TempData[3]; 
extern uint8_t DUAN_TABLE[11] ;
extern struct BEEP_INFOR beep_infor;
struct FUNCTION_PARAM function_param;
struct INTERRUPT_STATUE interrupt_statue;
struct DELAY_STATUE delay_statue;
void EEP_Read(uint8_t addr, uint8_t buf[], uint8_t len)
{
    uint8_t i;
    FLASH_Unlock(FLASH_MEMTYPE_DATA);
    for(i = 0; i < len; i++)
    {
            buf[i] = FLASH_ReadByte(OF_EEPROM_ADDRESS+addr+i);
    }
    FLASH_Lock(FLASH_MEMTYPE_DATA);
}

void EEP_Write(uint8_t addr, uint8_t buf[], uint8_t len)
{
	uint8_t i;

	FLASH_Unlock(FLASH_MEMTYPE_DATA);
	for (i = 0; i < len; i++) 
	{
            FLASH_ProgramByte(OF_EEPROM_ADDRESS+addr+i,buf[i]);
	}
	FLASH_Lock(FLASH_MEMTYPE_DATA);
}

void systemInit()
{
  uint32_t magic;
  EEP_Read(OF_MAGIC_NUM, (uint8_t *)&magic, OF_MAGIC_NUM_LEN);
   if (MAGIC_NUM != magic)
   {
      function_param.mode_p1=DEFAULT_P1;
      function_param.mode_p2=DEFAULT_P2;
      function_param.switch_=DEFAULT_SWITCH;
      function_param.ctrl=DEFAULT_CTRL;
      function_param.beep_on_off=DEFAULT_BEEP_ON_OFF;
      function_param.delay_time_on=DEFAULT_DELAY_TIME_ON;
      function_param.delay_time_off=DEFAULT_DELAY_TIME_OFF;
      function_param.beep=ENABLE;
      EEP_Write(OF_FUNCTION_PARAM, (uint8_t *)&function_param, SIZE_FUNCTION_PARAM);
      magic = MAGIC_NUM;
      EEP_Write(OF_MAGIC_NUM, (uint8_t *)&magic, OF_MAGIC_NUM_LEN);
   }
   EEP_Read(OF_FUNCTION_PARAM, (uint8_t *)&function_param, sizeof(function_param));
}

#define GPIO_KEY_NUM 3 ///< Defines the total number of key member
keyTypedef_t singleKey[GPIO_KEY_NUM]; ///< Defines a single key member array pointer
keysTypedef_t keys; 
uint8_t eep_write_flag;
struct AUTO_ADD_DEC auto_add_dec;
struct KEYS_STATUE keys_statue;
struct DISPLAY_CTRL display_ctrl;
void key1ShortPress(void)
{
    printf("key1ShortPressed!\r\n");
    if(SET_DELAY_TIME_OFF==keys_statue.key1)
    {
        return ;
    }
    if(function_param.beep)
    {
        BEEP_Cmd(ENABLE); 
        beep_infor.time=2;
    }
    else
    {
        BEEP_Cmd(DISABLE); 
        beep_infor.time=0;
    }
    function_param.switch_^=0x01;
    if(ENABLE==function_param.switch_)//当开机时，继电器控制开关 b关闭
    function_param.ctrl=DISABLE;
    printf("function_param.switch_=%d!\r\n",function_param.switch_);
    EEP_Write(OF_FUNCTION_PARAM, (uint8_t *)&function_param, SIZE_FUNCTION_PARAM);//写入EEPROM，
    
}

void key1LongPress(void)
{
    printf("key1LongPressed!\r\n");
    if(DISABLE==function_param.switch_)
        return ;
    if(function_param.beep)
    {
        BEEP_Cmd(ENABLE); 
        beep_infor.time=2;
    }
    else
    {
        BEEP_Cmd(DISABLE); 
        beep_infor.time=0;
    }
    keys_statue.key1++;
    if(SET_DELAY_TIME_OFF==keys_statue.key1)
    {
        keys_statue.counter=11;                //设置模式有11秒钟，多了1秒是为了防止两次按键间隔的时候闪烁。
    }
    if(keys_statue.key1>SET_DELAY_TIME_OFF)      //目前只有设置延时关闭时间
    {
        keys_statue.key1=NO_SET;           //
        keys_statue.counter=0;
    } 
}

void key2ShortPress(void)
{
    printf("key2ShortPressed!\r\n");
    if(SET_DELAY_TIME_OFF==keys_statue.key1)
    {
        keys_statue.counter=11;    
        if(++function_param.delay_time_off>999)
            function_param.delay_time_off=0;
        EEP_Write(OF_FUNCTION_PARAM, (uint8_t *)&function_param, SIZE_FUNCTION_PARAM);//写入EEPROM， 
    }
    else
    {
        keys_statue.counter=0;
    }
    if(function_param.switch_==DISABLE)
    {
        function_param.ctrl^=0x01;
        if(function_param.beep)
        {
            BEEP_Cmd(ENABLE); 
            beep_infor.time=2;
        }
        else
        {
            BEEP_Cmd(DISABLE); 
            beep_infor.time=0;
        }
    }
}
void key2LongPress(void)
{
    printf("key2LongPressed!\r\n");
    if(SET_DELAY_TIME_OFF==keys_statue.key1)
    {
        return ;
    }
    if(function_param.beep)
    {
        BEEP_Cmd(ENABLE); 
        beep_infor.time=2;
    }
    else
    {
        BEEP_Cmd(DISABLE); 
        beep_infor.time=0;
    }
    function_param.ctrl^=0x01;
    if(function_param.ctrl==ENABLE)
    {
        function_param.switch_=DISABLE;
    }
    printf("function_param.ctrl=%d!\r\n",function_param.ctrl);
    EEP_Write(OF_FUNCTION_PARAM, (uint8_t *)&function_param, SIZE_FUNCTION_PARAM);//写入EEPROM， 
}
void key3ShortPress(void)
{
    printf("key3ShortPressed!\r\n");
    if(SET_DELAY_TIME_OFF==keys_statue.key1)
    {
        keys_statue.counter=11;    
        if(--function_param.delay_time_off<1)
            function_param.delay_time_off=999;
        EEP_Write(OF_FUNCTION_PARAM, (uint8_t *)&function_param, SIZE_FUNCTION_PARAM);//写入EEPROM， 
    }
    else
    {
        keys_statue.counter=0;
    }
}
void key3LongPress(void)
{
    printf("key3LongPressed!\r\n");
    if(SET_DELAY_TIME_OFF==keys_statue.key1)
    {
        return ;
    }
    function_param.beep^=0x01;            //蜂鸣器开关反转
    printf("beep_infor.on_off=%d!\r\n",function_param.beep);
    if(function_param.beep)               //当蜂鸣器开时，长响一声
    {
        BEEP_Cmd(ENABLE); 
        beep_infor.time=3;
    }
    else                                //当蜂鸣器开时，短响一声
    {
        function_param.beep=ENABLE;       //因为蜂鸣器关闭了，所以先打开一个，响完后再关闭
        BEEP_Cmd(ENABLE); 
        beep_infor.time=2;
        function_param.beep=DISABLE;      //关闭蜂鸣器响动
    }
    
    EEP_Write(OF_FUNCTION_PARAM, (uint8_t *)&function_param, SIZE_FUNCTION_PARAM);//写入EEPROM，
}
void keyInit(void)
{
    singleKey[0] = keyInitOne(KEY1_PORT, KEY1_PIN, key1ShortPress, key1LongPress);
    singleKey[1] = keyInitOne(KEY2_PORT, KEY2_PIN, key2ShortPress, key2LongPress);
    singleKey[2] = keyInitOne(KEY3_PORT, KEY3_PIN, key3ShortPress, key3LongPress);
    keys.singleKey = (keyTypedef_t *)&singleKey;
    keyParaInit(&keys); 
}


uint8_t ITC_GetCPUCC(void)
{
#ifdef _COSMIC_
  _asm("push cc");
  _asm("pop a");
  return; /* Ignore compiler warning, the returned value is in A register */
#elif defined _RAISONANCE_ /* _RAISONANCE_ */
  return _getCC_();
#else /* _IAR_ */
  asm("push cc");
  asm("pop a"); /* Ignore compiler warning, the returned value is in A register */
#endif /* _COSMIC_*/
}

uint8_t ITC_GetSoftIntStatus(void)
{
    return(u8)(ITC_GetCPUCC() & CPU_CC_I1I0);
}
volatile uint16_t  __sys_tick_1 = 0;
extern volatile uint16_t __sys_tick;
void sysTick(void *args)
{
      static volatile uint16_t last_tick;
      static volatile uint16_t last_tick_1=0; 
      uint16_t deltms;
      static volatile uint8_t _msec = 0xff,tick_cnt = 0;
      static volatile uint8_t _sec = 0;

    if (0xff == _msec) 
    {
        _msec = 0;
        last_tick = __sys_tick;
        last_tick_1 = __sys_tick_1;
        return;
    }
    
    deltms = __sys_tick - last_tick;
    if(deltms >= 20)
    {
        last_tick += 20;
        notify(EV_20MS);
        if (++tick_cnt < 5) return; 
        tick_cnt = 0;
        notify(EV_100MS);        
        if(++_msec < 10) return;
        _msec = 0; 
        notify(EV_SEC);
        if(++_sec < 60) return;
        _sec = 0;
        notify(EV_MIN);
    }
}
void interrupt_handle()
{
    if(interrupt_statue.int1_full_counter>1)
       interrupt_statue.int1_full_counter--;
    if((interrupt_statue.int1_full_counter==1)&&(!GPIO_ReadInputPin(INT1_GPIO,INT1_PIN)))//滤波后是低电平
    {
        interrupt_statue.int1_statue=LOWING;            //低电平状态
        GPIO_WriteHigh(REALY_PORT,REALY_PIN);             //打开继电器
        delay_statue.start_stop=DISABLE;
        delay_statue.int1_delay_counter=0;
        
    }
    if(interrupt_statue.int1_rise_counter>1)
        interrupt_statue.int1_rise_counter--;
    if((interrupt_statue.int1_rise_counter==1)&&(GPIO_ReadInputPin(INT1_GPIO,INT1_PIN)))//滤波后是高电平
    {
        interrupt_statue.int1_statue=HIGHING;            //低电平状态
        delay_statue.start_stop=ENABLE;
        delay_statue.int1_delay_counter=0;
        interrupt_statue.int1_rise_counter=0;
        interrupt_statue.int1_full_counter=0;
    }
}
void task20ms(void *args)
{
   interrupt_handle();
}
void task100ms(void *args)
{

    if(beep_infor.time>0)
        beep_infor.time--;
    beepCtrl(); 
    if(SET_DELAY_TIME_OFF==keys_statue.key1)
    {
        TempData[0]=DUAN_TABLE[function_param.delay_time_off/100];
        TempData[1]=DUAN_TABLE[function_param.delay_time_off%100/10];
        TempData[2]=DUAN_TABLE[function_param.delay_time_off%10];	
    }
    else if((DISABLE==function_param.switch_)&&(DISABLE==function_param.ctrl))
    {
        TempData[0]=DUAN_TABLE[10];
        TempData[1]=DUAN_TABLE[10];
        TempData[2]=DUAN_TABLE[10];
        delay_statue.int1_delay_counter=function_param.delay_time_off;
    }
    else if((ENABLE==function_param.switch_)&&(DISABLE==function_param.ctrl))
    {
        TempData[0]=DUAN_TABLE[delay_statue.int1_delay_counter/100];
        TempData[1]=DUAN_TABLE[delay_statue.int1_delay_counter%100/10];
        TempData[2]=DUAN_TABLE[delay_statue.int1_delay_counter%10];
        if((GPIO_ReadOutputData(GPIOB)&0x02)==0x02)
            TempData[2]=DUAN_TABLE[delay_statue.int1_delay_counter%10]|0x01;
        else
            TempData[2]=DUAN_TABLE[delay_statue.int1_delay_counter%10];
    }
    realyCtrl();
}
void realyCtrl()
{
    if((DISABLE==function_param.switch_)&&(ENABLE==function_param.ctrl))
    {
        GPIO_WriteHigh(REALY_PORT,REALY_PIN);             //打开继电器
    }
    else if((DISABLE==function_param.switch_)&&(DISABLE==function_param.ctrl))
    {
        GPIO_WriteLow(REALY_PORT,REALY_PIN);             //关闭继电器
    }
}
void taskSec(void *args)
{
    static uint8_t counter=0x80;
    
    if((0==function_param.switch_)&&(1==function_param.ctrl))
    {
        TempData[0]=counter;
        TempData[1]=counter;
        TempData[2]=counter;
        delay_statue.int1_delay_counter=function_param.delay_time_off;
    }
    else
        counter=0x80;
    counter>>=1;
    if(counter<0x04)
    {
        counter=0x80;
    }
    if(SET_DELAY_TIME_OFF==keys_statue.key1&&keys_statue.counter>0)
    {
        keys_statue.counter--;
    }
    if((delay_statue.start_stop==ENABLE)&&(delay_statue.int1_delay_counter<function_param.delay_time_off))
    {
        delay_statue.int1_delay_counter++;
    }
    else
    {
        if(!GPIO_ReadInputPin(INT1_GPIO,INT1_PIN))
        {
            delay_statue.int1_delay_counter=0;
        }
        else 
        {
            delay_statue.int1_delay_counter=function_param.delay_time_off;
        }
    }
    if((delay_statue.int1_delay_counter>=function_param.delay_time_off)&&(ENABLE==function_param.switch_))
    {
        
        delay_statue.int1_delay_counter=function_param.delay_time_off;
        delay_statue.start_stop=DISABLE;
        if(GPIO_ReadInputPin(REALY_PORT,REALY_PIN))
        {
            if(function_param.beep)
            {
                BEEP_Cmd(ENABLE); 
                beep_infor.time=3;
            }
            else
            {
                BEEP_Cmd(DISABLE); 
                beep_infor.time=0;
            }
        }
        GPIO_WriteLow(REALY_PORT,REALY_PIN);             //关闭继电器
        interrupt_statue.int1_rise_counter=0;
        interrupt_statue.int1_full_counter=0;
    }
}
const struct task tasks[] =
{
   { EV_20MS,   0,            NULL,  task20ms },
   { EV_100MS,  0,            NULL,  task100ms },
   { EV_SEC,    0,            NULL,  taskSec },
   { EV_TICK,   ALWAYS_ALIVE, NULL,  sysTick },
};

void taskHandle(void)
{
   uint8_t i;

   for (i = 0; i < ARRAY_SIZE(tasks); ++i)
   {
      if ((is_task_set(tasks[i].id))
          || (is_task_always_alive(tasks[i].flags)))
      {
         reset_task(tasks[i].id);
         tasks[i].handle(tasks[i].args);
      }
   }
}

