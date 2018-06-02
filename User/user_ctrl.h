#ifndef __USER_CTRL_H
#define __USER_CTRL_H
#include "stm8s.h"
//EEPROM地址分配
#define MAGIC_NUM          0x2BAA2B2B//0xAAAA2B2B
#define ADDRESS_OFFSET      (0x04)

#define OF_MAGIC_NUM        (0)
#define OF_MAGIC_NUM_LEN    (4)
#define OF_EEPROM_ADDRESS   (0x4000)
#define OF_FUNCTION_PARAM        (OF_MAGIC_NUM+OF_MAGIC_NUM_LEN)
//默认参数
#define DEFAULT_P1 0
#define DEFAULT_P2 0
#define DEFAULT_SWITCH  ENABLE
#define DEFAULT_CTRL DISABLE
#define DEFAULT_BEEP_ON_OFF ENABLE
#define DEFAULT_DELAY_TIME_ON 20
#define DEFAULT_DELAY_TIME_OFF 20



typedef volatile unsigned char    OS_CPU_SR;
//#define  OS_ENTER_CRITICAL()  do{disableInterrupts();}while(0)
//#define  OS_EXIT_CRITICAL()   do{if(cpu_sr){ disableInterrupts();}else{ enableInterrupts(); }}while(0)
//#define  OS_CRITICAL_METHOD   3

#define  OS_CRITICAL_METHOD   3
#define  OS_ENTER_CRITICAL()  do{cpu_sr = ITC_GetSoftIntStatus() & 0x08;disableInterrupts();}while(0)
#define  OS_EXIT_CRITICAL()   do{if(cpu_sr){ disableInterrupts();}else{ enableInterrupts(); }}while(0)

#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))
#define set_bit(x, bit) 	do{enableInterrupts();(x) |= 1 << (bit);disableInterrupts();}while(0)
#define reset_bit(x, bit) 	do{enableInterrupts();(x) &= ~(1 << (bit));disableInterrupts();}while(0)
#define is_bit_set(x, bit) 	((x) & (1 << (bit)))

#define DATA_SEG @near
extern uint16_t task_monitor;

#define notify(task_bit)		set_bit(task_monitor, task_bit)
#define is_task_set(task_bit)	is_bit_set(task_monitor, task_bit)
#define is_task_always_alive(flags)	(flags & ALWAYS_ALIVE)//ALWAYS_ALIVE = 0x01
#define reset_task(task_bit) 	reset_bit(task_monitor, task_bit)
enum
{
    EV_CLRDOG,
    EV_RXCHAR,
    EV_REPORT,
    EV_20MS,
    EV_100MS,
    EV_SEC,
    EV_MIN,
    EV_KEY,
    EV_TICK,
    EV_STATE,
    EV_TOUCH_SCAN
};
enum
{
    ALWAYS_ALIVE = 0x01
};
struct task
{
    uint8_t id;
    uint8_t flags;
    void *args;
    void (*handle)(void *args);
};

struct FUNCTION_PARAM
{
  uint8_t mode_p1;              //模式1编号
  uint8_t mode_p2;              //模式2编号
  uint8_t switch_;              //模块开关
  uint8_t ctrl;                 //开关控制--手动控制阀门开或关。
  uint8_t beep_on_off;          //蜂鸣器开关
  uint16_t delay_time_on;       //延时开时间
  uint16_t delay_time_off;      //延时关时间
  uint8_t beep;
  
};
#define SIZE_FUNCTION_PARAM sizeof(struct FUNCTION_PARAM)

struct AUTO_ADD_DEC
{
  uint8_t auto_flag;
  uint8_t counter;
  uint8_t add_flag;
  uint8_t dec_flag;
};
struct KEYS_STATUE
{
  uint8_t key1;
  uint8_t key2;
  uint8_t key3;
  uint8_t counter;
};
enum KEY1_STATUE
{
  NO_SET,
  SET_DELAY_TIME_OFF,
};
enum KEY2_STATUE
{
  ADD=1,
};
enum KEY3_STATUE
{
  DEC=1,
};
struct DISPLAY_CTRL
{
  uint8_t mode;
  uint8_t counter;
};
struct INTERRUPT_STATUE
{
    uint8_t int1_rise_counter;
    uint8_t int1_statue;
    uint8_t int1_full_counter;
    uint16_t int1_delay_counter;
};
struct DELAY_STATUE
{
    uint8_t start_stop;
    uint16_t int1_delay_counter;
};
enum INT1_STATUE
{
    NO_INPUT,
    LOWING,
    HIGHING,
};
void keyInit(void);
void EEP_Read(uint8_t addr, uint8_t buf[], uint8_t len);
void EEP_Write(uint8_t addr, uint8_t buf[], uint8_t len);
void systemInit();
void sysTick(void *args);
void taskHandle(void);
void realyCtrl();
#endif