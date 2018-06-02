#include "user_digitron.h"
#include "user_ctrl.h"
GPIO_TypeDef *DUAN_GPIO_PORT[] = {GPIOA, GPIOA, GPIOF, GPIOC, GPIOC, GPIOC, GPIOC, GPIOD};
GPIO_Pin_TypeDef DUAN_GPIO_PIN[] = {GPIO_PIN_1, GPIO_PIN_2, GPIO_PIN_4, GPIO_PIN_4, GPIO_PIN_5, GPIO_PIN_6, GPIO_PIN_7, GPIO_PIN_0}; //段码表
GPIO_TypeDef *WEI_GPIO_PORT[] = {GPIOD, GPIOD, GPIOD, GPIOE};
GPIO_Pin_TypeDef WEI_GPIO_PIN[] = {GPIO_PIN_2, GPIO_PIN_3, GPIO_PIN_7, GPIO_PIN_5}; //位码表
uint8_t DUAN_TABLE[] = {0xFC, 0x60, 0xDA, 0xF2, 0x66, 0xB6, 0xBE, 0xE0, 0xFE, 0xF6, 0x02};
volatile uint8_t TempData[3]; 
extern struct DISPLAY_CTRL display_ctrl;
void delay(uint16_t nCount)
{
  /* Decrement nCount value */
  while (nCount != 0)
  {   
    nCount--;
  }
}                                                             //临时缓冲区
void digitronInit()
{
  uint8_t i;
  for (i = 0; i < 8; i++)
  {
    GPIO_Init(DUAN_GPIO_PORT[i], (GPIO_Pin_TypeDef)DUAN_GPIO_PIN[i], GPIO_MODE_OUT_PP_LOW_SLOW); //GPIO_MODE_OUT_OD_HIZ_SLOW
  }
  for (i = 0; i < 4; i++)
  {
    GPIO_Init(WEI_GPIO_PORT[i], (GPIO_Pin_TypeDef)WEI_GPIO_PIN[i], GPIO_MODE_OUT_PP_LOW_FAST);
  }
}

void showNum(uint8_t num)
{
  uint8_t i;
  uint8_t duan = DUAN_TABLE[num];
  GPIO_WriteLow(WEI_GPIO_PORT[0], WEI_GPIO_PIN[0]);
  GPIO_WriteHigh(WEI_GPIO_PORT[1], WEI_GPIO_PIN[1]);
  GPIO_WriteHigh(WEI_GPIO_PORT[2], WEI_GPIO_PIN[2]);
  for (i = 0; i < 8; i++)
  {
    if (duan & 0x80)
    {
      GPIO_WriteHigh(DUAN_GPIO_PORT[i], DUAN_GPIO_PIN[i]);
    }
    else
    {
      GPIO_WriteLow(DUAN_GPIO_PORT[i], DUAN_GPIO_PIN[i]);
    }
    duan <<= 1;
  }
}

void selectWei(uint8_t num)
{
  switch(num)
  {
    case  0: 
    
    GPIO_WriteHigh(WEI_GPIO_PORT[1], WEI_GPIO_PIN[1]);
    GPIO_WriteHigh(WEI_GPIO_PORT[2], WEI_GPIO_PIN[2]);
    GPIO_WriteLow(WEI_GPIO_PORT[0], WEI_GPIO_PIN[0]); 
    break;
    case 1:
    GPIO_WriteHigh(WEI_GPIO_PORT[0], WEI_GPIO_PIN[0]); 
    
    GPIO_WriteHigh(WEI_GPIO_PORT[2], WEI_GPIO_PIN[2]); 
    GPIO_WriteLow(WEI_GPIO_PORT[1], WEI_GPIO_PIN[1]);
    break;
    case 2:
    GPIO_WriteHigh(WEI_GPIO_PORT[0], WEI_GPIO_PIN[0]); 
    GPIO_WriteHigh(WEI_GPIO_PORT[1], WEI_GPIO_PIN[1]);
    GPIO_WriteLow(WEI_GPIO_PORT[2], WEI_GPIO_PIN[2]); 
    break;
    default : break;
  }  
}
extern uint16_t  printf_counter;
extern struct KEYS_STATUE keys_statue;
void showPositionNum(uint8_t firstbit, uint8_t number)
{
  uint8_t wei_pos,j;
  uint8_t duan; 
  for (wei_pos = 0; wei_pos < number; wei_pos++)
  {
        duan=TempData[wei_pos];
        if(keys_statue.counter%2==0||keys_statue.counter>10)//为防止加减的时候会闪，加入>20这样就可以防止两次按键中间出现闪烁现象。
        {
              for(j=0;j<8;j++)
            {
              if (duan & 0x80)
              {
                GPIO_WriteHigh(DUAN_GPIO_PORT[j], DUAN_GPIO_PIN[j]);
              }
              else
              {
                GPIO_WriteLow(DUAN_GPIO_PORT[j], DUAN_GPIO_PIN[j]);
              }
              duan <<= 1;
            }
        }
        else
        {
          for(j=0;j<8;j++)
            {
                GPIO_WriteLow(DUAN_GPIO_PORT[j], DUAN_GPIO_PIN[j]);
            }
        }
        selectWei(wei_pos);
        
        delay(500);
        duan=0;
        for(j=0;j<8;j++)
        {
            GPIO_WriteLow(DUAN_GPIO_PORT[j], DUAN_GPIO_PIN[j]);
        }
        GPIO_WriteHigh(WEI_GPIO_PORT[0], WEI_GPIO_PIN[0]); 
        GPIO_WriteHigh(WEI_GPIO_PORT[1], WEI_GPIO_PIN[1]);
        GPIO_WriteHigh(WEI_GPIO_PORT[2], WEI_GPIO_PIN[2]);
  }
}
