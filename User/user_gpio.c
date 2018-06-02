#include "user_gpio.h"
#include "stm8s_exti.h"
//#include "stm8s.h"
void gpioInit()
{
    //按键
    GPIO_Init(KEY1_PORT, (GPIO_Pin_TypeDef)KEY1_PIN, GPIO_MODE_IN_PU_NO_IT); //
    GPIO_Init(KEY2_PORT, (GPIO_Pin_TypeDef)KEY2_PIN, GPIO_MODE_IN_PU_NO_IT); //
    GPIO_Init(KEY3_PORT, (GPIO_Pin_TypeDef)KEY3_PIN, GPIO_MODE_IN_PU_NO_IT); //
    
    //继电器
    GPIO_Init(REALY_PORT, (GPIO_Pin_TypeDef)REALY_PIN, GPIO_MODE_OUT_PP_LOW_FAST); //
    //中断输入
    GPIO_Init(INT1_GPIO, (GPIO_Pin_TypeDef)INT1_PIN, GPIO_MODE_IN_PU_IT); //
    GPIO_Init(INT2_GPIO, (GPIO_Pin_TypeDef)INT2_PIN, GPIO_MODE_IN_PU_NO_IT); //
    EXTI_SetExtIntSensitivity(EXTI_PORT_GPIOB,EXTI_SENSITIVITY_RISE_FALL);
    GPIO_WriteHigh(INT1_GPIO, (GPIO_Pin_TypeDef)INT1_PIN);
    GPIO_WriteHigh(INT2_GPIO, (GPIO_Pin_TypeDef)INT2_PIN);
}