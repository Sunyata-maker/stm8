/**
  ******************************************************************************
  * @file    Project/main.c 
  * @author  MCD Application Team
  * @version V2.1.0
  * @date    18-November-2011
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */ 


/* Includes ------------------------------------------------------------------*/
#include "stm8s.h"
#include "stm8s_flash.h"
/* Private defines -----------------------------------------------------------*/
uint32_t i=0;
uint8_t start_up=1;
/* Private function prototypes -----------------------------------------------*/
extern void Cmd_Process(void);
/* Private functions ---------------------------------------------------------*/
void Init_GPIO(void)
{
        GPIO_Init(GPIOD, GPIO_PIN_4, GPIO_MODE_IN_FL_NO_IT);
	GPIO_Init(GPIOC, GPIO_PIN_6, GPIO_MODE_OUT_PP_LOW_FAST);//mcu wdt dog feed pin
	GPIO_Init(GPIOC, GPIO_PIN_7, GPIO_MODE_OUT_PP_LOW_FAST);//sta led
	GPIO_Init(GPIOC, GPIO_PIN_4, GPIO_MODE_IN_FL_NO_IT);//data
	GPIO_Init(GPIOC, GPIO_PIN_3, GPIO_MODE_IN_FL_NO_IT);//cs
	GPIO_Init(GPIOC, GPIO_PIN_5, GPIO_MODE_IN_FL_NO_IT);//reset cpu
	GPIO_Init(GPIOA, GPIO_PIN_3, GPIO_MODE_OUT_PP_LOW_FAST);//pwr on board
	GPIO_Init(GPIOD, GPIO_PIN_1, GPIO_MODE_OUT_PP_LOW_FAST);//SWIM pin for debug
	
}
void Init_Timer4(void)
{
	/*TIM4_UpdateDisableConfig(ENABLE);//��������¼�
	TIM4_ARRPreloadConfig(ENABLE);//�Զ���װ
	TIM4_ITConfig(TIM4_IT_UPDATE, ENABLE);//�ж����ã������ж�
	TIM4_SetCounter(0xff);//��������ֵ
	TIM4_SetAutoreload(0xFF);//�������Զ���װ�ĳ�ֵ
	TIM4_PrescalerConfig(TIM4_PRESCALER_128, TIM4_PSCRELOADMODE_UPDATE);//Ԥ��Ƶֵ
	*/
	TIM4_TimeBaseInit(TIM4_PRESCALER_4, 0xff);
  /* Clear TIM4 update flag */
  TIM4_ClearFlag(TIM4_FLAG_UPDATE);
	
	 /* Enable update interrupt */
	TIM4_ITConfig(TIM4_IT_UPDATE, ENABLE);

  TIM4_SelectOnePulseMode(TIM4_OPMODE_SINGLE);
	TIM4_Cmd(ENABLE);
  // Cmd_Process();
}

void main(void)
{
  unsigned int ii,j=0;
  
  CLK_HSIPrescalerConfig(CLK_PRESCALER_HSIDIV1);
	Init_GPIO();
	 GPIO_WriteHigh(GPIOC,GPIO_PIN_5);
	GPIO_WriteHigh(GPIOA,GPIO_PIN_3);
	Init_Timer4();
	for(ii=0;ii<25;ii++)
	{
		for(j=0;j<50000;j++);
		GPIO_WriteReverse(GPIOC, GPIO_PIN_7);
		GPIO_WriteReverse(GPIOC, GPIO_PIN_6);
	}
	enableInterrupts();
	ii=0;
	// start_up=0;
	/* Infinite loop */
  while (1)
  {
      Cmd_Process();
      GPIO_WriteHigh(GPIOD,GPIO_PIN_2);
       GPIO_WriteHigh(GPIOC,GPIO_PIN_7);
      for(j=0;j<8000;j++);
      GPIO_WriteLow(GPIOD,GPIO_PIN_2);
      // GPIO_WriteLow(GPIOC,GPIO_PIN_7);
      for(j=0;j<8000;j++);
  }
  
}

#ifdef USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param file: pointer to the source file name
  * @param line: assert_param error line source number
  * @retval : None
  */
void assert_failed(u8* file, u32 line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
