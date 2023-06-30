/**
  ******************************************************************************
  * @file     stm8s_it.c
  * @author   MCD Application Team
  * @version  V2.1.0
  * @date     18-November-2011
  * @brief    Main Interrupt Service Routines.
  *           This file provides template for all peripherals interrupt service 
  *           routine.
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
#include "stm8s_it.h"
extern uint32_t i;

extern uint8_t start_up;
/** @addtogroup Template_Project
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

#define DATA_GPIO   (GPIOC)
#define DATA_PIN    (GPIO_PIN_4)

#define CS_GPIO   (GPIOC)
#define CS_PIN    (GPIO_PIN_3)

#define TIME_WAIT_nS  0XfFFF
#define JUDGE_TIME    15//5S没通讯，重启cpu
#define JUDGE_TIME_NO_DEBUG    453//5S没通讯，重启cpu

#define GET_DATA()  (GPIO_ReadInputPin(DATA_GPIO,DATA_PIN)?1:0)

#define GET_CS()  (GPIO_ReadInputPin(CS_GPIO,CS_PIN)?1:0)

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint8_t   CsVal,DataVal;
uint8_t   CsValLast,DataValLast;
uint8_t   WdtEn=0;
uint16_t  PwmSta=0;
uint16_t  TimCnt=0;
uint8_t   CpuAlive=1;
uint32_t  CpuUnAliveCnt=0;
uint8_t   DebugMode=0;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/


void reset_cpu(void)
{ 
	unsigned short time;
   GPIO_WriteHigh(GPIOC,GPIO_PIN_5); 
   GPIO_WriteLow(GPIOC,GPIO_PIN_5);  
   for(time=TIME_WAIT_nS;time--;)  ;
   GPIO_WriteHigh(GPIOC,GPIO_PIN_5); 
}

void re_power_on(uint8_t times)
{
	uint32_t time;
  uint8_t i=0;
  GPIO_WriteHigh(GPIOA,GPIO_PIN_3);
  GPIO_WriteLow(GPIOA,GPIO_PIN_3);
  GPIO_WriteLow(GPIOA,GPIO_PIN_7);
  for(time=TIME_WAIT_nS;time--;)GPIO_WriteReverse(GPIOC, GPIO_PIN_6);//feed dog for mcu;
  GPIO_WriteHigh(GPIOA,GPIO_PIN_3);
  
  for( i=0;i<times;i++)
  {
    GPIO_WriteReverse(GPIOC, GPIO_PIN_7);
    for(time=TIME_WAIT_nS;time--;)GPIO_WriteReverse(GPIOC, GPIO_PIN_6);//feed dog for mcu;
  }

  for(time=TIME_WAIT_nS;time--;)GPIO_WriteReverse(GPIOC, GPIO_PIN_6);//feed dog for mcu;
  start_up=1;
}


void Cmd_Process(uint8_t new_cmd)
{
	unsigned int j,n;
    DebugMode = GPIO_ReadInputPin(GPIOD,GPIO_PIN_1)?1:0;
    DebugMode = GPIO_ReadInputPin(GPIOD,GPIO_PIN_4)?1:0;
//	if((0==DebugMode)&&new_cmd)
  //if(0)//==DebugMode)
  {
		//GPIO_WriteLow(GPIOC,GPIO_PIN_7);
    if(DebugMode == 0)
    {
      CpuUnAliveCnt = 0;
    }
    
    if(PwmSta>3)//=5 feed wdt
    {
      start_up=0;
      CpuUnAliveCnt = 0;
      if(PwmSta<10)
      {
        CpuAlive = 1;
      }
      if(PwmSta<20)//控制看门狗开始
      {
        CpuAlive = 1;
        WdtEn = 1;
      }
      else if(PwmSta<30)//控制看门狗结束
      {
        CpuAlive = 1;
        WdtEn = 0;
      }
      else if(PwmSta<40)//控制重启cpu
      {
        CpuAlive = 1;
        WdtEn = 0;
        reset_cpu();
      }
      else if(PwmSta<50)//控制整板重新上电
      {
        CpuAlive = 1;
        WdtEn = 0;
        re_power_on(7);
      }
      else if(PwmSta<60)     //开机
      {
        CpuAlive = 1;
        GPIO_WriteLow(GPIOD,GPIO_PIN_2);
        for(n=0;n<233;n++)
				{
					for(j=0;j<5000;j++);
          GPIO_WriteReverse(GPIOC, GPIO_PIN_6);//feed dog for mcu;
				}
        GPIO_WriteHigh(GPIOD,GPIO_PIN_2);
      }
      else if(PwmSta<70)
      {
        CpuAlive = 1;               //关机
        GPIO_WriteLow(GPIOD,GPIO_PIN_2);
				for(n=0;n<2330;n++)
				{
					for(j=0;j<5000;j++);
          GPIO_WriteReverse(GPIOC, GPIO_PIN_6);//feed dog for mcu;
				}
        GPIO_WriteHigh(GPIOD,GPIO_PIN_2);
      }
      
      PwmSta = 0;//执行完操作后，清零
    }
    
    if(WdtEn)
    {
      if(CpuUnAliveCnt>JUDGE_TIME)CpuAlive = 0;
      else CpuAlive = 1;
    }
    
    if(0==CpuAlive)
    {
      if(WdtEn)
      {
        WdtEn = 0;
				CpuUnAliveCnt = 0;
        re_power_on(5);
      }
    }
		if(CpuUnAliveCnt>JUDGE_TIME_NO_DEBUG)//如果非debug模式，通讯断了超过20s，不管使能不使能，都重新上电
    {
			CpuUnAliveCnt = 0;
			re_power_on(3);
		}
  }
//	else
//	{
//              //GPIO_WriteHigh(GPIOC,GPIO_PIN_7);
//	}
}
/* Public functions ----------------------------------------------------------*/

#ifdef _COSMIC_
/**
  * @brief Dummy Interrupt routine
  * @par Parameters:
  * None
  * @retval
  * None
*/
INTERRUPT_HANDLER(NonHandledInterrupt, 25)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
}
#endif /*_COSMIC_*/

/**
  * @brief TRAP Interrupt routine
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER_TRAP(TRAP_IRQHandler)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
}

/**
  * @brief Top Level Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(TLI_IRQHandler, 0)

{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
}

/**
  * @brief Auto Wake Up Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(AWU_IRQHandler, 1)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
}

/**
  * @brief Clock Controller Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(CLK_IRQHandler, 2)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
}

/**
  * @brief External Interrupt PORTA Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(EXTI_PORTA_IRQHandler, 3)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
}

/**
  * @brief External Interrupt PORTB Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(EXTI_PORTB_IRQHandler, 4)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
}

/**
  * @brief External Interrupt PORTC Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(EXTI_PORTC_IRQHandler, 5)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
}

/**
  * @brief External Interrupt PORTD Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(EXTI_PORTD_IRQHandler, 6)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
}

/**
  * @brief External Interrupt PORTE Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(EXTI_PORTE_IRQHandler, 7)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
}

#ifdef STM8S903
/**
  * @brief External Interrupt PORTF Interrupt routine.
  * @param  None
  * @retval None
  */
 INTERRUPT_HANDLER(EXTI_PORTF_IRQHandler, 8)
 {
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
 }
#endif /*STM8S903*/

#if defined (STM8S208) || defined (STM8AF52Ax)
/**
  * @brief CAN RX Interrupt routine.
  * @param  None
  * @retval None
  */
 INTERRUPT_HANDLER(CAN_RX_IRQHandler, 8)
 {
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
 }

/**
  * @brief CAN TX Interrupt routine.
  * @param  None
  * @retval None
  */
 INTERRUPT_HANDLER(CAN_TX_IRQHandler, 9)
 {
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
 }
#endif /*STM8S208 || STM8AF52Ax */

/**
  * @brief SPI Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(SPI_IRQHandler, 10)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
}

/**
  * @brief Timer1 Update/Overflow/Trigger/Break Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(TIM1_UPD_OVF_TRG_BRK_IRQHandler, 11)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
}

/**
  * @brief Timer1 Capture/Compare Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(TIM1_CAP_COM_IRQHandler, 12)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
}

#ifdef STM8S903
/**
  * @brief Timer5 Update/Overflow/Break/Trigger Interrupt routine.
  * @param  None
  * @retval None
  */
 INTERRUPT_HANDLER(TIM5_UPD_OVF_BRK_TRG_IRQHandler, 13)
 {
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
 }
 
/**
  * @brief Timer5 Capture/Compare Interrupt routine.
  * @param  None
  * @retval None
  */
 INTERRUPT_HANDLER(TIM5_CAP_COM_IRQHandler, 14)
 {
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
 }

#else /*STM8S208, STM8S207, STM8S105 or STM8S103 or STM8AF62Ax or STM8AF52Ax or STM8AF626x */
/**
  * @brief Timer2 Update/Overflow/Break Interrupt routine.
  * @param  None
  * @retval None
  */
 INTERRUPT_HANDLER(TIM2_UPD_OVF_BRK_IRQHandler, 13)
 {
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
 }

/**
  * @brief Timer2 Capture/Compare Interrupt routine.
  * @param  None
  * @retval None
  */
 INTERRUPT_HANDLER(TIM2_CAP_COM_IRQHandler, 14)
 {
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
 }
#endif /*STM8S903*/

#if defined (STM8S208) || defined(STM8S207) || defined(STM8S007) || defined(STM8S105) || \
    defined(STM8S005) ||  defined (STM8AF62Ax) || defined (STM8AF52Ax) || defined (STM8AF626x)
/**
  * @brief Timer3 Update/Overflow/Break Interrupt routine.
  * @param  None
  * @retval None
  */
 INTERRUPT_HANDLER(TIM3_UPD_OVF_BRK_IRQHandler, 15)
 {
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
 }

/**
  * @brief Timer3 Capture/Compare Interrupt routine.
  * @param  None
  * @retval None
  */
 INTERRUPT_HANDLER(TIM3_CAP_COM_IRQHandler, 16)
 {
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
 }
#endif /*STM8S208, STM8S207 or STM8S105 or STM8AF62Ax or STM8AF52Ax or STM8AF626x */

#if defined (STM8S208) || defined(STM8S207) || defined(STM8S007) || defined(STM8S103) || \
    defined(STM8S003) ||  defined (STM8AF62Ax) || defined (STM8AF52Ax) || defined (STM8S903)
/**
  * @brief UART1 TX Interrupt routine.
  * @param  None
  * @retval None
  */
 INTERRUPT_HANDLER(UART1_TX_IRQHandler, 17)
 {
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
 }

/**
  * @brief UART1 RX Interrupt routine.
  * @param  None
  * @retval None
  */
 INTERRUPT_HANDLER(UART1_RX_IRQHandler, 18)
 {
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
 }
#endif /*STM8S208 or STM8S207 or STM8S103 or STM8S903 or STM8AF62Ax or STM8AF52Ax */

/**
  * @brief I2C Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(I2C_IRQHandler, 19)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
}

#if defined(STM8S105) || defined(STM8S005) ||  defined (STM8AF626x)
/**
  * @brief UART2 TX interrupt routine.
  * @param  None
  * @retval None
  */
 INTERRUPT_HANDLER(UART2_TX_IRQHandler, 20)
 {
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
 }

/**
  * @brief UART2 RX interrupt routine.
  * @param  None
  * @retval None
  */
 INTERRUPT_HANDLER(UART2_RX_IRQHandler, 21)
 {
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
 }
#endif /* STM8S105 or STM8AF626x */

#if defined(STM8S207) || defined(STM8S007) || defined(STM8S208) || defined (STM8AF52Ax) || defined (STM8AF62Ax)
/**
  * @brief UART3 TX interrupt routine.
  * @param  None
  * @retval None
  */
 INTERRUPT_HANDLER(UART3_TX_IRQHandler, 20)
 {
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
 }

/**
  * @brief UART3 RX interrupt routine.
  * @param  None
  * @retval None
  */
 INTERRUPT_HANDLER(UART3_RX_IRQHandler, 21)
 {
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
 }
#endif /*STM8S208 or STM8S207 or STM8AF52Ax or STM8AF62Ax */

#if defined(STM8S207) || defined(STM8S007) || defined(STM8S208) || defined (STM8AF52Ax) || defined (STM8AF62Ax)
/**
  * @brief ADC2 interrupt routine.
  * @param  None
  * @retval None
  */
 INTERRUPT_HANDLER(ADC2_IRQHandler, 22)
 {
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
 }
#else /*STM8S105, STM8S103 or STM8S903 or STM8AF626x */
/**
  * @brief ADC1 interrupt routine.
  * @par Parameters:
  * None
  * @retval 
  * None
  */
 INTERRUPT_HANDLER(ADC1_IRQHandler, 22)
 {
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
 }
#endif /*STM8S208 or STM8S207 or STM8AF52Ax or STM8AF62Ax */

#ifdef STM8S903
/**
  * @brief Timer6 Update/Overflow/Trigger Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(TIM6_UPD_OVF_TRG_IRQHandler, 23)
 {
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
 }
#else /*STM8S208, STM8S207, STM8S105 or STM8S103 or STM8AF52Ax or STM8AF62Ax or STM8AF626x */
/**
  * @brief Timer4 Update/Overflow Interrupt routine.
  * @param  None
  * @retval None
  */
 INTERRUPT_HANDLER(TIM4_UPD_OVF_IRQHandler, 23)
 {
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
  static unsigned char work=0;
  static unsigned short pwm_val=0;
  static unsigned char new_cmd=0;
  CsVal = GET_CS();
  DataVal = GET_DATA();
  if(CsVal!=CsValLast)
  {
    if(CsValLast) //cs为低使能
    {
      work = 1;
    }
    else //cs 拉高后执行操作
    {
      work = 0;
      PwmSta = pwm_val;
      new_cmd = 1;
			pwm_val = 0;
    }
    CsValLast = CsVal;
  }
  if(DataVal!=DataValLast)
  {
    if(work)
    {
      if(DataValLast)
      {
        pwm_val++;
        if(0 == start_up)GPIO_WriteReverse(GPIOC, GPIO_PIN_7);
      }
    }
    else
    {
      pwm_val = 0;
    }
    DataValLast = DataVal;
  }
	i++;
	//if(i==61*128)//1秒
	//if(i==90*128)//1秒
  if(i==6776)//1秒
	{
    CpuUnAliveCnt++;//1s加一次，如果没有通讯超过1s，那么就断定有问题了，cpu那端要保证1s喂一次
    // if(0 == start_up)GPIO_WriteReverse(GPIOC, GPIO_PIN_7);
    i=0;
  }
  else if(i==61)GPIO_WriteReverse(GPIOC, GPIO_PIN_6);//feed dog for mcu
  
  
  
  
  

  Cmd_Process(new_cmd);

	TIM4_ClearITPendingBit(TIM4_IT_UPDATE);
  
	TIM4_Cmd(ENABLE);
 }
#endif /*STM8S903*/

/**
  * @brief Eeprom EEC Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(EEPROM_EEC_IRQHandler, 24)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
}

/**
  * @}
  */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/