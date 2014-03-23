/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.4.0
  * @date    10/15/2010
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2010 STMicroelectronics</center></h2>
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "I2CRoutines.h"
#include "main.h"
#include "batteryFunctions.h"

/** @addtogroup STM32F10x_StdPeriph_Template
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define PC_OFF_TIMEOUT 15
#define PC_ON_TIMEOUT 5
//#define SLAVE_DMA_USE
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern __IO uint32_t ADC_DualConvertedValueTab[3];
extern uint16_t * ADC_ConvertedValueTabPointer;

extern StateStructure state;

static __IO uint32_t TimingDelay;

uint8_t TurnOffDelyTime;
uint8_t TurnOnDelyTime;
uint8_t NoCurrentTimeout;
uint8_t BatteryKoTimeout;

bool goingOn=false;
bool goingOff=false;
bool sendPcPulse=false;
bool pcPulseStarted=false;
bool noCurrentFlag=false;
bool batteryKoFlag=false;

ErrorStatus HSEStartUpStatus;


__IO uint8_t Tx_Idx1=0, Rx_Idx1=0;
extern __IO uint32_t NumbOfBytes1;
extern uint8_t Buffer_Rx1[];
extern uint8_t Buffer_Tx1[];

extern __IO uint32_t I2CDirection ;
extern uint8_t Address;

extern bool pcOn;

extern uint8_t percent;


/* Private function prototypes -----------------------------------------------*/
uint8_t getStateId(StateStructure state) {
    uint8_t stateId=state.regulatedOutputState + (state.unregulatedOutputState<<1) + (state.externalPowerSourceState<<2) + (state.batteryState<<3);
    return stateId;
}

#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

/**
  * @brief  Configures system clock after wake-up from STOP: enable HSE, PLL
  *   and select PLL as system clock source.
  * @param  None
  * @retval None
  */
void SYSCLKConfig_STOP(void)
{
  /* Enable HSE */
  RCC_HSEConfig(RCC_HSE_ON);

  /* Wait till HSE is ready */
  HSEStartUpStatus = RCC_WaitForHSEStartUp();

  if(HSEStartUpStatus == SUCCESS)
  {

#ifdef STM32F10X_CL
    /* Enable PLL2 */ 
    RCC_PLL2Cmd(ENABLE);

    /* Wait till PLL2 is ready */
    while(RCC_GetFlagStatus(RCC_FLAG_PLL2RDY) == RESET)
    {
    }
#endif

    /* Enable PLL */ 
    RCC_PLLCmd(ENABLE);

    /* Wait till PLL is ready */
    while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
    {
    }

    /* Select PLL as system clock source */
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

    /* Wait till PLL is used as system clock source */
    while(RCC_GetSYSCLKSource() != 0x08)
    {
    }
  }
}

FunctionalState TIM_State(TIM_TypeDef* TIMx);
/* Private functions ---------------------------------------------------------*/

#ifdef DEBUG
/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART */
  USART_SendData(USART1, (uint8_t) ch);

  /* Loop until the end of transmission */
  while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
  {}

  return ch;
}
#endif

/**
  * @brief  Check the specified TIM peripheral.
  * @param  TIMx: where x can be 1 to 17 to select the TIMx peripheral.
  * @retval State: the state of the TIMx peripheral.
  *   This value can be: ENABLE or DISABLE.
  */
FunctionalState TIM_State(TIM_TypeDef* TIMx)
{
  /* Check the parameters */
  assert_param(IS_TIM_ALL_PERIPH(TIMx));
  
  if ((TIMx->CR1 && TIM_CR1_CEN) != 0)
  {
    /* TIM Counter Enabled */
    return ENABLE;
  }
  else
  {
    /* TIM Counter Disabled */
    return DISABLE;
  }
}

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief   This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  
  while (1)
  {
  }
  
  //NVIC_SystemReset();
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  
  while (1)
  {
  }
  
  //NVIC_SystemReset();
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  
  while (1)
  {
  }
  
  //NVIC_SystemReset();
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  
  while (1)
  {
  }
  
  //NVIC_SystemReset();
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
bool buttonPressed=false;
uint16_t timePressed=0;
#define TIME_TO_RESET 3000 //3 sec
void SysTick_Handler(void)
{
  TimingDelay_Decrement();
  //Check button for reset
  if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_10)==Bit_RESET)
    timePressed++;
  else
    timePressed=0;
  if(timePressed==100)
  {
    //throw push button exception
    EXTI_GenerateSWInterrupt(EXTI_Line14);
  }
  if(timePressed==TIME_TO_RESET)
    //Reset
    NVIC_SystemReset();
    
}

/******************************************************************************/
/*            STM32F10x Peripherals Interrupt Handlers                        */
/******************************************************************************/

void EXTI9_5_IRQHandler(void)
{
  if (EXTI_GetITStatus(EXTI_Line6) != RESET)
  {
    //Battery to KO status
    
#ifdef DEBUG_INFO_USART
    printf("Battery to KO ");
#endif
    
    /* Execute actions asociated */
    //Send "Turn OFF PC" signal
    sendPcPulse=true;
    //Start timer to turn off the PC after PC_OFF_TIMEOUT seconds
    TurnOffDelyTime=PC_OFF_TIMEOUT;
    goingOff=true;
    /* Change to the new state */
    state.batteryState=KO;
    
#ifdef DEBUG_INFO_USART
    printf("Status number %d\n", getStateId(state));
#endif
    
    EXTI_ClearITPendingBit(EXTI_Line6);
  }
  else if (EXTI_GetITStatus(EXTI_Line7) != RESET)
  {
    //Battery charged
    
#ifdef DEBUG_INFO_USART
    printf("Battery charged ");
#endif
    
    /* Execute actions asociated */
    /* Change to the new state */
    state.batteryState=CHARGED;
    
#ifdef DEBUG_INFO_USART
    printf("Status number %d\n", getStateId(state));
#endif
    
    TIM_SetCompare3(TIM3,1140);
    
    EXTI_ClearITPendingBit(EXTI_Line7);
  }
  else if (EXTI_GetITStatus(EXTI_Line8) != RESET)
  {
    //Battery to CV charging state
    
#ifdef DEBUG_INFO_USART
    printf("Battery to charging at CV ");
#endif
    
    /* Execute actions asociated */
    /* Change to the new state */
    state.batteryState=CHARGING_CV;
    
#ifdef DEBUG_INFO_USART
    printf("Status number %d\n", getStateId(state));
#endif
    
    TIM_SetCompare3(TIM3,760);
    
    EXTI_ClearITPendingBit(EXTI_Line8);
  }
  else if (EXTI_GetITStatus(EXTI_Line9) != RESET)
  {
    //No current at Regulated output
    
#ifdef DEBUG_INFO_USART
    printf("No load at Regulated Output ");
#endif
    pcOn=false;
    state.unregulatedOutputState=NOT_ACTIVE;
    GPIO_ResetBits(GPIOB,GPIO_Pin_14);
    //Apago alimentacion PC
    //Espero 3seg
    //Enciendo alimentacion PC
    
    if (state.externalPowerSourceState==DISCONNECTED)
    {
      /* Execute actions asociated */
      /* Turn UnRegulatedOutput off */
      //TurnPowerExternal(); /* PW_CTR to disabled state */
      TurnPowerOff();
      goingOff=false;
      //It is possible to change the led status
      /* Change to the new state */
      /* Turn RegulatedOutputDC off */
      state.regulatedOutputState=OFF; /* The PWM control signal goes to off state afterwards, in the main loop */
      
  #ifdef DEBUG_INFO_USART
      printf("Status number %d\n", getStateId(state));
  #endif
      
    
      /* Disable Power Lines */
#ifdef TR_INVERSE
      GPIO_SetBits(GPIOA,GPIO_Pin_8);
#else
      GPIO_ResetBits(GPIOA,GPIO_Pin_8);
#endif
      GPIO_ResetBits(GPIOA,GPIO_Pin_11);
      /* Disabel devices */
      TIM_SetCompare3(TIM3,0);
      /* Enter STOP mode */
      
#ifdef DEBUG_INFO_USART
      printf("Going to STOP mode\n");
#endif
      
      EXTI_ClearITPendingBit(EXTI_Line9);
    
#ifndef DEBUG_NO_STOP
      PWR_EnterSTOPMode(PWR_Regulator_LowPower, PWR_STOPEntry_WFI);
#endif
    }
    else
    {
      EXTI_ClearITPendingBit(EXTI_Line9);
    }
  }
}


void EXTI15_10_IRQHandler(void)
{
  if (EXTI_GetITStatus(EXTI_Line10) != RESET)
  {
    /* Wake up from STOP */
    SYSCLKConfig_STOP();
    EXTI_ClearITPendingBit(EXTI_Line10);
  }
  else if (EXTI_GetITStatus(EXTI_Line11) != RESET)
  {
    //Charger connector connected/disconnected
    /* Wake up from STOP */
    SYSCLKConfig_STOP();
    
#ifdef DEBUG_INFO_USART
    printf("Charger connector present\n");
#endif
#ifdef TEST_BOARD
    printf("Charger connector present\n");
#endif
    
    EXTI_ClearITPendingBit(EXTI_Line11);
  }
  else if (EXTI_GetITStatus(EXTI_Line12) != RESET)
  {
    //Charger pressent
    
#ifdef DEBUG_INFO_USART
    printf("Charger with power ");
#endif
    
    /* Execute actions asociated */
    TurnPowerExternal();
    TIM_SetCompare3(TIM3,320);
    /* Change to the new state */
    state.externalPowerSourceState=CONNECTED;
    state.batteryState=CHARGING_CC;
    /* Turn RegulatedOutput on */
    state.regulatedOutputState=ON;
      
#ifdef DEBUG_INFO_USART
    printf("Status number %d\n", getStateId(state));
#endif
    
    EXTI_ClearITPendingBit(EXTI_Line12);
  }
  else if (EXTI_GetITStatus(EXTI_Line13) != RESET)
  {
    //Charger not pressent
    
#ifdef DEBUG_INFO_USART
    printf("Charger disconnected ");
#endif
    
    /* Execute actions asociated */
    /* Change to the new state */
    TurnPowerBattery();
    state.externalPowerSourceState=DISCONNECTED;
    state.batteryState=DISCHARGING;
    
#ifdef DEBUG_INFO_USART
    printf("Status number %d\n", getStateId(state));
#endif
    
    EXTI_ClearITPendingBit(EXTI_Line13);
    
    if(/*state.regulatedOutputState==OFF*/!pcOn)
    {
      state.regulatedOutputState=OFF;
      /* Disable Power Lines */
      //TurnPowerExternal(); /* PW_CTR to disabled state */
      TurnPowerOff();
      state.unregulatedOutputState=NOT_ACTIVE;
#ifdef TR_INVERSE
      GPIO_SetBits(GPIOA,GPIO_Pin_8);
#else
      GPIO_ResetBits(GPIOA,GPIO_Pin_8);
#endif
      GPIO_ResetBits(GPIOA,GPIO_Pin_11);
      GPIO_ResetBits(GPIOB,GPIO_Pin_14);
      /* Disabel devices */
      TIM_SetCompare3(TIM3,0);
      /* Enter STOP mode */
      
#ifdef DEBUG_INFO_USART
      printf("Going to STOP mode\n");
#endif
    
#ifndef DEBUG_NO_STOP
      PWR_EnterSTOPMode(PWR_Regulator_LowPower, PWR_STOPEntry_WFI);
#endif
    }
    else
    {
      TIM_SetCompare3(TIM3,1440);
    }
  }
  if (EXTI_GetITStatus(EXTI_Line14) != RESET)
  {
    //Switch Button pressed
    
#ifdef DEBUG_INFO_USART
    printf("Start/stop button pressed ");
#endif
#ifdef TEST_BOARD
    printf("Start/stop button pressed ");
#endif
    
    if(/*state.regulatedOutputState==ON*/pcOn && !goingOff && !goingOn)
    {
      /* Generate "Turn OFF PC" signal */
      sendPcPulse=true;
      /* Start timer to turn off the PC after X seconds */
      TurnOffDelyTime=PC_OFF_TIMEOUT;
      goingOff=true;
      
#ifdef DEBUG_INFO_USART
      printf("Going OFF\n");
#endif
#ifdef TEST_BOARD
      printf("Going OFF\n");
#endif
    
    }
    else if(/*state.regulatedOutputState==OFF*/!pcOn && !goingOn)
    {
      /* Turn RegulatedOutput on */
      state.regulatedOutputState=ON;
      pcOn=true;
      /* Turn UnRegulatedOutput on */
      if(state.externalPowerSourceState==DISCONNECTED)
      {
        TurnPowerBattery(); /* PW_CTR to enable state */
        TIM_SetCompare3(TIM3,1440);
      }
      else
        TurnPowerExternal();
      state.unregulatedOutputState=ACTIVE;
      GPIO_SetBits(GPIOB,GPIO_Pin_14);
      /* Generate "Turn ON PC" signal */
      sendPcPulse=true;
      /* Turn RegulatedOutput timeout On */
      TurnOnDelyTime=PC_ON_TIMEOUT;
      goingOn=true;
      
#ifdef DEBUG_INFO_USART
      printf("Going ON\n");
#endif
#ifdef TEST_BOARD
      printf("Going ON\n");
#endif
    
    }
    EXTI_ClearITPendingBit(EXTI_Line14);
  }
}


/**
  * @brief  This function handles TIM2 global interrupt request.
  * @param  None
  * @retval None
  */
void TIM2_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
  {
    if(goingOn)
    {
      TurnOnDelyTime--;
      if(TurnOnDelyTime<=0)
      {
        goingOn=false;
      }
    }
    if(goingOff)
    {
      TurnOffDelyTime--;
      if(TurnOffDelyTime<=0)
      {
        pcOn=false;
        goingOff=false;
        state.unregulatedOutputState=NOT_ACTIVE;
        GPIO_ResetBits(GPIOB,GPIO_Pin_14);
        
        if (state.externalPowerSourceState==DISCONNECTED)
        {
          
          /* Execute actions asociated */
          /* Turn UnRegulatedOutput off */
          //TurnPowerExternal(); /* PW_CTR to disabled state */
          TurnPowerOff();
          //It is possible to change the led status
          /* Change to the new state */
          /* Turn RegulatedOutputDC off */
          state.regulatedOutputState=OFF; /* The PWM control signal goes to off state afterwards, in the main loop */
          
      #ifdef DEBUG_INFO_USART
          printf("Status number %d\n", getStateId(state));
      #endif
        
          /* Disable Power Lines */
#ifdef TR_INVERSE
          GPIO_SetBits(GPIOA,GPIO_Pin_8);
#else
          GPIO_ResetBits(GPIOA,GPIO_Pin_8);
#endif
          GPIO_ResetBits(GPIOA,GPIO_Pin_11);
          /* Disabel devices */
          TIM_SetCompare3(TIM3,0);
          /* Enter STOP mode */
          
    #ifdef DEBUG_INFO_USART
          printf("Going to STOP mode\n");
    #endif
        
    #ifndef DEBUG_NO_STOP    
          TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
          PWR_EnterSTOPMode(PWR_Regulator_LowPower, PWR_STOPEntry_WFI);
    #endif
        }
      }
    }
    if(batteryKoFlag)
    {
      BatteryKoTimeout--;
    }
    if(noCurrentFlag)
    {
      NoCurrentTimeout--;
    }
    if(sendPcPulse && pcPulseStarted)
    {
      GPIO_SetBits(GPIOB,GPIO_Pin_8);
      sendPcPulse=false;
      pcPulseStarted=false;
    }
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
  }
  else if (TIM_GetITStatus(TIM2, TIM_IT_CC1) != RESET)
  {
    if(sendPcPulse)
    {
      GPIO_ResetBits(GPIOB,GPIO_Pin_8);
      pcPulseStarted=true;
    }
    TIM_ClearITPendingBit(TIM2, TIM_IT_CC1);
  }
}

/**
  * @brief  This function handles TIM3 global interrupt request.
  * @param  None
  * @retval None
  */
void TIM3_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
  {
    /* Get ADC values */
    uint32_t bat_level_value=ADC_ConvertedValueTabPointer[1]*/*41*/ADC_TO_MV_CONSTANT;
    uint32_t regulatedOutputCurrentValue=ADC_ConvertedValueTabPointer[2]*ADC_TO_MA_CONSTANT;
    uint32_t bat_current_value=ADC_ConvertedValueTabPointer[3]*/*32*/ADC_TO_MA_CONSTANT;
    /*
    printf("Bat Level: %d\n\r", bat_level_value);
    printf("Bat charge current: %d\n\r", bat_current_value);
    printf("Reg output Level: %d\n\r", ADC_ConvertedValueTabPointer[0]*ADC_TO_MV_CONSTANT);
    printf("Reg output current: %d\n\r", regulatedOutputCurrentValue);
    printf("Charger Level: %d\n\r", ADC_ConvertedValueTabPointer[4]*ADC_TO_MV_CONSTANT);
    */
    /* Check for the events related to the ADC values */
    /* PC is turned off (it was previously on) */
    if (state.regulatedOutputState==ON && regulatedOutputCurrentValue<CURRENT_ON_OFF_THRESHOLD && TurnOnDelyTime==0 && !noCurrentFlag /* && state.externalPowerSourceState==DISCONNECTED */ && pcOn)
    {
      NoCurrentTimeout=2;
      noCurrentFlag=true;
    }
    else if(noCurrentFlag && regulatedOutputCurrentValue>CURRENT_ON_OFF_THRESHOLD)
    {
      noCurrentFlag=false;
      if(!pcOn)
        pcOn=true;
    }
    if(noCurrentFlag && NoCurrentTimeout<=0 && pcOn)
    {
      /* Event EV.3 detected */
      noCurrentFlag=false;
      EXTI_GenerateSWInterrupt(EXTI_Line9);
    }
    /* Battery below KO level */
    if (state.batteryState==DISCHARGING && bat_level_value<BATTERY_KO_THRESHOLD && !batteryKoFlag)
    {
      BatteryKoTimeout=5;
      batteryKoFlag=true;
    }
    else if(batteryKoFlag)
    {
      if(bat_level_value>BATTERY_KO_THRESHOLD || state.externalPowerSourceState==CONNECTED)
        batteryKoFlag=false;
    }
    if(batteryKoFlag && BatteryKoTimeout<=0)
    {
      /* Event EV.6 detected */
      batteryKoFlag=false;
      EXTI_GenerateSWInterrupt(EXTI_Line6);
    }
    /* Battery charge to constant voltage mode */
    else if (state.batteryState==CHARGING_CC && bat_level_value>BAT_CC_CV_THRESHOLD)
    {
      /* Event EV.4 detected */
      EXTI_GenerateSWInterrupt(EXTI_Line8);
    }
    /* Battery fully charged */
    else if (state.batteryState==CHARGING_CV && bat_current_value<BAT_CURRENT_CHARGED_THRESHOLD)
    {
      /* Event EV.5 detected */
      EXTI_GenerateSWInterrupt(EXTI_Line7);
    }
    
    getBatteryLevel(4, 3300, 3600, state.batteryState, bat_level_value);
    
    TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
  }
}

/**
  * @brief  This function handles I2C1 Event interrupt request.
  * @param  None
  * @retval : None
  */
void I2C1_EV_IRQHandler(void)
{

    __IO uint32_t SR1Register =0;
    __IO uint32_t SR2Register =0;

#ifdef SLAVE_DMA_USE
    /* Read SR1 register */
    SR1Register = I2C1->SR1;

    /* If ADDR is set */
    if ((SR1Register & 0x0002) == 0x0002)
    {
        /* In slave Transmitter/Receiver mode, when using DMA, it is recommended to update the buffer
          base address and the buffer size before clearing ADDR flag. In fact, the only
          period when the slave has control  on the bus(SCL is stretched so master can not initiate
          transfers) is the period between ADDR is set and ADDR is cleared. Otherwise, the master can
          initiate transfers and the buffer size & the buffer address have not yet been updated.*/

        /* Update the DMA channels memory base address and count */
    	Buffer_Tx1[0]=getStateId(state);
    	//Buffer_Tx1[1]=getBatteryLevel(4, 3300, 3600, state.batteryState, ((uint32_t)ADC_ConvertedValueTabPointer[1])*/*41*/ADC_TO_MV_CONSTANT);
        //Buffer_Tx1[1]=percent;
        Buffer_Tx1[1]=(uint8_t)((((uint32_t)ADC_ConvertedValueTabPointer[1])*ADC_TO_MV_CONSTANT)/1000);
        I2C_DMAConfig (I2C1, Buffer_Tx1, 0xFFFF, I2C_DIRECTION_TX);
        I2C_DMAConfig (I2C1, Buffer_Rx1, 0xFFFF, I2C_DIRECTION_RX);
        /* Clear ADDR by reading SR2 register */
        SR2Register = I2C1->SR2;
    }
#else
    /* Read the I2C1 SR1 and SR2 status registers */
    SR1Register = I2C1->SR1;
    SR2Register = I2C1->SR2;

    /* If I2C1 is slave (MSL flag = 0) */
    if ((SR2Register &0x0001) != 0x0001)
    {
        /* If ADDR = 1: EV1 */
        if ((SR1Register & 0x0002) == 0x0002)
        {
            /* Clear SR1Register and SR2Register variables to prepare for next IT */
            SR1Register = 0;
            SR2Register = 0;
            /* Initialize the transmit/receive counters for next transmission/reception
            using Interrupt  */
            Tx_Idx1 = 0;
            Rx_Idx1 = 0;
            /* Set the values in the buffer */
            Buffer_Tx1[0]=getStateId(state);
            //Buffer_Tx1[1]=getBatteryLevel(4, 3300, 3600, state.batteryState, ((uint32_t)ADC_ConvertedValueTabPointer[1])*/*41*/ADC_TO_MV_CONSTANT);
            //Buffer_Tx1[1]=percent;
            Buffer_Tx1[1]=(uint8_t)((((uint32_t)ADC_ConvertedValueTabPointer[1])*ADC_TO_MV_CONSTANT)/1000);
        }
        /* If TXE = 1: EV3 */
        if ((SR1Register & 0x0080) == 0x0080)
        {
            /* Write data in data register */
            I2C1->DR = Buffer_Tx1[Tx_Idx1++];
            SR1Register = 0;
            SR2Register = 0;
        }
        /* If RXNE = 1: EV2 */
        //if ((SR1Register & 0x0040) == 0x0040)
        //{
        //    /* Read data from data register */
        //    Buffer_Rx1[Rx_Idx1++] = I2C1->DR;
        //    SR1Register = 0;
        //    SR2Register = 0;

        //}
        /* If STOPF =1: EV4 (Slave has detected a STOP condition on the bus */
        if (( SR1Register & 0x0010) == 0x0010)
        {
            I2C1->CR1 |= CR1_PE_Set;
            SR1Register = 0;
            SR2Register = 0;

        }
    } /* End slave mode */

#endif

    /* If SB = 1, I2C1 master sent a START on the bus: EV5) */
    if ((SR1Register &0x0001) == 0x0001)
    {

        /* Send the slave address for transmssion or for reception (according to the configured value
            in the write master write routine */
        I2C1->DR = Address;
        SR1Register = 0;
        SR2Register = 0;
    }


}
/**
  * @}
  */

/**
  * @brief  This function handles I2C1 Error interrupt request.
  * @param  None
  * @retval : None
  */
void I2C1_ER_IRQHandler(void)
{

    __IO uint32_t SR1Register =0;

    /* Read the I2C1 status register */
    SR1Register = I2C1->SR1;
    /* If AF = 1 */
    if ((SR1Register & 0x0400) == 0x0400)
    {
        I2C1->SR1 &= 0xFBFF;
        SR1Register = 0;
    }
    /* If ARLO = 1 */
    if ((SR1Register & 0x0200) == 0x0200)
    {
        I2C1->SR1 &= 0xFBFF;
        SR1Register = 0;
    }
    /* If BERR = 1 */
    if ((SR1Register & 0x0100) == 0x0100)
    {
        I2C1->SR1 &= 0xFEFF;
        SR1Register = 0;
    }

    /* If OVR = 1 */

    if ((SR1Register & 0x0800) == 0x0800)
    {
        I2C1->SR1 &= 0xF7FF;
        SR1Register = 0;
    }
}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 

/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
