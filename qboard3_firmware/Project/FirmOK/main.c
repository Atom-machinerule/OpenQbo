/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/main.c 
  * @author  MCD Application Team
  * @version V3.4.0
  * @date    10/15/2010
  * @brief   Main program body
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
#include "main.h"
#include "I2CRoutines.h"

/** @addtogroup STM32F10x_StdPeriph_Template
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define ADC1_DR_Address    ((uint32_t)0x4001244C)
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
USART_InitTypeDef USART_InitStructure;
TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
TIM_OCInitTypeDef  TIM_OCInitStructure;

__IO uint32_t ADC_DualConvertedValueTab[3];
uint16_t * ADC_ConvertedValueTabPointer;

StateStructure state;

static __IO uint32_t TimingDelay;

uint16_t chargeCurrentMax=0;

bool pcOn=false;


/* Buffer of data to be received by I2C1 */
uint8_t Buffer_Rx1[255];
/* Buffer of data to be transmitted by I2C1 */
uint8_t Buffer_Tx1[255] = {0x0, 0x0};
extern __IO uint8_t Tx_Idx1 , Rx_Idx1;

/* Private function prototypes -----------------------------------------------*/

void RCC_Configuration(void);
void GPIO_Configuration(void);
void NVIC_Configuration(void);
void InitADCs(void);
void TIMxinit(void);
void InitI2C(void);
void InitUSART(void);
void Delay(__IO uint32_t nTime);

/* Private functions ---------------------------------------------------------*/


#ifdef DEBUG
char getch(void) 
{ 
   char tmp; 
   while(USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET); 
   tmp = USART_ReceiveData(USART1); 
   return tmp;    
}
#endif

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{
  /*!< At this stage the microcontroller clock setting is already configured, 
       this is done through SystemInit() function which is called from startup
       file (startup_stm32f10x_xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32f10x.c file
     */     
  
  /* System clocks configuration ---------------------------------------------*/
  RCC_Configuration();

  /* GPIO configuration ------------------------------------------------------*/
  GPIO_Configuration();
  
  GPIO_SetBits(GPIOB,GPIO_Pin_8);
  GPIO_ResetBits(GPIOB,GPIO_Pin_14);
  GPIO_ResetBits(GPIOA,GPIO_Pin_11);
#ifdef TR_INVERSE
  GPIO_SetBits(GPIOA,GPIO_Pin_8);
#else
  GPIO_ResetBits(GPIOA,GPIO_Pin_8);
#endif
  
  ADC_ConvertedValueTabPointer=(uint16_t *)ADC_DualConvertedValueTab;
  
  uint32_t regulatedOutputValue;
  uint32_t regulatedOutputCurrentValue;
  uint32_t bat_level_value;
  uint32_t bat_current_value;
  uint32_t externalPower_value;
  
  int32_t diffExternalBat;
  
  state.regulatedOutputState=OFF;
  state.unregulatedOutputState=NOT_ACTIVE;
  state.externalPowerSourceState=DISCONNECTED;
  state.batteryState=DISCHARGING;

  /* Turn UnRegulatedOutput on */
  TurnPowerExternal();

  while(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_10)==Bit_RESET);

   /* Setup SysTick Timer for 1 msec interrupts  */
  if (SysTick_Config(SystemCoreClock / 1000))
  { 
    /* Capture error */ 
    while (1);
  }
  
#ifdef DEBUG
  InitUSART();
#endif

  InitADCs();
  
  TIMxinit();
  
  NVIC_Configuration();

  InitI2C();

  //uint16_t battCurrentArray[16]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
  //uint8_t battIndex=0;
  
  while (1)
  {
    regulatedOutputValue=ADC_ConvertedValueTabPointer[0]*/*40*/ADC_TO_MV_CONSTANT; //El valor del multiplicador ha de ser 40, pero tras el problema del transistor casi quemado ya no se mide bien este valor. Con este multiplicador vuelve a funcionar
    bat_level_value=ADC_ConvertedValueTabPointer[1]*/*41*/ADC_TO_MV_CONSTANT; //Ajustado para el valor real de las resistencias usadas
    regulatedOutputCurrentValue=ADC_ConvertedValueTabPointer[2]*ADC_TO_MA_CONSTANT;
    bat_current_value=ADC_ConvertedValueTabPointer[3]*/*32*/ADC_TO_MA_CONSTANT; //Ajustado a mano
    externalPower_value=ADC_ConvertedValueTabPointer[4]*/*46*/ADC_TO_MV_CONSTANT; //Ajustado para el valor real de las resistencias usadas
    diffExternalBat=ADC_ConvertedValueTabPointer[4];
    diffExternalBat-=ADC_ConvertedValueTabPointer[1];
    diffExternalBat*=ADC_TO_MV_CONSTANT;
    //printf("\n\r%d %d",externalPower_value,diffExternalBat);
    /* External Power Supply disconnected */
    if (state.externalPowerSourceState==CONNECTED && (/*diffExternalBat<14000 || */externalPower_value<EXTERNAL_POWER_NOT_PRESENT_THRESHOLD || GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_11)==Bit_RESET))
    {
      //printf("Apago!");
      /* Event EV.1 detected */
      EXTI_GenerateSWInterrupt(EXTI_Line13);
    }
    /* External Power Suply connected */
    else if (state.externalPowerSourceState==DISCONNECTED && externalPower_value>EXTERNAL_POWER_PRESENT_THRESHOLD && GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_11)==Bit_SET )
    {
      /* Event EV.0 detected */
      EXTI_GenerateSWInterrupt(EXTI_Line12);
    }

    /* Regulated Output DC DC control */
    if(state.regulatedOutputState==ON) //Output enabled
    {
      if (regulatedOutputValue>DESIRED_REGULATED_OUTPUT_VALUE)
      {
    	  GPIO_ResetBits(GPIOA,GPIO_Pin_11);
      }
      else
      {
    	  GPIO_SetBits(GPIOA,GPIO_Pin_11);
      }
    }
    else //Output disabled
      GPIO_ResetBits(GPIOA,GPIO_Pin_11);

    if(state.regulatedOutputState==ON)
    {
      chargeCurrentMax=MAX_EXTERNAL_POWER_CURRENT-regulatedOutputCurrentValue;
      chargeCurrentMax=chargeCurrentMax>MAX_BAT_CC_CURRENT_VALUE ? MAX_BAT_CC_CURRENT_VALUE : chargeCurrentMax;
      /*
      if(pcOn)
      {
        chargeCurrentMax=0;
      }
      */
    }
    else
      chargeCurrentMax=MAX_BAT_CC_CURRENT_VALUE;
    
    if(bat_level_value<100000)
    {
#ifdef TR_INVERSE
      GPIO_ResetBits(GPIOA,GPIO_Pin_8);
#else
      GPIO_SetBits(GPIOA,GPIO_Pin_8);
#endif
    }
    else if(state.externalPowerSourceState==CONNECTED && chargeCurrentMax>0) //Charge battery
    {

      switch(state.batteryState)
      {
      case KO: case CHARGING_CC:
        if (bat_current_value>chargeCurrentMax)
        {
#ifdef TR_INVERSE
                GPIO_SetBits(GPIOA,GPIO_Pin_8);
#else
                GPIO_ResetBits(GPIOA,GPIO_Pin_8);
#endif
        }
        else
        {
#ifdef TR_INVERSE
                GPIO_ResetBits(GPIOA,GPIO_Pin_8);
#else
                GPIO_SetBits(GPIOA,GPIO_Pin_8);
#endif
        }
        break;
      case CHARGING_CV: case CHARGED:
        if (bat_level_value>BAT_CV_CHARGED_THRESHOLD)
        {
#ifdef TR_INVERSE
                GPIO_SetBits(GPIOA,GPIO_Pin_8);
#else
                GPIO_ResetBits(GPIOA,GPIO_Pin_8);
#endif
        }
        else
        {
#ifdef TR_INVERSE
                GPIO_ResetBits(GPIOA,GPIO_Pin_8);
#else
                GPIO_SetBits(GPIOA,GPIO_Pin_8);
#endif
        }
        break;
      default:
#ifdef TR_INVERSE
        GPIO_SetBits(GPIOA,GPIO_Pin_8);
#else
    	GPIO_ResetBits(GPIOA,GPIO_Pin_8);
#endif
        break;
      }

    }
    else //Not charge battery
#ifdef TR_INVERSE
          GPIO_SetBits(GPIOA,GPIO_Pin_8);
#else
  	  GPIO_ResetBits(GPIOA,GPIO_Pin_8);
#endif

  }
}

/**
  * @brief  Configures the different system clocks.
  * @param  None
  * @retval None
  */
void RCC_Configuration(void)
{
  /* PCLK1 = HCLK/4 */
  RCC_PCLK1Config(RCC_HCLK_Div1);
  
#if defined (STM32F10X_LD_VL) || defined (STM32F10X_MD_VL) || defined (STM32F10X_HD_VL) || defined (STM32F10X_MD)
  /* ADCCLK = PCLK2/2 */
  RCC_ADCCLKConfig(RCC_PCLK2_Div2); 
#else
  /* ADCCLK = PCLK2/4 */
  RCC_ADCCLKConfig(RCC_PCLK2_Div4); 
#endif
  
  /* DMA1 Clock activation */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
  
  /* Periphepals clock activation */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3
                          | RCC_APB1Periph_I2C1 | RCC_APB1Periph_PWR, ENABLE);

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_ADC1
                          | RCC_APB2Periph_ADC2 | RCC_APB2Periph_AFIO | RCC_APB2Periph_TIM1, ENABLE);
}

/**
  * @brief  Configure the GPIO Pins.
  * @param  None
  * @retval None
  */
void GPIO_Configuration(void)
{  
  //PWM charge -> PA8
  //PWM Regula -> PA11
  //Bat On     -> PB12
  //FA On      -> PB13
  //Unreg      -> PB14
  
  //Bat curren -> PA4
  //Bat Level  -> PA5
  //Reg Level  -> PA6
  //Reg curren -> PA7
  //FA Level   -> PB1
  
  //Led        -> PB0
  //PC Pulse   -> PB8
  //Switch     -> PB10
  //FA Present -> PB11
  
  GPIO_InitTypeDef GPIO_InitStructure;
  EXTI_InitTypeDef EXTI_InitStructure;
  
  /* PW_CTR */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init (GPIOB, &GPIO_InitStructure);
  
  /* PWM_Charge and PWM_DC_Conv */
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_8 | GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  /* Led */
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  //GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  /* PC_CTR */
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_8;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  /* Analog Inputs */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Speed = (GPIOSpeed_TypeDef)0;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init (GPIOA, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
  GPIO_Init (GPIOB, &GPIO_InitStructure);
  
  /* Digital Inputs */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init (GPIOB, &GPIO_InitStructure);
  
  /* I2C */
  //GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
  //GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  //GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//GPIO_Mode_AF_OD;
  //GPIO_Init (GPIOB, &GPIO_InitStructure);
  
  /* Connect EXTI Line to Button and Charger GPIO Pins */
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource10);
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource11);
  
  /* Configure Button EXTI line */
  EXTI_InitStructure.EXTI_Line = EXTI_Line10;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
  
  /* Configure Charger EXTI line */
  EXTI_InitStructure.EXTI_Line = EXTI_Line11;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
	
  /* Set EXTI line configuration for software interrupts */
  EXTI->IMR |= EXTI_Line6 | EXTI_Line7 | EXTI_Line8 | EXTI_Line9 | EXTI_Line12 | EXTI_Line13 | EXTI_Line14;
  EXTI->EMR |= EXTI_Line6 | EXTI_Line7 | EXTI_Line8 | EXTI_Line9 | EXTI_Line12 | EXTI_Line13 | EXTI_Line14;
}


void NVIC_Configuration(void)
{

  NVIC_InitTypeDef NVIC_InitStructure;
  
  /* Enable the DMA gloabal Interrupt */
  /*
  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  
  NVIC_Init(&NVIC_InitStructure);*/
  
  /* Enable and set Button, Charger and software EXTI Interrupt */
  
  NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  
  NVIC_Init(&NVIC_InitStructure);
  
  
  /* Enable and set Button, Charger and software EXTI Interrupt */
  
  NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  
  NVIC_Init(&NVIC_InitStructure);
  
  
  /* Enable the TIM2 gloabal Interrupt */
  
  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  
  NVIC_Init(&NVIC_InitStructure);
  
  /* Enable the TIM1 gloabal Interrupt */
  
  NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;//TIM1_UP_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  
  NVIC_Init(&NVIC_InitStructure);
  
  /* Enable the I2C1 Interruptions */
  /*
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

  NVIC_SetPriority(I2C1_EV_IRQn, 0x00);
  NVIC_EnableIRQ(I2C1_EV_IRQn);

  //NVIC_SetPriority(I2C1_ER_IRQn, 0x01);
  //NVIC_EnableIRQ(I2C1_ER_IRQn);
  */
  NVIC_InitStructure.NVIC_IRQChannel = I2C1_EV_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  
  NVIC_Init(&NVIC_InitStructure);
}

/*************************************************************************
 * Function Name: InitADCs
 * Parameters: none
 * Return: none
 *
 * Description: ADC Init subroutine
 *
 *************************************************************************/
void InitADCs(void)
{
  ADC_InitTypeDef   ADC_InitStructure;
  DMA_InitTypeDef DMA_InitStructure;

  /* DMA1 channel1 configuration ----------------------------------------------*/
  DMA_DeInit(DMA1_Channel1);
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)ADC1_DR_Address;
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)ADC_DualConvertedValueTab;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize = 3;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel1, &DMA_InitStructure);
  
  /* Enable DMA1 channel1 */
  DMA_Cmd(DMA1_Channel1, ENABLE);
  //DMA_ITConfig(DMA1_Channel1,DMA_IT_TC,ENABLE);

  /* ADC1 configuration ------------------------------------------------------*/
  ADC_InitStructure.ADC_Mode = ADC_Mode_RegSimult;
  ADC_InitStructure.ADC_ScanConvMode = ENABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfChannel = 3;
  ADC_Init(ADC1, &ADC_InitStructure);
  /* ADC1 regular channels configuration */ 
  ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 1, ADC_SampleTime_7Cycles5);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 2, ADC_SampleTime_7Cycles5);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_9, 3, ADC_SampleTime_7Cycles5);
  /* Enable ADC1 DMA */
  ADC_DMACmd(ADC1, ENABLE);

  /* ADC2 configuration ------------------------------------------------------*/
  ADC_InitStructure.ADC_Mode = ADC_Mode_RegSimult;
  ADC_InitStructure.ADC_ScanConvMode = ENABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfChannel = 3;
  ADC_Init(ADC2, &ADC_InitStructure);
  /* ADC2 regular channels configuration */ 
  ADC_RegularChannelConfig(ADC2, ADC_Channel_5, 1, ADC_SampleTime_7Cycles5);
  ADC_RegularChannelConfig(ADC2, ADC_Channel_4, 2, ADC_SampleTime_7Cycles5);
  ADC_RegularChannelConfig(ADC2, ADC_Channel_9, 3, ADC_SampleTime_7Cycles5);
  /* Enable ADC2 external trigger conversion */
  ADC_ExternalTrigConvCmd(ADC2, ENABLE);

  /* Enable ADC1 */
  ADC_Cmd(ADC1, ENABLE);

  /* Enable ADC1 reset calibaration register */   
  ADC_ResetCalibration(ADC1);
  /* Check the end of ADC1 reset calibration register */
  while(ADC_GetResetCalibrationStatus(ADC1));

  /* Start ADC1 calibaration */
  ADC_StartCalibration(ADC1);
  /* Check the end of ADC1 calibration */
  while(ADC_GetCalibrationStatus(ADC1));

  /* Enable ADC2 */
  ADC_Cmd(ADC2, ENABLE);

  /* Enable ADC2 reset calibaration register */   
  ADC_ResetCalibration(ADC2);
  /* Check the end of ADC2 reset calibration register */
  while(ADC_GetResetCalibrationStatus(ADC2));

  /* Start ADC2 calibaration */
  ADC_StartCalibration(ADC2);
  /* Check the end of ADC2 calibration */
  while(ADC_GetCalibrationStatus(ADC2));

  /* Start ADC1 Software Conversion */ 
  ADC_SoftwareStartConvCmd(ADC1, ENABLE);

}

/**
  * @brief  Configure the TIMERx functions.
  * @param  None
  * @retval None
  */
void TIMxinit(void) 
{
  
  /* Timeout timmers config */
  /* TIM2 configured to generate interruptions with a frecuency of */
  /* one second. Its function is to serve as a timer for the On and Off */
  /* timeouts delays and as a PC On/Off pulse generator */
  uint16_t PrescalerValue = (uint16_t) (SystemCoreClock / 65535) - 1;
  
  TIM_TimeBaseStructure.TIM_Period = 65535; //One second period
  TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

  /* TIM2 PWM1 Mode configuration: Channel1 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 43690;//65536/2;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

  TIM_OC1Init(TIM2, &TIM_OCInitStructure);
  TIM_ITConfig(TIM2, TIM_IT_CC1, ENABLE);
  
  TIM_Cmd(TIM2, ENABLE);
  
  /* LED PWM Config */
  //Ahora no funciona como PWM para el led, sino como fuente de interrupci�n peri�dica para comprobar el estado de carga de la bater�a

  /*
  PrescalerValue = (uint16_t) (SystemCoreClock / 65535) - 1;
  
  TIM_TimeBaseStructure.TIM_Period = 65535; //One second period
  TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  */
  
  
  TIM_TimeBaseStructure.TIM_Period = 1440;
  TIM_TimeBaseStructure.TIM_Prescaler = 65535;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

  /* TIM3 PWM2 Mode configuration: Channel3 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
//#ifdef DEBUG_TIM_OUTPUTS
//  TIM_OCInitStructure.TIM_Pulse = 0xEFFF;
//#elif defined TEST_BOARD
//  TIM_OCInitStructure.TIM_Pulse = 65536/2;
//#else
  TIM_OCInitStructure.TIM_Pulse = 0;
//#endif
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

  TIM_OC3Init(TIM3, &TIM_OCInitStructure);
  
  TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
  
  TIM_Cmd(TIM3, ENABLE);
}

/**
  * @brief  Configure the I2C.
  * @param  None
  * @retval None
  */
void InitI2C(void)
{
  I2C_LowLevel_Init(I2C1);
  I2C_Slave_BufferReadWrite(I2C1, Interrupt);
}


#ifdef DEBUG
/**
  * @brief  Configure the USART.
  * @param  None
  * @retval None
  */
void InitUSART(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1 | RCC_APB2Periph_AFIO, ENABLE);
  
  /* Configure USART1 Tx as alternate function push-pull */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* Configure USART1 Rx as input floating */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  /* USARTx configured as follow:
        - BaudRate = 115200 baud  
        - Word Length = 8 Bits
        - One Stop Bit
        - No parity
        - Hardware flow control disabled (RTS and CTS signals)
        - Receive and transmit enabled
  */
  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  
  /* USART configuration */
  USART_Init(USART1, &USART_InitStructure);
    
  /* Enable USART */
  USART_Cmd(USART1, ENABLE);

  /* Retarget the C library printf function to the USARTx, can be USART1 or USART2
     depending on the EVAL board you are using ********************************/
  printf("\n\r QBO Energy board");
}
#endif

/**
  * @brief  Inserts a delay time.
  * @param  nTime: specifies the delay time length, in milliseconds.
  * @retval None
  */
void Delay(__IO uint32_t nTime)
{ 
  TimingDelay = nTime;

  while(TimingDelay != 0);
}

/**
  * @brief  Decrements the TimingDelay variable.
  * @param  None
  * @retval None
  */
void TimingDelay_Decrement(void)
{
  if (TimingDelay != 0x00)
  { 
    TimingDelay--;
  }
}


void TurnPowerBattery(void)
{
  GPIO_ResetBits(GPIOB,GPIO_Pin_13);
  GPIO_SetBits(GPIOB,GPIO_Pin_12);
}
void TurnPowerExternal(void)
{
  GPIO_SetBits(GPIOA,GPIO_Pin_11);
  GPIO_ResetBits(GPIOB,GPIO_Pin_12);
  GPIO_ResetBits(GPIOB,GPIO_Pin_12); //A small dely
  GPIO_SetBits(GPIOB,GPIO_Pin_13);
}
void TurnPowerOff(void)
{
  GPIO_ResetBits(GPIOB,GPIO_Pin_12 | GPIO_Pin_13);
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  /*
  while (1)
  {
  }
  */
  NVIC_SystemReset();
}
#endif

/**
  * @}
  */


/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
