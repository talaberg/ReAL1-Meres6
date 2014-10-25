/* Device dependent functions */

/* Includes */
#include "stm32f4xx.h"
#include "ucos_ii.h"
#include "bsp.h"

/* Private Definitions */
#define UART_BUFFER_SIZE	32
#define TempAddr ((uint8_t) 0x30)
#define TempConfReg ((uint8_t) 0x01)
#define TempReg ((uint8_t) 0x05)
#define I2C_TIMEOUT ((uint16_t) 0xFFFF)

/* Global Variables */
uint16_t TempVal = 0; // Temperature
uint16_t LimitVal = 0; // Limit
uint16_t* pDisplayVal; // Pointer to the value to display

/* Private Functions */
void USARTSendByte (USART_TypeDef* USARTx, uint8_t data);

/* SysTick Initialization */
void BSP_SysTick_Init(void)
{
    RCC_ClocksTypeDef  rcc_clocks;
    RCC_GetClocksFreq(&rcc_clocks);
	OS_CPU_SysTickInit(rcc_clocks.HCLK_Frequency / (INT32U)OS_TICKS_PER_SEC);                                   /* Init uC/OS periodic time src (SysTick).                  */
}

/* LED Initialization */
void BSP_LED_Init()
{
	GPIO_InitTypeDef  GPIO_InitStructure;

	/* Enable GPIOD Periphery clock */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	/* Configure PD12, PD13, PD14 and PD15 in output pushpull mode */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13| GPIO_Pin_14| GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
}

/* ADC Initialization */
void BSP_ADC_Init(void)
{
	  ADC_InitTypeDef ADC_InitStructure;
	  ADC_CommonInitTypeDef ADC_CommonInitStructure;
	  GPIO_InitTypeDef GPIO_InitStructure;

	  /* Enable ADC2 clock */
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE);
	  RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOB, ENABLE );

	  /* Configure ADC Channel 9 as analog input */
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
	  GPIO_Init(GPIOB, &GPIO_InitStructure);


	  /* ADC Common Init */
	  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
	  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div6;
	  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
	  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
	  ADC_CommonInit(&ADC_CommonInitStructure);

	  /* ADC2 Configuration ------------------------------------------------------*/
	  ADC_StructInit(&ADC_InitStructure);
	  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	  ADC_InitStructure.ADC_ScanConvMode = DISABLE;
	  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	  ADC_InitStructure.ADC_NbrOfConversion = 1;
	  ADC_Init(ADC2, &ADC_InitStructure);

	  /* ADC2 Regular Channel Config */
	  ADC_RegularChannelConfig(ADC2, ADC_Channel_9, 1, ADC_SampleTime_56Cycles);

	  /* Enable ADC2 */
	  ADC_Cmd(ADC2, ENABLE);

	  /* ADC2 regular Software Start Conv */
	  ADC_SoftwareStartConv(ADC2);
}

/* 7 Segment Display Initialization */
void BSP_Display_Init(void)
{
	  GPIO_InitTypeDef GPIO_InitStructure;
	  TIM_TimeBaseInitTypeDef Timer_InitStructure;
	  NVIC_InitTypeDef NVIC_InitStructure;


	  RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOB |RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_GPIOE, ENABLE );
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	  GPIO_Init( GPIOE, &GPIO_InitStructure );
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_10;
	  GPIO_Init( GPIOD, &GPIO_InitStructure );
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14 | GPIO_Pin_15;
	  GPIO_Init( GPIOB, &GPIO_InitStructure );

	  RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM4, ENABLE );
	  TIM_TimeBaseStructInit( &Timer_InitStructure );
	  Timer_InitStructure.TIM_Prescaler = 9999;
	  Timer_InitStructure.TIM_Period = 19;
	  Timer_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	  TIM_TimeBaseInit( TIM4, &Timer_InitStructure);
	  TIM_ITConfig( TIM4, TIM_IT_Update, ENABLE );
	  TIM_Cmd(TIM4, ENABLE );

	  NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	  NVIC_Init( &NVIC_InitStructure );
}

/* Temperature Sensor Initialization */
void BSP_TempSensor_Init(void)
{
	I2C_InitTypeDef  I2C_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOB, ENABLE );

	GPIO_PinAFConfig(GPIOB,GPIO_PinSource6,GPIO_AF_I2C1);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource9,GPIO_AF_I2C1);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);


	/* I2C1 periph clk enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
	I2C_DeInit(I2C1);
	/* Init I2C in master mode, scl speed = 100kHz */
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_ClockSpeed = 100000;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_OwnAddress1 = 0xA0;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_Init(I2C1, &I2C_InitStructure);
	/* Enable I2C1 */
	I2C_Cmd(I2C1, ENABLE);

	/* Init the sensor - Resolution 0.5°C*/
	/* Start */
	I2C_GenerateSTART(I2C1,ENABLE);
	while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_MODE_SELECT));
	/* Send Addres + Write cmd */
	I2C_Send7bitAddress(I2C1,TempAddr,I2C_Direction_Transmitter);
	while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
	/* Send Data = set the register pointer */
	I2C_SendData(I2C1,0x08);
	while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	/* Set the register */
	I2C_SendData(I2C1,0x00);
	while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	/* Stop Condition */
	I2C_GenerateSTOP(I2C1,ENABLE);

}

/* USART Initialization */
void BSP_USART_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	/* GPIOD Pin8,Pin9 AF-set */
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource8,GPIO_AF_USART3);
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource9,GPIO_AF_USART3);

	// USART RX,TX - GPIOD Pin Configuration
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	//Enable USART3 periph clk
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	/* USART configured as follow:
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
	USART_Init(USART3, &USART_InitStructure);

	USART_Cmd(USART3, ENABLE);

	USART_ITConfig(USART3,USART_IT_RXNE,ENABLE);

	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&NVIC_InitStructure);
}

/* Switch Initialization */
void BSP_Switch_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOE,&GPIO_InitStructure);

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init( GPIOC, &GPIO_InitStructure );

	GPIO_SetBits( GPIOC, GPIO_Pin_6 );
}

/* Read the temperature */
uint8_t GetI2CTemp(void)
{
	uint8_t temperature = 0x00;
	uint8_t UpperByte = 0x00;
	uint8_t LowerByte = 0x00;
	/* Start */
	I2C_GenerateSTART(I2C1,ENABLE);
	while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_MODE_SELECT));
	/* Send Slave Address + Write command */
	I2C_Send7bitAddress(I2C1,TempAddr,I2C_Direction_Transmitter);
	while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
	/* Send data = set the register pointer */
	I2C_SendData(I2C1,TempReg);
	while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	/* ReStart */
	I2C_GenerateSTART(I2C1,ENABLE);
	while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_MODE_SELECT));
	/* Send Slave Addres + Read Command */
	I2C_Send7bitAddress(I2C1,TempAddr,I2C_Direction_Receiver);
	while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
	/* Recieve data */
	while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_RECEIVED));
	UpperByte = I2C_ReceiveData(I2C1);
	/* Recieve data */
	while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_RECEIVED));
	LowerByte = I2C_ReceiveData(I2C1);
	/* Stop condition */
	I2C_GenerateSTOP(I2C1,ENABLE);

	UpperByte = UpperByte & 0x1F;
	if ((UpperByte & 0x10) == 0x10){	//TA<0°C
		UpperByte = UpperByte & 0x0F;
		temperature = 256 - (UpperByte*16+LowerByte/16);
	}else{	//TA>0°C
		temperature = UpperByte*16+LowerByte/16;
	}
	return temperature;
}

/* Read the ADC value */
uint8_t GetADCValue(void)
{
	return ADC_GetConversionValue(ADC2)>>6;
}

/* Read the display switch state */
uint8_t GetDisplaySwitchState()
{
	return GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_2);
}

/* Read the mode selector switch state */
uint8_t GetModeSwitchState()
{
	return GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_4);
}

/* Send a byte on USART */
void USARTSendByte (USART_TypeDef* USARTx, uint8_t data)
{
	USART_SendData(USARTx,data);
	while (USART_GetFlagStatus(USART3, USART_FLAG_TXE)==RESET);
}

/* Send a string on USART */
void USARTSendString(char* data)
{
	while (*data)
	{
		USARTSendByte(USART3,*data);
		data++;
	}
}

void SetRedLED()
{
	GPIO_SetBits(GPIOD, GPIO_Pin_14);
}
void SetGreenLED()
{
	GPIO_SetBits(GPIOD, GPIO_Pin_12);
}
void ResetRedLED()
{
	GPIO_ResetBits(GPIOD, GPIO_Pin_14);
}
void ResetGreenLED()
{
	GPIO_ResetBits(GPIOD, GPIO_Pin_12);
}
