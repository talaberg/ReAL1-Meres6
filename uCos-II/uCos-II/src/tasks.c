/* Includes */
#include "stm32f4xx.h"
#include "ucos_ii.h"
#include "bsp.h"
#include "stdio.h"

/* Variables */
//2. feladat TODO : globális változó a hőmérséklet tárolására


/* Private function declarations */
void SetTemperatureValue(uint16_t value);
void SetLimitValue(uint16_t value);
uint16_t GetLimitValue();
uint16_t GetTemperatureValue();
void SetDisplayValue(uint8_t value);
void DecrementLimitValue();
void IncrementLimitValue();
void SendMessage(char* message);

/* The Demo Task */
void Task_Demo(void* param)
{
	uint16_t ledVal = 0x1000;
	uint16_t portVal;
	while(1)
	{
		ledVal <<= 1;
		if ( ledVal == 0)
			ledVal = 0x1000;
		portVal = GPIO_ReadOutputData(GPIOD);
		portVal &= 0x0fff;
		portVal |= ledVal;
		GPIO_Write(GPIOD,portVal);
		OSTimeDly(OS_TICKS_PER_SEC/4);
	}
	
	//4. feladat --> jelző flag figyelése, ha jelzett, akkor piros led-et villantunk, ha nem, akkor zöldet
	//OSFlagPend, OSFlagPost, SetRedLED, SetGreenLED, ResetRedLED, ResetGreenLED
	
	//6. feladat --> 4 feladat kibővítése --> riasztás küldése
	//SendMessage

}
void Task_Main(void* param)
{
	//1. feladat TODO : tizedmásodperecenként számol, számlálót megjelenít a kijelzőn
	//meghívni: OSTimeDly INT32U ticks
	
	//2. feladat: hőmérő - TODO: hőmérés --> glob változó
	// GetI2CTemp, SetTemperatureValue --> megírni
	
	//3. feladat : TODO: 1-es kapcsoló beolvasása
	//Kapcsoló állapotától függően: hőm. vagy limit a kijelzőre
	//GetTemperatureValue, SetTemperatureValue, GetLimitValue,SetLimitValue, SetDisplayValue, GetDisplaySwitchState, GetADCValue
	//Timer4 --> megjelnítésért felelős, pDisplayVal -t kéri le
	
	//4. feladat --> 2-es kapcsoló állásának figyelése --> ha 1 akkor jelzőflag beállítása
	//GetModeSwitchState
	
	//5. feladat --> 4-es módosítása, nem a kapcsolót figyeljük, hanem az aktuális hőmérsékletet
	//GetTemperatureValue, GetLimitValue, OSFlagPost
	
	
}

/*
void Task_UI(void* param)
{	//7. feladat - UI kezelő taszk
}*/

/* Set the temperature value */
void SetTemperatureValue(uint16_t value)
{
	/* TODO: Set the value of TempVal */
	//2. feladat TODO: OSSemPend, OSSemPost, ... glob. vált. írás
}

/* Read the temperature value */
uint16_t GetTemperatureValue()
{
	/* TODO: Return the value of TempVal */
	return 0;
	//3. feladat TODO: OSSemPend, OSSemPost
}

/* Set the limit value */
void SetLimitValue(uint16_t value)
{
	/* TODO: Set the value of LimitVal */
}

/* Read the temperature value */
uint16_t GetLimitValue()
{
	/* TODO: Return the value of LimitVal */
	return 0;
}

/* Increment the limit value */
void IncrementLimitValue()
{
	/* TODO: Change the value of LimitVal */
}

/* Decrement the limit value */
void DecrementLimitValue()
{
	/* TODO: Change the value of LimitVal */
}

/* Set the display value */
void SetDisplayValue(uint8_t value)
{
	// The input parameter is a boolean
	value &=1;

	/* TODO: Change the value of pDisplayVal */
}

/* Send a message on USART */
void SendMessage(char* message)
{
//6. feladat
//SendMessage, USARTSendString -->(bsp.c-ben)
}
