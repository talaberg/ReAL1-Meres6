/**
*****************************************************************************
**
**	ARM C mérés
**  Feladat2        : 7seg.c
**
*****************************************************************************
*/



#include "7seg.h"


const uint8_t SegmentTable[16] = { 0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F, 0x77, 0x7C, 0x39, 0x5E, 0x79, 0x71 };


void DisplayDigit( uint8_t digit, uint8_t value, uint8_t dp  )
{
	uint16_t t;
	if ((0<=value) && (value <=15) && (digit >= 0) && (digit <= 3))
	{
		GPIO_SetBits( GPIOD, GPIO_Pin_10 );
		digit % 2 ? GPIO_SetBits( GPIOB, GPIO_Pin_14 ) : GPIO_ResetBits( GPIOB, GPIO_Pin_14 );
		(digit/2) % 2 ? GPIO_SetBits( GPIOB, GPIO_Pin_15 ) : GPIO_ResetBits( GPIOB, GPIO_Pin_15 );
		t = (GPIO_ReadOutputData( GPIOE ) & 0x00FF);
		t |= (((uint16_t)( SegmentTable[value] + (dp ? 0x80 : 0x00) ))<<8);
		GPIO_Write( GPIOE , t );
		GPIO_SetBits( GPIOD, GPIO_Pin_2 );
		GPIO_ResetBits( GPIOD, GPIO_Pin_2 );
		GPIO_ResetBits( GPIOD, GPIO_Pin_10 );
	}
}
