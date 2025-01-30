#ifndef _BH1750_H_
#define _BH1750_H_

#include "stm32f3xx_hal.h"
#include "stm32f3xx.h"

#define BH1750_Addr_H 0x5C
#define BH1750_Addr_L 0x46

#define GOOG   						0x00
#define FAIL   						0x01

#define BH1750_DOWN   		0x00
#define BH1750_ON					0x01
#define BH1750_RESET			0x07

#define BH_CON_H_MODE			0x10
#define BH_CON_H_MODE2		0x11
#define BH_CON_L_MODE			0x13
#define BH_ONE_H_MODE			0x20
#define BH_ONE_H_MODE2		0x21
#define BH_ONE_L_MODE			0x23

typedef struct
	{
		uint8_t 	mode;
		uint32_t	time;
		uint16_t	data_re[2];
		uint16_t	result_lx;
		uint8_t		status;
	} BH1750_DATA ;

void BH1750_Init(void);  
void BH1750_Start(void);			
uint16_t BH1750_Read(void);
void BH1750_Shutdown(void);

#endif

