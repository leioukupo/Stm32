#ifndef __DAC_H
#define __DAC_H	 
#include "sys.h"
#include "stm32f10x_rcc.h"	    
void Dac1_init(void);
void Dac1_Set_Vol(u16 vol);
#endif