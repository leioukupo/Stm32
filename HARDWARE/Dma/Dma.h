#ifndef __DMA_H
#define __DMA_H
#include "sys.h"

void MYDMA_Config(DMA_Channel_TypeDef*DMA_CHx,u32 cpar,u32 cmar,u16 cndtr);//配置DMA几_CH几

void MYDMA_Enable(DMA_Channel_TypeDef*DMA_CHx);//使能DMA几 _CH几
		   
#endif
