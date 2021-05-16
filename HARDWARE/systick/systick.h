#ifndef __SYSTICK_H
#define __SYSTICK_H 			   


#include "stm32f4xx.h"  
void Systick_init(void);
void delay_ms(u16 nms);
void delay_us(u32 nus);

#endif





























