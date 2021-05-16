#include "Systick.h"

		   
/*初始化延迟函数
SYSTICK的时钟固定为AHB时钟(180MHZ频率)，SYSCLK:系统时钟频率  */
void Systick_init()
{
 	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);  // 8分频   频率为22.5MHZ   1s = 22500000次

}								    

/*延时nus
nus为要延时的us数.  注意:nus的值,SysTick->LOAD为24位寄存器，不要大于798915us(最大值即2^24/fac_us   其中fac_us=21)*/
void delay_us(u32 nus)
{		
	u32 temp;	    	 
	SysTick->LOAD=nus*22.5; 				//时间加载	  		 
	SysTick->VAL=0x00;        				//清空计数器
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ; //使能滴答定时器开始倒数 	 
	do
	{
		temp=SysTick->CTRL;
	}while((temp&0x01)&&!(temp&(1<<16)));	//等待时间到达   
	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk; //关闭计数器
	SysTick->VAL =0X00;       				//清空计数器 
}

/*延时nms
注意nms的范围，SysTick->LOAD为24位寄存器,所以,最大延时为:
nms<=0xffffff*8*1000/SYSCLK，SYSCLK单位为Hz,nms单位为ms，对168M条件下,nms<=798ms */
void delay_ms(u16 nms)
{	 		  	  
	u32 temp;		   
	SysTick->LOAD=(u32)nms*22500;				//时间加载(SysTick->LOAD为24bit)
	SysTick->VAL =0x00;           				//清空计数器
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ;    //能滴答定时器开始倒数 
	do
	{
		temp=SysTick->CTRL;
	}while((temp&0x01)&&!(temp&(1<<16)));		//等待时间到达   
	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;    //关闭计数器
	SysTick->VAL =0X00;     		  			//清空计数器	  	    
} 




































