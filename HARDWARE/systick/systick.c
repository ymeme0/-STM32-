#include "Systick.h"

		   
/*��ʼ���ӳٺ���
SYSTICK��ʱ�ӹ̶�ΪAHBʱ��(180MHZƵ��)��SYSCLK:ϵͳʱ��Ƶ��  */
void Systick_init()
{
 	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);  // 8��Ƶ   Ƶ��Ϊ22.5MHZ   1s = 22500000��

}								    

/*��ʱnus
nusΪҪ��ʱ��us��.  ע��:nus��ֵ,SysTick->LOADΪ24λ�Ĵ�������Ҫ����798915us(���ֵ��2^24/fac_us   ����fac_us=21)*/
void delay_us(u32 nus)
{		
	u32 temp;	    	 
	SysTick->LOAD=nus*22.5; 				//ʱ�����	  		 
	SysTick->VAL=0x00;        				//��ռ�����
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ; //ʹ�ܵδ�ʱ����ʼ���� 	 
	do
	{
		temp=SysTick->CTRL;
	}while((temp&0x01)&&!(temp&(1<<16)));	//�ȴ�ʱ�䵽��   
	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk; //�رռ�����
	SysTick->VAL =0X00;       				//��ռ����� 
}

/*��ʱnms
ע��nms�ķ�Χ��SysTick->LOADΪ24λ�Ĵ���,����,�����ʱΪ:
nms<=0xffffff*8*1000/SYSCLK��SYSCLK��λΪHz,nms��λΪms����168M������,nms<=798ms */
void delay_ms(u16 nms)
{	 		  	  
	u32 temp;		   
	SysTick->LOAD=(u32)nms*22500;				//ʱ�����(SysTick->LOADΪ24bit)
	SysTick->VAL =0x00;           				//��ռ�����
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ;    //�ܵδ�ʱ����ʼ���� 
	do
	{
		temp=SysTick->CTRL;
	}while((temp&0x01)&&!(temp&(1<<16)));		//�ȴ�ʱ�䵽��   
	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;    //�رռ�����
	SysTick->VAL =0X00;     		  			//��ռ�����	  	    
} 




































