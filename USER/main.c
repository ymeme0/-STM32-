#include "stm32f4xx.h"
#include "Systick.h"
#include <stdio.h>

#define DHT_SET	  	GPIO_SetBits(GPIOI,GPIO_Pin_11)
#define DHT_RESET	GPIO_ResetBits(GPIOI,GPIO_Pin_11)
#define DHT_READ	GPIO_ReadInputDataBit(GPIOI, GPIO_Pin_11)
unsigned char dht_buf[5] = {0};
unsigned int distance = 0;

static  uint8_t  uart1_recv_buf[128] = {0};  //保存串口1接收到的数据
static  uint8_t  uart3_recv_buf[128] = {0};  //保存串口3接收到的数据

uint8_t	 uart1_recv_cnt = 0;  //记录串口1接收到的数据的个数
uint8_t	 uart3_recv_cnt = 0;  //记录串口3接收到的数据的个数
//因为printf()函数内部调用
int fputc(int ch,FILE * f)
{
	USART_SendData(USART1,ch);//利用UART1把收到的数据发送出去
	while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);//等待数据发送完成
	
	return ch;
}


//串口1的初始化   针对于电脑
void  uart1_init()
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	//1.打开时钟  PA9  PA10
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

	//2.配置GPIO  + 初始化GPIO
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9|GPIO_Pin_10;   					//引脚
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF; 								//复用模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 								//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;  						//引脚速率
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;  
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	//3.设置引脚复用
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);
	
	//4.配置UART + 初始化UART
	USART_InitStructure.USART_BaudRate = 9600;		//波特率 9600
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;  //数据位 8bit
	USART_InitStructure.USART_StopBits = USART_StopBits_1;		//停止位  1bit
	USART_InitStructure.USART_Parity = USART_Parity_No;     //无校验
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  //无硬件流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;  //收发模式
	USART_Init(USART1,&USART_InitStructure);

	//5.配置NVIC + 初始化NVIC
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	//6.打开串口中断   接收到数据后发生中断
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

	//7.打开串口
	 USART_Cmd(USART1,ENABLE);
}

//串口3的初始化  针对于蓝牙
void  usart3_init()
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	//1.打开时钟  PB10  PB11  USART3
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);
	
	//2.配置GPIO  + 初始化GPIO
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_11|GPIO_Pin_10;   					//引脚
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF; 								//复用模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 								//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;  						//引脚速率
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;  
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	//3.设置引脚复用
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);
	
	//4.配置UART + 初始化UART
	USART_InitStructure.USART_BaudRate = 9600;		//波特率 9600
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;  //数据位 8bit
	USART_InitStructure.USART_StopBits = USART_StopBits_1;		//停止位  1bit
	USART_InitStructure.USART_Parity = USART_Parity_No;     //无校验
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  //无硬件流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;  //收发模式
	USART_Init(USART3,&USART_InitStructure);

	//5.配置NVIC + 初始化NVIC
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	//6.打开串口中断   接收到数据后发生中断
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);

	//7.打开串口
	USART_Cmd(USART3,ENABLE);

}

//利用串口3发送多个字节  pbuf表示数组    len表示数据个数
void usart3_sendbytes(uint8_t * pbuf,uint8_t len)
{
	while(len--)
	{
		USART_SendData(USART3,*pbuf++);
		while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);//等待数据发送完成
	}
}


//DHT切换输出模式
void  dht_outputmode()
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOI, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_11;   								//引脚
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT; 								//输出模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 								//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;  						//引脚速率
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;  
	GPIO_Init(GPIOI, &GPIO_InitStructure);
	
	GPIO_SetBits(GPIOI, GPIO_Pin_11);//输出高电平
}


//DHT切换输入模式
void dht_inputmode()
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOI, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_11;   								//引脚
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN; 								//输入模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 								//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;  						//引脚速率
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;  
	GPIO_Init(GPIOI, &GPIO_InitStructure);

}

//开始信号
void dht_start()
{
	//1.设置PI11为输出模式
	dht_outputmode();
	//2.把总线拉低  至少持续18ms
	DHT_RESET;
	delay_ms(20);
	//3.把总线拉高  持续20~40us
	DHT_SET;
	delay_us(30);
}


//检测是否响应  返回1  表示没响应  返回0  表示响应了
unsigned int dht_check()
{
	unsigned int i = 0;
	//1.设置引脚为输入模式
	dht_inputmode();
	//2.判断引脚是否出现低电平  		（超时处理）
	while( DHT_READ == 1 && i<100 )
	{
		i++;
		delay_us(1);
	}
	if(i>100)
		return 1; //说明没响应
	else
		i=0;
	//3.判断低电平的时间是否足够80us  	（超时处理）
	while( DHT_READ == 0 && i<100 )
	{
		i++;
		delay_us(1);
	}
	if(i>100)
		return 1; //说明没响应
	else
		return 0; //说明响应了
}

//读取1bit数据  返回0  表示数字0   返回1  表示数字1
unsigned char dht_read_bit()
{
	unsigned int i = 0;
	//1.判断是否出现了低电平			（超时处理）
	while(DHT_READ == 1 && i<100)
	{
		i++;
		delay_us(1);
	}
	i=0;
	//2.判断是否出现了高电平
	while(DHT_READ == 0 && i<100)
	{
		i++;
		delay_us(1);
	}
	//3.延时一段时间  建议40us
	delay_us(40);
	//4.判断引脚电平
	if(DHT_READ == 1)
		return 1;                                                                
	else
		return 0;
}

//读取一个字节
unsigned char dht_read_byte()
{
	//1.定义变量保存读取到的数据
	unsigned char i = 0;
	unsigned char data = 0; //0000 0000
	
	//2.循环读取8次
	for(i=0;i<8;i++)
	{
		data <<= 1; 
		data |= dht_read_bit();
		
	}

	return data;
}

//读取整个数据  返回1  表示失败  返回0 表示成功
unsigned int dht_read_data(unsigned char *buf)
{
	unsigned int i = 0;
	//1.发送开始信号
	dht_start();
	//2.判断是否响应
	if( dht_check() == 0 )
	{
		for(i=0;i<5;i++)
		{
			buf[i] = dht_read_byte();
		}
		if(buf[0]+buf[1]+buf[2]+buf[3] != buf[4])
			return 1;
	}
	else
		return 1;

	return 0;
}


//超声波的初始化
void sr04_init()
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	
	//1.打开时钟  PE2--TRIG  PE3--ECHO
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

	//2.配置GPIO  + 初始化GPIO
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 					//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;  			//引脚速率
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;  
	
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_2;						//引脚
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT; 					//输出模式
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN; 					//输入模式
	GPIO_Init(GPIOE, &GPIO_InitStructure);
}

//led灯的初始化
void led123_init()
{
	GPIO_InitTypeDef  GPIO_InitStructure;

	//1.打开对应端口的时钟  GPIOH
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH, ENABLE);

	//2.配置GPIO的结构体  0400   
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12;   		//引脚
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT; 								//输出模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 								//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;  						//引脚速率
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;   

	//3.初始化GPIO
	GPIO_Init(GPIOH, &GPIO_InitStructure);
}

//蜂鸣器的初始化
void beep123_init()
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	
	//1.打开对应端口的时钟  GPIOB
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	//2.配置GPIO的结构体    
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_5;   								//引脚
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT; 								//输出模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 								//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;  						//引脚速率
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;   

	//3.初始化GPIO
	GPIO_Init(GPIOB, &GPIO_InitStructure);

}
	
//获取超声波距离
unsigned int sr04_get_distance()
{
	unsigned int i = 0;
	
	//1.TRIG空闲状态下输出低电平
	GPIO_ResetBits(GPIOE,GPIO_Pin_2);
	
	//2.TRIG输出高电平，并且持续至少10us  建议15us
	GPIO_SetBits(GPIOE,GPIO_Pin_2);
	delay_us(15);
	GPIO_ResetBits(GPIOE,GPIO_Pin_2);
	
	//3.等待ECHO出现高电平
	while( GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_3) == 0 );
	
	//4.计算时间
	while( GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_3) == 1 )
	{
		i++;
		delay_us(9);
	}
	
	//5.计算距离
	i = i / 2;

	return i * 3;  //距离 = 时间 * 精度  单位mm
}




int main()
{
	led123_init();
	beep123_init();
	uart1_init();
	Systick_init();//系统时钟初始化
	sr04_init();
	//usart3_init(); //波特率9600
	while(1)
	{	distance = sr04_get_distance();
		dht_read_data(dht_buf);
		if(distance<200)
		{
			//1.蜂鸣器进行报警提示
			//2.手机会收到提示信息
			while(1)
			{
				GPIO_SetBits(GPIOB,GPIO_Pin_5); //输出高电平  蜂鸣器响	 
				delay_ms(500);
				delay_ms(500);
				printf("主人，有陌生人靠近！\n");
			}
		}else if(distance>=200&&distance<400)
		{
			//1.LED灯亮两盏
			//2.蜂鸣器不报警
			//3.手机不会收到信息
			while(1)
			{
				GPIO_ResetBits(GPIOH,GPIO_Pin_10|GPIO_Pin_11); //输出低电平  灯亮	 
				GPIO_ResetBits(GPIOB,GPIO_Pin_5); //输出低电平  蜂鸣器不响	 
			}	
		}else
		{
			//1.LED灯不亮
			//2.蜂鸣器不报警
			//3.手机不会收到信息	
			while(1)
			{
				GPIO_SetBits(GPIOH,GPIO_Pin_10|GPIO_Pin_11); //输出高电平  灯灭	
				GPIO_ResetBits(GPIOB,GPIO_Pin_5); //输出低电平  蜂鸣器不响	 
			}		
		}
		
		if(dht_buf[2]>=26)
		{
			delay_ms(500);
			delay_ms(500);
			delay_ms(500);
			printf("主人，室内温度过高，请及时降温\n");
		}
		if(dht_buf[0]>70)
		{
			delay_ms(500);
			delay_ms(500);
			delay_ms(500);
			printf("主人，室内湿度过高，请及时通风\n");
		}
	}
}

void  USART1_IRQHandler(void)
{
	uint8_t recv_data;//保存接收到的数据
	
	//判断是否发生了中断
	if(  USART_GetITStatus(USART1,USART_IT_RXNE) != RESET )
	{
		uart1_recv_buf[uart1_recv_cnt] = USART_ReceiveData(USART1);//把接收到的数据保存在这个数组中
		uart1_recv_cnt++;
		
		//确保所接收到数据必须为\r\n结尾
		if(uart1_recv_buf[uart1_recv_cnt-1] == '\n')
		{
		
			usart3_sendbytes(uart1_recv_buf,uart1_recv_cnt);
			
			uart1_recv_cnt = 0;
		}
	}
}

//针对于蓝牙
void  USART3_IRQHandler(void)
{
	uint8_t recv_data;//保存接收到的数据
	
	//判断是否发生了中断
	if(  USART_GetITStatus(USART3,USART_IT_RXNE) != RESET )
	{
		recv_data = USART_ReceiveData(USART3);//把串口3接收到的数据保存在这个变量中
		USART_SendData(USART1,recv_data);//利用UART1把收到的数据发送出去
		while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)//等待串口1的数据发送完成
		{
			while(recv_data =='t')
			{
				while(1)
				{
					delay_ms(500);
					delay_ms(500);
					delay_ms(500);
					dht_read_data(dht_buf);
					printf("温度= %d ℃\n",dht_buf[2]);
				}	
			}
			while(recv_data =='h')
			{
				while(1)
				{
					delay_ms(500);
					delay_ms(500);
					delay_ms(500);
					dht_read_data(dht_buf);
					printf("湿度 = %d RH\n",dht_buf[0]);
				}	
			}
			while(recv_data =='0')
			{
				while(1)
				{
					GPIO_ResetBits(GPIOB,GPIO_Pin_5); //蜂鸣器关闭	 
				}
			}
			while(recv_data =='1')
			{
				while(1)
				{
					GPIO_SetBits(GPIOB,GPIO_Pin_5); //蜂鸣器开启
				}
			}
			while(recv_data =='2')
			{
				while(1)
				{
					GPIO_SetBits(GPIOH,GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12); //LED灯全亮
				}
			}
			while(recv_data =='3')
			{
				while(1)
				{
					GPIO_ResetBits(GPIOH,GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12); //LED灯全灭 
				}
			}
			while(recv_data =='4')
			{
				while(1)
				{
					GPIO_ResetBits(GPIOH,GPIO_Pin_10); //第一盏LED灯亮 
				}
			}		
		}
	}
}
