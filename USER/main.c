#include "stm32f4xx.h"
#include "Systick.h"
#include <stdio.h>

#define DHT_SET	  	GPIO_SetBits(GPIOI,GPIO_Pin_11)
#define DHT_RESET	GPIO_ResetBits(GPIOI,GPIO_Pin_11)
#define DHT_READ	GPIO_ReadInputDataBit(GPIOI, GPIO_Pin_11)
unsigned char dht_buf[5] = {0};
unsigned int distance = 0;

static  uint8_t  uart1_recv_buf[128] = {0};  //���洮��1���յ�������
static  uint8_t  uart3_recv_buf[128] = {0};  //���洮��3���յ�������

uint8_t	 uart1_recv_cnt = 0;  //��¼����1���յ������ݵĸ���
uint8_t	 uart3_recv_cnt = 0;  //��¼����3���յ������ݵĸ���
//��Ϊprintf()�����ڲ�����
int fputc(int ch,FILE * f)
{
	USART_SendData(USART1,ch);//����UART1���յ������ݷ��ͳ�ȥ
	while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);//�ȴ����ݷ������
	
	return ch;
}


//����1�ĳ�ʼ��   ����ڵ���
void  uart1_init()
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	//1.��ʱ��  PA9  PA10
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

	//2.����GPIO  + ��ʼ��GPIO
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9|GPIO_Pin_10;   					//����
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF; 								//����ģʽ
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 								//�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;  						//��������
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;  
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	//3.�������Ÿ���
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);
	
	//4.����UART + ��ʼ��UART
	USART_InitStructure.USART_BaudRate = 9600;		//������ 9600
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;  //����λ 8bit
	USART_InitStructure.USART_StopBits = USART_StopBits_1;		//ֹͣλ  1bit
	USART_InitStructure.USART_Parity = USART_Parity_No;     //��У��
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  //��Ӳ��������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;  //�շ�ģʽ
	USART_Init(USART1,&USART_InitStructure);

	//5.����NVIC + ��ʼ��NVIC
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	//6.�򿪴����ж�   ���յ����ݺ����ж�
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

	//7.�򿪴���
	 USART_Cmd(USART1,ENABLE);
}

//����3�ĳ�ʼ��  ���������
void  usart3_init()
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	//1.��ʱ��  PB10  PB11  USART3
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);
	
	//2.����GPIO  + ��ʼ��GPIO
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_11|GPIO_Pin_10;   					//����
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF; 								//����ģʽ
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 								//�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;  						//��������
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;  
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	//3.�������Ÿ���
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);
	
	//4.����UART + ��ʼ��UART
	USART_InitStructure.USART_BaudRate = 9600;		//������ 9600
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;  //����λ 8bit
	USART_InitStructure.USART_StopBits = USART_StopBits_1;		//ֹͣλ  1bit
	USART_InitStructure.USART_Parity = USART_Parity_No;     //��У��
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  //��Ӳ��������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;  //�շ�ģʽ
	USART_Init(USART3,&USART_InitStructure);

	//5.����NVIC + ��ʼ��NVIC
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	//6.�򿪴����ж�   ���յ����ݺ����ж�
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);

	//7.�򿪴���
	USART_Cmd(USART3,ENABLE);

}

//���ô���3���Ͷ���ֽ�  pbuf��ʾ����    len��ʾ���ݸ���
void usart3_sendbytes(uint8_t * pbuf,uint8_t len)
{
	while(len--)
	{
		USART_SendData(USART3,*pbuf++);
		while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);//�ȴ����ݷ������
	}
}


//DHT�л����ģʽ
void  dht_outputmode()
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOI, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_11;   								//����
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT; 								//���ģʽ
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 								//�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;  						//��������
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;  
	GPIO_Init(GPIOI, &GPIO_InitStructure);
	
	GPIO_SetBits(GPIOI, GPIO_Pin_11);//����ߵ�ƽ
}


//DHT�л�����ģʽ
void dht_inputmode()
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOI, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_11;   								//����
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN; 								//����ģʽ
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 								//�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;  						//��������
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;  
	GPIO_Init(GPIOI, &GPIO_InitStructure);

}

//��ʼ�ź�
void dht_start()
{
	//1.����PI11Ϊ���ģʽ
	dht_outputmode();
	//2.����������  ���ٳ���18ms
	DHT_RESET;
	delay_ms(20);
	//3.����������  ����20~40us
	DHT_SET;
	delay_us(30);
}


//����Ƿ���Ӧ  ����1  ��ʾû��Ӧ  ����0  ��ʾ��Ӧ��
unsigned int dht_check()
{
	unsigned int i = 0;
	//1.��������Ϊ����ģʽ
	dht_inputmode();
	//2.�ж������Ƿ���ֵ͵�ƽ  		����ʱ����
	while( DHT_READ == 1 && i<100 )
	{
		i++;
		delay_us(1);
	}
	if(i>100)
		return 1; //˵��û��Ӧ
	else
		i=0;
	//3.�жϵ͵�ƽ��ʱ���Ƿ��㹻80us  	����ʱ����
	while( DHT_READ == 0 && i<100 )
	{
		i++;
		delay_us(1);
	}
	if(i>100)
		return 1; //˵��û��Ӧ
	else
		return 0; //˵����Ӧ��
}

//��ȡ1bit����  ����0  ��ʾ����0   ����1  ��ʾ����1
unsigned char dht_read_bit()
{
	unsigned int i = 0;
	//1.�ж��Ƿ�����˵͵�ƽ			����ʱ����
	while(DHT_READ == 1 && i<100)
	{
		i++;
		delay_us(1);
	}
	i=0;
	//2.�ж��Ƿ�����˸ߵ�ƽ
	while(DHT_READ == 0 && i<100)
	{
		i++;
		delay_us(1);
	}
	//3.��ʱһ��ʱ��  ����40us
	delay_us(40);
	//4.�ж����ŵ�ƽ
	if(DHT_READ == 1)
		return 1;                                                                
	else
		return 0;
}

//��ȡһ���ֽ�
unsigned char dht_read_byte()
{
	//1.������������ȡ��������
	unsigned char i = 0;
	unsigned char data = 0; //0000 0000
	
	//2.ѭ����ȡ8��
	for(i=0;i<8;i++)
	{
		data <<= 1; 
		data |= dht_read_bit();
		
	}

	return data;
}

//��ȡ��������  ����1  ��ʾʧ��  ����0 ��ʾ�ɹ�
unsigned int dht_read_data(unsigned char *buf)
{
	unsigned int i = 0;
	//1.���Ϳ�ʼ�ź�
	dht_start();
	//2.�ж��Ƿ���Ӧ
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


//�������ĳ�ʼ��
void sr04_init()
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	
	//1.��ʱ��  PE2--TRIG  PE3--ECHO
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

	//2.����GPIO  + ��ʼ��GPIO
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 					//�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;  			//��������
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;  
	
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_2;						//����
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT; 					//���ģʽ
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN; 					//����ģʽ
	GPIO_Init(GPIOE, &GPIO_InitStructure);
}

//led�Ƶĳ�ʼ��
void led123_init()
{
	GPIO_InitTypeDef  GPIO_InitStructure;

	//1.�򿪶�Ӧ�˿ڵ�ʱ��  GPIOH
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH, ENABLE);

	//2.����GPIO�Ľṹ��  0400   
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12;   		//����
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT; 								//���ģʽ
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 								//�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;  						//��������
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;   

	//3.��ʼ��GPIO
	GPIO_Init(GPIOH, &GPIO_InitStructure);
}

//�������ĳ�ʼ��
void beep123_init()
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	
	//1.�򿪶�Ӧ�˿ڵ�ʱ��  GPIOB
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	//2.����GPIO�Ľṹ��    
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_5;   								//����
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT; 								//���ģʽ
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 								//�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;  						//��������
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;   

	//3.��ʼ��GPIO
	GPIO_Init(GPIOB, &GPIO_InitStructure);

}
	
//��ȡ����������
unsigned int sr04_get_distance()
{
	unsigned int i = 0;
	
	//1.TRIG����״̬������͵�ƽ
	GPIO_ResetBits(GPIOE,GPIO_Pin_2);
	
	//2.TRIG����ߵ�ƽ�����ҳ�������10us  ����15us
	GPIO_SetBits(GPIOE,GPIO_Pin_2);
	delay_us(15);
	GPIO_ResetBits(GPIOE,GPIO_Pin_2);
	
	//3.�ȴ�ECHO���ָߵ�ƽ
	while( GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_3) == 0 );
	
	//4.����ʱ��
	while( GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_3) == 1 )
	{
		i++;
		delay_us(9);
	}
	
	//5.�������
	i = i / 2;

	return i * 3;  //���� = ʱ�� * ����  ��λmm
}




int main()
{
	led123_init();
	beep123_init();
	uart1_init();
	Systick_init();//ϵͳʱ�ӳ�ʼ��
	sr04_init();
	//usart3_init(); //������9600
	while(1)
	{	distance = sr04_get_distance();
		dht_read_data(dht_buf);
		if(distance<200)
		{
			//1.���������б�����ʾ
			//2.�ֻ����յ���ʾ��Ϣ
			while(1)
			{
				GPIO_SetBits(GPIOB,GPIO_Pin_5); //����ߵ�ƽ  ��������	 
				delay_ms(500);
				delay_ms(500);
				printf("���ˣ���İ���˿�����\n");
			}
		}else if(distance>=200&&distance<400)
		{
			//1.LED������յ
			//2.������������
			//3.�ֻ������յ���Ϣ
			while(1)
			{
				GPIO_ResetBits(GPIOH,GPIO_Pin_10|GPIO_Pin_11); //����͵�ƽ  ����	 
				GPIO_ResetBits(GPIOB,GPIO_Pin_5); //����͵�ƽ  ����������	 
			}	
		}else
		{
			//1.LED�Ʋ���
			//2.������������
			//3.�ֻ������յ���Ϣ	
			while(1)
			{
				GPIO_SetBits(GPIOH,GPIO_Pin_10|GPIO_Pin_11); //����ߵ�ƽ  ����	
				GPIO_ResetBits(GPIOB,GPIO_Pin_5); //����͵�ƽ  ����������	 
			}		
		}
		
		if(dht_buf[2]>=26)
		{
			delay_ms(500);
			delay_ms(500);
			delay_ms(500);
			printf("���ˣ������¶ȹ��ߣ��뼰ʱ����\n");
		}
		if(dht_buf[0]>70)
		{
			delay_ms(500);
			delay_ms(500);
			delay_ms(500);
			printf("���ˣ�����ʪ�ȹ��ߣ��뼰ʱͨ��\n");
		}
	}
}

void  USART1_IRQHandler(void)
{
	uint8_t recv_data;//������յ�������
	
	//�ж��Ƿ������ж�
	if(  USART_GetITStatus(USART1,USART_IT_RXNE) != RESET )
	{
		uart1_recv_buf[uart1_recv_cnt] = USART_ReceiveData(USART1);//�ѽ��յ������ݱ��������������
		uart1_recv_cnt++;
		
		//ȷ�������յ����ݱ���Ϊ\r\n��β
		if(uart1_recv_buf[uart1_recv_cnt-1] == '\n')
		{
		
			usart3_sendbytes(uart1_recv_buf,uart1_recv_cnt);
			
			uart1_recv_cnt = 0;
		}
	}
}

//���������
void  USART3_IRQHandler(void)
{
	uint8_t recv_data;//������յ�������
	
	//�ж��Ƿ������ж�
	if(  USART_GetITStatus(USART3,USART_IT_RXNE) != RESET )
	{
		recv_data = USART_ReceiveData(USART3);//�Ѵ���3���յ������ݱ��������������
		USART_SendData(USART1,recv_data);//����UART1���յ������ݷ��ͳ�ȥ
		while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)//�ȴ�����1�����ݷ������
		{
			while(recv_data =='t')
			{
				while(1)
				{
					delay_ms(500);
					delay_ms(500);
					delay_ms(500);
					dht_read_data(dht_buf);
					printf("�¶�= %d ��\n",dht_buf[2]);
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
					printf("ʪ�� = %d RH\n",dht_buf[0]);
				}	
			}
			while(recv_data =='0')
			{
				while(1)
				{
					GPIO_ResetBits(GPIOB,GPIO_Pin_5); //�������ر�	 
				}
			}
			while(recv_data =='1')
			{
				while(1)
				{
					GPIO_SetBits(GPIOB,GPIO_Pin_5); //����������
				}
			}
			while(recv_data =='2')
			{
				while(1)
				{
					GPIO_SetBits(GPIOH,GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12); //LED��ȫ��
				}
			}
			while(recv_data =='3')
			{
				while(1)
				{
					GPIO_ResetBits(GPIOH,GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12); //LED��ȫ�� 
				}
			}
			while(recv_data =='4')
			{
				while(1)
				{
					GPIO_ResetBits(GPIOH,GPIO_Pin_10); //��һյLED���� 
				}
			}		
		}
	}
}
