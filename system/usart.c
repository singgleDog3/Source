#include "sys.h"
#include "usart.h"
#include "uart2_download.h"
#include "timer.h"

////////////////////////////////////////////////////////////////////////////////// 	 
//如果使用ucos,则包括下面的头文件即可.
#if SYSTEM_SUPPORT_UCOS
#include "includes.h"					//ucos 使用	  
#endif  
 

//////////////////////////////////////////////////////////////////
//加入以下代码,支持printf函数,而不需要选择use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 
	/* Whatever you require here. If the only file you are using is */ 
	/* standard output using printf() for debugging, no file handling */ 
	/* is required. */ 
}; 
/* FILE is typedef’ d in stdio.h. */ 
FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
_sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{      
	while((USART2->SR&0X40)==0);//循环发送,直到发送完毕   
	USART2->DR = (u8) ch;      
	return ch;
}

int fputs(const char *str, FILE * FIL )
{
	uint32_t num = 0;
	
	while(*(str + num) != '\0'){
		fputc(*(str + num), FIL);
		num++;
	}
	return 0;
}

#endif 
//end
//////////////////////////////////////////////////////////////////

#if EN_USART1_RX   //如果使能了接收
//串口1中断服务程序
//注意,读取USARTx->SR能避免莫名其妙的错误   	
u8 USART_RX_BUF[USART_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.
//接收状态
//bit15，	接收完成标志
//bit14，	接收到0x0d
//bit13~0，	接收到的有效字节数目
u16 USART_RX_STA=0;       //接收状态标记	  
  
void USART1_IRQHandler(void)
{
	u8 res;	
#ifdef OS_CRITICAL_METHOD 	//如果OS_CRITICAL_METHOD定义了,说明使用ucosII了.
	OSIntEnter();    
#endif
	if(USART1->SR&(1<<5))//接收到数据
	{	 
		res=USART1->DR; 
		if((USART_RX_STA&0x8000)==0)//接收未完成
		{
			if(USART_RX_STA&0x4000)//接收到了0x0d
			{
				if(res!=0x0a)USART_RX_STA=0;//接收错误,重新开始
				else USART_RX_STA|=0x8000;	//接收完成了 
			}else //还没收到0X0D
			{	
				if(res==0x0d)USART_RX_STA|=0x4000;
				else
				{
					USART_RX_BUF[USART_RX_STA&0X3FFF]=res;
					USART_RX_STA++;
					if(USART_RX_STA>(USART_REC_LEN-1))USART_RX_STA=0;//接收数据错误,重新开始接收	  
				}		 
			}
		}  		 									     
	}
#ifdef OS_CRITICAL_METHOD 	//如果OS_CRITICAL_METHOD定义了,说明使用ucosII了.
	OSIntExit();  											 
#endif
} 
#endif										 

extern void lower_uart_rx(u8 data);
static u32 ReceiveDataLenght = 0;


void USART2_IRQHandler(void)
{
    u8 res;
		uint8_t temp_str[250];
		uint32_t temp;
		static uint8_t uart2_flag = 1;
	
    if(USART2->SR & (1<<5)) {
        res = USART2->DR;		
				Timer3_Clear();
				uart2_DownloadEntry(res);
				
        //lower_uart_rx(res);
    }
}

void uart2_tx(u8 c)
{
    while((USART2->SR&0X40)==0);
    USART2->DR = c;
}

void uart1_tx(u8 c)
{
    while((USART1->SR&0X40)==0);
	USART1->DR = (u8) c;
}

//初始化IO 串口1
//pclk2:PCLK2时钟频率(Mhz)
//bound:波特率
//CHECK OK
//091209
void uart_init(u32 pclk2,u32 bound)
{  	 
	float temp;
	u16 mantissa;
	u16 fraction;
    
	temp=(float)(pclk2*1000000)/(bound*16);//得到USARTDIV
	mantissa=temp;				 //得到整数部分
	fraction=(temp-mantissa)*16; //得到小数部分	 
    mantissa<<=4;
	mantissa+=fraction; 
    
	RCC->APB2ENR|=1<<2;   //使能PORTA口时钟  
	RCC->APB2ENR|=1<<14;  //使能串口时钟 
	GPIOA->CRH&=0XFFFFF00F;//IO状态设置
	GPIOA->CRH|=0X000008B0;//IO状态设置

	RCC->APB2RSTR|=1<<14;   //复位串口1
	RCC->APB2RSTR&=~(1<<14);//停止复位	   	   
	//波特率设置
 	USART1->BRR=mantissa; // 波特率设置	 
	USART1->CR1|=0X200C;  //1位停止,无校验位.
#if EN_USART1_RX		  //如果使能了接收
	//使能接收中断

	USART1->CR1|=1<<8;    //PE中断使能
	USART1->CR1|=1<<5;    //接收缓冲区非空中断使能	    	
	MY_NVIC_Init(3,3,USART1_IRQChannel,2);//组2，最低优先级 

#endif

#if 1   //USART2 initialization
  RCC->APB1ENR|=1<<17;  //使能串口时钟 
#if UART_DOWNLOAD
  GPIOA->CRL&=0XFFFF00FF;//IO状态设置
	GPIOA->CRL|=0X00008B00;//IO状态设置
#elif SPI_DOWNLOAD
	 
#endif

	RCC->APB1RSTR|=1<<17;   //复位串口2
	RCC->APB1RSTR&=~(1<<17);//停止复位	   	   
	//波特率设置
 	USART2->BRR=mantissa; // 波特率设置	 
	USART2->CR1|=0X200C;  //1位停止,无校验位.

	//使能接收中断
	USART2->CR1|=1<<8;    //PE中断使能
	USART2->CR1|=1<<5;    //接收缓冲区非空中断使能	    	
	MY_NVIC_Init(3,3,USART2_IRQChannel,2);//组2，最低优先级 
#endif 


}

void uart2_ChangeBaudRate(unsigned int bound)
{
		float temp;
	u16 mantissa;
	u16 fraction;
    
	temp=(float)(36*1000000)/(bound*16);//得到USARTDIV
	mantissa=temp;				 //得到整数部分
	fraction=(temp-mantissa)*16; //得到小数部分	 
    mantissa<<=4;
	mantissa+=fraction; 
	
	USART2->CR1&= ~ (1<<8);    //PE中断使能
	USART2->BRR=mantissa; // 波特率设置	
	USART2->CR1 |=  (1<<8); 	
}

void uart2_WriteChar(u8 ch)
{
	while((USART2->SR & (0x01<<6)) == 0);
	USART2->DR = ch;
}

void uart2_WriteStr(char *str, unsigned int length)
{
	u32 num = 0;
	while(num < length){
		uart2_WriteChar(*(str+num));
		num++;
	}
}


void uart2_DownloadData(u8 *str, u32 length)
{

}


//检测开始信号
u32 CheckStartSingle(void)
{
	u8 StartFlag = 0;

	if(GPIOB->IDR & 0x01 == 0x01){
		delay_ms(20);
		if(GPIOB->IDR & 0x01 == 0x01)
			StartFlag = 0x01;
	}
	else 
		StartFlag = 0x00;
	return StartFlag;
}

//初始化PB0脚，输入；
void InitStartPin(void)
{
	// mode 
	GPIOB->CRL &= (~(0x03<<0));
	GPIOB->CRL |= (0x00 << 0);
	
	//pull_up and pull_down
	GPIOB->CRL &= (~(0x03<<2));
	GPIOB->CRL |= (0x02 << 2);
	
	// open MR0
	EXTI->IMR &= ~(0x01 << 0);
	EXTI->IMR |= 0x01;
	
	//上升沿触发
	EXTI->RTSR &= ~(0x01 << 0);
	EXTI->RTSR |= 0x01;
	// 禁止下降沿触发
	EXTI->FTSR &= ~(0x01 <<0);
	
	//
	AFIO->EXTICR[0] &= (~(0x0f <<0));
	AFIO->EXTICR[0] |= 0x01;
	
	MY_NVIC_Init(3,3,EXTI0_IRQChannel,2);
	
}


u8 StartFlag = 0;
void EXTI0_IRQHandler(void)
{
	EXTI->IMR &= ~(0x01 << 0);
	EXTI->PR &= (0x01<<0);
	
	StartFlag = 1;
	EXTI->IMR |= (0x01 << 0);
}


void Timer1_Init(void)
{
	//
//	TIM1->CR1
//	
//	
//	TIM1->PSC = 0x0009;
//	TIM1->ARR = 60000 ;
}




