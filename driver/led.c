#include "led.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK战舰STM32开发板
//LED驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2012/9/2
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	   

//初始化PB5和PE5为输出口.并使能这两个口的时钟		    
//LED IO初始化
void LED_Init(void)
{
	//RCC->APB2ENR|=1<<3;    //使能PORTB时钟	   	 
	RCC->APB2ENR|=1<<6;    //使能PORTE时钟	
	   	 
	//GPIOB->CRL&=0XFF0FFFFF; 
	//GPIOB->CRL|=0X00300000;//PB.5 推挽输出   	 
   // GPIOB->ODR|=1<<5;      //PB.5 输出高
											  
	GPIOE->CRL&=0XFF00FFFF;
	GPIOE->CRL|=0X00330000;//PE.45推挽输出
	//GPIOE->ODR|=0x03<<4;      //PE.45输出高//...
	GPIOE->ODR  &= ~(0x03<<4);
	
//	GPIOE->ODR &= ~(0x03<<4);      //PE.45输出高
}

