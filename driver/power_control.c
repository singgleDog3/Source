#include "sys.h"

void power_control_pin_on(void)
{
    GPIOE->ODR |= 1<<2;
}

void power_control_pin_off(void)
{
    GPIOE->ODR &= (~(1<<2));
}

void power_control_pin_init(void)
{
    RCC->APB2ENR |= 1<<6;

    GPIOE->CRL &= 0XFFFFF0FF;
//    GPIOE->CRL |= 0X00000500;//...
		GPIOE->CRL |= 1<<8;
    power_control_pin_off();
}


void HC244_disable(void)
{
		GPIOC->ODR |= 1<<2;
}


void HC244_enable(void)
{
		GPIOC->ODR &= ~(1<<2);
}


void HC244_ctrl_pin_init(void)
{
		RCC->APB2ENR |= 1<<4;

		GPIOC->CRL &= ~(0xf<<8);
		GPIOC->CRL |= 1<<8;
		HC244_disable();
}

