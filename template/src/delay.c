#include "include.h"


void delay_Ms(int delay)
{
	int i=0;
	for(; delay>0;delay--)
		for(i=0;i<2460;i++); //measured with oscilloscope
}

void delay_Us(int delay)
{
	for(int i = 0;i<(delay*2);i++)	//accurate range 10us-100us
	{
		asm("mov r0, r0");
	}
}

void delay_us(unsigned long delay)
{
	unsigned long i = 0;
	RCC->APB2ENR|=(1<<4); 	//TIM11EN: Time 11 clock enable. p160
	TIM11->PSC=32-1;			//32 000 000 MHz / 32 = 1 000 000 Hz. p435
	TIM11->ARR=1;			//TIM11 counter. 1 000 000 Hz / 1000 = 1000 Hz ~ 1ms. p435
	TIM11->CNT=0;			//counter start value = 0
	TIM11->CR1=1;			//TIM11 Counter enabled. p421

	while(i<delay)
	{
		while(!((TIM11->SR)&1)){}	//Update interrupt flag. p427
		i++;
		TIM11->SR &= ~1;			//flag cleared. p427
		TIM11->CNT=0;				//counter start value =0
	}
	TIM11->CR1&=0;					//TIM11 Counter disabled. p421
}