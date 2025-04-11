#include "include.h"

void read_dht22_humidity_and_temperature(uint16_t* temp,uint16_t* hum,uint16_t*  c_sum, signed char* minus)
{
	uint16_t humidity=0x0000, temperature=0x0000; //(16 bits)
	uint8_t checksum=0x00; //8bits checksum
	unsigned char i=0;//i: counter the number of a frame (40 bit).
	*minus=1; //initial alway positive

	/*Start Signal*/
	RCC->AHBENR|=1; //GPIOA ABH bus clock On.
	GPIOA->MODER|=0x1000; //GPIOA pin 6 to OUTPUT. p184
	GPIOA->ODR|=0x40; //0100 0000 set Pin 6 HIGH. p186
	delay_Ms(1);
	GPIOA->ODR&=~0x40; //0011 1111 low-state at least 500 us -> Pin 6 LOW
	delay_Ms(1);
	GPIOA->ODR|=0x40; //pin 6 high state and sensor gives this 20us-40us -> Pin 6 HIGH
	delay_Us(40);
	GPIOA->MODER&=~0x3000; // ~0011 0000 0000 0000 GPIOA pin 6 to INPUT. p184


	//wait for response from sensor
	while((GPIOA->IDR & 0x40)){}//Wait forever until Input Data Register Pin 6 LOW
	while(!(GPIOA->IDR & 0x40)){}//Wait forever until Input Data Register Pin 6 HIGH
	while((GPIOA->IDR & 0x40)){}//Wait forever until Input Data Register Pin 6 LOW

	//read values from sensor
	while(i<40)
	{
		while(!(GPIOA->IDR&0x40)){}//Wait forever until Input Data Register Pin 6 HIGH
		delay_Us(50);
		/*check if PIN6 still HIGH over 45us, the vol-lengh mean data "1"
		 * because vol-lengh 26-28us mean "0" and 70us mean "1"
		 * then it have 80us LOW before next data because "0"or"1" depend on TIME of HIGH.
		*/
		if((GPIOA->IDR & 0x40)&&i<16)
		{
			humidity|=(0x8000>>i); //if "1" put it from left to right

		}
		else if((GPIOA->IDR & 0x40) && i>=16 && i<32)
		{
			//Check minus temperature
			if(i==16){
				*minus=-1; //if first bit "1" mean negative temp datasheet AM2302 p5
			}
			temperature|=(0x8000>>(i-16)); //because temp have 16 bit start from 0 so i-16
		}
		else if((GPIOA->IDR & 0x40)&&i>=32){
			checksum|=(0x80>>(i-32)); //because sum have 8 bit start from 0 so i-32
		}
		i++;
		while((GPIOA->IDR & 0x40)){}
	}

	/*Check sum
	 * Checksum=(Byte 0+Byte 1+Byte 2+Byte 3)&0xFF
	 * * The & 0xFF ensures only the least significant 8 bits are considered.
	 * * if the computed checksum matches Byte 4, the data is valid.
	 * */
	uint8_t h_t=0x00, l_t=0x00,h_h=0x00, l_h=0x00,s=0x00;
	h_t=(temperature>>8); //first byte of temp
	l_t=(temperature&0x00FF); //second byte of temp
	h_h=(humidity>>8);
	l_h=(humidity&0x00FF);
	s=(h_t+l_t+h_h+l_h)&0xFF;
	if(s==checksum){
		*c_sum=checksum;
	}

	/*Return results*/
	temperature &= ~(1<<15); //Change first bit to 0
	*hum=humidity;
	*temp=temperature;
}