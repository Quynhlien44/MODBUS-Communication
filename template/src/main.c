/* Includes */
#include "stm32l1xx.h"
#define HSI_VALUE    ((uint32_t)16000000)
#include <stdio.h>
#include <stdlib.h>
#include "include.h"

#define SLAVE_ADDRESS 0x01
#define REG_DHT22_TEMP 0x01
#define REG_DHT22_HUM  0x02
#define REG_LMT84LP    0x03
#define REG_NSL19M51   0x04
#define REG_HIH4000    0x05
#define REG_GROVE_VOC  0x06
#define REG_GROVE_ECO2 0x07
#define REG_AC_VOLTAGE 0x08
#define REG_ENERGY_KWH 0x09

/* Private typedef */
/* Private define  */
/* Private macro */
/* Private variables */
/* Private function prototypes */
/* Private functions */



/************************************************************************/
/* Flags are volatile, since the flag value can change during interrupt */
/* handler. Therefore compiler can handle it correctly.                 */
/************************************************************************/
volatile char mFlag=0; // 0: idle, 1: correct address, 2: wrong address
volatile uint8_t neFlag = 0;
volatile uint8_t frameFlag = 0;
volatile uint8_t sensor_read_flag = 0; // Timer-based sensor read trigger

void TIM2_IRQHandler(void) {
    if (TIM2->SR & TIM_SR_UIF) {
        sensor_read_flag = 1; // Trigger sensor reading
        TIM2->SR &= ~TIM_SR_UIF; // Clear interrupt flag
    }
}

/*
 * Clear buffer. This is used only for receiver buffer, hence fixed size (8 bytes)
 */
void clear_buffer(char *b)
{
	for (int i=0;i<8;i++)
		b[i] = 0;
}


#define led_on()	GPIOA->ODR|=0x20; //0010 0000 set bit 5. p186
#define led_off()	GPIOA->ODR&=~0x20; //0000 0000 clear bit 5. p186


/**
 **===========================================================================
 **
 **  Abstract: main program
 **
 **===========================================================================
 */


int main(void)
{
	__disable_irq();			//global disable IRQs, M3_Generic_User_Guide p135.

	USART1_Init(); // ModBus
	USART2_Init(); // Used as debugging terminal

	/* Configure the system clock to 32 MHz and update SystemCoreClock */
	SetSysClock();
	SystemCoreClockUpdate();

	TIM2_Init(); // Timer for sensor scheduling
    LED_Init();


	/* TODO - Add your application code here */


	USART1->CR1 |= 0x0020;			//enable RX interrupt
	NVIC_EnableIRQ(USART1_IRQn); 	//enable interrupt in NVIC
	NVIC_EnableIRQ(TIM2_IRQn);
	__enable_irq();					//global enable IRQs, M3_Generic_User_Guide p135

	RCC->AHBENR|=1; 				//GPIOA ABH bus clock ON. p154
	GPIOA->MODER&=~0x00000C00;		//clear (input reset state for PA5). p184
	GPIOA->MODER|=0x400; 			//GPIOA pin 5 to output. p184

	//setup ADC1. p272
	RCC->APB2ENR|=0x00000200;		//enable ADC1 clock
	ADC1->SQR5=0;					//conversion sequence starts at ch0
	ADC1->CR2=0;					//bit 1=0: Single conversion mode, bit 11=0 align right
	ADC1->SMPR3=7;				//384 cycles sampling time for channel 0 (longest)
	ADC1->CR1&=~0x03000000;		//resolution 12-bit
	ADC1->CR2|=1;					//bit 0, ADC on/off (1=on, 0=off)


	char received_frame[8]={0};
	/* Infinite loop */
	unsigned short int crc=0; //16 bitts
	char crc_high_byte=0;
	char crc_low_byte=0;

	char *framingErrorString = "Framing Error Detected";
	char *noiseErrorString = "Noise Error Detected";

	// Sensor data
    uint16_t dht22_temp = 0, dht22_hum = 0, grove_voc = 0, grove_eco2 = 0;
    uint16_t c_sum = 0;
    signed char dht22_minus = 1;
    int lmt84lp_temp = 0, nsl19m51_light = 0, hih4000_hum = 0;
    int ac_voltage = 0, energy_kwh = 0;

    char received_frame[8] = {0};
    char error_msg[30];

	USART1_IRQHandler();

	while (1)
	{
		if (frameFlag == 1)
		{
			/* Here we have encountered a framing erorr. I tshould not occur, but if it does, here we handle it */
			write_debug_msg(framingErrorString, 22);
			frameFlag = 0;
			clear_buffer(received_frame);
		}
		if (neFlag == 1)
		{
			/* If we have noise error in communication, we handle it here */
			write_debug_msg(noiseErrorString, 22);
			neFlag = 0;
			clear_buffer(received_frame);
		}
		// Periodic sensor reading
        if (sensor_read_flag) {
            // Read DHT22
            read_dht22_humidity_and_temperature(&dht22_temp, &dht22_hum, &c_sum, &dht22_minus);
            if (c_sum) {
                dht22_temp *= dht22_minus;
            } else {
                dht22_temp = dht22_hum = -9999; // Error value
            }

            // Read other sensors
            lmt84lp_temp = read_lmt84lp_temperature();
            nsl19m51_light = read_nsl19m51_light();
            hih4000_hum = read_hih4000_humidity();
            read_grove_voc_eco2(&grove_voc, &grove_eco2);
            ac_voltage = read_ac_voltage();
            energy_kwh = read_energy_kwh();

            // Control logic (e.g., turn off appliances if CO2 > 1000 ppm or energy > 10 kWh)
            control_energy(grove_eco2, energy_kwh);

            sensor_read_flag = 0;
        }
		if(mFlag==1) //correct slave address
		{
			read_7_bytes_from_usartx(&received_frame[1]);
			received_frame[0]=SLAVE_ADDRESS;
			crc=CRC16(received_frame,6);
			crc_high_byte=crc>>8|crc_high_byte; //high byte
			crc_low_byte=crc|crc_low_byte; //low byte


			if (received_frame[6] == crc_low && received_frame[7] == crc_high) {
                uint8_t reg = received_frame[3];
                int value = -9999;

                // Map register to sensor value
                switch (reg) {
                    case REG_DHT22_TEMP: value = dht22_temp; break;
                    case REG_DHT22_HUM:  value = dht22_hum; break;
                    case REG_LMT84LP:    value = lmt84lp_temp; break;
                    case REG_NSL19M51:   value = nsl19m51_light; break;
                    case REG_HIH4000:    value = hih4000_hum; break;
                    case REG_GROVE_VOC:  value = grove_voc; break;
                    case REG_GROVE_ECO2: value = grove_eco2; break;
                    case REG_AC_VOLTAGE: value = ac_voltage; break;
                    case REG_ENERGY_KWH: value = energy_kwh; break;
                    default: break;
                }

                if (value != -9999) {
                    respond_frame(reg, value);
                }
            }
			crc=0;
			crc_low_byte=0;
			crc_high_byte=0;
			mFlag=0;
			//GPIOA->ODR|=0x20;				//0010 0000 or bit 5. p186
			USART1->CR1 |= 0x0020;			//enable RX interrupt
		}
		else if(mFlag==2) //wrong slave address
		{
			wrong_slave_address();
			mFlag = 0;
			received_frame[0] = 0;
		}
	}

	return 0;
}


void TIM2_Init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; // Enable TIM2 clock
    TIM2->PSC = 32000 - 1; // 32 MHz / 32000 = 1 kHz
    TIM2->ARR = 1000 - 1;  // 1 kHz / 1000 = 1 Hz (1-second interval)
    TIM2->DIER |= TIM_DIER_UIE; // Enable update interrupt
    TIM2->CR1 |= TIM_CR1_CEN;   // Enable timer
}

/**
 * Initialize Modbus pins for UART1
 */
void USART1_Init(void)
{
	RCC->APB2ENR|=(1<<14);	 	//set bit 14 (USART1 EN) p.156
	RCC->AHBENR|=0x00000001; 	//enable GPIOA port clock bit 0 (GPIOA EN)
	GPIOA->AFR[1]=0x00000700;	//GPIOx_AFRL p.189,AF7 p.177 (AFRH10[3:0])
	GPIOA->AFR[1]|=0x00000070;	//GPIOx_AFRL p.189,AF7 p.177 (AFRH9[3:0])
	GPIOA->MODER|=0x00080000; 	//MODER2=PA9(TX)D8 to mode 10=alternate function mode. p184
	GPIOA->MODER|=0x00200000; 	//MODER2=PA10(RX)D2 to mode 10=alternate function mode. p184

	USART1->BRR = 0x00000D05;	//9600 BAUD and crystal 32MHz. p710, D05
	USART1->CR1 = 0x00000008;	//TE bit. p739-740. Enable transmit
	USART1->CR1 |= 0x00000004;	//RE bit. p739-740. Enable receiver
	USART1->CR1 |= 0x00002000;	//UE bit. p739-740. Uart enable

	USART1->CR1 |= USART_CR1_SBK; // SBK bit. Send break enabled

	USART1->CR2 = 0x00; // reset

	USART1->CR3 = 0;   // Set to default state
	USART1->CR3 |= 1;  // Enable error interrupt,  p744
	/* Error Interrupt Enable Bit is required to enable interrupt generation in case of a framing
	error, overrun error or noise flag (FE=1 or ORE=1 or NF=1 in the USART_SR register) in
	case of Multi Buffer Communication (DMAR=1 in the USART_CR3 register).
	 */
}

/**
 * Initialize TERMINAL pins for UART2, used for debugging
 */
void USART2_Init(void)
{
	RCC->APB1ENR|=0x00020000; 	//set bit 17 (USART2 EN)
	RCC->AHBENR|=0x00000001; 	//enable GPIOA port clock bit 0 (GPIOA EN)
	GPIOA->AFR[0]=0x00000700;	//GPIOx_AFRL p.188,AF7 p.177
	GPIOA->AFR[0]|=0x00007000;	//GPIOx_AFRL p.188,AF7 p.177
	GPIOA->MODER|=0x00000020; 	//MODER2=PA2(TX) to mode 10=alternate function mode. p184
	GPIOA->MODER|=0x00000080; 	//MODER2=PA3(RX) to mode 10=alternate function mode. p184

	USART2->BRR = 0x00000D05;	//9600 BAUD and crystal 32MHz. p710, D05
	USART2->CR1 = 0x00000008;	//TE bit. p739-740. Enable transmit
	USART2->CR1 |= 0x00000004;	//RE bit. p739-740. Enable receiver
	USART2->CR1 |= 0x00002000;	//UE bit. p739-740. Uart enable
}

void USART1_write(char data)
{
	//wait while TX buffer is empty
	while(!(USART1->SR&0x0080)){} 	//TXE: Transmit data register empty. p736-737
	USART1->DR=(data);			//p739
}

void USART2_write(char data)
{
	//wait while TX buffer is empty
	while(!(USART2->SR&0x0080)){} 	//TXE: Transmit data register empty. p736-737
	USART2->DR=(data);			//p739
}

void USART1_IRQHandler(void)
{
	char received_slave_address=0;

	/**
	 * If there is framing error (physical) in the modbus, we raise a flag here. Since inside
	 * IRQ handler actions should be simple and fast.  We will then investigate the flag in main()
	 */
	if (USART1->SR & USART_SR_FE)
	{
		frameFlag = 1;
	}

	/**
	 * If there is noise error (physical) in the modbus, we raise a flag here. Since inside
	 * IRQ handler actions should be simple and fast. We will then investigate the flag in main()
	 */
	if (USART1->SR & USART_SR_NE)
	{
		neFlag = 1;
	}

	if(USART1->SR & 0x0020) 		//if data available in DR register. p737
	{
		received_slave_address=USART1->DR;
	}
	if(received_slave_address==SLAVE_ADDRESS) //if we have right address
	{
		mFlag=1;

	}
	else
	{
		mFlag=2;


	}
	USART1->CR1 &= ~0x0020;			//disable RX interrupt

}

unsigned short int CRC16(char *nData,unsigned short int wLength)
{
	static const unsigned short int wCRCTable[] = {
			0X0000, 0XC0C1, 0XC181, 0X0140, 0XC301, 0X03C0, 0X0280, 0XC241,
			0XC601, 0X06C0, 0X0780, 0XC741, 0X0500, 0XC5C1, 0XC481, 0X0440,
			0XCC01, 0X0CC0, 0X0D80, 0XCD41, 0X0F00, 0XCFC1, 0XCE81, 0X0E40,
			0X0A00, 0XCAC1, 0XCB81, 0X0B40, 0XC901, 0X09C0, 0X0880, 0XC841,
			0XD801, 0X18C0, 0X1980, 0XD941, 0X1B00, 0XDBC1, 0XDA81, 0X1A40,
			0X1E00, 0XDEC1, 0XDF81, 0X1F40, 0XDD01, 0X1DC0, 0X1C80, 0XDC41,
			0X1400, 0XD4C1, 0XD581, 0X1540, 0XD701, 0X17C0, 0X1680, 0XD641,
			0XD201, 0X12C0, 0X1380, 0XD341, 0X1100, 0XD1C1, 0XD081, 0X1040,
			0XF001, 0X30C0, 0X3180, 0XF141, 0X3300, 0XF3C1, 0XF281, 0X3240,
			0X3600, 0XF6C1, 0XF781, 0X3740, 0XF501, 0X35C0, 0X3480, 0XF441,
			0X3C00, 0XFCC1, 0XFD81, 0X3D40, 0XFF01, 0X3FC0, 0X3E80, 0XFE41,
			0XFA01, 0X3AC0, 0X3B80, 0XFB41, 0X3900, 0XF9C1, 0XF881, 0X3840,
			0X2800, 0XE8C1, 0XE981, 0X2940, 0XEB01, 0X2BC0, 0X2A80, 0XEA41,
			0XEE01, 0X2EC0, 0X2F80, 0XEF41, 0X2D00, 0XEDC1, 0XEC81, 0X2C40,
			0XE401, 0X24C0, 0X2580, 0XE541, 0X2700, 0XE7C1, 0XE681, 0X2640,
			0X2200, 0XE2C1, 0XE381, 0X2340, 0XE101, 0X21C0, 0X2080, 0XE041,
			0XA001, 0X60C0, 0X6180, 0XA141, 0X6300, 0XA3C1, 0XA281, 0X6240,
			0X6600, 0XA6C1, 0XA781, 0X6740, 0XA501, 0X65C0, 0X6480, 0XA441,
			0X6C00, 0XACC1, 0XAD81, 0X6D40, 0XAF01, 0X6FC0, 0X6E80, 0XAE41,
			0XAA01, 0X6AC0, 0X6B80, 0XAB41, 0X6900, 0XA9C1, 0XA881, 0X6840,
			0X7800, 0XB8C1, 0XB981, 0X7940, 0XBB01, 0X7BC0, 0X7A80, 0XBA41,
			0XBE01, 0X7EC0, 0X7F80, 0XBF41, 0X7D00, 0XBDC1, 0XBC81, 0X7C40,
			0XB401, 0X74C0, 0X7580, 0XB541, 0X7700, 0XB7C1, 0XB681, 0X7640,
			0X7200, 0XB2C1, 0XB381, 0X7340, 0XB101, 0X71C0, 0X7080, 0XB041,
			0X5000, 0X90C1, 0X9181, 0X5140, 0X9301, 0X53C0, 0X5280, 0X9241,
			0X9601, 0X56C0, 0X5780, 0X9741, 0X5500, 0X95C1, 0X9481, 0X5440,
			0X9C01, 0X5CC0, 0X5D80, 0X9D41, 0X5F00, 0X9FC1, 0X9E81, 0X5E40,
			0X5A00, 0X9AC1, 0X9B81, 0X5B40, 0X9901, 0X59C0, 0X5880, 0X9841,
			0X8801, 0X48C0, 0X4980, 0X8941, 0X4B00, 0X8BC1, 0X8A81, 0X4A40,
			0X4E00, 0X8EC1, 0X8F81, 0X4F40, 0X8D01, 0X4DC0, 0X4C80, 0X8C41,
			0X4400, 0X84C1, 0X8581, 0X4540, 0X8701, 0X47C0, 0X4680, 0X8641,
			0X8201, 0X42C0, 0X4380, 0X8341, 0X4100, 0X81C1, 0X8081, 0X4040 };

	unsigned char nTemp;
	unsigned short int wCRCWord = 0xFFFF;

	while (wLength--)
	{
		nTemp = *nData++ ^ wCRCWord;
		wCRCWord >>= 8;
		wCRCWord ^= wCRCTable[nTemp];
	}
	return wCRCWord;

}

char USART1_read()
{
	char data=0;
	//wait while RX buffer is data is ready to be read
	while(!(USART1->SR&0x0020)){} 	//Bit 5 RXNE: Read data register not empty
	data=USART1->DR;			//p739
	return data;
}

char USART2_read()
{
	char data=0;
	//wait while RX buffer is data is ready to be read
	while(!(USART2->SR&0x0020)){} 	//Bit 5 RXNE: Read data register not empty
	data=USART2->DR;			//p739
	return data;
}

void wrong_slave_address(void)
{
	USART1->CR1 &= ~0x00000004;		//RE bit. p739-740. Disable receiver
	delay_Ms(10); 					//time=1/9600 x 10 bits x 7 byte = 7,29 ms
	USART1->CR1 |= 0x00000004;		//RE bit. p739-740. Enable receiver
	USART1->CR1 |= 0x0020;			//enable RX interrupt
	mFlag=0;
}


void read_7_bytes_from_usartx(char *received_frame)
{
	char frame[7]={0};
	uint8_t i=0;

	while(i<7)
	{
		*received_frame=USART1_read();
		frame[i]=*received_frame;
		received_frame++;
		i++;
	}
	write_debug_frame(frame, 7);
}

int read_sensor(int input_address)
{
	//if(input_address==0x01)
	int result=0;
	ADC1->CR2|=0x40000000;	//start conversion
	while(!(ADC1->SR & 2)){}	//wait for conversion complete
	result=ADC1->DR;			//read conversion result
	return result;
}

void respond_frame(int sensor_value)
{
	GPIOA->ODR|=0x20;				//led on, transmitting mode
	//example response should be like this: 0104020B057E03
	char respond_frame[7]={SLAVE_ADDRESS,0x04,0x02,0,0,0,0};
	char sensor_high_bits=0;
	char sensor_low_bits=0;
	char crc_high_byte=0;
	char crc_low_byte=0;
	unsigned short int crc=0; //16 bits

	sensor_high_bits=(sensor_value>>8)|sensor_high_bits;
	sensor_low_bits=sensor_value|sensor_low_bits;
	respond_frame[3]=sensor_high_bits;
	respond_frame[4]=sensor_low_bits;
	crc=CRC16(respond_frame,5);
	crc_high_byte=(crc>>8)|crc_high_byte; //high byte
	crc_low_byte=crc|crc_low_byte; //low byte

	respond_frame[6]=crc_high_byte;
	respond_frame[5]=crc_low_byte;

	for(int i=0;i<7;i++)
	{
		//blink_led();
		led_on();
		USART1_write(respond_frame[i]);
	}
	write_debug_frame(respond_frame, 7);
	led_off();
	GPIOA->ODR&=~0x20;				//led off, receiving mode

}

/**
 * Initialize LED
 */
void LED_Init()
{
	GPIOA->MODER|=0x400; //GPIOA pin 5 to output. p184
}

/**
 * Debug write a string to debug terminal
 */
void write_debug_msg(char *str, int maxchars)
{
	int i = 0;
	while(str[i] != '\0') {
		USART2_write(str[i]);
		if (++i == maxchars)
			break;
	}
	USART2_write('\r');
	USART2_write('\n');
}

char bytestr[] = {'0', '1', '2', '3', '4','5','6','7','8','9','a','b','c','d','e','f'};

/**
 * Write Modbus frame bytes to debug terminal
 */
void write_debug_frame(char *str, int maxchars)
{
	int i;
	for (i=0;i<8;i++) {
		USART2_write('0');
		USART2_write('x');
		USART2_write(bytestr[(str[i] & 0xf0) >> 4]);
		USART2_write(bytestr[str[i] & 0x0f]);
		USART2_write(44);
	}
	USART2_write('\r');
	USART2_write('\n');
}


uint8_t led_value = 0;

/**
 * Just turn led on/off, depending on previous state
 */
void blink_led()
{
	if (!led_value) {
		led_on();
	} else {
		led_off();
	}
	led_value = !led_value;
}


