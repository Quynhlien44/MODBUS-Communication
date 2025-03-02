#include "stm32l1xx.h"
#define HSI_VALUE ((uint32_t)16000000)
#include "nucleo152start.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

// MODBUS and Sensor Register Definitions
#define SLAVE_ADDRESS 0x01
#define FREEZER_TEMP_REG 0x01
#define OUTDOOR_TEMP_REG 0x02
#define OUTDOOR_HUM_REG 0x03
#define LIGHT_INTENSITY_REG 0x04
#define BATHROOM_HUM_REG 0x05
#define CO2_REG 0x06
#define AC_VOLTAGE_REG 0x07
#define ENERGY_REG 0x08
#define SGP30_ADDRESS 0x58

// Function Prototypes
void delay_Ms(int delay);
void delay_Us(int delay);
void USART1_Init(void);
void USART2_Init(void);
void I2C1_Init(void);
void I2C1_Write(uint8_t address, uint8_t command, int n, uint8_t* data);
void I2C1_SimpleRead(uint8_t address, int n, uint8_t* data);
void USART1_write(char data);
void USART2_write(char data);
void USART2_write_string(const char* str);
char USART1_read(void);
char USART2_read(void);
unsigned short int CRC16(char *nData, unsigned short int wLength);
int read_sensor(int input_address);
void respond_frame(int sensor_value);
void wrong_slave_address(void);
void read_dht22(int *temp, int *hum);

// Global Variables
volatile char rx_buffer[8];       // Buffer for MODBUS frame reception
volatile int rx_index = 0;        // Index for received bytes
volatile char mFlag = 0;          // MODBUS reception flag
int sgp30_initialized = 0;        // CO2 sensor initialization flag
volatile uint32_t pulse_count = 0;// Energy pulse counter

int main(void) {
    __disable_irq();

    // Initialize peripherals
    USART1_Init();  // MODBUS communication
    USART2_Init();  // Debug output
    I2C1_Init();    // For CO2 sensor
    SetSysClock();
    SystemCoreClockUpdate();

    // Enable USART1 RX interrupt
    USART1->CR1 |= 0x0020;
    NVIC_EnableIRQ(USART1_IRQn);
    __enable_irq();

    // Clock enables
    RCC->AHBENR |= 1;        // GPIOA clock
    RCC->AHBENR |= 2;        // GPIOB clock for I2C
    RCC->APB2ENR |= 1;       // SYSCFG clock for EXTI

    // LED on PA5 (also used for MAX3485 DE)
    GPIOA->MODER &= ~0x00000C00;
    GPIOA->MODER |= 0x00000400;

    // ADC setup for PA0, PA1, PA2, PA3
    RCC->APB2ENR |= 0x00000200; // ADC1 clock
    ADC1->CR1 &= ~0x03000000;   // 12-bit resolution
    ADC1->CR2 |= 1;             // ADC on

    // Energy measurement on PA4 (EXTI4)
    GPIOA->MODER &= ~0x00000300; // PA4 input
    SYSCFG->EXTICR[1] &= ~0x000F;// PA4 for EXTI4
    EXTI->IMR |= 0x0010;         // Unmask EXTI4
    EXTI->FTSR |= 0x0010;        // Falling edge trigger
    NVIC_EnableIRQ(EXTI4_IRQn);

    // I2C pins for CO2 sensor (PB8 SCL, PB9 SDA)
    GPIOB->AFR[1] &= ~0x000000FF;
    GPIOB->AFR[1] |= 0x00000044; // AF4 for I2C1
    GPIOB->MODER &= ~0x000F0000;
    GPIOB->MODER |= 0x000A0000;  // Alternate function
    GPIOB->OTYPER |= 0x00000300; // Open-drain
    GPIOB->PUPDR &= ~0x000F0000; // No pull-up/pull-down

    USART2_write_string("Home Automation System Started\r\n");

    while (1) {
        if (mFlag == 1) {
            // Validate CRC
            unsigned short int crc = CRC16((char*)rx_buffer, 6);
            char crc_low_byte = crc & 0xFF;
            char crc_high_byte = (crc >> 8) & 0xFF;

            if ((rx_buffer[6] == crc_low_byte) && (rx_buffer[7] == crc_high_byte)) {
                // Check function code (0x04) and quantity (1)
                if (rx_buffer[1] == 0x04 && rx_buffer[4] == 0x00 && rx_buffer[5] == 0x01) {
                    int input_address = rx_buffer[3];
                    int sensor_value = read_sensor(input_address);
                    respond_frame(sensor_value);
                    USART2_write_string("MODBUS Response Sent\r\n");
                } else {
                    USART2_write_string("Unsupported Function Code or Quantity\r\n");
                }
            } else {
                USART2_write_string("CRC Check Failed\r\n");
            }
            mFlag = 0;
            rx_index = 0;
            USART1->CR1 |= 0x0020; // Re-enable RX interrupt
        } else if (mFlag == 2) {
            wrong_slave_address();
        }
    }
    return 0;
}

// Delay Functions
void delay_Ms(int delay) {
    for (; delay > 0; delay--)
        for (int i = 0; i < 2460; i++);
}

void delay_Us(int delay) {
    for (int i = 0; i < (delay * 2); i++) {
        asm("nop");
    }
}

// USART Initialization
void USART1_Init(void) {
    RCC->APB2ENR |= (1 << 14);  // USART1 clock
    RCC->AHBENR |= 0x00000001;  // GPIOA clock
    GPIOA->AFR[1] = 0x00000770; // PA9, PA10 AF7
    GPIOA->MODER |= 0x00280000; // PA9, PA10 alternate function
    USART1->BRR = 0x00000D05;   // 9600 baud @ 32MHz
    USART1->CR1 = 0x0000200C;   // TE, RE, UE
}

void USART2_Init(void) {
    RCC->APB1ENR |= 0x00020000; // USART2 clock
    RCC->AHBENR |= 0x00000001;  // GPIOA clock
    GPIOA->AFR[0] = 0x00007700; // PA2, PA3 AF7
    GPIOA->MODER |= 0x000000A0; // PA2, PA3 alternate function
    USART2->BRR = 0x00000D05;   // 9600 baud
    USART2->CR1 = 0x0000200C;   // TE, RE, UE
}

// I2C Initialization and Functions
void I2C1_Init(void) {
    RCC->AHBENR |= 2;          // GPIOB clock
    RCC->APB1ENR |= (1 << 21); // I2C1 clock
    I2C1->CR1 = 0x8000;        // Software reset
    I2C1->CR1 &= ~0x8000;
    I2C1->CR2 = 0x0020;        // 32MHz peripheral clock
    I2C1->CCR = 160;           // 100kHz I2C clock
    I2C1->TRISE = 33;
    I2C1->CR1 |= 0x0001;       // Enable I2C1
}

void I2C1_Write(uint8_t address, uint8_t command, int n, uint8_t* data) {
    volatile int tmp;
    while (I2C1->SR2 & 2) {}
    I2C1->CR1 &= ~0x800;
    I2C1->CR1 |= 0x100;
    while (!(I2C1->SR1 & 1)) {}
    I2C1->DR = address << 1;
    while (!(I2C1->SR1 & 2)) {}
    tmp = I2C1->SR2;
    while (!(I2C1->SR1 & 0x80)) {}
    I2C1->DR = command;
    for (int i = 0; i < n; i++) {
        while (!(I2C1->SR1 & 0x80)) {}
        I2C1->DR = *data++;
    }
    while (!(I2C1->SR1 & 4)) {}
    I2C1->CR1 |= (1 << 9);
}

void I2C1_SimpleRead(uint8_t address, int n, uint8_t* data) {
    volatile int tmp;
    while (I2C1->SR2 & 2) {}
    I2C1->CR1 &= ~0x800;
    I2C1->CR1 |= 0x100;
    while (!(I2C1->SR1 & 1)) {}
    I2C1->DR = (address << 1) | 1;
    while (!(I2C1->SR1 & 2)) {}
    tmp = I2C1->SR2;
    I2C1->CR1 |= (1 << 10);
    for (int i = 0; i < n; i++) {
        if (i == n - 1) I2C1->CR1 &= ~(1 << 10);
        while (!(I2C1->SR1 & 0x40)) {}
        data[i] = I2C1->DR;
    }
    I2C1->CR1 |= (1 << 9);
}

// USART Read/Write Functions
void USART1_write(char data) {
    while (!(USART1->SR & 0x0080)) {}
    USART1->DR = data;
}

void USART2_write(char data) {
    while (!(USART2->SR & 0x0080)) {}
    USART2->DR = data;
}

void USART2_write_string(const char* str) {
    while (*str) {
        USART2_write(*str++);
    }
}

char USART1_read(void) {
    while (!(USART1->SR & 0x0020)) {}
    return USART1->DR;
}

char USART2_read(void) {
    while (!(USART2->SR & 0x0020)) {}
    return USART2->DR;
}

// CRC16 Calculation
unsigned short int CRC16(char *nData, unsigned short int wLength) {
    static const unsigned short int wCRCTable[] = {
        0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
        0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
        0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
        0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
        0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
        0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
        0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
        0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
        0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
        0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
        0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
        0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
        0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
        0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
        0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
        0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
        0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
        0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
        0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
        0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
        0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
        0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
        0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
        0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
        0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
        0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
        0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
        0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
        0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
        0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
        0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
        0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
    };
    unsigned char nTemp;
    unsigned short int wCRCWord = 0xFFFF;
    while (wLength--) {
        nTemp = *nData++ ^ wCRCWord;
        wCRCWord >>= 8;
        wCRCWord ^= wCRCTable[nTemp];
    }
    return wCRCWord;
}

// MODBUS Frame Handling
void respond_frame(int sensor_value) {
    GPIOA->ODR |= 0x20; // Enable MAX3485 driver (DE on PA5)
    char respond_frame[7] = {SLAVE_ADDRESS, 0x04, 0x02, 0, 0, 0, 0};
    respond_frame[3] = (sensor_value >> 8) & 0xFF;
    respond_frame[4] = sensor_value & 0xFF;
    unsigned short int crc = CRC16(respond_frame, 5);
    respond_frame[5] = crc & 0xFF;
    respond_frame[6] = (crc >> 8) & 0xFF;
    for (int i = 0; i < 7; i++) {
        USART1_write(respond_frame[i]);
    }
    GPIOA->ODR &= ~0x20; // Disable driver
}

void wrong_slave_address(void) {
    mFlag = 0;
    rx_index = 0;
    USART1->CR1 |= 0x0020; // Re-enable RX interrupt
    USART2_write_string("Wrong Slave Address Received\r\n");
}

// Interrupt Handlers
void EXTI4_IRQHandler(void) {
    if (EXTI->PR & 0x0010) {
        pulse_count++;
        EXTI->PR = 0x0010; // Clear pending bit
    }
}

void USART1_IRQHandler(void) {
    if (USART1->SR & 0x0020) {
        rx_buffer[rx_index] = USART1->DR;
        rx_index++;
        if (rx_index == 8) {
            mFlag = (rx_buffer[0] == SLAVE_ADDRESS) ? 1 : 2;
            rx_index = 0;
            USART1->CR1 &= ~0x0020; // Disable RX interrupt
        }
    }
}

// Sensor Reading Functions
void read_dht22(int *temp, int *hum) {
    uint8_t data[5] = {0};
    GPIOA->MODER |= 0x1000;  // PA6 output
    GPIOA->ODR &= ~0x40;     // Low
    delay_Ms(1);
    GPIOA->ODR |= 0x40;      // High
    GPIOA->MODER &= ~0x3000; // PA6 input
    while (GPIOA->IDR & 0x40) {}
    while (!(GPIOA->IDR & 0x40)) {}
    while (GPIOA->IDR & 0x40) {}
    for (int i = 0; i < 40; i++) {
        while (!(GPIOA->IDR & 0x40)) {}
        delay_Us(40);
        if (GPIOA->IDR & 0x40) {
            data[i / 8] |= (1 << (7 - (i % 8)));
        }
        while (GPIOA->IDR & 0x40) {}
    }
    *hum = (data[0] << 8 | data[1]); // Humidity in 0.1%
    int temp_val = (data[2] & 0x7F) << 8 | data[3]; // Temp in 0.1°C
    if (data[2] & 0x80) temp_val = -temp_val;
    *temp = temp_val;
}

int read_sensor(int input_address) {
    switch (input_address) {
        case FREEZER_TEMP_REG: // LMT84LP on PA0
            ADC1->SQR5 = 0;
            ADC1->CR2 |= 0x40000000;
            while (!(ADC1->SR & 2)) {}
            int result = ADC1->DR;
            float voltage_mV = (result * 3300.0f) / 4095.0f; // No divider assumed
            float temp_c = (1845.0f - voltage_mV) / 10.9f;
            return (int)(temp_c * 100);

        case OUTDOOR_TEMP_REG: // DHT22 temp on PA6
        case OUTDOOR_HUM_REG:  // DHT22 humidity on PA6
            int temp, hum;
            read_dht22(&temp, &hum);
            return (input_address == OUTDOOR_TEMP_REG) ? temp : hum;

        case LIGHT_INTENSITY_REG: // NSL19M51 on PA1
            ADC1->SQR5 = 1;
            ADC1->CR2 |= 0x40000000;
            while (!(ADC1->SR & 2)) {}
            int result2 = ADC1->DR;
            float voltage = (result2 / 4095.0f) * 3.3f;
            float lux = voltage * 1000.0f; // Approximate
            return (int)lux;

        case BATHROOM_HUM_REG: // HIH-4000-001 on PA2
            int sum = 0;
            for (int i = 0; i < 10; i++) {
                ADC1->SQR5 = 2;
                ADC1->CR2 |= 0x40000000;
                while (!(ADC1->SR & 2)) {}
                sum += ADC1->DR;
                delay_Ms(1);
            }
            int adc_value = sum / 10;
            float vout = (adc_value / 4095.0f) * 3.3f;
            float rh = (vout / 3.3f - 0.16f) / 0.0062f;
            rh = fmaxf(fminf(rh, 100.0f), 0.0f);
            return (int)(rh * 100);

        case CO2_REG: // SGP30 on I2C
            if (!sgp30_initialized) {
                uint8_t init_cmd_data[] = {0x03};
                I2C1_Write(SGP30_ADDRESS, 0x20, 1, init_cmd_data);
                delay_Ms(500);
                sgp30_initialized = 1;
            }
            uint8_t measure_cmd_data[] = {0x08};
            I2C1_Write(SGP30_ADDRESS, 0x20, 1, measure_cmd_data);
            delay_Ms(12);
            uint8_t data[6];
            I2C1_SimpleRead(SGP30_ADDRESS, 6, data);
            uint16_t eco2 = (data[3] << 8) | data[4];
            return eco2; // ppm

        case AC_VOLTAGE_REG: // Placeholder on PA3
            return 0;

        case ENERGY_REG: // Pulse counting on PA4
            float energy_kwh = (float)pulse_count / 1000.0f; // 1000 pulses/kWh
            return (int)(energy_kwh * 100);

        default:
            return 0;
    }
}
