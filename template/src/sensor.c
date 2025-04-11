#include "include.h"

// ADC channel mappings (example, adjust based on Nucleo pinout)
#define ADC_CH_LMT84LP  1 // PA1
#define ADC_CH_NSL19M51 2 // PA2
#define ADC_CH_HIH4000  3 // PA3
#define ADC_CH_AC_VOLT  4 // PA4

int read_lmt84lp_temperature(void) {
    ADC1->SQR5 = ADC_CH_LMT84LP; // Select channel
    ADC1->CR2 |= ADC_CR2_SWSTART; // Start conversion
    while (!(ADC1->SR & ADC_SR_EOC)) {} // Wait for end of conversion
    int adc_value = ADC1->DR;
    // LMT84LP: Vout = (-5.5mV/°C * T) + 1035mV
    // ADC value (0-4095) maps to 0-3.3V
    float voltage = (adc_value * 3.3) / 4095.0;
    int temp = (int)((1035.0 - voltage * 1000.0) / 5.5);
    return temp; // °C
}

int read_nsl19m51_light(void) {
    ADC1->SQR5 = ADC_CH_NSL19M51;
    ADC1->CR2 |= ADC_CR2_SWSTART;
    while (!(ADC1->SR & ADC_SR_EOC)) {}
    int adc_value = ADC1->DR;
    // NSL19M51: Resistance decreases with light, use voltage divider
    // Assume 10k resistor in series, convert to lux (simplified)
    float voltage = (adc_value * 3.3) / 4095.0;
    int lux = (int)(1000.0 * (3.3 - voltage) / voltage); // Approximate
    return lux;
}

int read_hih4000_humidity(void) {
    ADC1->SQR5 = ADC_CH_HIH4000;
    ADC1->CR2 |= ADC_CR2_SWSTART;
    while (!(ADC1->SR & ADC_SR_EOC)) {}
    int adc_value = ADC1->DR;
    // HIH-4000: Vout = (0.0062 * RH + 0.16) * Vsupply
    float voltage = (adc_value * 3.3) / 4095.0;
    int rh = (int)(((voltage / 3.3) - 0.16) / 0.0062);
    if (rh < 0) rh = 0;
    if (rh > 100) rh = 100;
    return rh; // %RH
}

void read_grove_voc_eco2(uint16_t* voc, uint16_t* eco2) {
    // Grove-VOC-and-eCO2 uses I2C (SGP30 sensor)
    // Initialize I2C1 (PA9: SCL, PA10: SDA, assuming alternate function setup)
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    GPIOA->MODER |= (GPIO_MODER_MODER9_1 | GPIO_MODER_MODER10_1); // AF mode
    GPIOA->AFR[1] |= (0x4 << 4) | (0x4 << 8); // AF4 for I2C1
    I2C1->CR1 = 0;
    I2C1->CR2 = 32; // 32 MHz clock
    I2C1->CCR = 160; // 100 kHz
    I2C1->TRISE = 33;
    I2C1->CR1 |= I2C_CR1_PE;

    // SGP30: Send measure air quality command (0x2008)
    uint8_t cmd[] = {0x20, 0x08};
    I2C1->CR1 |= I2C_CR1_START;
    while (!(I2C1->SR1 & I2C_SR1_SB)) {}
    I2C1->DR = 0x58 << 1; // SGP30 address
    while (!(I2C1->SR1 & I2C_SR1_ADDR)) {}
    (void)I2C1->SR2;
    for (int i = 0; i < 2; i++) {
        I2C1->DR = cmd[i];
        while (!(I2C1->SR1 & I2C_SR1_TXE)) {}
    }
    I2C1->CR1 |= I2C_CR1_STOP;

    delay_Ms(25); // Wait for measurement

    // Read 6 bytes (CO2: 2 bytes + CRC, TVOC: 2 bytes + CRC)
    uint8_t data[6];
    I2C1->CR1 |= I2C_CR1_START;
    while (!(I2C1->SR1 & I2C_SR1_SB)) {}
    I2C1->DR = (0x58 << 1) | 1; // Read mode
    while (!(I2C1->SR1 & I2C_SR1_ADDR)) {}
    (void)I2C1->SR2;
    for (int i = 0; i < 5; i++) {
        while (!(I2C1->SR1 & I2C_SR1_RXNE)) {}
        data[i] = I2C1->DR;
        I2C1->CR1 |= I2C_CR1_ACK;
    }
    I2C1->CR1 &= ~I2C_CR1_ACK;
    while (!(I2C1->SR1 & I2C_SR1_RXNE)) {}
    data[5] = I2C1->DR;
    I2C1->CR1 |= I2C_CR1_STOP;

    *eco2 = (data[0] << 8) | data[1]; // CO2 in ppm
    *voc = (data[3] << 8) | data[4];  // TVOC in ppb
}

int read_ac_voltage(void) {
    ADC1->SQR5 = ADC_CH_AC_VOLT;
    ADC1->CR2 |= ADC_CR2_SWSTART;
    while (!(ADC1->SR & ADC_SR_EOC)) {}
    int adc_value = ADC1->DR;
    // 77DE-06-09: Assume transformer scales 230V to 0-3.3V range
    float voltage = (adc_value * 3.3) / 4095.0;
    int ac_volt = (int)(voltage * (230.0 / 3.3)); // Scale to actual voltage
    return ac_volt; // Volts
}

int read_energy_kwh(void) {
    // L-934LGD: Assume pulse counting for energy (e.g., 3200 pulses/kWh)
    static uint32_t pulse_count = 0;
    static uint32_t last_time = 0;
    uint32_t current_time = TIM2->CNT; // Use timer for timekeeping

    // Assume PA7 for pulse input (external interrupt)
    if (GPIOA->IDR & GPIO_IDR_IDR7) {
        pulse_count++;
    }

    // Calculate kWh: 3200 pulses = 1 kWh
    int kwh = (int)(pulse_count / 3200.0);
    return kwh;
}

void control_energy(uint16_t eco2, int energy_kwh) {
    // Example: Control relay on PA8 if CO2 > 1000 ppm or energy > 10 kWh
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    GPIOA->MODER |= GPIO_MODER_MODER8_0; // PA8 output
    if (eco2 > 1000 || energy_kwh > 10) {
        GPIOA->ODR &= ~GPIO_ODR_ODR8; // Relay off
    } else {
        GPIOA->ODR |= GPIO_ODR_ODR8;  // Relay on
    }
}