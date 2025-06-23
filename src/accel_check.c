#include "stm32f4xx.h"

#define ADXL345_ADDR 0x53

void delay(volatile uint32_t d) { while (d--) __NOP(); }

void usart2_init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    GPIOA->MODER &= ~((0x3 << (2*2)) | (0x3 << (3*2)));
    GPIOA->MODER |= (0x2 << (2*2)) | (0x2 << (3*2));
    GPIOA->AFR[0] &= ~((0xF << (2*4)) | (0xF << (3*4)));
    GPIOA->AFR[0] |= (0x7 << (2*4)) | (0x7 << (3*4));
    USART2->BRR = 16000000/9600;
    USART2->CR1 = USART_CR1_TE | USART_CR1_UE;
}

void usart2_print(const char *s) {
    while (*s) {
        while (!(USART2->SR & USART_SR_TXE));
        USART2->DR = *s++;
    }
}

void led_init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    GPIOA->MODER &= ~(0x3 << (5*2));
    GPIOA->MODER |= (0x1 << (5*2));
}

void led_toggle(void) {
    GPIOA->ODR ^= (1 << 5);
}

void i2c1_init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
    GPIOB->MODER &= ~((0x3 << (8*2)) | (0x3 << (9*2)));
    GPIOB->MODER |= (0x2 << (8*2)) | (0x2 << (9*2));
    GPIOB->AFR[1] &= ~((0xF << ((8-8)*4)) | (0xF << ((9-8)*4)));
    GPIOB->AFR[1] |= (0x4 << ((8-8)*4)) | (0x4 << ((9-8)*4));
    GPIOB->OTYPER |= (1 << 8) | (1 << 9);
    GPIOB->PUPDR &= ~((0x3 << (8*2)) | (0x3 << (9*2)));
    GPIOB->PUPDR |= (0x1 << (8*2)) | (0x1 << (9*2));
    GPIOB->ODR |= (1 << 8) | (1 << 9);
    I2C1->CR2 = 16;
    I2C1->CCR = 80;
    I2C1->TRISE = 17;
    I2C1->CR1 |= I2C_CR1_PE;
}

int i2c1_read_reg(uint8_t dev_addr, uint8_t reg, uint8_t *data) {
    uint32_t timeout;
    I2C1->CR1 |= I2C_CR1_START;
    timeout = 10000; while (!(I2C1->SR1 & I2C_SR1_SB) && --timeout);
    if (!timeout) return -1;
    (void)I2C1->SR1;
    I2C1->DR = (dev_addr << 1);
    timeout = 10000; while (!(I2C1->SR1 & I2C_SR1_ADDR) && --timeout);
    if (!timeout) return -2;
    (void)I2C1->SR2;
    I2C1->DR = reg;
    timeout = 10000; while (!(I2C1->SR1 & I2C_SR1_TXE) && --timeout);
    if (!timeout) return -3;
    I2C1->CR1 |= I2C_CR1_START;
    timeout = 10000; while (!(I2C1->SR1 & I2C_SR1_SB) && --timeout);
    if (!timeout) return -4;
    (void)I2C1->SR1;
    I2C1->DR = (dev_addr << 1) | 1;
    timeout = 10000; while (!(I2C1->SR1 & I2C_SR1_ADDR) && --timeout);
    if (!timeout) return -5;
    (void)I2C1->SR2;
    I2C1->CR1 &= ~I2C_CR1_ACK;
    timeout = 10000; while (!(I2C1->SR1 & I2C_SR1_RXNE) && --timeout);
    if (!timeout) return -6;
    *data = I2C1->DR;
    I2C1->CR1 |= I2C_CR1_STOP;
    I2C1->CR1 |= I2C_CR1_ACK;
    return 0;
}

int main(void) {
    usart2_init();
    led_init();
    i2c1_init();
    delay(1600000);

    usart2_print("ADXL345 Minimal Test\r\n");

    uint8_t devid = 0;
    int result = i2c1_read_reg(ADXL345_ADDR, 0x00, &devid);

    char msg[64];
    if (result == 0 && devid == 0xE5) {
        usart2_print("ADXL345 detected! DEVID=0xE5\r\n");
        while (1) {
            led_toggle();
            delay(800000);
        }
    } else {
        snprintf(msg, sizeof(msg), "ERROR: result=%d, DEVID=0x%02X\r\n", result, devid);
        usart2_print(msg);
        while (1) {
            delay(400000);
            led_toggle();
        }
    }
}