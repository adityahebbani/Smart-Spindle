// #include "stm32f4xx.h"
// #include <stdio.h>

// #define ADXL345_ADDR 0x53
// #define DS3231_ADDR  0x68
// #define TSL2591_ADDR 0x29
// #define MPU6050_ADDR 0x68
// #define MPU6050_WHO_AM_I 0x75

// void delay(volatile uint32_t d) { while (d--) __NOP(); }

// void usart2_init(void) {
//     RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
//     RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
//     GPIOA->MODER &= ~((0x3 << (2*2)) | (0x3 << (3*2)));
//     GPIOA->MODER |= (0x2 << (2*2)) | (0x2 << (3*2));
//     GPIOA->AFR[0] &= ~((0xF << (2*4)) | (0xF << (3*4)));
//     GPIOA->AFR[0] |= (0x7 << (2*4)) | (0x7 << (3*4));
//     USART2->BRR = 16000000/9600;
//     USART2->CR1 = USART_CR1_TE | USART_CR1_UE;
// }

// void usart2_print(const char *s) {
//     while (*s) {
//         while (!(USART2->SR & USART_SR_TXE));
//         USART2->DR = *s++;
//     }
// }

// void led_init(void) {
//     RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
//     GPIOA->MODER &= ~(0x3 << (5*2));
//     GPIOA->MODER |= (0x1 << (5*2));
// }

// void led_toggle(void) {
//     GPIOA->ODR ^= (1 << 5);
// }

// void i2c1_init(void) {
//     RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
//     RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
//     GPIOB->MODER &= ~((0x3 << (8*2)) | (0x3 << (9*2)));
//     GPIOB->MODER |= (0x2 << (8*2)) | (0x2 << (9*2));
//     GPIOB->AFR[1] &= ~((0xF << ((8-8)*4)) | (0xF << ((9-8)*4)));
//     GPIOB->AFR[1] |= (0x4 << ((8-8)*4)) | (0x4 << ((9-8)*4));
//     GPIOB->OTYPER |= (1 << 8) | (1 << 9);
//     GPIOB->PUPDR &= ~((0x3 << (8*2)) | (0x3 << (9*2)));
//     GPIOB->PUPDR |= (0x1 << (8*2)) | (0x1 << (9*2));
//     GPIOB->ODR |= (1 << 8) | (1 << 9);
//     I2C1->CR2 = 16;
//     I2C1->CCR = 80;
//     I2C1->TRISE = 17;
//     I2C1->CR1 |= I2C_CR1_PE;
// }

// void i2c2_init(void) {
//     RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
//     RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
//     // Set PB10 (D6) to AF4 (I2C2_SCL)
//     GPIOB->MODER &= ~(0x3 << (10*2));
//     GPIOB->MODER |=  (0x2 << (10*2));
//     GPIOB->AFR[1] &= ~(0xF << ((10-8)*4));
//     GPIOB->AFR[1] |=  (0x4 << ((10-8)*4));
//     // Set PB3 (D3) to AF9 (I2C2_SDA)
//     GPIOB->MODER &= ~(0x3 << (3*2));
//     GPIOB->MODER |=  (0x2 << (3*2));
//     GPIOB->AFR[0] &= ~(0xF << (3*4));
//     GPIOB->AFR[0] |=  (0x9 << (3*4));
//     // Open-drain and pull-up
//     GPIOB->OTYPER |= (1 << 10) | (1 << 3);
//     GPIOB->PUPDR &= ~((0x3 << (10*2)) | (0x3 << (3*2)));
//     GPIOB->PUPDR |=  (0x1 << (10*2)) | (0x1 << (3*2));
//     GPIOB->ODR |= (1 << 10) | (1 << 3);
//     // I2C2 peripheral setup
//     I2C2->CR2 = 16;
//     I2C2->CCR = 80;
//     I2C2->TRISE = 17;
//     I2C2->CR1 |= I2C_CR1_PE;
// }

// int i2c1_read_reg(uint8_t dev_addr, uint8_t reg, uint8_t *data) {
//     uint32_t timeout;
//     I2C1->CR1 |= I2C_CR1_START;
//     timeout = 10000; while (!(I2C1->SR1 & I2C_SR1_SB) && --timeout);
//     if (!timeout) return -1;
//     (void)I2C1->SR1;
//     I2C1->DR = (dev_addr << 1);
//     timeout = 10000; while (!(I2C1->SR1 & I2C_SR1_ADDR) && --timeout);
//     if (!timeout) return -2;
//     (void)I2C1->SR2;
//     I2C1->DR = reg;
//     timeout = 10000; while (!(I2C1->SR1 & I2C_SR1_TXE) && --timeout);
//     if (!timeout) return -3;
//     I2C1->CR1 |= I2C_CR1_START;
//     timeout = 10000; while (!(I2C1->SR1 & I2C_SR1_SB) && --timeout);
//     if (!timeout) return -4;
//     (void)I2C1->SR1;
//     I2C1->DR = (dev_addr << 1) | 1;
//     timeout = 10000; while (!(I2C1->SR1 & I2C_SR1_ADDR) && --timeout);
//     if (!timeout) return -5;
//     (void)I2C1->SR2;
//     I2C1->CR1 &= ~I2C_CR1_ACK;
//     timeout = 10000; while (!(I2C1->SR1 & I2C_SR1_RXNE) && --timeout);
//     if (!timeout) return -6;
//     *data = I2C1->DR;
//     I2C1->CR1 |= I2C_CR1_STOP;
//     I2C1->CR1 |= I2C_CR1_ACK;
//     return 0;
// }

// int i2c2_read_reg(uint8_t dev_addr, uint8_t reg, uint8_t *data) {
//     uint32_t timeout;
//     I2C2->CR1 |= I2C_CR1_START;
//     timeout = 10000; while (!(I2C2->SR1 & I2C_SR1_SB) && --timeout);
//     if (!timeout) return -1;
//     (void)I2C2->SR1;
//     I2C2->DR = (dev_addr << 1);
//     timeout = 10000; while (!(I2C2->SR1 & I2C_SR1_ADDR) && --timeout);
//     if (!timeout) return -2;
//     (void)I2C2->SR2;
//     I2C2->DR = reg;
//     timeout = 10000; while (!(I2C2->SR1 & I2C_SR1_TXE) && --timeout);
//     if (!timeout) return -3;
//     I2C2->CR1 |= I2C_CR1_START;
//     timeout = 10000; while (!(I2C2->SR1 & I2C_SR1_SB) && --timeout);
//     if (!timeout) return -4;
//     (void)I2C2->SR1;
//     I2C2->DR = (dev_addr << 1) | 1;
//     timeout = 10000; while (!(I2C2->SR1 & I2C_SR1_ADDR) && --timeout);
//     if (!timeout) return -5;
//     (void)I2C2->SR2;
//     I2C2->CR1 &= ~I2C_CR1_ACK;
//     timeout = 10000; while (!(I2C2->SR1 & I2C_SR1_RXNE) && --timeout);
//     if (!timeout) return -6;
//     *data = I2C2->DR;
//     I2C2->CR1 |= I2C_CR1_STOP;
//     I2C2->CR1 |= I2C_CR1_ACK;
//     return 0;
// }

// int i2c1_write_reg(uint8_t dev_addr, uint8_t reg, uint8_t data) {
//     const uint32_t MAX_TIMEOUT = 50000; // Same timeout as in read function
//     uint32_t timeout;
    
//     // Generate start condition
//     I2C1->CR1 |= I2C_CR1_START;
    
//     // Wait for start condition with timeout
//     timeout = MAX_TIMEOUT;
//     while (!(I2C1->SR1 & I2C_SR1_SB) && timeout) timeout--;
//     if (!timeout) {
//         I2C1->CR1 |= I2C_CR1_STOP; // Send stop to release the bus
//         return -1;  // Timeout error
//     }
    
//     (void)I2C1->SR1; // Clear SB flag
//     I2C1->DR = (dev_addr << 1); // Send device address (write mode)
    
//     // Wait for address sent with timeout
//     timeout = MAX_TIMEOUT;
//     while (!(I2C1->SR1 & I2C_SR1_ADDR) && timeout) timeout--;
//     if (!timeout) {
//         I2C1->CR1 |= I2C_CR1_STOP;
//         return -2;  // Address error
//     }
    
//     (void)I2C1->SR2; // Clear ADDR flag
    
//     // Send register address
//     I2C1->DR = reg;
    
//     // Wait for TXE with timeout
//     timeout = MAX_TIMEOUT;
//     while (!(I2C1->SR1 & I2C_SR1_TXE) && timeout) timeout--;
//     if (!timeout) {
//         I2C1->CR1 |= I2C_CR1_STOP;
//         return -3;  // Register address error
//     }
    
//     // Send data
//     I2C1->DR = data;
    
//     // Wait for TXE with timeout
//     timeout = MAX_TIMEOUT;
//     while (!(I2C1->SR1 & I2C_SR1_TXE) && timeout) timeout--;
//     if (!timeout) {
//         I2C1->CR1 |= I2C_CR1_STOP;
//         return -4;  // Data write error
//     }
    
//     // Generate stop condition
//     I2C1->CR1 |= I2C_CR1_STOP;
    
//     return 0;  // Success
// }

// int i2c2_write_reg(uint8_t dev_addr, uint8_t reg, uint8_t data) {
//     uint32_t timeout;
//     I2C2->CR1 |= I2C_CR1_START;
//     timeout = 10000; while (!(I2C2->SR1 & I2C_SR1_SB) && --timeout);
//     if (!timeout) return -1;
//     (void)I2C2->SR1;
//     I2C2->DR = (dev_addr << 1);
//     timeout = 10000; while (!(I2C2->SR1 & I2C_SR1_ADDR) && --timeout);
//     if (!timeout) return -2;
//     (void)I2C2->SR2;
//     I2C2->DR = reg;
//     timeout = 10000; while (!(I2C2->SR1 & I2C_SR1_TXE) && --timeout);
//     if (!timeout) return -3;
//     I2C2->DR = data;
//     timeout = 10000; while (!(I2C2->SR1 & I2C_SR1_TXE) && --timeout);
//     if (!timeout) return -4;
//     I2C2->CR1 |= I2C_CR1_STOP;
//     return 0;
// }

// int main(void) {
//     usart2_init();
//     led_init();
//     i2c1_init();
//     i2c2_init();
//     delay(1600000);
//     int result;
//     char msg[64];

//     // // usart2_print("ADXL345 Minimal Test\r\n");
//     // uint8_t devid = 0;
//     // result= i2c1_read_reg(ADXL345_ADDR, 0x00, &devid);
//     // if (result == 0 && devid == 0xE5) {
//     //     usart2_print("ADXL345 detected! DEVID=0xE5\r\n");
//     // } else {
//     //     snprintf(msg, sizeof(msg), "ERROR: result=%d, DEVID=0x%02X\r\n", result, devid);
//     //     usart2_print(msg);
//     // }

//     // DS3231 check (read seconds register, should be valid BCD)
//     uint8_t seconds = 0;
//     result = i2c2_read_reg(DS3231_ADDR, 0x00, &seconds);
//     if (result == 0) {
//         snprintf(msg, sizeof(msg), "DS3231 detected! Seconds=0x%02X\r\n", seconds);
//         usart2_print(msg);
//     } else {
//         snprintf(msg, sizeof(msg), "DS3231 ERROR: result=%d\r\n", result);
//         usart2_print(msg);
//     }

//     // TSL2591 check (read ID register, should be 0x50 per datasheet)
//     uint8_t tsl_id = 0;
//     result = i2c1_read_reg(TSL2591_ADDR, 0xA0 | 0x12, &tsl_id);
//     if (result == 0 && tsl_id == 0x50) {
//         usart2_print("TSL2591 detected! ID=0x50\r\n");
//     } else {
//         snprintf(msg, sizeof(msg), "TSL2591 ERROR: result=%d, ID=0x%02X\r\n", result, tsl_id);
//         usart2_print(msg);
//     }

//     // MPU6050 check (WHO_AM_I register)
//     uint8_t mpu_id = 0;
//     result = i2c1_read_reg(MPU6050_ADDR, MPU6050_WHO_AM_I, &mpu_id);
//     if (result == 0 && (mpu_id == 0x68)) {
//         snprintf(msg, sizeof(msg), "MPU6050 detected! WHO_AM_I=0x%02X\r\n", mpu_id);
//         usart2_print(msg);

//         // Try reading the gyroscope Z high byte as a functional check
//         i2c1_write_reg(MPU6050_ADDR, 0x6B, 0x00); // 0x6B = PWR_MGMT_1, 0x00 = wake up
//         delay(10000);
        
//         uint8_t gyro_z_h = 0;
//         result = i2c1_read_reg(MPU6050_ADDR, 0x47, &gyro_z_h); // GYRO_ZOUT_H register
//         if (result == 0) {
//             snprintf(msg, sizeof(msg), "MPU6050 GYRO_ZOUT_H=0x%02X\r\n", gyro_z_h);
//             usart2_print(msg);
//         } else {
//             snprintf(msg, sizeof(msg), "MPU6050 GYRO_ZOUT_H read error: result=%d\r\n", result);
//             usart2_print(msg);
//         }
//     } else {
//         snprintf(msg, sizeof(msg), "MPU6050 ERROR: result=%d, WHO_AM_I=0x%02X\r\n", result, mpu_id);
//         usart2_print(msg);
//     }

//     while (1) {
//         led_toggle();
//         delay(800000);
//     }
// }