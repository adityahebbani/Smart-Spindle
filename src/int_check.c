#include "stm32f4xx.h"

// --- MPU6050 definitions ---
#define MPU6050_ADDR         0x68
#define MPU6050_PWR_MGMT_1   0x6B
#define MPU6050_INT_ENABLE   0x38
#define MPU6050_MOT_THR      0x1F
#define MPU6050_MOT_DUR      0x20
#define MPU6050_INT_PIN_CFG  0x37

// --- Minimal I2C1 functions ---
// Note: In production these functions should include error checking.
int I2C1_WriteReg(uint8_t dev_addr, uint8_t reg, uint8_t data) {
    I2C1->CR1 |= I2C_CR1_START;
    while (!(I2C1->SR1 & I2C_SR1_SB));
    (void)I2C1->SR1;
    I2C1->DR = (dev_addr << 1); // write mode
    while (!(I2C1->SR1 & I2C_SR1_ADDR));
    (void)I2C1->SR2;
    I2C1->DR = reg;
    while (!(I2C1->SR1 & I2C_SR1_TXE));
    I2C1->DR = data;
    while (!(I2C1->SR1 & I2C_SR1_TXE));
    I2C1->CR1 |= I2C_CR1_STOP;
    return 0;
}

void I2C1_Init(void) {
    // Enable clocks for GPIOB and I2C1
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
    // Configure PB8 (SCL) and PB9 (SDA) as alternate function (AF4), open-drain with pull-up
    GPIOB->MODER &= ~((0x3 << (8*2)) | (0x3 << (9*2)));
    GPIOB->MODER |= ((0x2 << (8*2)) | (0x2 << (9*2)));
    GPIOB->OTYPER |= ((1 << 8) | (1 << 9));
    GPIOB->PUPDR &= ~((0x3 << (8*2)) | (0x3 << (9*2)));
    GPIOB->PUPDR |= ((0x1 << (8*2)) | (0x1 << (9*2)));
    GPIOB->AFR[1] &= ~((0xF << ((8-8)*4)) | (0xF << ((9-8)*4)));
    GPIOB->AFR[1] |= ((0x4 << ((8-8)*4)) | (0x4 << ((9-8)*4)));
    
    // Configure I2C timing for 100kHz (assumes 16MHz APB1 clock)
    I2C1->CR2 = 16;
    I2C1->CCR = 80;
    I2C1->TRISE = 17;
    I2C1->CR1 |= I2C_CR1_PE;
}

void MPU6050_Init(void) {
    // Wake up the MPU6050 by writing 0 to PWR_MGMT_1.
    I2C1_WriteReg(MPU6050_ADDR, MPU6050_PWR_MGMT_1, 0x00);
    for (volatile int i = 0; i < 100000; i++); // brief delay
}

void MPU6050_Enable_Motion_Interrupt(void) {
    // Set motion threshold (tweak the value as needed; here 60 LSB)
    I2C1_WriteReg(MPU6050_ADDR, MPU6050_MOT_THR, 60);
    // Set motion duration (here 40 ms)
    I2C1_WriteReg(MPU6050_ADDR, MPU6050_MOT_DUR, 40);
    // Enable motion detection interrupt: set bit6 in INT_ENABLE
    I2C1_WriteReg(MPU6050_ADDR, MPU6050_INT_ENABLE, 0x40);
    // Configure interrupt pin: active high, push-pull (0x20)
    I2C1_WriteReg(MPU6050_ADDR, MPU6050_INT_PIN_CFG, 0x20);
}

// --- UART functions for terminal output ---
void UART_Init(void) {
    // Enable GPIOA and USART2 clocks
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    // Configure PA2 as TX (Alternate function AF7)
    GPIOA->MODER &= ~(0x3 << (2*2));
    GPIOA->MODER |= (0x2 << (2*2));
    GPIOA->AFR[0] &= ~(0xF << (2*4));
    GPIOA->AFR[0] |= (0x7 << (2*4));
    // Configure USART2: 9600 baud, 8-bit, no parity, 1 stop bit. (Assuming 16MHz clock)
    USART2->BRR = 16000000 / 9600;
    USART2->CR1 = USART_CR1_TE | USART_CR1_UE;
}

void UART_Print(const char *str) {
    while(*str) {
        while(!(USART2->SR & USART_SR_TXE));
        USART2->DR = *str++;
    }
}

// --- LED and external interrupt on PA10 ---

// Initialize LED on PA5
void LED_Init(void) {
    // Enable GPIOA clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    // Set PA5 as output
    GPIOA->MODER &= ~(0x3 << (5 * 2));
    GPIOA->MODER |=  (0x1 << (5 * 2));
    // Initially turn LED off
    GPIOA->ODR &= ~(1 << 5);
}

// Initialize external interrupt on PA10 (connected to MPU6050 INT)
void Interrupt_Pin_Init(void) {
    // Enable GPIOA clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    // Configure PA10 as input
    GPIOA->MODER &= ~(0x3 << (10 * 2));
    // Enable internal pull-up on PA10 (so it stays high by default)
    GPIOA->PUPDR &= ~(0x3 << (10 * 2));
    GPIOA->PUPDR |=  (0x1 << (10 * 2));
    
    // Enable SYSCFG clock for EXTI configuration
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    // Map EXTI10 to PA10
    SYSCFG->EXTICR[2] &= ~SYSCFG_EXTICR3_EXTI10; 
    SYSCFG->EXTICR[2] |=  (0x0 << SYSCFG_EXTICR3_EXTI10_Pos);
    
    // Unmask EXTI line 10 and configure for rising edge trigger
    EXTI->IMR |= EXTI_IMR_MR10;
    EXTI->RTSR |= EXTI_RTSR_TR10;
    
    // Enable NVIC interrupt for EXTI15_10 group
    NVIC_EnableIRQ(EXTI15_10_IRQn);
}

// EXTI15_10 Interrupt Handler: Flash LED on PA5 and print a message when an interrupt is triggered
void EXTI15_10_IRQHandler(void) {
    if (EXTI->PR & EXTI_PR_PR10) {
        // Clear pending flag for EXTI10
        EXTI->PR = EXTI_PR_PR10;
        
        // Turn on LED on PA5
        GPIOA->ODR |= (1 << 5);
        
        // Print message to terminal
        UART_Print("Motion Interrupt Triggered!\r\n");
        
        // Delay for visible LED flash (adjust delay as needed)
        for (volatile int i = 0; i < 500000; i++);
        
        // Turn off LED on PA5
        GPIOA->ODR &= ~(1 << 5);
    }
}

int main(void) {
    // Initialize UART for terminal output
    UART_Init();
    
    // Initialize I2C, MPU6050, and enable motion interrupt
    I2C1_Init();
    MPU6050_Init();
    MPU6050_Enable_Motion_Interrupt();
    
    // Initialize LED and external interrupt on PA10 (connected to MPU6050 INT)
    LED_Init();
    Interrupt_Pin_Init();
    
    // Main loop: enter sleep mode, waiting for interrupts (motion-triggered)
    while (1) {
        __WFI();
    }
}