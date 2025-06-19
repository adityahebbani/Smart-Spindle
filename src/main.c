#include "stm32f4xx.h"
#include <stdio.h>
#include <string.h>

// Uncomment the next line to use the internal clock (84 MHz system, 42 MHz APB1)
// #define USE_INTERNAL_CLOCK

void internal_clock(void)
{
    // Enable HSI
    RCC->CR |= RCC_CR_HSION;
    while ((RCC->CR & RCC_CR_HSIRDY) == 0);
    // Configure Flash prefetch, Instruction cache, Data cache and wait state
    FLASH->ACR = FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_PRFTEN | FLASH_ACR_LATENCY_2WS;
    // Configure PLL: PLLCLK = (HSI/16) * 336 / 4 = 84 MHz
    RCC->PLLCFGR = (16 << RCC_PLLCFGR_PLLM_Pos) | (336 << RCC_PLLCFGR_PLLN_Pos) |
                   (0 << RCC_PLLCFGR_PLLP_Pos) | (RCC_PLLCFGR_PLLSRC_HSI) |
                   (7 << RCC_PLLCFGR_PLLQ_Pos);
    // Enable PLL
    RCC->CR |= RCC_CR_PLLON;
    while ((RCC->CR & RCC_CR_PLLRDY) == 0);
    // Select PLL as system clock source
    RCC->CFGR &= ~RCC_CFGR_SW;
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);
}

int main(void) {
    // SystemInit(); // Not needed, called before main by startup code
#ifdef USE_INTERNAL_CLOCK
    internal_clock();
#endif
    // Enable GPIOA clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    // Set PA5 as output (LED)
    GPIOA->MODER &= ~(0x3 << (5 * 2));
    GPIOA->MODER |= (0x1 << (5 * 2));
    // Enable USART2 clock
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    // Set PA2 to alternate function (AF7)
    GPIOA->MODER &= ~(0x3 << (2 * 2));
    GPIOA->MODER |= (0x2 << (2 * 2));
    GPIOA->AFR[0] &= ~(0xF << (2 * 4));
    GPIOA->AFR[0] |= (0x7 << (2 * 4));
    // Set PA3 to alternate function (AF7) if you want RX (not needed for TX only)
    GPIOA->MODER &= ~(0x3 << (3 * 2));
    GPIOA->MODER |= (0x2 << (3 * 2));
    GPIOA->AFR[0] &= ~(0xF << (3 * 4));
    GPIOA->AFR[0] |= (0x7 << (3 * 4));
#ifdef USE_INTERNAL_CLOCK
    // APB1 is 42 MHz after internal_clock()
    USART2->BRR = (uint16_t)(42000000 / 9600);
#else
    // APB1 is 16 MHz by default
    USART2->BRR = (uint16_t)(16000000 / 9600);
#endif
    USART2->CR1 = USART_CR1_TE | USART_CR1_UE;

    // Delay to allow USART to stabilize
    for (volatile int i = 0; i < 8000000; ++i);

    // Toggle LED before serial print
    GPIOA->ODR ^= (1 << 5);
    // Print hello message
    const char *msg = "Hello from STM32F401RE!\r\n";
    for (const char *p = msg; *p; ++p) {
        while (!(USART2->SR & USART_SR_TXE));
        USART2->DR = *p;
    }
    // Toggle LED after serial print
    GPIOA->ODR ^= (1 << 5);
    while (1) {
        const char *msg = "Hello from STM32F401RE!\r\n";
        for (const char *p = msg; *p; ++p) {
            while (!(USART2->SR & USART_SR_TXE));
            USART2->DR = *p;
        }
        // Blink LED
        GPIOA->ODR ^= (1 << 5);
        for (volatile int i = 0; i < 800000; ++i);
    }
}