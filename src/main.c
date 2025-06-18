// #include "stm32f4xx.h"
// #include <stdio.h>
// #include <string.h>

// /* Internal clock set to 84 MHz */
// void internal_clock(void)
// {
//     // Enable HSI
//     RCC->CR |= RCC_CR_HSION;
//     while ((RCC->CR & RCC_CR_HSIRDY) == 0);

//     // Configure Flash prefetch, Instruction cache, Data cache and wait state
//     FLASH->ACR = FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_PRFTEN | FLASH_ACR_LATENCY_2WS;

//     // Configure PLL: PLLCLK = (HSI/16) * 336 / 4 = 84 MHz
//     RCC->PLLCFGR = (16 << RCC_PLLCFGR_PLLM_Pos) | (336 << RCC_PLLCFGR_PLLN_Pos) |
//                    (0 << RCC_PLLCFGR_PLLP_Pos) | (RCC_PLLCFGR_PLLSRC_HSI) |
//                    (7 << RCC_PLLCFGR_PLLQ_Pos);

//     // Enable PLL
//     RCC->CR |= RCC_CR_PLLON;
//     while ((RCC->CR & RCC_CR_PLLRDY) == 0);

//     // Select PLL as system clock source
//     RCC->CFGR &= ~RCC_CFGR_SW;
//     RCC->CFGR |= RCC_CFGR_SW_PLL;
//     while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);
// }

// /* Logger */
// typedef enum { LOG_OFF, LOG_ERROR, LOG_INFO, LOG_DEBUG, LOG_SILLY } LogLevel;
// LogLevel log_level = LOG_INFO;

// void log(LogLevel level, const char* msg) {
//     if (level <= log_level) {
//         printf("%s\n", msg);
//     }
// }

// // Event types
// typedef enum { EVENT_ROLL_CHANGE, EVENT_DISPENSE } EventType;

// typedef struct {
//     EventType type;
//     char date[16]; // YYMMDDhhmmss
//     float revs;
// } Event;

// #define MAX_EVENTS 32
// Event eventArray[MAX_EVENTS];
// int eventCount = 0;


// /* Main */
// int main(void) {
//     // System Initialization
//     SystemInit();
//     internal_clock();

//     // Initialize GPIOA for LED (PA5)
//     RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
//     GPIOA->MODER &= ~(0x3 << (5 * 2)); // Clear mode bits for PA5
//     GPIOA->MODER |= (0x1 << (5 * 2));  // Set PA5 to output

//     // Simple LED blink and serial print test
//     // Initialize USART2 for serial (PA2: TX, PA3: RX)
//     // Enable clocks
//     RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
//     RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
//     // Set PA2 and PA3 to alternate function
//     GPIOA->MODER &= ~((0x3 << (2 * 2)) | (0x3 << (3 * 2)));
//     GPIOA->MODER |= (0x2 << (2 * 2)) | (0x2 << (3 * 2));
//     GPIOA->AFR[0] |= (0x7 << (2 * 4)) | (0x7 << (3 * 4)); // AF7 for USART2
//     // Configure USART2: 9600 baud, 8N1
//     USART2->BRR = (uint16_t)(42000000 / 9600); // APB1 at 42MHz
//     USART2->CR1 = USART_CR1_TE | USART_CR1_UE; // Enable TX, USART

//     // Print hello message
//     const char *msg = "Hello from STM32F401RE!\r\n";
//     for (const char *p = msg; *p; ++p) {
//         while (!(USART2->SR & USART_SR_TXE));
//         USART2->DR = *p;
//     }

//     while (1) {
//         // Toggle LED
//         GPIOA->ODR ^= (1 << 5);
//         // Delay
//         for (volatile int i = 0; i < 800000; ++i);
//     }
// }


#include "stm32f4xx.h"

int main(void) {
    // Enable GPIOA clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    // Set PA5 as output
    GPIOA->MODER &= ~(0x3 << (5 * 2));
    GPIOA->MODER |= (0x1 << (5 * 2));

    while (1) {
        GPIOA->ODR ^= (1 << 5); // Toggle LED
        for (volatile int i = 0; i < 800000; ++i); // Delay
    }
}