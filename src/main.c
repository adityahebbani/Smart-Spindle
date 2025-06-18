#include "stm32f4xx.h"
#include <stdio.h>
#include <string.h>

/* Internal clock set to 84 MHz */
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

/* Logger */
typedef enum { LOG_OFF, LOG_ERROR, LOG_INFO, LOG_DEBUG, LOG_SILLY } LogLevel;
LogLevel log_level = LOG_INFO;

void log(LogLevel level, const char* msg) {
    if (level <= log_level) {
        printf("%s\n", msg);
    }
}

// Event types
typedef enum { EVENT_ROLL_CHANGE, EVENT_DISPENSE } EventType;

typedef struct {
    EventType type;
    char date[16]; // YYMMDDhhmmss
    float revs;
} Event;

#define MAX_EVENTS 32
Event eventArray[MAX_EVENTS];
int eventCount = 0;


/* Main */
int main(void) {
    // System Initialization
    SystemInit();
    internal_clock();

    // TODO: Initialize peripherals and port your application logic here

    while (1) {
        // Main loop
    }
}
