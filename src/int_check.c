#include "stm32f4xx.h"

// Initialize LED on PA5
void LED_Init(void) {
    // Enable GPIOA clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    // Clear mode bits for PA5 and set as output (01)
    GPIOA->MODER &= ~(0x3 << (5 * 2));
    GPIOA->MODER |=  (0x1 << (5 * 2));
    // Initially turn LED off
    GPIOA->ODR &= ~(1 << 5);
}

// Initialize external interrupt on PA10
void Interrupt_Pin_Init(void) {
    // Enable GPIOA clock (if not already enabled)
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    // Configure PA10 as input
    GPIOA->MODER &= ~(0x3 << (10 * 2));
    // Enable internal pull-up on PA10
    GPIOA->PUPDR &= ~(0x3 << (10 * 2));
    GPIOA->PUPDR |=  (0x1 << (10 * 2)); // 01 = Pull-up

    // Enable SYSCFG clock for EXTI configuration
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    // Map EXTI10 to PA10 (PA for bits 8-11 in EXTICR[2])
    SYSCFG->EXTICR[2] &= ~SYSCFG_EXTICR3_EXTI10; 
    SYSCFG->EXTICR[2] |=  (0x0 << SYSCFG_EXTICR3_EXTI10_Pos);
    
    // Unmask EXTI line 10 and configure trigger on rising edge
    EXTI->IMR |= EXTI_IMR_MR10;
    EXTI->RTSR |= EXTI_RTSR_TR10;
    
    // Enable NVIC interrupt for EXTI15_10 group
    NVIC_EnableIRQ(EXTI15_10_IRQn);
}

// EXTI15_10 Interrupt Handler: Toggle LED on PA5 each time PA10 triggers
void EXTI15_10_IRQHandler(void) {
    if (EXTI->PR & EXTI_PR_PR10) {
        // Clear pending flag for EXTI10
        EXTI->PR = EXTI_PR_PR10;
        // Toggle LED on PA5
        GPIOA->ODR ^= (1 << 5);
    }
}

int main(void) {
    LED_Init();
    Interrupt_Pin_Init();
    
    // Main loop does nothing; wait for interrupts to toggle the LED.
    while(1) {
        // Optionally, insert a low power sleep here:
        __WFI();
    }
}