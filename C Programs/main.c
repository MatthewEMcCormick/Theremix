
/**
  SENIOR DESIGN
*/
const char username = "lmoloney";
#include "stm32f0xx.h"

void init_gpio(void) {
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;  // Enable clock to GPIOA
    GPIOA->MODER &= ~(3 << (5 * 2));
    GPIOA->MODER |= (1 << (5 * 2));     // Set PA5 to output mode
}

//===========================================================================
// init_tim6()
// This sets up Timer 6 to toggle the GPIO at the required rate (270 kHz).
//===========================================================================

void init_tim6(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;    // Enable clock to TIM6
    TIM6->PSC = 1 - 1;
    TIM6->ARR = (48000000 / (270000*2)) - 1; // Set auto-reload for 270 kHz (toggle every half-period)
    TIM6->DIER |= TIM_DIER_UIE;
    TIM6->CR1 |= TIM_CR1_CEN;
    NVIC->ISER[0] = 1 << TIM6_DAC_IRQn;
}

//===========================================================================
// Timer 6 ISR
// This interrupt service routine toggles the GPIO pin.
//===========================================================================

void TIM6_DAC_IRQHandler(void) {
    TIM6->SR &= ~TIM_SR_UIF;  // Clear the update interrupt flag
    GPIOA->ODR ^= (1 << 5);   // Toggle PA5 (connected to LED or scope)
}

//===========================================================================
// main()
// Initializes the system and enters an infinite loop.
//===========================================================================
int main(void) {
    internal_clock();
    init_gpio();    // Initialize PA5 as output
    init_tim6();    // Set up Timer 6 for 270 kHz toggling

    while (1) {
        // Infinite loop, all action happens in the Timer 6 ISR
        asm("wfi");  // Wait for interrupt
    }
}