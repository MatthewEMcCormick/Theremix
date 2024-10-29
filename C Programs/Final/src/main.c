 #include "stm32f4xx.h"
#include <stdint.h>
#include "IO_DSP.h"
#include "clock.h"
#include "DSP.h"


int activeSetting = 0;  // If running into errors, maybe make this volatile
int Settings[12];
int currentPage = 0;
char* currentSetting = "";

void nano_wait(unsigned int n) {
    asm(    "        mov r0,%0\n"
            "repeat: sub r0,#83\n"
            "        bgt repeat\n" : : "r"(n) : "r0", "cc");
}

void init_fixed_rate(void) {
    // Enable GPIOA clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;

    // Set PA4 as output (01 in MODER5)
    GPIOC->MODER &= ~(3 << (0 * 2));  // Clear mode bits for PA5
    GPIOC->MODER |= (1 << (0 * 2));   // Set PA5 to output mode
}

void togglexn(GPIO_TypeDef *port, int n) {
    port->ODR ^= (1 << n);  // Toggle pin n
}


int main(void) {
    internal_clock();  // Set clock
    //init_fixed_rate();        // Initialize FRO (PA4 as output)
    init_DSP();

    while(1)
    {
        //togglexn(GPIOC, 0);
        continue;
    }
}