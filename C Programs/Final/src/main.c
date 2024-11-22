#include "stm32f4xx.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "IO_DSP.h"
#include "DSP.h"
#include "global.h"
#include "lcd.h"
#include "updateSettings.h"

#define TC GREEN
#define BC RED
#define BS BLUE

// Function to initialize SPI4 for OLED
void internal_clock(void);

// Function to create a small delay
// void nano_wait(unsigned int n) {
//     asm(    "        mov r0,%0\n"
//             "repeat: sub r0,#83\n"
//             "        bgt repeat\n" : : "r"(n) : "r0", "cc");
// }


//===========================================================================
// 4.4 SPI OLED Display
//===========================================================================

void init_spi1() {
    // PB3  SPI1_SCK  
    // PB5  SPI1_MOSI/SDI
    // PA15 SPI1_NSS/CS
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
                    //CS on PB8                  nReset on 11         DC on 14/ 9!
                    //   pin 3                      pin 4               pin5
    GPIOB->MODER &= ~(GPIO_MODER_MODER8 | GPIO_MODER_MODER11 | GPIO_MODER_MODER9 |
                     GPIO_MODER_MODER5 | GPIO_MODER_MODER3);
                    //SDI(MOSI)PB5 pin 6   SCK on PB3 Pin 7
    // 8 9 and 11 as gpio outputs
    GPIOB->MODER |= (GPIO_MODER_MODER8_0 | GPIO_MODER_MODER9_0 |GPIO_MODER_MODER11_0 );

    /// THE INIT_SPI1_SLOW PART OF THE CODE
    GPIOB->MODER |= (GPIO_MODER_MODER3_1 | GPIO_MODER_MODER5_1);
    GPIOB->AFR[0] &= ~(GPIO_AFRL_AFRL3 | GPIO_AFRL_AFRL5);
    GPIOB->AFR[0] |= ((0x5 << 12) | (0x5 << 20));


    //Probably dont need all the PB4 stuff

    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

    SPI1->CR1 &= ~SPI_CR1_SPE;
    SPI1->CR1 &= ~SPI_CR1_BR; //Testing different speeds, LCD wants 24Mhz
    SPI1->CR1 |= 0b000<<3; //<--- /2BR  //24Mhz as specified in lab7

    SPI1->CR1 |= SPI_CR1_MSTR;
    //8bit mode
    SPI1->CR1 &= ~SPI_CR1_DFF;
    // SPI1->CR2 |= SPI_CR2_SSOE | SPI_CR2_NSSP;
    SPI1->CR1 |= SPI_CR1_SSM | SPI_CR1_SSI;
    SPI1->CR1 |= SPI_CR1_SPE;
}


void internal_clock() {
    RCC->CR &= ~RCC_CR_HSEON;          // Disable HSE to allow use of GPIOs
    FLASH->ACR = FLASH_ACR_PRFTEN | FLASH_ACR_LATENCY_1WS; // Enable prefetch and set flash latency
    RCC->CFGR |= RCC_CFGR_HPRE_DIV1;      // HCLK = SYSCLK
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;     // APB1 (PCLK1) max is 42MHz, so divide by 2
    RCC->CFGR |= RCC_CFGR_PPRE2_DIV1;     // APB2 (PCLK2) can run at full speed
    RCC->PLLCFGR = (16 << RCC_PLLCFGR_PLLM_Pos)  // PLLM = 16 (input divider)
                 | (96 << RCC_PLLCFGR_PLLN_Pos)  // PLLN = 96 (multiplier)
                 | (0 << RCC_PLLCFGR_PLLP_Pos)   // PLLP = 2 (output divider)
                 | (4 << RCC_PLLCFGR_PLLQ_Pos)   // PLLQ = 4 (USB OTG FS requires 48 MHz)
                 | RCC_PLLCFGR_PLLSRC_HSI;       // Use HSI as PLL source // Use HSI as PLL source
    RCC->CR |= RCC_CR_PLLON;                // Enable PLL
    while((RCC->CR & RCC_CR_PLLRDY) == 0); // Wait till PLL is ready
    RCC->CFGR &= (uint32_t)(~RCC_CFGR_SW);
    RCC->CFGR |= (uint32_t)RCC_CFGR_SW_PLL; // Select PLL as system clock source
    while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS) != (uint32_t)RCC_CFGR_SWS_PLL); // Wait till PLL is used as system clock source
}

void test_pin()
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    GPIOB->MODER &= ~(GPIO_MODER_MODER7);
    GPIOB->MODER |= GPIO_MODER_MODER7_0;
    GPIOB->ODR |= GPIO_ODR_OD7;
}
int main(void) {
    internal_clock();         // Initialize system clock
    
    init_spi1();
    LCD_Setup();
    LCD_Clear(BLACK); // If the screen turns black, that means it's working.
    
    Display_Top('s');
    
    //switchToCom();
    //switchToSat();
    init_DSP();
    
    /*RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    GPIOB->MODER &= ~(GPIO_MODER_MODE4 |GPIO_MODER_MODE5 |GPIO_MODER_MODE6 |GPIO_MODER_MODE7);
    GPIOB->ODR |= GPIO_ODR_OD4 | GPIO_ODR_OD5 | GPIO_ODR_OD6 | GPIO_ODR_OD7;*/
    
    while(1) {
        asm("wfi");           // Enter sleep mode to save power
    }
}


