#include "stm32f4xx.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "IO_DSP.h"
#include "DSP.h"
#include "global.h"
#include "lcd.h"

void nano_wait(unsigned int n);
// Function to initialize SPI4 for OLED
void init_spi4(void);
// Function to send a command to the OLED
void spi4_cmd(unsigned int data);
// Function to send data to the OLED
void spi4_data(unsigned int data);
// Function to initialize the OLED display with SPI commands
void spi4_init_oled(void);
// Function to display a message on the first line of the OLED
void spi4_display1(const char *string);
// Function to display a message on the second line of the OLED
void spi4_display2(const char *string);
void internal_clock(void);


// Function to create a small delay
void nano_wait(unsigned int n) {
    asm(    "        mov r0,%0\n"
            "repeat: sub r0,#83\n"
            "        bgt repeat\n" : : "r"(n) : "r0", "cc");
}

//===========================================================================
// 4.4 SPI OLED Display
//===========================================================================
/*void init_spi1() {
    // PB3  SPI1_SCK  
    // PB5  SPI1_MOSI/SDI
    // PA15 SPI1_NSS/CS
    // RCC->AHBENR |= RCC_AHBENR_GPIOEEN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    GPIOB->MODER &= ~(GPIO_MODER_MODER8 | GPIO_MODER_MODER11 | GPIO_MODER_MODER14);


    //This needs converted all to the right pins
    // I              I             I 
    // I              I             I 
    // V              V             V
    // Clear all pb's used
    //2 and 6 are the ones we care abour here on the 439
    //SCK   MOSI   nss doesnt get used, just tied to ground i think   NSS (not used)
    GPIOE->MODER &= ~(GPIO_MODER_MODER2 | GPIO_MODER_MODER6);
    // 8 11 and 14 as gpio outputs
    
    /// THE INIT_SPI1_SLOW PART OF THE CODE
    GPIOE->MODER |= (GPIO_MODER_MODER2_1 | GPIO_MODER_MODER6_1 );
    GPIOE->AFR[0] &= ~(GPIO_AFRL_AFRL2 | GPIO_AFRL_AFRL6);


    //Probably dont need all the PB4 stuff

    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

    SPI4->CR1 &= ~SPI_CR1_SPE;
    SPI4->CR1 &= ~SPI_CR1_BR; //Testing different speeds, LCD wants 24Mhz
    // SPI4->CR1 |= 0x1<<3; // 6Mhz instead of 48, might need to make this flat 0 to be /3Mhz which is still *8 slower than the 24BR that we get on the STM32F091
    SPI4->CR1 |= 0b001<<3;
    SPI4->CR1 |= SPI_CR1_MSTR;

    // SPI4->CR2 |= 0x0700; // This is 8 bit, the whole reason i have to switch
    // SPI4->CR2 &= ~0x0800; //This is the key difference. Using 8-bit frames.
    //8 bit on the 439 is:
    SPI4->CR1 &= ~SPI_CR1_DFF;
   

    // SPI1->CR2 |= SPI_CR2_SSOE | SPI_CR2_NSSP;
    SPI4->CR1 |= SPI_CR1_SSM | SPI_CR1_SSI;
    //Dont think this is needed either because it has to do with reading the 8 bits i hope
    // SPI4->CR2 |= SPI_CR2_FR;

    SPI4->CR1 |= SPI_CR1_SPE;



    //End of SPI stuff now on 2.7 init LCD
    // GPIOB->MODER |= GPIO_MODER_MODER8_0;

}*/


void init_spi1() {
    // PB3  SPI1_SCK  
    // PB5  SPI1_MOSI/SDI
    // PA15 SPI1_NSS/CS
    // RCC->AHBENR |= RCC_AHBENR_GPIOEEN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    GPIOB->MODER &= ~(GPIO_MODER_MODER3 | GPIO_MODER_MODER5);


    //This needs converted all to the right pins
    // I              I             I 
    // I              I             I 
    // V              V             V
    // Clear all pb's used
    //2 and 6 are the ones we care abour here on the 439
    //SCK   MOSI   nss doesnt get used, just tied to ground i think   NSS (not used)
    GPIOB->MODER &= ~(GPIO_MODER_MODER8 | GPIO_MODER_MODER9 |GPIO_MODER_MODER11);
    // 8 11 and 14 as gpio outputs
    GPIOB->MODER |= (GPIO_MODER_MODER8_0 | GPIO_MODER_MODER9_0 |GPIO_MODER_MODER11_0 );
    /// THE INIT_SPI1_SLOW PART OF THE CODE
    GPIOB->MODER |= (GPIO_MODER_MODER3_1 | GPIO_MODER_MODER5_1 );
    GPIOB->AFR[0] &= ~(GPIO_AFRL_AFRL3 | GPIO_AFRL_AFRL5);
    GPIOB->AFR[0] |= ((0x5 << 12) | (0x5 << 20));


    //Probably dont need all the PB4 stuff

    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

    SPI1->CR1 &= ~SPI_CR1_SPE;
    SPI1->CR1 &= ~SPI_CR1_BR; //Testing different speeds, LCD wants 24Mhz
    // SPI4->CR1 |= 0x1<<3; // 6Mhz instead of 48, might need to make this flat 0 to be /3Mhz which is still *8 slower than the 24BR that we get on the STM32F091
    SPI1->CR1 |= 0b001<<3;
    SPI1->CR1 |= SPI_CR1_MSTR;

    // SPI4->CR2 |= 0x0700; // This is 8 bit, the whole reason i have to switch
    // SPI4->CR2 &= ~0x0800; //This is the key difference. Using 8-bit frames.
    //8 bit on the 439 is:
    SPI1->CR1 &= ~SPI_CR1_DFF;
   

    // SPI1->CR2 |= SPI_CR2_SSOE | SPI_CR2_NSSP;
    SPI1->CR1 |= SPI_CR1_SSM | SPI_CR1_SSI;
    //Dont think this is needed either because it has to do with reading the 8 bits i hope
    // SPI4->CR2 |= SPI_CR2_FR;

    SPI1->CR1 |= SPI_CR1_SPE;



    //End of SPI stuff now on 2.7 init LCD
    // GPIOB->MODER |= GPIO_MODER_MODER8_0;

}

// Function to initialize the system clock
void internal_clock() {
    RCC->CR &= ~RCC_CR_HSEON;               // Disable HSE to allow use of GPIOs
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


// Main function
int main(void) {
    internal_clock();         // Initialize system clock
    //init_DSP(); 
    init_spi1();
    LCD_Setup();
    LCD_Clear(0xFFFF); // If the screen turns black, that means it's working.
    // Insert your picture code here...

    LCD_DrawString(20,20, BLACK, WHITE, "SAT  1  2  3    Com  1  2  3", 16, 0);
    LCD_DrawFillRectangle(20, 100, 30, 200, RED); // chimneys
    LCD_DrawFillRectangle(35, 150, 45, 200, GREEN);
    LCD_DrawFillRectangle(50, 125, 60, 200, BLUE);
    /*while(1) {
        //asm("wfi");           // Enter sleep mode to save power
    }*/
}
