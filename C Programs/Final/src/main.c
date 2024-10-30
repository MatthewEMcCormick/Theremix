#include "stm32f4xx.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "IO_DSP.h"
#include "clock.h"
#include "DSP.h"
#include "global.h"

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

// Function to initialize SPI4 for OLED
void init_spi4() {
    // Enable GPIO and SPI clocks
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;  // Enable GPIOE clock
    // Configure PE2 for SCK, PE4 for NSS, PE6 for MOSI (Alternate Function mode)
    GPIOE->MODER &= ~(GPIO_MODER_MODER2 | GPIO_MODER_MODER4 | GPIO_MODER_MODER6);
    GPIOE->MODER |= ((2 << GPIO_MODER_MODER2_Pos) | (2 << GPIO_MODER_MODER4_Pos) | (2 << GPIO_MODER_MODER6_Pos)); // Alternate function mode for PE2, PE4, PE6

    GPIOE->AFR[0] &= ~(GPIO_AFRL_AFRL2 | GPIO_AFRL_AFRL4 | GPIO_AFRL_AFRL6); // AF5 for SPI4
    GPIOE->AFR[0] |= 0x05050500;


    RCC->APB2ENR |= RCC_APB2ENR_SPI4EN;   // Enable SPI4 clock
    // SPI4 Configuration
    SPI4->CR1 &= ~SPI_CR1_SPE;             // Disable SPI4 for configuration
    SPI4->CR1 |= SPI_CR1_BR;               // Set baud rate (adjust as needed)


    //uncertain on setting bit size here:
    // SPI4->CR1 &= ~SPI_CR1_DFF;
    SPI4->CR1 |= SPI_CR1_DFF;
    ///


    SPI4->CR1 |= SPI_CR1_MSTR;             // Master mode
    SPI4->CR1 |= SPI_CR1_SSM;
    SPI4->CR1 |= SPI_CR1_SSI; // Enable software NSS management

                //SPI4->CR1 &= ~SPI_CR1_SSI; maybe clear instead?
    SPI4->CR1 &= ~SPI_CR1_LSBFIRST;

    SPI4->CR2 |= SPI_CR2_SSOE;             // Enable SSOE to drive NSS low when active
    
    // SPI1->CR2 |= SPI_CR2_TXDMAEN; UNCOMMENT THIS WHEN YOU ENABLE DMA
    
    SPI4->CR1 |= SPI_CR1_SPE;              // Enable SPI4
   
}// Go through with working version, DS, and a few others dont perfectly align.

//For 10 bit but hoping will work with 16
void spi4_cmd(unsigned int data) {
    while(!(SPI4->SR & SPI_SR_TXE)) {}     // Wait until the transmit buffer is empty
    SPI4->DR = data;                       // Send the command
}

// Function to send data to the OLED
void spi4_data(unsigned int data) {
    spi4_cmd(data | 0x200);                // Set the data flag and send
}

// Function to initialize the OLED display with SPI commands
void spi4_init_oled() {
    nano_wait(1000000);                     // Wait for power-up
    spi4_cmd(0x38);                        // Function set
    spi4_cmd(0x08);                        // Display off
    spi4_cmd(0x01);                        // Clear display
    nano_wait(2000000);                    // Wait for clear to complete
    spi4_cmd(0x06);                        // Entry mode set
    spi4_cmd(0x02);  
    spi4_cmd(0x0C);                        // Display on
}

// Function to display a message on the first line of the OLED
void spi4_display1(const char *string) {
    spi4_cmd(0x02);                        // Set cursor to the first position of line 1
    while (*string != '\0') {
        spi4_data(*string);                // Send each character
        string++;
    }
}

// Function to display a message on the second line of the OLED
void spi4_display2(const char *string) {
    spi4_cmd(0xc0);                        // Set cursor to the first position of line 2
    while (*string != '\0') {
        spi4_data(*string);                // Send each character
        string++;
    }
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
                 | RCC_PLLCFGR_PLLSRC_HSI;       // Use HSI as PLL source
    RCC->CR |= RCC_CR_PLLON;                // Enable PLL
    while((RCC->CR & RCC_CR_PLLRDY) == 0); // Wait till PLL is ready
    RCC->CFGR &= (uint32_t)(~RCC_CFGR_SW);
    RCC->CFGR |= (uint32_t)RCC_CFGR_SW_PLL; // Select PLL as system clock source
    while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS) != (uint32_t)RCC_CFGR_SWS_PLL); // Wait till PLL is used as system clock source
}

// Main function
int main(void) {
    internal_clock();         // Initialize system clock
    init_spi4();             // Initialize SPI4
    spi4_init_oled();        // Initialize OLED

    // Display a message on the OLED
    spi4_display1("Hello World!"); // Change this message as needed

    while(1) {
        asm("wfi");           // Enter sleep mode to save power
    }
}
