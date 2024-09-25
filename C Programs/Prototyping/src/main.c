/**
  ******************************************************************************
 SENIOR DESIGN
  ******************************************************************************
*/
/******************************************************************************
*/ 

#include "stm32f0xx.h"
#include <stdint.h>
const char* username = "lmoloney";

int activeSetting = 0; //If running into errors maybe make this volatile
int Settings[12];
int currentPage = 0;
char* currentSetting = "";


void initc(void);
void initb(void);
void togglexn(GPIO_TypeDef *port, int n);
void init_exti(void);
void set_col(int col);
void SysTick_Handler(void);
void adjust_priorities(void);
void nano_wait(unsigned int n);

void init_spi1(void);
void spi_cmd(unsigned int data);
void spi_data(unsigned int data);
void spi1_init_oled(void);
void spi1_display1(const char *string);
void spi1_display2(const char *string);
void spi1_setup_dma(void);
void spi1_enable_dma(void);

void change_message(int page);

void setup_adc(void);
void TIM2_IRQHandler(void);
void init_tim2(void);


// extern void autotest();
extern void internal_clock();
void nano_wait(unsigned int n) {
    asm(    "        mov r0,%0\n"
            "repeat: sub r0,#83\n"
            "        bgt repeat\n" : : "r"(n) : "r0", "cc");
}
// extern void nano_wait(int);

int main(void) {
    internal_clock();
    // Uncomment when most things are working
    // autotest();
    
    // initb(); // B really only handled the 7 segment display which im not actually using right now. Might be better to just comment out
    initc();
    init_exti();
    // init_systick();
    // adjust_priorities();
    init_spi1();
    spi1_init_oled();
    spi1_setup_dma();
    spi1_enable_dma();

    setup_adc();
    init_tim2();
    // Slowly blinking
    // for(;;) {
        // togglexn(GPIOC, 9);
        // nano_wait(500000000);
    // }
}

void initc() {
    // Enable GPIOC clock
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;

    // Set PC10 and PC11 as input (00 in MODER)
    GPIOC->MODER &= ~(3 << (10 * 2)); // PC10 as input
    GPIOC->MODER &= ~(3 << (11 * 2)); // PC11 as input

    // Enable pull-up resistors for PC10 and PC11
    GPIOC->PUPDR |= (1 << (10 * 2));  // Pull-up for PC10
    GPIOC->PUPDR |= (1 << (11 * 2));  // Pull-up for PC11
}


/**
 * @brief Init GPIO port B
 *        PB0, PB2, PB3, PB4 as input pins
 *          enable pull down resistor on PB2 and PB3
 *        PB8-PB11 as output pins
 * 
 */


/**
 * @brief Change the ODR value from 0 to 1 or 1 to 0 for a specified 
 *        pin of a port.
 * 
 * @param port : The passed in GPIO Port
 * @param n    : The pin number
 */
void togglexn(GPIO_TypeDef *port, int n) {
  port->ODR ^= (1 << n);
}

/**
 * @brief Follow the lab manual to initialize EXTI.  In a gist:
 *        (1-2) Enable the SYSCFG subsystem, and select Port B for
 *            pins 0, 2, 3, and 4.
 *        (3) Configure the EXTI_RTSR register so that an EXTI
 *            interrupt is generated on the rising edge of 
 *            each of the pins.
 *        (4) Configure the EXTI_IMR register so that the EXTI
 *            interrupts are unmasked for each of the pins.
 *        (5) Enable the three interupts for EXTI pins 0-1, 2-3 and
 *            4-15. Don't enable any other interrupts.
 */
void init_exti() {
    // Enable SYSCFG clock (required for EXTI configuration)
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN;

    // Configure EXTI lines 10 and 11 for port C
    SYSCFG->EXTICR[2] &= ~(0xF << (2 * 4)); // Clear EXTI10 configuration bits
    SYSCFG->EXTICR[2] &= ~(0xF << (3 * 4)); // Clear EXTI11 configuration bits
    SYSCFG->EXTICR[2] |= (2 << (2 * 4));    // Map PC10 to EXTI10
    SYSCFG->EXTICR[2] |= (2 << (3 * 4));    // Map PC11 to EXTI11

    // Enable rising and falling trigger for EXTI10 and EXTI11
    EXTI->RTSR |= (1 << 10) | (1 << 11);
    // EXTI->FTSR |= (1 << 10) | (1 << 11);

    // Unmask EXTI10 and EXTI11
    EXTI->IMR |= (1 << 10) | (1 << 11);

    // Enable the EXTI4_15 interrupt in the NVIC (EXTI10 and EXTI11 fall under this)
    NVIC->ISER[0] = (1 << EXTI4_15_IRQn);
}

//==========================================================
// Write the EXTI interrupt handler for pins 0 and 1 below.
// Copy the name from the startup file as explained in the 
// lab manual, create a label of that name below, and declare 
// it to be a function.
// It should acknowledge the pending bit for pin 0, and 
// it should call togglexn(GPIOB, 8).


//==========================================================
// Write the EXTI interrupt handler for pins 4-15 below.
// It should acknowledge the pending bit for pin 4, and 
// it should call togglexn(GPIOB, 10).

void EXTI4_15_IRQHandler() {
    // Check if EXTI10 caused the interrupt
    if (EXTI->PR & (1 << 10)) {
        EXTI->PR = (1 << 10);  // Clear the pending interrupt flag for EXTI10
        // togglexn(GPIOB, 8);     // Call your function to toggle pin PB8
        if (currentPage == 1)
        {
          currentPage = 2;
        }
        else{
          currentPage = 1;
        }
        
    }

    // Check if EXTI11 caused the interrupt
    if (EXTI->PR & (1 << 11)) {
        EXTI->PR = (1 << 11);  // Clear the pending interrupt flag for EXTI11
        if (currentPage == 3)
        {
          currentPage = 4;
        }
        else
        {
          currentPage = 3;
        }
    }
  change_message(currentPage);
}


void adjust_priorities() {
  NVIC_SetPriority(EXTI2_3_IRQn, 192 >> 6);
  NVIC_SetPriority(EXTI4_15_IRQn, 128 >> 6);
}




///OLED shenanigans
uint16_t display[34] = {
        0x002, // Command to set the cursor at the first position line 1
        0x200+'C', 0x200+'h', 0x200+'o', 0x200+'o', 0x200+'s', + 0x200+'e', 0x200+' ', 0x200+'S',
        0x200+'e', 0x200+'t', 0x200+'t', 0x200+'i', + 0x200+'n', 0x200+'g', 0x200+' ', 0x200+' ',
        0x0c0, // Command to set the cursor at the first position line 2
        0x200+'E', 0x200+'Q', 0x200+':', 0x200+'1', 0x200+' ', + 0x200+'S', 0x200+'a', 0x200+'t',
        0x200+':', 0x200+'2', 0x200+' ', 0x200+' C', + 0x200+'o', 0x200+'m', 0x200+':', 0x200+'3',
};

//===========================================================================
// Configure the proper DMA channel to be triggered by SPI1_TX.
// Set the SPI1 peripheral to trigger a DMA when the transmitter is empty.
//===========================================================================
void spi1_setup_dma(void) {
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;
    // Turn off DMA
    // CPAR to SPI1 DR adr
    // CMAR to msg adr
    // 34 units in array
    // mem to per direction
    // Increment mem adr
    // no inc per adr
    // Set data size to 16b
    // Operate circularly
    // Configure DMA. They should be using channel 3.
    DMA1_Channel3->CCR &= ~DMA_CCR_EN;
    DMA1_Channel3->CPAR = (uint32_t) &(SPI1->DR);
    DMA1_Channel3->CMAR = (uint32_t) &(display);
    DMA1_Channel3->CNDTR = 34;
    DMA1_Channel3->CCR |= DMA_CCR_DIR;
    DMA1_Channel3->CCR |= DMA_CCR_MINC;
    DMA1_Channel3->CCR &= ~DMA_CCR_PINC;
    DMA1_Channel3->CCR |= 0x0500;
    DMA1_Channel3->CCR |= DMA_CCR_CIRC;
}

//===========================================================================
// Enable the DMA channel triggered by SPI1_TX.
//===========================================================================
void spi1_enable_dma(void) {
    DMA1_Channel3->CCR |= DMA_CCR_EN;
}


//===========================================================================
// 4.4 SPI OLED Display
//===========================================================================
void init_spi1() {
    // PB3  SPI1_SCK
    // PB5  SPI1_MOSI/SDI
    // PA15 SPI1_NSS/CS
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

    // Configure PA15 for NSS
    GPIOA->MODER &= ~(3 << (2 * 15));
    GPIOA->MODER |= (2 << (2 * 15)); // Alternate function mode
    GPIOA->AFR[1] &= ~(0xF << (4 * (15 - 8)));
    // GPIOA->AFR[1] |= (0x0 << (4 * (15 - 8))); // AF0 for PA15 (SPI1_NSS)

    // Configure PB3 and PB5 for SCK and MOSI respectively
    GPIOB->MODER &= ~(3 << (2 * 3)) | ~(3 << (2 * 5));
    GPIOB->MODER |= (2 << (2 * 3)) | (2 << (2 * 5)); // Alternate function mode
    GPIOB->AFR[0] &= ~(0xF << (4 * 3)) | ~(0xF << (4 * 5));
    // GPIOB->AFR[0] |= (0x0 << (4 * 3)) | (0x0 << (4 * 5)); // AF0 for PB3 and PB5 (SPI1_SCK, SPI1_MOSI)

    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

    SPI1->CR1 &= ~SPI_CR1_SPE;

    SPI1->CR1 |= SPI_CR1_BR;

    SPI1->CR2 |= 0x0900;
    SPI1->CR2 &= ~0x0600;

    SPI1->CR1 |= SPI_CR1_MSTR;

    SPI1->CR2 |= SPI_CR2_SSOE | SPI_CR2_NSSP;

    SPI1->CR2 |= SPI_CR2_TXDMAEN;

    SPI1->CR1 |= SPI_CR1_SPE;
}

void spi_cmd(unsigned int data) {
    while(!(SPI1->SR & SPI_SR_TXE)) {}
    SPI1->DR = data;
}
void spi_data(unsigned int data) {
    spi_cmd(data | 0x200);
}
void spi1_init_oled() {
    nano_wait(1000000);
    spi_cmd(0x38);
    spi_cmd(0x08);
    spi_cmd(0x01);
    nano_wait(2000000);
    spi_cmd(0x06);
    spi_cmd(0x02);
    spi_cmd(0x0c);
}
void spi1_display1(const char *string) {
    spi_cmd(0x02);
    while(*string != '\0') {
        spi_data(*string);
        string++;
    }
}
void spi1_display2(const char *string) {
    spi_cmd(0xc0);
    while(*string != '\0') {
        spi_data(*string);
        string++;
    }
}


// settings[0]  - PA0 -> ADC_IN0
// settings[1]  - PA1 -> ADC_IN1
// settings[2]  - PA2 -> ADC_IN2
// settings[3]  - PA3 -> ADC_IN3
// settings[4]  - PA4 -> ADC_IN4
// settings[5]  - PC0 -> ADC_IN10
// settings[6]  - PC1 -> ADC_IN11
// settings[7]  - PC2 -> ADC_IN12
// settings[8]  - PC3 -> ADC_IN13
// settings[9]  - PB0 -> ADC_IN8
// settings[10] - PB1 -> ADC_IN9
// settings[11] - PA6 -> ADC_IN6
void change_message(int page) {
    char buffer[34];
    // Check if the message is "Saturator" or "Compressor" and append the correct setting
    if (page == 1) {
        //  snprintf(buffer, sizeof(buffer), "S1: %d %d %d %d %d", Settings[0], Settings[1], Settings[2], Settings[3], Settings[4]); // Show Settings[1] for Compressor
         snprintf(buffer, sizeof(buffer), "PA0: %d", Settings[0]);
    } else if (page == 2) {
        snprintf(buffer, sizeof(buffer), "S2: %d %d", Settings[5], Settings[6]); //
        // snprintf(buffer, sizeof(buffer), "S2");
    }
    else if (page == 3) {
        snprintf(buffer, sizeof(buffer), "C1: %d %d %d", Settings[7], Settings[8], Settings[9]);
    }
    else if (page == 4) {
        snprintf(buffer, sizeof(buffer), "C2: %d %d", Settings[10], Settings[11]);
    }
    else {
        strncpy(buffer, "Press PB for SAT or COM", sizeof(buffer)); // Use the message as is for other cases
    }
    
    // Ensure the message is exactly 34 characters (padded with spaces if necessary)
    for (int i = 0; i < 34; i++) {
        if (i < 16) {
            // Update the first line of the display (line 1 starts at index 1)
            display[i+1] = (i < strlen(buffer)) ? 0x200 + buffer[i] : 0x200 + ' ';
        } else if (i >= 16 && i < 32) {
            // Update the second line of the display (line 2 starts at index 17)
            display[i+2] = (i < strlen(buffer) ? 0x200 + buffer[i] : 0x200 + ' ');
        }
    }

    // Reset the DMA transfer by disabling and re-enabling it
    DMA1_Channel3->CCR &= ~DMA_CCR_EN;   // Disable DMA
    DMA1_Channel3->CMAR = (uint32_t) &(display);  // Update memory address
    DMA1_Channel3->CNDTR = 34;           // Reload data size
    DMA1_Channel3->CCR |= DMA_CCR_EN;    // Re-enable DMA
}



// settings[0]  - PA0 -> ADC_IN0
// settings[1]  - PA1 -> ADC_IN1
// settings[2]  - PA2 -> ADC_IN2
// settings[3]  - PA3 -> ADC_IN3
// settings[4]  - PA4 -> ADC_IN4
// settings[5]  - PC0 -> ADC_IN10
// settings[6]  - PC1 -> ADC_IN11
// settings[7]  - PC2 -> ADC_IN12
// settings[8]  - PC3 -> ADC_IN13
// settings[9]  - PB0 -> ADC_IN8
// settings[10] - PB1 -> ADC_IN9
// settings[11] - PA6 -> ADC_IN6
//This is all ADC for reading in extra
void setup_adc(void) {

    // Enable GPIOA, GPIOB, and GPIOC clocks
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIOCEN;
    
    // Enable ADC clock
    RCC->APB2ENR |= RCC_APB2ENR_ADCEN;
    
    // Configure PA0-4, PA6-7 for analog input
    GPIOA->MODER |= GPIO_MODER_MODER0 | GPIO_MODER_MODER1 | GPIO_MODER_MODER2 |
                    GPIO_MODER_MODER3 | GPIO_MODER_MODER4 | GPIO_MODER_MODER6 | GPIO_MODER_MODER7;
    
    // Configure PB0 and PB1 for analog input (ADC_IN8 and ADC_IN9)
    GPIOB->MODER |= GPIO_MODER_MODER0 | GPIO_MODER_MODER1;
    
    // Configure PC0–PC3 for analog input (ADC_IN10 to ADC_IN13)
    GPIOC->MODER |= GPIO_MODER_MODER0 | GPIO_MODER_MODER1 | GPIO_MODER_MODER2;// | GPIO_MODER_MODER3;

    // Turn on HSI14 clock for ADC
    RCC->CR2 |= RCC_CR2_HSI14ON;
    while(!(RCC->CR2 & RCC_CR2_HSI14RDY));
    
    // Set ADC resolution to 8 bits (RES = 0b10)
    ADC1->CFGR1 &= ~ADC_CFGR1_RES;
    ADC1->CFGR1 |= (0b10 << ADC_CFGR1_RES_Pos);  // Set 8-bit resolution

    ADC1->SMPR &= ~0x7;  // <---This line needs to be removed DEBUGCODE

    // Enable ADC
    ADC1->CR |= ADC_CR_ADEN;
    while(!(ADC1->ISR & ADC_ISR_ADRDY));

    // Configure all required ADC channels: PA0-4, PA6-7, PB0-1, and PC0-3
    ADC1->CHSELR |= ADC_CHSELR_CHSEL0 | ADC_CHSELR_CHSEL1 | ADC_CHSELR_CHSEL2 |
                    ADC_CHSELR_CHSEL3 | ADC_CHSELR_CHSEL4 | ADC_CHSELR_CHSEL6 | ADC_CHSELR_CHSEL7 |
                    ADC_CHSELR_CHSEL8 | ADC_CHSELR_CHSEL9 | ADC_CHSELR_CHSEL10 | ADC_CHSELR_CHSEL11 |
                    ADC_CHSELR_CHSEL12;// | ADC_CHSELR_CHSEL13;
    while(!(ADC1->ISR & ADC_ISR_ADRDY));
}


void TIM2_IRQHandler() {
    TIM2->SR &= ~TIM_SR_UIF;

    int current_channel = activeSetting;

    // Handle skipping PA5 by incrementing the channel index accordingly
    if (activeSetting >= 5 && activeSetting <= 11) {
        current_channel += 1; // Skip PA5 (ADC_IN5)
    }

    // Set ADC channel based on current setting
    ADC1->CHSELR = (1 << current_channel);
    ADC1->CR |= ADC_CR_ADSTART;

    // Wait for conversion to complete and read value
    while (!(ADC1->ISR & ADC_ISR_EOC));
    Settings[activeSetting] = (ADC1->DR) / 2.56;  // Adjust scaling if needed

    change_message(currentPage);

    // Move to the next setting
    activeSetting = (activeSetting + 1) % 12;
}




//============================================================================
// init_tim2()
//============================================================================
void init_tim2(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    TIM2->PSC = (48000 - 1);
    TIM2->ARR = (100 - 1);
    TIM2->DIER |= TIM_DIER_UIE;
    TIM2->CR1 |= TIM_CR1_CEN;
    NVIC->ISER[0] = 1 << TIM2_IRQn;
}