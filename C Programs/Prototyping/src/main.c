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

volatile int activeSetting = 0;
volatile int Settings[2];
char* Setting = "2:Sat 3:Com";


void initc();
void initb();
void togglexn(GPIO_TypeDef *port, int n);
void init_exti();
void set_col(int col);
void SysTick_Handler();
void init_systick();
void adjust_priorities();

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
    // init_exti();
    init_systick();
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

/**
 * @brief Init GPIO port C
 *        PC0-PC3 as input pins with the pull down resistor enabled
 *        PC4-PC9 as output pins
 * 
 */
void initc() {
  RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
  for (int i = 0; i <= 9; i++) {
    GPIOC->MODER &= ~(3 << 2*i);
    GPIOC->PUPDR &= ~(3 << 2*i);
  }
  GPIOC->MODER |= (1 << 9*2) | (1 << 8*2) | (1 << 7*2) | (1 << 6*2) | (1 << 5*2) | (1 << 4*2);
  GPIOC->PUPDR |= (2 << 3*2) | (2 << 2*2) | (2 << 1*2) | (2 << 0*2);
}

/**
 * @brief Init GPIO port B
 *        PB0, PB2, PB3, PB4 as input pins
 *          enable pull down resistor on PB2 and PB3
 *        PB8-PB11 as output pins
 * 
 */
void initb() {
  RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
  GPIOB->MODER &= ~((3 << 2*0) | (3 << 2*2) | (3 << 2*3) | (3 << 2*4));
  GPIOB->PUPDR &= ~((3 << 2*2) | (3 << 2*3));
  GPIOB->PUPDR |= (2 << 2*2) | (2 << 2*3);

  GPIOB->MODER &= ~((3 << 2*8) | (3 << 2*9) | (3 << 2*10) | (3 << 2*11));
  GPIOB->MODER |= (1 << 2*8) | (1 << 2*9) | (1 << 2*10) | (1 << 2*11);
}

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
  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN; 
  SYSCFG->EXTICR[0] &= ~((0xF << 0*4) | (0xF << 2*4) | (0xF << 3*4));
  SYSCFG->EXTICR[1] &= ~((0xF << (4-4)*4));
  SYSCFG->EXTICR[0] |= (1 << 0*4) | (1 << 2*4) | (1 << 3*4);
  SYSCFG->EXTICR[1] |= (1 << (4-4)*4);
  EXTI->RTSR |= (1 << 0) | (1 << 2) | (1 << 3) | (1 << 4);
  EXTI->IMR |= (1 << 0) | (1 << 2) | (1 << 3) | (1 << 4);
  NVIC->ISER[0] = (1 << EXTI0_1_IRQn) | (1 << EXTI2_3_IRQn) | (1 << EXTI4_15_IRQn);
}

//==========================================================
// Write the EXTI interrupt handler for pins 0 and 1 below.
// Copy the name from the startup file as explained in the 
// lab manual, create a label of that name below, and declare 
// it to be a function.
// It should acknowledge the pending bit for pin 0, and 
// it should call togglexn(GPIOB, 8).
void EXTI0_1_IRQHandler() {
  EXTI->PR = (1 << 0);
  togglexn(GPIOB, 8);
}

//==========================================================
// Write the EXTI interrupt handler for pins 2-3 below.
// It should acknowledge the pending bit for pin 2, and 
// it should call togglexn(GPIOB, 9).
void EXTI2_3_IRQHandler() {
  EXTI->PR = (1 << 2);
  togglexn(GPIOB, 9);
}

//==========================================================
// Write the EXTI interrupt handler for pins 4-15 below.
// It should acknowledge the pending bit for pin 4, and 
// it should call togglexn(GPIOB, 10).
void EXTI4_15_IRQHandler() {
  EXTI->PR = (1 << 4);
  togglexn(GPIOB, 10);
}


/**
 * @brief Enable the SysTick interrupt to occur every 1/16 seconds.
 * 
 */
void init_systick() {
  SysTick->LOAD = 375000 - 1;
  SysTick->VAL = 0;
  SysTick->CTRL &= ~SysTick_CTRL_CLKSOURCE_Msk;
  // SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk;
  SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;
}

volatile int current_col = 1;

/**
 * @brief The ISR for the SysTick interrupt.
 * 
 */
void SysTick_Handler() {
  int r = GPIOC->IDR & 0xf;
  if ((current_col == 1) && (r & 8)) {
      change_message("EQ");
  } else if ((current_col == 2) && (r & 8)) {
      change_message("Saturator");
  } else if ((current_col == 3) && (r & 8)) {
      // togglexn(GPIOC, 7);
      change_message("Compressor");
  } else if ((current_col == 4) && (r & 8)) {
      // togglexn(GPIOC, 6);
  }

  current_col += 1;
  if (current_col > 4)
    current_col = 1;
  set_col(current_col);
}

/**
 * @brief For the keypad pins, 
 *        Set the specified column level to logic "high".
 *        Set the other three three columns to logic "low".
 * 
 * @param col 
 */
void set_col(int col) {
  GPIOC->BRR = (1 << 4) | (1 << 5) | (1 << 6) | (1 << 7);
  GPIOC->BSRR = (1 << (8-col));
}

/**
 * @brief Set the priority for EXTI pins 2-3 interrupt to 192.
 *        Set the priority for EXTI pins 4-15 interrupt to 128.
 *        Do not adjust the priority for any other interrupts.
 * 
 */
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
    // PA5  SPI1_SCK
    // PA7  SPI1_MOSI/SDI
    // PA15 SPI1_NSS/CS
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

    GPIOA->MODER &= ~0xc000fc00;
    GPIOA->MODER |=  0x8a00a800;

    GPIOA->AFR[0] &= ~0xfff00000;
    GPIOA->AFR[1] &= ~0xf0000000;

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


void change_message(const char* new_message) {
    Setting = new_message;
    char buffer[34];
    // Check if the message is "Saturator" or "Compressor" and append the correct setting
    if (strcmp(new_message, "Saturator") == 0) {
        snprintf(buffer, sizeof(buffer), "Saturator: %d", Settings[0]);  // Show Settings[0] for Saturator
    } else if (strcmp(new_message, "Compressor") == 0) {
        snprintf(buffer, sizeof(buffer), "Compressor: %d", Settings[1]); // Show Settings[1] for Compressor
    } else {
        strncpy(buffer, new_message, sizeof(buffer)); // Use the message as is for other cases
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



//This is all ADC for reading in extra
void setup_adc(void) {
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    GPIOA->MODER |= GPIO_MODER_MODER1;  // PA1 analog
    RCC->APB2ENR |= RCC_APB2ENR_ADCEN;
    RCC->CR2 |= RCC_CR2_HSI14ON;
    while(!(RCC->CR2 & RCC_CR2_HSI14RDY));
    ADC1->CR |= ADC_CR_ADEN;
    while(!(ADC1->ISR & ADC_ISR_ADRDY));
    ADC1->CHSELR |= ADC_CHSELR_CHSEL1;
    while(!(ADC1->ISR & ADC_ISR_ADRDY));
}

void TIM2_IRQHandler() {
    TIM2->SR &= ~TIM_SR_UIF;
    ADC1->CR |= ADC_CR_ADSTART;
    while(!(ADC1->ISR & ADC_ISR_EOC));
    
    Settings[activeSetting] = (ADC1->DR/39);

    change_message(Setting);


    /**/
    

    activeSetting = (activeSetting + 1) %2;
}



//============================================================================
// init_tim2()
//============================================================================
void init_tim2(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    TIM2->PSC = (48000 - 1);
    TIM2->ARR = (1000 - 1);
    TIM2->DIER |= TIM_DIER_UIE;
    TIM2->CR1 |= TIM_CR1_CEN;
    NVIC->ISER[0] = 1 << TIM2_IRQn;
}