#include "stm32f4xx.h"
#include <stdint.h>
#include "IO_DSP.h"
#include <math.h>
#include "DSP.h"
#include "global.h"
#include <stdio.h>
#include <stdlib.h>
//float lookUp[300];
int activeSetting = 0;
int newVal = 0;
volatile int Settings[12];
void init_DSP()
{
    initLookUpTan();
    setup_adc();
    init_tim3();
    init_GPIO();
    init_ADC();
    init_TIM5();
    init_DAC();
    init_TIM2();
    
}

void init_GPIO(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;  // Enable clock to the GPIOA
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOFEN;
    //adc config 
    GPIOF->MODER &= ~(3 << (3 * 2));; //1111 1111 1111 1111 1111 1111 1111 1100
    GPIOF->MODER |= (3 << (3 * 2));    // Set PF3 to Analog mode

    //dac config
    GPIOA->MODER &= ~(3 << (5 * 2));; //1111 1111 1111 1111 1111 1111 1111 1100
    GPIOA->MODER |= (3 << (5* 2));    // Set PA4 to Analog mode

    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    GPIOB->MODER &= ~(3 << (7 * 2));; //1111 1111 1111 1111 1111 1111 1111 1100
    GPIOB->MODER |= (1 << (7 * 2)); 

}

void init_ADC(void)
{
    //enable clock
    RCC->APB2ENR |= RCC_APB2ENR_ADC3EN;
    //enable high speed clock
    RCC->CR |= RCC_CR_HSION;
    while (!(RCC->CR & RCC_CR_HSIRDY))
    //set 12 bit resoltion
    ADC3->CR1 &= ~(0x3 << ADC_CR1_RES_Pos); 
    ADC3->CR1 &= ~ADC_CR1_RES;
    //enable adc
    ADC3->CR2 |= ADC_CR2_ADON; 
    ADC3->CR2 |= ADC_CR2_CONT;
    //wait for adc3
    for (int i = 0; i < 10000; i++); 
    //enable chanel
    ADC3->SQR3 = 9;
    for (int i = 0; i < 10000; i++); 
    
}

void init_TIM5(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM5EN; // Enable Timer 3 clock
    TIM5->PSC = 10 - 1;//96-1;  //samples at 5000hz * 2 so 10hz since 10k hz
    TIM5->ARR = 10 - 1;//100-1;
    TIM5->DIER |= TIM_DIER_UIE;
    TIM5->CR1 |= TIM_CR1_CEN;
    NVIC_EnableIRQ(TIM5_IRQn);
    NVIC_SetPriority(TIM5_IRQn,2);
    //NVIC->ISER[0] = 1 << TIM5_IRQn;
}

//adc interupt
void TIM5_IRQHandler(void) 
{
    if (TIM5->SR & TIM_SR_UIF) {
    TIM5->SR &= ~TIM_SR_UIF;
    ADC3->CR2 |= ADC_CR2_SWSTART;
    //GPIOB->ODR ^= (1 << 7); 
    while (!(ADC3->SR & ADC_SR_EOC));
    }
}

void init_DAC(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_DACEN;

    //clear trigger selection //trigger was causing an issue 
    DAC->CR &= ~DAC_CR_TSEL2;       
    //DAC->CR |= DAC_CR_TEN1;  
    //DAC->CR |= (4 << DAC_CR_TSEL1_Pos); 
    //DAC->CR &= ~DAC_CR_TEN1;    
    DAC->CR |= DAC_CR_BOFF2;
    DAC->CR |= DAC_CR_EN2;
    //DAC->CR &= ~DAC_CR_EN1;
}

void init_TIM2() //clock speed made it smoother 
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; // Enable Timer 3 clock
    TIM2->PSC = 10 - 1;
    TIM2->ARR = 10 - 1;
    TIM2->DIER |= TIM_DIER_UIE;
    TIM2->CR1 |= TIM_CR1_CEN;
    NVIC_EnableIRQ(TIM2_IRQn);
    NVIC_SetPriority(TIM2_IRQn,2);
}

void TIM2_IRQHandler(void) 
{
    if (TIM2->SR & TIM_SR_UIF) {
    TIM2->SR &= ~TIM_SR_UIF;
    //uint16_t sat = saturator(1,20,1,ADC3->DR);
    //if (ADC3->SR & ADC_SR_EOC)
    //{
    
    if (ADC3->SR & ADC_SR_EOC) {
        /*if(ADC3->DR < 0)
        {
            GPIOB->ODR ^= (1 << 7); 
        }*/
        //GPIOB->ODR ^= (1 << 7); 
        
        //uint16_t val = (uint16_t)(lookUp[280]);
        //(Drive, WD, Curve) //Default 50
        //Settings[0] = 100;
        uint16_t sat = saturator(100,100,100,ADC3->DR);
        //compressor(Input Gain, Output Gain, Wet/Dry, ratio, threshold, attack, release, audio sample)
        //upper ratio is 0-6 
        /**
         * Ratio 0 1 2 3 4 5 6 7 8 
         * If(currentRatio >= Max Ratio)
         * 
         * 
         * 
         * if(c 
         * 
         */
        //Settings from threshold should not be scaled to 0-100!!!!!!!!!!!!!!
        //compressor(Input Gain, Output Gain, Wet/Dry, ratio, threshold, attack, release, audio sample)
        uint16_t val = compressor(0, 0, Settings[6], 2, Settings[7], 0, 0,sat);
        DAC->DHR12R2 = val;//val;//saturator(50,50,100,ADC3->DR);
        
    }
    }
}


void setup_adc(void) {

    // Enable GPIOA, GPIOB, and GPIOC clocks
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN;
    
    // Enable ADC clock
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    
    // Configure PA0-4, PA6-7 for analog input
    GPIOA->MODER |= GPIO_MODER_MODER0 | GPIO_MODER_MODER1 | GPIO_MODER_MODER2 |
                    GPIO_MODER_MODER3 | GPIO_MODER_MODER4 | GPIO_MODER_MODER6 | GPIO_MODER_MODER7;
    
    // Configure PB0 and PB1 for analog input (ADC_IN8 and ADC_IN9)
    GPIOB->MODER |= GPIO_MODER_MODER0 | GPIO_MODER_MODER1;
    
    // Configure PC0–PC3 for analog input (ADC_IN10 to ADC_IN13)
    GPIOC->MODER |= GPIO_MODER_MODER0 | GPIO_MODER_MODER1 | GPIO_MODER_MODER2;// | GPIO_MODER_MODER3;

    // Turn on HSI14 clock for ADC
    RCC->CR |= RCC_CR_HSION;
    while (!(RCC->CR & RCC_CR_HSIRDY));
    
    // Set ADC resolution to 8 bits (RES = 0b10)
    ADC1->CR1 &= ~ADC_CR1_RES;
    ADC1->CR1 |= (0x2 << ADC_CR1_RES_Pos);  // Set 8-bit resolution

   //ADC1->SMPR &= ~0x7;  // <---This line needs to be removed DEBUGCODE

    // Enable ADC
    ADC1->CR2 |= ADC_CR2_ADON; 
    //ADC1->CR2 |= ADC_CR2_CONT;
    //wait for adc3
    for (int i = 0; i < 10000; i++); 
    //enable chanel
    // Configure all required ADC channels: PA0-4, PA6-7, PB0-1, and PC0-3
    //ADC1->SQR3 |= (0 << ADC_SQR3_SQ1_Pos) | (1 << ADC_SQR3_SQ2_Pos) | (2 << ADC_SQR3_SQ3_Pos)|(3 << ADC_SQR3_SQ4_Pos) | (4 << ADC_SQR3_SQ5_Pos)
    //            | (6 << ADC_SQR3_SQ6_Pos);
    //ADC1->SQR2 |= (7 << ADC_SQR2_SQ7_Pos) | (8 << ADC_SQR2_SQ8_Pos) | (9 << ADC_SQR2_SQ9_Pos) | (10 << ADC_SQR2_SQ10_Pos)
    //                | (11 << ADC_SQR2_SQ11_Pos) | (12 << ADC_SQR2_SQ12_Pos); // | ADC_CHSELR_CHSEL13;
    //wait
    //for (int i = 0; i < 10000; i++); 
}

void TIM3_IRQHandler() {
    TIM3->SR &= ~TIM_SR_UIF;

    int current_channel = activeSetting;

    // Handle skipping PA5 by incrementing the channel index accordingly
    /*if (activeSetting >= 5 && activeSetting <= 11) {
        current_channel += 1; // Skip PA5 (ADC_IN5)
    }*/

    // Set ADC channel based on current settin
    ADC1->SQR3 = current_channel;
    ADC1->SQR1 &= ~ADC_SQR1_L;

    ADC1->CR2 |= ADC_CR2_SWSTART;

    //GPIOB->ODR ^= (1 << 7); 
    while (!(ADC1->SR & ADC_SR_EOC));
   
    // Wait for conversion to complete and read value
    newVal = (ADC1->DR * 100) >> 8;
    if(abs(Settings[activeSetting] - newVal) > 4)
    {
        if(activeSetting == 7)
        {
            Settings[activeSetting] = ADC1->DR;
        }
        else
        {
            Settings[activeSetting] = newVal;
        }
        
    }
    //Settings[activeSetting] = 100;
    //Settings[activeSetting] = 100;
      // Adjust scaling if needed
    if(Settings[activeSetting] >= 90 && activeSetting == 6)
    {
        GPIOB->ODR = (1 << 7); 
    }
    if(Settings[activeSetting] <= 30 && activeSetting == 6)
    {
        GPIOB->ODR &= ~(1 << 7); 
    }
    //GPIOB->ODR ^= (1 << 7); 

    //change_message(currentPage);

    // Move to the next setting
    activeSetting = (activeSetting + 1) % 12;
}

void init_tim3(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    TIM3->PSC = (3000 - 1);
    TIM3->ARR = (10 - 1);
    TIM3->DIER |= TIM_DIER_UIE;
    TIM3->CR1 |= TIM_CR1_CEN;
    NVIC_EnableIRQ(TIM3_IRQn);
    NVIC_SetPriority(TIM3_IRQn,1);
}