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
int numHighs = 0;
int numInterupts = 0;


void init_DSP()
{
    initLookUpTan();
    init_GPIO();
    setup_adc();
    init_ADC();
    init_DAC();
    init_tim3();
    init_TIM5();
    init_TIM2();
    
}
void togglexn(GPIO_TypeDef *port, int n) {
  port->ODR ^= (1 << n);
}
void init_GPIO(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;  // Enable clock to the GPIOA
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOFEN;
    //adc config // For FINAL version we want it to be PA0
    GPIOF->MODER &= ~(3 << (3 * 2));; //1111 1111 1111 1111 1111 1111 1111 1100
    GPIOF->MODER |= (3 << (3 * 2));    // Set PF3 to Analog mode

    //dac config // For FINAL version we want it to be PA5
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
    TIM5->PSC = 30 - 1;//96-1;  //samples at 5000hz * 2 so 10hz since 10k hz
    TIM5->ARR = 10 - 1;//100-1;
    TIM5->DIER |= TIM_DIER_UIE;
    TIM5->CR1 |= TIM_CR1_CEN;
    NVIC_EnableIRQ(TIM5_IRQn);
    NVIC_SetPriority(TIM5_IRQn,2);
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
        
        //uint16_t val = (uint16_t)(lookUp[280]);
        //(Drive, WD, Curve) //Default 50
        //Settings[0] = 100;
        //(assuming 1V offset for testing need to change to 1.5V (1861))
        int scaleInput = ADC3->DR - 1861; //1861 shifts wave down by 1.5V to center around 0 for processing.
        int preSat = 0;

        //calculations only work on postive portion of a wave so have to invert any portion of sine wave below 0
        if(scaleInput <=  0)
        {
            preSat = scaleInput * -1;
        }
        else
        {
            preSat = scaleInput;
        }
        uint16_t sat = saturator(100,0,100,preSat);
        /**
         * Ratio 0 1 2 3 4 
         
         */
        //Settings from threshold should not be scaled to 0-100!!!!!!!!!!!!!!
        //compressor(Input Gain, Output Gain, Wet/Dry, ratio, threshold, attack, release, audio sample)
        // Call selectRatio before calling compressor to get target Ratio value from user Parameters. It returns the ratio value to pass to compressor function.
        //currentRatio starts 


        //scales threshold level from 0 - 2000 (0-1.6V) depedning on the user input.  
        uint16_t thres = (Settings[7] * 3000 * 41) >> 12;
        uint16_t targetRatio = selectRatio(Settings[6]);
        uint16_t comp = compressor(100, 100, 100, targetRatio,thres, 0, 0,sat);

        //reverts bottom half of wave back
        if(scaleInput <=  0)
        {
            comp = comp * -1;
        }
        DAC->DHR12R2 = comp + 1861; // shift back up to 1.5V
        
    }
    }
}

uint16_t selectRatio(uint16_t ratio)
{
    if(ratio > 80)
    {
        return 4;
    }
    else if(ratio > 60)
    {
        return 3;
    }
    else if(ratio > 40)
    {
        return 2;
    }
    else if( ratio > 20)
    {
        return 1;
    }
    return 0;
}
void setup_adc(void) {

    // Enable GPIOA, GPIOB, and GPIOC clocks
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN;
    
    // Enable ADC clock
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    
    // Configure PA0-4, PA6-7 for analog input
    // For FInal we want PA1-PA3, PA6-PA-7
    GPIOA->MODER |= GPIO_MODER_MODER0 | GPIO_MODER_MODER1 | GPIO_MODER_MODER2 |
                    GPIO_MODER_MODER3 | GPIO_MODER_MODER4 | GPIO_MODER_MODER6 | GPIO_MODER_MODER7;
    
    // Configure PB0 and PB1 for analog input (ADC_IN8 and ADC_IN9)
    //
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
    
    //for (int i = 0; i < 10000; i++); 
}

void TIM3_IRQHandler() {
    TIM3->SR &= ~TIM_SR_UIF;

    int current_channel = activeSetting;

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
        Settings[activeSetting] = newVal;
        
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

/*void init_tim7(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
    TIM6->PSC = (3000 - 1);
    TIM6->ARR = (10 - 1);
    TIM6->DIER |= TIM_DIER_UIE;
    TIM6->CR1 |= TIM_CR1_CEN;
    NVIC_EnableIRQ(TIM6_IRQn);
    NVIC_SetPriority(TIM6_IRQn,2);
}

void TIM6_IRQHandler() {


}*/