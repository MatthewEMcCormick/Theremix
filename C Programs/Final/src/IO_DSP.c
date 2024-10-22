#include "stm32f4xx.h"
#include <stdint.h>
#include "IO_DSP.h"
#include "DSP.h"

void init_DSP()
{
    init_GPIO();
    init_ADC();
    init_TIM3();
    init_DAC();
    init_TIM2();
}
void init_GPIO(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;  // Enable clock to the GPIOA
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOFEN;
    //adc config 
    GPIOA->MODER &= ~(3 << (3 * 2));; //1111 1111 1111 1111 1111 1111 1111 1100
    GPIOA->MODER |= (3 << (3 * 2));    // Set PF3 to Analog mode

    //dac config
    GPIOA->MODER &= ~(3 << (4 * 2));; //1111 1111 1111 1111 1111 1111 1111 1100
    GPIOA->MODER |= (3 << (4* 2));    // Set PA4 to Analog mode

    //led config for testing
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    GPIOB->MODER &= ~(3 << (7 * 2));; //1111 1111 1111 1111 1111 1111 1111 1100
    GPIOB->MODER |= (1 << (7 * 2)); 

}

void init_ADC(void)
{
    //enable clock
    RCC->APB2ENR |= RCC_APB2ENR_ADC3EN;
    //enable high speed clock
    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR & RCC_CR_HSERDY))
    //set 12 bit resoltion
    ADC3->CR1 &= ~(0x3 << ADC_CR1_RES_Pos); 
    //ADC3->CR1 |= (0x00 << ADC_CR1_RES_Pos);
    ADC3->CR1 &= ~ADC_CR1_RES;
    //enable adc
    ADC3->CR2 |= ADC_CR2_ADON; 
    //wait for adc3
    for (int i = 0; i < 1000; i++); 
    //enable chanel
    ADC3->SQR3 = 9;
    for (int i = 0; i < 1000; i++); 
}

void init_TIM3(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; // Enable Timer 3 clock
    TIM3->PSC = 4800 - 1;//96-1;  //samples at 5000hz * 2 so 10hz since 10k hz
    TIM3->ARR = 100 - 1;//100-1;
    TIM3->DIER |= TIM_DIER_UIE;
    TIM3->CR1 |= TIM_CR1_CEN;
    NVIC->ISER[0] = 1 << TIM3_IRQn;
}

//adc interupt
void TIM3_IRQHandler(void) 
{
    TIM3->SR &= ~TIM_SR_UIF;
    ADC3->CR2 |= ADC_CR2_SWSTART;
    while (!(ADC3->SR & ADC_SR_EOC));
    //GPIOB->ODR ^= (1 << 7);
   /* adcOut = ADC1->DR;

    int saturatorOut;
    saturatorOut = saturator(1,1,1,adcOut);
    comp.audio = saturatorOut;
    comp = compressor(saturatorOut, 1, 1,1,4,1, comp);
    dacIn = comp.audio;*/

}

void init_DAC(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_DACEN;

    //clear trigger selection 
    DAC->CR &= ~DAC_CR_TSEL1;       
    DAC->CR |= DAC_CR_TEN1;  
    DAC->CR |= (4 << DAC_CR_TSEL1_Pos);             
    DAC->CR |= DAC_CR_EN1;
}

void init_TIM2()
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; // Enable Timer 3 clock
    TIM2->PSC = 4800 - 1;//96-1;  //samples at 5000hz * 2 so 10hz since 10k hz
    TIM2->ARR = 100 - 1;//100-1;
    TIM2->DIER |= TIM_DIER_UIE;
    TIM2->CR1 |= TIM_CR1_CEN;
    NVIC->ISER[0] = 1 << TIM2_IRQn;
}

void TIM2_IRQHandler(void) 
{
    TIM2->SR &= ~TIM_SR_UIF;
    uint16_t sat = saturator(1,20,1,ADC3->DR);
    DAC->DHR12R1 = 4000;
}