#include "stm32f4xx.h"
#include <stdint.h>
#include "IO_DSP.h"
#include <math.h>
#include "DSP.h"
#include "global.h"
void init_DSP()
{
    init_GPIO();
    init_ADC();
    init_TIM5();
    init_DAC();
    init_TIM2();
    initLookUpTan();
    SCB->CPACR |= (0xF << 20);
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
    //ADC3->CR1 |= (0x00 << ADC_CR1_RES_Pos);
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
}

void TIM2_IRQHandler(void) 
{
    if (TIM2->SR & TIM_SR_UIF) {
    TIM2->SR &= ~TIM_SR_UIF;
    //uint16_t sat = saturator(1,20,1,ADC3->DR);
    //if (ADC3->SR & ADC_SR_EOC)
    //{
    if (ADC3->SR & ADC_SR_EOC) {
        if(ADC3->DR < 0)
        {
            GPIOB->ODR ^= (1 << 7); 
        }
        //GPIOB->ODR ^= (1 << 7); 
        //saturator(1,100,5,ADC3->DR);
        
        DAC->DHR12R2 = (int)(lookUp[0]*4000.0f);//*4000.0 + .5);//(int)(lookUp[0]*4000.0 + .5);//lookUp[295]* 4095;//saturator(1,100,5,ADC3->DR); //ADC3->DR;
    }
    }
}