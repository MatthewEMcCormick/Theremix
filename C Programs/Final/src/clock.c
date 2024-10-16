#include "stm32f4xx.h"

//Currently using HSI at 48 Mhz
void internal_clock()
{
    /* Disable HSE to allow use of the GPIOs */
    RCC->CR &= ~RCC_CR_HSEON;
    
    /* Enable Prefetch Buffer, set Flash Latency */
    FLASH->ACR = FLASH_ACR_PRFTEN | FLASH_ACR_LATENCY_1WS;  // STM32F4 needs FLASH_ACR_PRFTEN

    /* HCLK = SYSCLK */
    RCC->CFGR |= (uint32_t)RCC_CFGR_HPRE_DIV1;

    /* PCLK1 = HCLK */
    RCC->CFGR |= (uint32_t)RCC_CFGR_PPRE1_DIV2;  // APB1 (PCLK1) max is 42MHz, so divide by 2

    /* PCLK2 = HCLK */
    RCC->CFGR |= (uint32_t)RCC_CFGR_PPRE2_DIV1;  // APB2 (PCLK2) can run at full speed

    /* PLL configuration: PLLP = 2, PLLQ = 4, PLLN = 96, PLLM = 16 */
    RCC->PLLCFGR = (16 << RCC_PLLCFGR_PLLM_Pos)  // PLLM = 16 (input divider)
                 | (96 << RCC_PLLCFGR_PLLN_Pos)  // PLLN = 96 (multiplier)
                 | (0 << RCC_PLLCFGR_PLLP_Pos)   // PLLP = 2 (output divider)
                 | (4 << RCC_PLLCFGR_PLLQ_Pos)   // PLLQ = 4 (USB OTG FS requires 48 MHz)
                 | RCC_PLLCFGR_PLLSRC_HSI;       // Use HSI as PLL source

    /* Enable PLL */
    RCC->CR |= RCC_CR_PLLON;

    /* Wait till PLL is ready */
    while((RCC->CR & RCC_CR_PLLRDY) == 0);

    /* Select PLL as system clock source */
    RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));
    RCC->CFGR |= (uint32_t)RCC_CFGR_SW_PLL;

    /* Wait till PLL is used as system clock source */
    while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS) != (uint32_t)RCC_CFGR_SWS_PLL);
}

//This would work at 180 Mhz Clock
/*
#include "stm32f4xx.h"

void internal_clock()
{
    //Disable HSE to allow use of the GPIOs
    RCC->CR &= ~RCC_CR_HSEON;
    
    //Enable Prefetch Buffer and set Flash Latency for 180 MHz
    FLASH->ACR = FLASH_ACR_PRFTEN | FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_LATENCY_5WS;

    //HCLK = SYSCLK
    RCC->CFGR |= (uint32_t)RCC_CFGR_HPRE_DIV1;

    /* PCLK1 = HCLK / 4 (max 45 MHz)
    RCC->CFGR |= (uint32_t)RCC_CFGR_PPRE1_DIV4;

    /* PCLK2 = HCLK / 2 (max 90 MHz)
    RCC->CFGR |= (uint32_t)RCC_CFGR_PPRE2_DIV2;

    //  PLL configuration:
    //    - PLLM = 16 (divides HSI 16 MHz to 1 MHz)
    //    - PLLN = 360 (multiplies 1 MHz to 360 MHz)
    //    - PLLP = 2 (divides 360 MHz by 2 to get 180 MHz)
    //    - PLLQ = 7 (for USB, SDIO, etc.)
    
    RCC->PLLCFGR = (16 << RCC_PLLCFGR_PLLM_Pos)  // PLLM = 16 (divides HSI 16 MHz)
                 | (360 << RCC_PLLCFGR_PLLN_Pos) // PLLN = 360 (multiplies to 360 MHz)
                 | (0 << RCC_PLLCFGR_PLLP_Pos)   // PLLP = 2 (divides to get 180 MHz)
                 | (7 << RCC_PLLCFGR_PLLQ_Pos)   // PLLQ = 7 (for USB, SDIO, etc.)
                 | RCC_PLLCFGR_PLLSRC_HSI;       // Use HSI as PLL source

    Enable PLL
    RCC->CR |= RCC_CR_PLLON;

    while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS) != (uint32_t)RCC_CFGR_SWS_PLL);

*/