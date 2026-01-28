/*
 * bsp.c
 *
 *  Created on: Oct 8, 2025
 *      Author: thalya.morice-roy-le
 */

#include "bsp.h"

/*
 * BSP_LED_Init()
 * Initialize LED pin (PA5) as a High-Speed Push-Pull output
 * Set LED initial state to OFF
 */

void BSP_LED_Init()
{
	// Enable GPIOA clock
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

	// Configure PA5 as output
	GPIOA->MODER &= ~GPIO_MODER_MODER5_Msk;
	GPIOA->MODER |= (0x01 <<GPIO_MODER_MODER5_Pos);

	// Configure PA5 as Push-Pull output
	GPIOA->OTYPER &= ~GPIO_OTYPER_OT_5;

	// Configure PA5 as High-Speed Output
	GPIOA->OSPEEDR &= ~GPIO_OSPEEDR_OSPEEDR5_Msk;
	GPIOA->OSPEEDR |= (0x03 <<GPIO_OSPEEDR_OSPEEDR5_Pos);

	// Disable PA5 Pull-up/Pull-down
	GPIOA->PUPDR &= ~GPIO_PUPDR_PUPDR5_Msk;

	// Set Initial State OFF
	GPIOA->BSRR |= GPIO_BSRR_BR_5;
}

/*
 * BSP_LED_On()
 * Turn ON LED on PA5
 */

void BSP_LED_On()
{
	GPIOA->BSRR = GPIO_BSRR_BS_5;
}

/*
 * BSP_LED_Off()
 * Turn OFF LED on PA5
 */

void BSP_LED_Off()
{
	GPIOA->BSRR = GPIO_BSRR_BR_5;
}

/*
 * BSP_LED_Toggle()
 * Toggle LED on PA5
 */

void BSP_LED_Toggle()
{
	GPIOA->ODR ^= GPIO_ODR_5;
}

/*
 * BSP_PB_Init()
 * Initialize Push-Button pin (PC13) as input without Pull-up/Pull-down
 */

void BSP_PB_Init()
{
	// Enable GPIOC clock
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;

	// Configure PC13 as input
	GPIOC->MODER &= ~GPIO_MODER_MODER13_Msk;
	GPIOC->MODER |= (0x00 <<GPIO_MODER_MODER13_Pos);

	// Disable PC13 Pull-up/Pull-down
	GPIOC->PUPDR &= ~GPIO_PUPDR_PUPDR13_Msk;
}

/*
 * BSP_PB_GetState()
 * Returns the state of the button (0=released, 1=pressed)
 */

uint8_t BSP_PB_GetState()
{
	uint8_t state;
	if ((GPIOC->IDR & GPIO_IDR_13) == GPIO_IDR_13)
	{
		state = 0;
	}
	else
	{
		state = 1;
	}
	return state;
}


//void BSP_Console_Init()
//{
//	// Enable GPIOA clock
//	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
//
//	// Configure PA2 and PA3 as Alternate function
//	GPIOA->MODER &= ~(GPIO_MODER_MODER9_Msk | GPIO_MODER_MODER10_Msk);
//	GPIOA->MODER |= (0x02 <<GPIO_MODER_MODER9_Pos) | (0x02 <<GPIO_MODER_MODER10_Pos);
//
//	// Set PA2 and PA3 to AF1 (USART2)
//	GPIOA->AFR[1] &= ~(0x00000FF0);
//	GPIOA->AFR[1] |=  (0x00000110);
//
//	// Enable USART2 clock
//	RCC -> APB1ENR |= RCC_APB1ENR_USART1EN;
//
//	// Clear USART2 configuration (reset state)
//	// 8-bit, 1 start, 1 stop, CTS/RTS disabled
//	USART1->CR1 = 0x00000000;
//	USART1->CR2 = 0x00000000;
//	USART1->CR3 = 0x00000000;
//
//	// Select PCLK (APB1) as clock source
//	// PCLK -> 48 MHz
//	RCC->CFGR3 &= ~RCC_CFGR3_USART1SW_Msk;
//
//	// Baud Rate = 115200
//	// With OVER8=0 and Fck=48MHz, USARTDIV =   48E6/115200 = 416.6666
//	// BRR = 417 -> Baud Rate = 115107.9137 -> 0.08% error
//	//
//	// With OVER8=1 and Fck=48MHz, USARTDIV = 2*48E6/115200 = 833.3333
//	// BRR = 833 -> Baud Rate = 115246.0984 -> 0.04% error (better)
//	USART1->CR1 |= USART_CR1_OVER8;
//	USART1->BRR = 833;
//
//	// Enable both Transmitter and Receiver
//	USART1->CR1 |= USART_CR1_TE | USART_CR1_RE;
//
//	// Enable USART2
//	USART1->CR1 |= USART_CR1_UE;
//}


void BSP_Console_Init()
{
	// Enable GPIOA clock
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

	// Configure PA2 and PA3 as Alternate function
	GPIOA->MODER &= ~(GPIO_MODER_MODER2_Msk | GPIO_MODER_MODER3_Msk);
	GPIOA->MODER |= (0x02 <<GPIO_MODER_MODER2_Pos) | (0x02 <<GPIO_MODER_MODER3_Pos);

	// Set PA2 and PA3 to AF1 (USART2)
	GPIOA->AFR[0] &= ~(0x0000FF00);
	GPIOA->AFR[0] |=  (0x00001100);

	// Enable USART2 clock
	RCC -> APB1ENR |= RCC_APB1ENR_USART2EN;

	// Clear USART2 configuration (reset state)
	// 8-bit, 1 start, 1 stop, CTS/RTS disabled
	USART2->CR1 = 0x00000000;
	USART2->CR2 = 0x00000000;
	USART2->CR3 = 0x00000000;

	// Select PCLK (APB1) as clock source
	// PCLK -> 48 MHz
	RCC->CFGR3 &= ~RCC_CFGR3_USART2SW_Msk;

	// Baud Rate = 115200
	// With OVER8=0 and Fck=48MHz, USARTDIV =   48E6/115200 = 416.6666
	// BRR = 417 -> Baud Rate = 115107.9137 -> 0.08% error
	//
	// With OVER8=1 and Fck=48MHz, USARTDIV = 2*48E6/115200 = 833.3333
	// BRR = 833 -> Baud Rate = 115246.0984 -> 0.04% error (better)
	USART2->CR1 |= USART_CR1_OVER8;
	USART2->BRR = 833;

	// Enable both Transmitter and Receiver
	USART2->CR1 |= USART_CR1_TE | USART_CR1_RE;

	// Enable USART2
	USART2->CR1 |= USART_CR1_UE;
}

extern uint16_t adc_dma_buffer[];
void BSP_ADC_DMA_Init() {
	// Enable GPIOC clock
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	// Configure pin PC0 and PC1 as analog
	GPIOC->MODER &= ~(GPIO_MODER_MODER0_Msk | GPIO_MODER_MODER1_Msk);
	GPIOC->MODER |= (0x03 << GPIO_MODER_MODER0_Pos)
			| (0x03 << GPIO_MODER_MODER1_Pos);
	// Enable ADC clock
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
	// Reset ADC configuration
	ADC1->CR = 0x00000000;
	ADC1->CFGR1 = 0x00000000;
	ADC1->CFGR2 = 0x00000000;
	ADC1->CHSELR = 0x00000000;
	// Enable continuous conversion mode
	ADC1->CFGR1 |= ADC_CFGR1_CONT;
	// 12-bit resolution
	ADC1->CFGR1 |= (0x00 << ADC_CFGR1_RES_Pos);
	// Select PCLK/2 as ADC clock
	ADC1->CFGR2 |= (0x01 << ADC_CFGR2_CKMODE_Pos);
	// Set sampling time to 239.5 ADC clock cycles environ 10 mico secondes
	ADC1->SMPR = 0x07;
	// Select channel 10 and 11 (Correspondant à PC0 et PC1)
	ADC1->CHSELR |= ADC_CHSELR_CHSEL10 | ADC_CHSELR_CHSEL11;
	// Start DMA clock
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;
	// Reset DMA1 Channel 1 configuration
	DMA1_Channel1->CCR = 0x00000000;
	// Configure DMA1 Channel 1
	// Peripheral -> Memory
	// Peripheral is 16-bit, no increment
	// Memory is 16-bit, increment
	// Circular mode
	DMA1_Channel1->CCR |= (0x01 << DMA_CCR_PSIZE_Pos)
			| (0x01 << DMA_CCR_MSIZE_Pos) | DMA_CCR_MINC | DMA_CCR_CIRC;
	// Peripheral is ADC1 DR
	DMA1_Channel1->CPAR = (uint32_t) &ADC1->DR;
	// Memory is adc_dma_buffer
	DMA1_Channel1->CMAR = (uint32_t) adc_dma_buffer;
	// Set Memory Buffer size
	DMA1_Channel1->CNDTR = 2;
	// Enable DMA1 Channel 1
	DMA1_Channel1->CCR |= DMA_CCR_EN;
	// Enable ADC DMA Request in circular mode
	ADC1->CFGR1 |= ADC_CFGR1_DMACFG | ADC_CFGR1_DMAEN;
	// Enable ADC
	ADC1->CR |= ADC_CR_ADEN;
	// Start conversion
	ADC1->CR |= ADC_CR_ADSTART;
}


//void BSP_ADC_Init_1()
//{
//	// Enable GPIOC clock
//	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
//
//	// Configure pin PC1 as analog
//	GPIOC->MODER &= ~GPIO_MODER_MODER1_Msk;
//	GPIOC->MODER &= ~GPIO_MODER_MODER0_Msk;
//	GPIOC->MODER |= (0x03 <<GPIO_MODER_MODER1_Pos) | (0x03 <<GPIO_MODER_MODER0_Pos);
//
//	// Enable ADC clock
//	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
//
//	// Reset ADC configuration
//	ADC1->CR 	= 0x00000000;
//	ADC1->CFGR1  = 0x00000000;
//	ADC1->CFGR2  = 0x00000000;
//	ADC1->CHSELR = 0x00000000;
//
//	// Enable continuous conversion mode
//	ADC1->CFGR1 |= ADC_CFGR1_CONT;
//
//	// 12-bit resolution
//	ADC1->CFGR1 |= (0x00 <<ADC_CFGR1_RES_Pos);
//
//	// Select PCLK/2 as ADC clock
//	ADC1->CFGR2 |= (0x01 <<ADC_CFGR2_CKMODE_Pos);
//
//	// Set sampling time to 28.5 ADC clock cycles
//	ADC1->SMPR = 0x03;
//
//	// Select channel 11
//	ADC1->CHSELR |= ADC_CHSELR_CHSEL11;
//	ADC1->CHSELR |= ADC_CHSELR_CHSEL10;
//
//	// Enable ADC
//	ADC1->CR |= ADC_CR_ADEN;
//
//	// Start conversion
//	ADC1->CR |= ADC_CR_ADSTART;
//}



