/*
 * main.c
 *
 *  Created on: Oct 1, 2025
 *      Author: thalya.morice-roy-le
 */

#include "main.h"
#include "stm32f0xx_it.h"
#include "stm32f0xx.h"
#include "bsp.h"
#include "servo.h"

#define ID_SERVO4 0x04
#define ID_SERVO5 0x05
#define ID_SERVOFD 0xFD //253
#define ID_SERVO03 0x03

// Static functions
static void SystemClock_Config(void);

#include "main.h"
#include "stm32f0xx.h"
#include "bsp.h"
#include "servo.h"

#define ID_SERVO4 0x04
#define ID_SERVOFD 0xFD

static void SystemClock_Config();
// Buffer DMA
uint16_t adc_dma_buffer[2];

int main(void)
{
    uint32_t position1, position2;
    volatile uint32_t delay;

    // Initialisations
    SystemClock_Config();
    servo_uart_init();
    BSP_Console_Init();
    BSP_ADC_DMA_Init(); 

    // Allumage initial
    herkulex_torque_on(ID_SERVO4);
    herkulex_torque_on(ID_SERVOFD);

    // LEDs initiales (Juste pour le style au démarrage)
    herkulex_led_green(ID_SERVO4);
    herkulex_led_cyan(ID_SERVOFD);

    my_printf("Systeme pret.\r\n");

    while (1)
    {
        position1 = (adc_dma_buffer[0] * 1023) / 4095;
        position2 = (adc_dma_buffer[1] * 1023) / 4095;

        //Debug Console
        my_printf("--- Cycle ---\r\n");
        my_printf("ADC: [%d, %d] -> POS: [%d, %d]\r\n",
                  adc_dma_buffer[0], adc_dma_buffer[1],
                  position1, position2);

        //Commande Servos
        herkulex_set_position(ID_SERVO4, position1, 60);
        herkulex_set_position(ID_SERVOFD, position2, 60);
		
        for (delay = 0; delay < 500000; delay++);
    }
}

// int main(void)
// {
// 	uint16_t adc_values[2];
// 	uint32_t position1, position2;
// 	volatile uint32_t delay;

// 	// Setup clocks
// 	SystemClock_Config();

// 	// UART / Console
// 	servo_uart_init();
// 	BSP_Console_Init();
// 	my_printf("Console ready!\r\n");

// 	// ADC avec 2 canaux
// 	BSP_ADC_Init();
// 	my_printf("ADC ready (2 channels)!\r\n");

// 	// __Servos__ ON
// 	herkulex_torque_on(ID_SERVO4);
// 	herkulex_torque_on(ID_SERVOFD);

// 	herkulex_led_green(ID_SERVO4);
// 	herkulex_led_cyan(ID_SERVOFD);

// 	while (1)
// 	{
// 		/* ===== Lecture Potentiomètre 1 (Canal 10) ===== */
// 		ADC1->CHSELR = ADC_CHSELR_CHSEL10;       // Sélectionner SEULEMENT canal 10
// 		ADC1->CR |= ADC_CR_ADSTART;              // Lancer la conversion
// 		while (!(ADC1->ISR & ADC_ISR_EOC));      // Attendre la fin
// 		adc_values[0] = ADC1->DR;                // Lire le résultat

// 		/* ===== Lecture Potentiomètre 2 (Canal 11) ===== */
// 		ADC1->CHSELR = ADC_CHSELR_CHSEL11;       // Sélectionner SEULEMENT canal 11
// 		ADC1->CR |= ADC_CR_ADSTART;              // Lancer la conversion
// 		while (!(ADC1->ISR & ADC_ISR_EOC));      // Attendre la fin
// 		adc_values[1] = ADC1->DR;                // Lire le résultat

// 		/* ===== Commande des Servos ===== */
// 		// Servo 1 (ID 4)
// 		position1 = (adc_values[0] * 1023) / 4095;
// 		// Filtrage simple : si la valeur bouge peu, on n'envoie pas pour éviter de surcharger le bus
// 		herkulex_set_position(ID_SERVO4, position1, 60); // 60 = temps de déplacement (PlayTime)

// 		// Servo 2 (ID 253/FD)
// 		position2 = (adc_values[1] * 1023) / 4095;
// 		herkulex_set_position(ID_SERVOFD, position2, 60);

// 		// Debug
// 		my_printf("Servo4 -> %d | ServoFD -> %d\r\n", adc_values[0], adc_values[1]);

// 		// Petit délai pour ne pas saturer le port série et les servos
// 		for (delay = 0; delay < 100000; delay++);
// 	}
// }

//int main(void)
//{
//	uint32_t adc_value;
//	uint32_t position;
//	volatile uint32_t delay;
//
//	// Setup clocks
//	SystemClock_Config();
//
//	// UART / Console
//	servo_uart_init();
//	BSP_Console_Init();
//	my_printf("Console ready!\r\n");
//
//	// ADC
//	BSP_ADC_Init_1();
//	my_printf("ADC ready!\r\n");
//
//	// Servos ON
//	herkulex_torque_on(ID_SERVO4);
//	herkulex_torque_on(ID_SERVOFD);
//
//	herkulex_led_green(ID_SERVO4);
//	herkulex_led_cyan(ID_SERVOFD);
//
//	while (1)
//	{
//		/* ===== Servo 1 ===== */
//		ADC1->CR |= ADC_CR_ADSTART;                 // Start ADC
//		while (!(ADC1->ISR & ADC_ISR_EOC));         // Wait EOC
//		adc_value = ADC1->DR;                       // Clear EOC
//
//		position = (adc_value * 1023) / 4095;
//		herkulex_set_position(ID_SERVO4, position, 250);
//
//		my_printf("ADC moteur 1 = %d\t", position);
//
//		for (delay = 0; delay < 500000; delay++);
//
//		/* ===== Servo 2 ===== */
//		//ADC1->CR |= ADC_CR_ADSTART;
//		while (!(ADC1->ISR & ADC_ISR_EOC));
//		adc_value = ADC1->DR;
//
//		position = (adc_value * 1023) / 4095;
//		herkulex_set_position(ID_SERVOFD, position, 250);
//
//		my_printf("ADC moteur 2 = %d\r\n", position);
//
//		for (delay = 0; delay < 500000; delay++);
//	}
//}


//int main(void)
//{
//	uint32_t i,j;
//	// Setup clocks
//	SystemClock_Config();
//
//	// Initialize UART
//	servo_uart_init();
//	//BSP_Console_Init();
//
//	// Initialize Debug Console
//	BSP_Console_Init();
//	my_printf("Console ready!\r\n");
//
//	// Initialize and start ADC on PC1
//	BSP_ADC_Init_1();
//	my_printf("ADC ready!\r\n");
//	herkulex_torque_on(ID_SERVO4);//active le torque
//	herkulex_torque_on(ID_SERVOFD);//active le torque
//
//	herkulex_led_green(ID_SERVO4); //allume la led vert
//	herkulex_led_cyan(ID_SERVOFD); //allume la led
//
//	// Main loop
//	while(1)
//	{
//		// Wait here until ADC EOC
//		while ((ADC1->ISR & ADC_ISR_EOC) != ADC_ISR_EOC);
//
//		// Report result to console
//		uint32_t valeur_normaliser_1=(ADC1->DR)*1024/4035;
//		//my_printf("ADC value moteur 1 = %d\r\n", valeur_normaliser_1);
//		herkulex_set_position(ID_SERVO4, valeur_normaliser_1, 250); //durée = playtime × 11,2 ms
//
//		my_printf("ADC value moteur 1 = %d\t",valeur_normaliser_1);
//
//		// Wait about 200ms
//		for (i=0; i<500000; i++);
//
//		// Wait here until ADC EOC
//		while ((ADC1->ISR & ADC_ISR_EOC) != ADC_ISR_EOC);
//
//		// Report result to console
//
//		uint32_t valeur_normaliser_2=(ADC1->DR)*1024/4035;
//
//		herkulex_set_position(ID_SERVOFD, valeur_normaliser_2, 250); //durée = playtime × 11,2 ms
//		my_printf("ADC value moteur 2 = %d\r\n", valeur_normaliser_2);
//
//
//		// Wait about 200ms
//		for (j=0; j<500000; j++);
//	}
//
//	//Servo 1
//
//
//
//
//	//Servo 2
//
//	//herkulex_set_position(ID_SERVO5, 100, 250);
//
//
//	//Servo 3
//	herkulex_torque_on(ID_SERVOFD);//active le torque
//	herkulex_set_position(ID_SERVOFD, 10, 250);
//
//	herkulex_led_red(ID_SERVOFD); //allume la led rouge
//
//	//Servo 3
//	herkulex_torque_on(ID_SERVO03);//active le torque
//	herkulex_set_position(ID_SERVO03, 10, 250);
//	herkulex_led_cyan(ID_SERVO03); //allume la led rouge
//	herkulex_ram_read(ID_SERVO03, 0x34, 1);
//	my_printf("Console ready!\r\n");
//
//	// Main loop
//	//while(1);
//}



static void SystemClock_Config()
{
	uint32_t	HSE_Status;
	uint32_t	PLL_Status;
	uint32_t	SW_Status;
	uint32_t	timeout = 0;
	timeout = 1000000;

	// Start HSE in Bypass Mode
	RCC->CR |= RCC_CR_HSEBYP;
	RCC->CR |= RCC_CR_HSEON;

	// Wait until HSE is ready
	do
	{
		HSE_Status = RCC->CR & RCC_CR_HSERDY_Msk;
		timeout--;
	} while ((HSE_Status == 0) && (timeout > 0));

	// Select HSE as PLL input source
	RCC->CFGR &= ~RCC_CFGR_PLLSRC_Msk;
	RCC->CFGR |= (0x02 <<RCC_CFGR_PLLSRC_Pos);

	// Set PLL PREDIV to /1
	RCC->CFGR2 = 0x00000000;

	// Set PLL MUL to x6
	RCC->CFGR &= ~RCC_CFGR_PLLMUL_Msk;
	RCC->CFGR |= (0x04 <<RCC_CFGR_PLLMUL_Pos);

	// Enable the main PLL
	RCC-> CR |= RCC_CR_PLLON;
	// Wait until PLL is ready
	do
	{
		PLL_Status = RCC->CR & RCC_CR_PLLRDY_Msk;
		timeout--;
	} while ((PLL_Status == 0) && (timeout > 0));

	// Set AHB prescaler to /1
	RCC->CFGR &= ~RCC_CFGR_HPRE_Msk;
	RCC->CFGR |= RCC_CFGR_HPRE_DIV1;

	//Set APB1 prescaler to /1
	RCC->CFGR &= ~RCC_CFGR_PPRE_Msk;
	RCC->CFGR |= RCC_CFGR_PPRE_DIV1;

	// Enable FLASH Prefetch Buffer and set Flash Latency
	FLASH->ACR = FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY;

	// Select the main PLL as system clock source
	RCC->CFGR &= ~RCC_CFGR_SW;
	RCC->CFGR |= RCC_CFGR_SW_PLL;

	// Wait until PLL becomes main switch input
	do
	{
		SW_Status = (RCC->CFGR & RCC_CFGR_SWS_Msk);
		timeout--;
	} while ((SW_Status != RCC_CFGR_SWS_PLL) && (timeout > 0));

	// Update SystemCoreClock global variable
	SystemCoreClockUpdate();
}



