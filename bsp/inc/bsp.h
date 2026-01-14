/*
 * bsp.h
 *
 *  Created on: Oct 8, 2025
 *      Author: thalya.morice-roy-le
 */

#ifndef BSP_INC_BSP_H_
#define BSP_INC_BSP_H_

#include "stm32f0xx.h"

/*
 * LED driver functions
 */

void	BSP_LED_Init	(void);
void	BSP_LED_On	(void);
void	BSP_LED_Off	(void);
void	BSP_LED_Toggle	(void);

void       BSP_PB_Init		(void);
uint8_t    BSP_PB_GetState	(void);
void BSP_Console_Init();
void BSP_ADC_Init();



#endif /* BSP_INC_BSP_H_ */
