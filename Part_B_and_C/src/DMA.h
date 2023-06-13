/*
 * ECE 153B - Spring 2023
 *
 * Name(s):
 * Section:
 * Lab: 6C
 */

#ifndef __STM32L476G_DISCOVERY_DMA_H
#define __STM32L476G_DISCOVERY_DMA_H

#include "stm32l476xx.h"

void DMA_Init_UARTx(DMA_Channel_TypeDef * tx, USART_TypeDef * uart);

void DMA1_Channel7_IRQHandler(void);
void DMA1_Channel5_IRQHandler(void);

#endif /* __STM32L476G_DISCOVERY_DMA_H */
