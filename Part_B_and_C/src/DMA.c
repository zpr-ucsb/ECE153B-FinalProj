/*
 * ECE 153B - Spring 2023
 *
 * Name(s):
 * Section:
 * Lab: 6C
 */
 
#include "DMA.h"
#include "SysTimer.h"
#include "UART.h"

void DMA_Init_UARTx(DMA_Channel_TypeDef * tx_channel, USART_TypeDef * uart) {
	//TODO
  RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN; // Enable the clock for DMA
  for (volatile uint32_t i = 0; i < 1000; ++i); // Wait for DMA setup (20us delay, adjust as necessary)
	
  tx_channel->CCR &= ~DMA_CCR_EN; // Disable DMA channel
  tx_channel->CCR &= ~DMA_CCR_MEM2MEM; // Disable Memory-to-Memory mode
	
  tx_channel->CCR &= ~DMA_CCR_PSIZE; // Set peripheral and memory data size to 8 bits
  tx_channel->CCR &= ~DMA_CCR_MSIZE;
  tx_channel->CCR &= ~DMA_CCR_PINC; // Disable peripheral increment mode
	 
  tx_channel->CCR |= DMA_CCR_MINC;  // Enable memory increment mode
	
  tx_channel->CCR |= DMA_CCR_DIR; // Configure the transfer direction
  tx_channel->CCR &= ~DMA_CCR_PL; // Set channel priority to high
  tx_channel->CCR &= ~DMA_CCR_CIRC; // Disable circular mode
	
  tx_channel->CCR &= ~DMA_CCR_HTIE; // Disable half transfer interrupt
  tx_channel->CCR &= ~DMA_CCR_TEIE; // Disable transfer error interrupt
  tx_channel->CCR |= DMA_CCR_TCIE; // Enable transfer complete interrupt
	
  tx_channel->CPAR = (uint32_t)&(uart->TDR); // Set the data destination to the data register of the data
	
  NVIC_ClearPendingIRQ(DMA1_Channel7_IRQn); // Enable DMA1_Channel7_IRQn interrupt
  NVIC_SetPriority(DMA1_Channel7_IRQn, 0);
  NVIC_EnableIRQ(DMA1_Channel7_IRQn);
	
  NVIC_ClearPendingIRQ(DMA1_Channel5_IRQn); // Enable DMA1_Channel5_IRQn interrupt
  NVIC_SetPriority(DMA1_Channel5_IRQn, 0);
  NVIC_EnableIRQ(DMA1_Channel5_IRQn);
}

void DMA1_Channel7_IRQHandler(void)
{
    if (DMA1->ISR & DMA_ISR_TCIF7) {
        DMA1->IFCR = DMA_IFCR_CTCIF7; // Clear transfer complete flag
        on_complete_transfer();
    }
		
		//Disable the DMA
		DMA1_Channel7->CCR &= ~DMA_CCR_EN;
		
    DMA1->IFCR |= DMA_IFCR_CGIF7; // Clear global DMA interrupt flag
}	


void DMA1_Channel5_IRQHandler(void)
{
	
}
