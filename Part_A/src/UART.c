#include "UART.h"

void UART1_Init(void) {

	// (a) Enable the USART1 clock in the peripheral clock register.
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

	// (b) Select the system clock as the USART2 clock source in the peripheral independent
	//     clock configuration register.
	RCC->CCIPR &= ~(RCC_CCIPR_USART1SEL);	// 01: SYSCLCK Selected 
	RCC->CCIPR |= RCC_CCIPR_USART1SEL_0;
}

void UART2_Init(void) {
	// (a) Enable the USART2 clock in the peripheral clock register.
	RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN;

	// (b) Select the system clock as the USART2 clock source in the peripheral independent
	//     clock configuration register.
	RCC->CCIPR &= ~(RCC_CCIPR_USART2SEL);	// 01: SYSCLCK Selected 
	RCC->CCIPR |= RCC_CCIPR_USART2SEL_0;
}

void UART1_GPIO_Init(void) {
		// [TODO]
		RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;		// Enable GPIOB CLK

		// Configure PB6 and PB7 to operate as UART transmitters and receivers.
		GPIOB->MODER &= ~(GPIO_MODER_MODE6);        // Clear Port B, Pin 6 MODER
			GPIOB->MODER |= GPIO_MODER_MODE6_1;         // Set Port B, Pin 6 MODER to Alternative Mode
		GPIOB->AFR[0] &= ~(GPIO_AFRL_AFSEL6);		// Clear AF for Port B, Pin 6
		GPIOB->AFR[0] |= GPIO_AFRL_AFSEL6_0;		// Choose AF7 (USART1_TX) for Port B, Pin 6
		GPIOB->AFR[0] |= GPIO_AFRL_AFSEL6_1;
		GPIOB->AFR[0] |= GPIO_AFRL_AFSEL6_2;

		GPIOB->MODER &= ~(GPIO_MODER_MODE7);        // Clear Port B, Pin 7 MODER
			GPIOB->MODER |= GPIO_MODER_MODE7_1;         // Set Port B, Pin 7 MODER to Alternative Mode
		GPIOB->AFR[0] &= ~(GPIO_AFRL_AFSEL7);		// Clear AF for Port B, Pin 7
		GPIOB->AFR[0] |= GPIO_AFRL_AFSEL7_0;		// Choose AF7 (USART1_RX) for Port B, Pin 7
		GPIOB->AFR[0] |= GPIO_AFRL_AFSEL7_1;
		GPIOB->AFR[0] |= GPIO_AFRL_AFSEL7_2;

		// (a) Both GPIO pins should operate at very high speed.
		GPIOB->OSPEEDR |= (GPIO_OSPEEDR_OSPEED6); 	// 11 = very high speed
		GPIOB->OSPEEDR |= (GPIO_OSPEEDR_OSPEED7);

		// (b) Both GPIO pins should have a push-pull output type.
		GPIOB->OTYPER &= ~(GPIO_OTYPER_OT6); 	// 0 = push-pull, 1 = opendrain
		GPIOB->OTYPER &= ~(GPIO_OTYPER_OT7);


		// (c) Configure both GPIO pins to use pull-up resistors for I/O.
		GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPD6);	// 01 = pull-up
		GPIOB->PUPDR |= GPIO_PUPDR_PUPD6_0;
		GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPD7);
		GPIOB->PUPDR |= GPIO_PUPDR_PUPD7_0;
}

void UART2_GPIO_Init(void) {
	// [TODO]
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN; 		// Enable GPIOA CLK

	// Configure PA2 and PA3 to operate as UART transmitters and receivers.
	GPIOA->MODER &= ~(GPIO_MODER_MODE2);        // Clear Port A, Pin 2 MODER
  	GPIOA->MODER |= GPIO_MODER_MODE2_1;         // Set Port A, Pin 2 MODER to Alternative Mode
	GPIOA->AFR[0] &= ~(GPIO_AFRL_AFSEL2);		// Clear AF for Port A, Pin 2
	GPIOA->AFR[0] |= GPIO_AFRL_AFSEL2_0;		// Choose AF7 (USART2_TX) for Port A, Pin 2
	GPIOA->AFR[0] |= GPIO_AFRL_AFSEL2_1;
	GPIOA->AFR[0] |= GPIO_AFRL_AFSEL2_2;

	GPIOA->MODER &= ~(GPIO_MODER_MODE3);        // Clear Port A, Pin 3 MODER
  	GPIOA->MODER |= GPIO_MODER_MODE3_1;         // Set Port A, Pin 3 MODER to Alternative Mode
	GPIOA->AFR[0] &= ~(GPIO_AFRL_AFSEL3);		// Clear AF for Port A, Pin 3
	GPIOA->AFR[0] |= GPIO_AFRL_AFSEL3_0;		// Choose AF7 (USART2_RX) for Port A, Pin 3
	GPIOA->AFR[0] |= GPIO_AFRL_AFSEL3_1;
	GPIOA->AFR[0] |= GPIO_AFRL_AFSEL3_2;

	// (a) Both GPIO pins should operate at very high speed.
	GPIOA->OSPEEDR |= (GPIO_OSPEEDR_OSPEED2); 	// 11 = very high speed
	GPIOA->OSPEEDR |= (GPIO_OSPEEDR_OSPEED3);

	// (b) Both GPIO pins should have a push-pull output type.
	GPIOA->OTYPER &= ~(GPIO_OTYPER_OT2); 	// 0 = push-pull, 1 = opendrain
	GPIOA->OTYPER &= ~(GPIO_OTYPER_OT3);


	// (c) Configure both GPIO pins to use pull-up resistors for I/O.
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD2);	// 01 = pull-up
	GPIOA->PUPDR |= GPIO_PUPDR_PUPD2_0;
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD3);
	GPIOA->PUPDR |= GPIO_PUPDR_PUPD3_0;
}

void USART_Init(USART_TypeDef* USARTx) {
	// [TODO]
	// Note that USART must be disabled to modify the settings ensure that USART is
	// disabled before you start to modify the registers. 
	USARTx->CR1 &= ~(USART_CR1_UE);

	// 	(a) In the control registers, set word length to 8 bits, oversampling mode to oversample
	// 	    by 16, and number of stop bits to 1.
	USARTx->CR1 &= USART_CR1_M1;		// 00 = 8 bits
	USARTx->CR1 &= USART_CR1_M0;
	
	USARTx->CR1 &= ~(USART_CR1_OVER8); 	// 0 = oversampling by 16
	USARTx->CR2 &= ~(USART_CR2_STOP);  // 00 = 1 stop bit
	
	// 	(b) Set the baud rate to 9600. To generate the baud rate, you will have to write a value
	// 	    into USARTx BRR.
	USARTx->BRR = 8333; // To obtain 9600 baud with f_CK = 80 MHz. In case of oversampling by 16: USARTDIV = 80000000/9600 = 8333

	// (c) In the control registers, enable both the transmitter and receiver.
	USARTx->CR1 |= USART_CR1_TE;
	USARTx->CR1 |= USART_CR1_RE;

	// (d) Now that everything has been set up, enable USART in the control registers
	USARTx->CR1 |= USART_CR1_UE;
}

uint8_t USART_Read (USART_TypeDef * USARTx) {
	// SR_RXNE (Read data register not empty) bit is set by hardware
	while (!(USARTx->ISR & USART_ISR_RXNE));  // Wait until RXNE (RX not empty) bit is set
	// USART resets the RXNE flag automatically after reading DR
	return ((uint8_t)(USARTx->RDR & 0xFF));
	// Reading USART_DR automatically clears the RXNE flag 
}

void USART_Write(USART_TypeDef * USARTx, uint8_t *buffer, uint32_t nBytes) {
	int i;
	// TXE is cleared by a write to the USART_DR register.
	// TXE is set by hardware when the content of the TDR 
	// register has been transferred into the shift register.
	for (i = 0; i < nBytes; i++) {
		while (!(USARTx->ISR & USART_ISR_TXE));   	// wait until TXE (TX empty) bit is set
		// Writing USART_DR automatically clears the TXE flag 	
		USARTx->TDR = buffer[i] & 0xFF;
		USART_Delay(300);
	}
	while (!(USARTx->ISR & USART_ISR_TC));   		  // wait until TC bit is set
	USARTx->ISR &= ~USART_ISR_TC;
}   

void USART_Delay(uint32_t us) {
	uint32_t time = 100*us/7;    
	while(--time);   
}