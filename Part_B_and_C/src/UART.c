/*
 * ECE 153B - Spring 2023
 *
 * Name(s):
 * Section:
 * Lab: 6C
 */


#include "UART.h"
#include "DMA.h"

static volatile DMA_Channel_TypeDef * tx;
static volatile char inputs[IO_SIZE];
static volatile uint8_t data_t_0[IO_SIZE];
static volatile uint8_t data_t_1[IO_SIZE];
static volatile uint8_t input_size = 0;
static volatile uint8_t pending_size = 0;
static volatile uint8_t * active = data_t_0;
static volatile uint8_t * pending = data_t_1;

#define SEL_0 1
#define BUF_0_EMPTY 2
#define BUF_1_EMPTY 4
#define BUF_0_PENDING 8
#define BUF_1_PENDING 16

void transfer_data(char ch);
void on_complete_transfer(void);

void UART1_Init(void) {
	//TODO
}

void UART2_Init(void) {
	//TODO
	UART2_GPIO_Init();
	
	// Enable the USART2 clock in the peripheral clock register.
	RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN;

	// Select the system clock as the USART2 clock source in the peripheral independent clock configuration register.
	RCC->CCIPR &= ~(RCC_CCIPR_USART2SEL);	// 01: SYSCLCK Selected 
	RCC->CCIPR |= RCC_CCIPR_USART2SEL_0;
	
	tx = DMA1_Channel7;
	
	DMA_Init_UARTx(tx, USART2);
	tx->CMAR = (uint32_t)active;
	DMA1_CSELR->CSELR &= ~DMA_CSELR_C7S;
	DMA1_CSELR->CSELR |= (1<<25); //(0010: Channel 7 mapped on USART2_TX)
	
	USART_Init(USART2);
	NVIC_EnableIRQ(USART2_IRQn);
	NVIC_SetPriority(USART2_IRQn, 1);  // Set USART2 interrupt priority to 1
}

void UART1_GPIO_Init(void) {
	//TODO
}

void UART2_GPIO_Init(void) {
	//TODO
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN; 		// Enable GPIOA CLK

	// Configure PA2 and PA3 to operate as UART transmitters and receivers.
	GPIOA->MODER &= ~(GPIO_MODER_MODE2);        // Clear Port A, Pin 2 MODER
	GPIOA->MODER |= GPIO_MODER_MODE2_1;         // Set Port A, Pin 2 MODER to Alternative Mode
	
	GPIOA->AFR[0] &= ~(GPIO_AFRL_AFSEL2);				// Clear AF for Port A, Pin 2
	GPIOA->AFR[0] |= GPIO_AFRL_AFSEL2_0;				// Choose AF7 (USART2_TX) for Port A, Pin 2
	GPIOA->AFR[0] |= GPIO_AFRL_AFSEL2_1;
	GPIOA->AFR[0] |= GPIO_AFRL_AFSEL2_2;
	
	GPIOA->MODER &= ~(GPIO_MODER_MODE3);        // Clear Port A, Pin 3 MODER
	GPIOA->MODER |= GPIO_MODER_MODE3_1;         // Set Port A, Pin 3 MODER to Alternative Mode
	
	GPIOA->AFR[0] &= ~(GPIO_AFRL_AFSEL3);				// Clear AF for Port A, Pin 3
	GPIOA->AFR[0] |= GPIO_AFRL_AFSEL3_0;				// Choose AF7 (USART2_RX) for Port A, Pin 3
	GPIOA->AFR[0] |= GPIO_AFRL_AFSEL3_1;
	GPIOA->AFR[0] |= GPIO_AFRL_AFSEL3_2;
	
	GPIOA->OSPEEDR |= (GPIO_OSPEEDR_OSPEED2); 	// 11 = very high speed
	GPIOA->OSPEEDR |= (GPIO_OSPEEDR_OSPEED3);
	
	GPIOA->OTYPER &= ~(GPIO_OTYPER_OT2); 				// 0 = push-pull, 1 = opendrain
	GPIOA->OTYPER &= ~(GPIO_OTYPER_OT3);
	
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD2);				// 01 = pull-up
	GPIOA->PUPDR |= GPIO_PUPDR_PUPD2_0;
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD3);
	GPIOA->PUPDR |= GPIO_PUPDR_PUPD3_0;
}

void USART_Init(USART_TypeDef * USARTx) {
		// [TODO]
	
	// Note that USART must be disabled to modify the settings ensure that USART is
	// disabled before you start to modify the registers. 
	USARTx->CR1 &= ~(USART_CR1_UE);

	// 	(a) In the control registers, set word length to 8 bits, oversampling mode to oversample
	// 	    by 16, and number of stop bits to 1.
	USARTx->CR1 &= USART_CR1_M1;		// 00 = 8 bits
	USARTx->CR1 &= USART_CR1_M0;
	
	USART2->CR3 |= USART_CR3_DMAT;  // Enable UART DMA transmission for USART2
	
	USARTx->CR1 &= ~(USART_CR1_OVER8); 	// 0 = oversampling by 16
	USARTx->CR2 &= ~(USART_CR2_STOP);  // 00 = 1 stop bit
	
	// 	(b) Set the baud rate to 9600. To generate the baud rate, you will have to write a value
	// 	    into USARTx BRR.
	USARTx->BRR = 8333; // To obtain 9600 baud with f_CK = 80 MHz. In case of oversampling by 16: USARTDIV = 80000000/9600 = 8333

	// (c) In the control registers, enable both the transmitter and receiver.
	USARTx->CR1 |= USART_CR1_TE;
	USARTx->CR1 |= USART_CR1_RE;
	USARTx->CR1 |= USART_CR1_RXNEIE | USART_CR1_TCIE;  // Enable RXNE and TC interrupts for USART2

	// (d) Now that everything has been set up, enable USART in the control registers
	USARTx->CR1 |= USART_CR1_UE;
}

/**
 * This function accepts a string that should be sent through UART
*/
void UART_print(char* data) {
	//TODO

	//Transfer char array to buffer
	//Check DMA status. If DMA is ready, send data
	//If DMA is not ready, put the data aside
	
	
		uint8_t data_length = 0;
    while (data[data_length] != '\0') {
        data_length++;
    }
    
		uint8_t test = (tx->CCR & DMA_CCR_EN);
		
    if (!(tx->CCR & DMA_CCR_EN)) { // Check DMA status. If DMA is ready, send data
			
				for (uint8_t i = 0; i < data_length; i++) { // Transfer char array to buffer
					active[i] = (uint8_t)data[i];
				}	
			
        // If DMA is not enabled, start the transfer
        tx->CNDTR = data_length;
        tx->CCR |= DMA_CCR_EN;
    }
    else { // If DMA is already enabled, store the data in the pending buffer
        for (uint8_t i = 0; i < data_length; i++) {
            pending[i] = (uint8_t)data[i];
        }
				
        pending_size = data_length;
    }
}

/**
 * This function should be invoked when a character is accepted through UART
*/
void transfer_data(char ch) {
	//TODO
	// Append character to input buffer.
	// If the character is end-of-line, invoke UART_onInput
	
		inputs[input_size++] = ch;
    
    if (ch == '\n') {
        UART_onInput(inputs, input_size);
        input_size = 0;
    }
}

/**
 * This function should be invoked when DMA transaction is completed
*/
void on_complete_transfer(void) {
    if (pending_size > 0) {
        uint8_t *temp = active;
        active = pending;
        pending = temp;
        UART_print(active);
        pending_size = 0;
    }
}

void USART1_IRQHandler(void){
	//TODO
	// When receive a character, invoke transfer_data
	// When complete sending data, invoke on_complete_transfer
}

void USART2_IRQHandler(void){
	//TODO
	// When receive a character, invoke transfer_data
	// When complete sending data, invoke on_complete_transfer
	
		NVIC_ClearPendingIRQ(USART2_IRQn);
	
		if (USART2->ISR & USART_ISR_RXNE) {
			USART2->RQR |= USART_RQR_RXFRQ;
			char ch = USART2->RDR;
			transfer_data(ch);
    }
		
    if (USART2->ISR & USART_ISR_TC) {
				USART2->ICR |= USART_ICR_TCCF;
        on_complete_transfer();
    }
}
