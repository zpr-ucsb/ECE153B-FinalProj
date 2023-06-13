#include "SPI.h"
#include "SysTimer.h"

void SPI1_GPIO_Init(void) {
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN;
	GPIOA->MODER &= ~GPIO_MODER_MODE4;
	GPIOA->MODER |= GPIO_MODER_MODE4_1;
	
	GPIOB->MODER &= ~(GPIO_MODER_MODE3 | GPIO_MODER_MODE4 | GPIO_MODER_MODE5);
	GPIOB->MODER |= (GPIO_MODER_MODE3_1 | GPIO_MODER_MODE4_1 | GPIO_MODER_MODE5_1);
	
	GPIOA->AFR[0] &= ~(GPIO_AFRL_AFSEL4);
	GPIOA->AFR[0] |= GPIO_AFRL_AFSEL4_2 | GPIO_AFRL_AFSEL4_0;
	GPIOB->AFR[0] &= ~(GPIO_AFRL_AFSEL3 | GPIO_AFRL_AFSEL4 | GPIO_AFRL_AFSEL5);
	GPIOB->AFR[0] |= GPIO_AFRL_AFSEL3_2 | GPIO_AFRL_AFSEL3_0 | GPIO_AFRL_AFSEL4_2 | GPIO_AFRL_AFSEL4_0 | GPIO_AFRL_AFSEL5_2 | GPIO_AFRL_AFSEL5_0;
	
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR4;
	GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR3 | GPIO_OSPEEDER_OSPEEDR4 | GPIO_OSPEEDER_OSPEEDR5;
	
}

void SPI1_Init(void){
	// (a) Enable the SPI clock.
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

	// (b) Set the RCC SPI reset bit, then clear it to reset the SPI1 or SPI2 peripheral.
	RCC->APB2RSTR |= RCC_APB2RSTR_SPI1RST;
	RCC->APB2RSTR &= ~(RCC_APB2RSTR_SPI1RST);

	// (c) Disable the SPI enable bit. The peripheral must be configured while it is disabled.
	SPI1->CR1 &= ~(SPI_CR1_SPE);			// 0 = disabled

	// (d) Conigure the peripheral for full-duplex communication.
	SPI1->CR1 &= ~(SPI_CR1_RXONLY);			// 0 = full-duplex (Transmit and receive)

	// (e) Configure the peripheral for 2-line unidirectional data mode.
	SPI1->CR1 &= ~(SPI_CR1_BIDIMODE);		// 0 = 2-line unidirectional data mode selected

	// (f) Disable output in bidirectional mode.
	SPI1->CR1 &= ~(SPI_CR1_BIDIOE); 		// 0 = Output disabled (receive-only mode) 

	// (g) Configure the frame format as MSB first.
	SPI1->CR1 &= ~(SPI_CR1_LSBFIRST);		// 0 = data is transmitted / received with the MSB first

	// (h) Configure the frame format to 16-bit mode.
	SPI1->CR2 |= SPI_CR2_DS; // 16 bit data size		

	// (i) Use Motorola SPI mode.
	SPI1->CR2 &= ~(SPI_CR2_FRF); 			// 0 = SPI Motorola mode

	// (j) Configure the clock to low polarity.
	SPI1->CR1 |= SPI_CR1_CPOL | SPI_CR1_CPHA;

	// (l) Set the baud rate prescaler to 16.
	SPI1->CR1 &= ~SPI_CR1_BR;
	SPI1->CR1 |= SPI_CR1_BR_0 | SPI_CR1_BR_1;

	// (m) Disable hardware CRC calculation.
	SPI1->CR1 &= ~(SPI_CR1_CRCEN);			// 0 = CRC calculation disabled

	// (n) Set SPI1 to master mode and SPI2 to slave mode.
	SPI1->CR1 |= SPI_CR1_MSTR;				// 1 = Master

	// Disable Software Slave Management
	SPI1->CR1 &= ~SPI_CR1_SSM;
	
	// (p) Enable NSS pulse generation.
	SPI1->CR2 |= SPI_CR2_NSSP;				// 1 = NSS pulse generated

	// Enable Output
	SPI1->CR2 |= SPI_CR2_SSOE;

	// (q)  Configure the internal slave select bit, 1 for master and 0 for slave.
	SPI1->CR1 |= SPI_CR1_SSI;

	// Set FIFO Reception Threshold to 1/2
	SPI1->CR2 &= ~SPI_CR2_FRXTH;

	// (s) Enable the SPI peripheral.
	SPI1->CR1 |= SPI_CR1_SPE;	
}


uint16_t SPI_Transfer_Data(uint16_t write_data){
// Wait for TXE (Transmit buffer empty)
	while ((SPI1->SR & SPI_SR_TXE) != SPI_SR_TXE) {}
	// Write data
	SPI1->DR = write_data;
	// Wait for not busy
	while ((SPI1->SR & SPI_SR_BSY) == SPI_SR_BSY) {}
	// Wait for RXNE (Receive buffer not empty)
	while ((SPI1->SR & SPI_SR_RXNE) != SPI_SR_RXNE) {}
	// Read data
	return SPI1->DR; // DONE
}
