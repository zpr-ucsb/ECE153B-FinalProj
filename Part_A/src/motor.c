/*
 * ECE 153B - Spring 2023
 *
 * Lab: 6A
 */

#include "stm32l476xx.h"
#include "motor.h"

static const uint32_t HalfStep[8] = {0b0001, 0b0011, 0b0010, 0b0110, 0b0100, 0b1100, 0b1000, 0b1001};

static volatile int8_t dire = 0;
static volatile uint8_t step = 0;

void Motor_Init(void) {	
// Enable GPIO Clock For Port C
	RCC->AHB2ENR |= (1<<2);

	// Enable OUTPUT FOR PINS 5,6,8,9
	GPIOC->MODER &= ~(1<<11); //5
	GPIOC->MODER &= ~(1<<13); //6
	GPIOC->MODER &= ~(1<<17); //8
	GPIOC->MODER &= ~(1<<19); //9

	//Output Speed
	GPIOC->OSPEEDR |= (1<<11);
	GPIOC->OSPEEDR |= (1<<13);
	GPIOC->OSPEEDR |= (1<<17);
	GPIOC->OSPEEDR |= (1<<19);

	GPIOC->OSPEEDR &= (~(1<<10));
	GPIOC->OSPEEDR &= (~(1<<12));
	GPIOC->OSPEEDR &= (~(1<<16));
	GPIOC->OSPEEDR &= (~(1<<18));

	//Output Type Push-Pull
	GPIOC->OTYPER &= (~(1<<5));
	GPIOC->OTYPER &= (~(1<<6));
	GPIOC->OTYPER &= (~(1<<8));
	GPIOC->OTYPER &= (~(1<<9));

	// Pull Up No Pull Down
	GPIOC->PUPDR &= ~((1<<11)+(1<<10));
	GPIOC->PUPDR &= ~((1<<13)+(1<<12));
	GPIOC->PUPDR &= ~((1<<17)+(1<<16));
	GPIOC->PUPDR &= ~((1<<19)+(1<<18));
	
	GPIOC->ODR |= GPIO_ODR_OD5;		
	GPIOC->ODR &= ~(GPIO_ODR_OD6);
	GPIOC->ODR &= ~(GPIO_ODR_OD8);
	GPIOC->ODR |= GPIO_ODR_OD9;

	// Enable the clock for GPIO
	//RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;

	// Configure GPIOC pins as output
	//GPIOC->MODER &= ~(GPIO_MODER_MODE5 | GPIO_MODER_MODE6 | GPIO_MODER_MODE8 | GPIO_MODER_MODE9);
	//GPIOC->MODER |= (GPIO_MODER_MODE5_0 | GPIO_MODER_MODE6_0 | GPIO_MODER_MODE8_0 | GPIO_MODER_MODE9_0);

	// Configure GPIOC pins as push-pull outputs
	//GPIOC->OTYPER &= ~(GPIO_OTYPER_OT5 | GPIO_OTYPER_OT6 | GPIO_OTYPER_OT8 | GPIO_OTYPER_OT9);

	// Configure GPIOC pins with no pull-up/pull-down
	//GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPD5 | GPIO_PUPDR_PUPD6 | GPIO_PUPDR_PUPD8 | GPIO_PUPDR_PUPD9);

	// Configure the timer
	configureTimer();

	step = 0;
}

void configureTimer(void)
{
    // Enable the clock for TIM3
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM3EN;

    // Set the timer prescaler and period
    TIM3->PSC = 200; // Adjust the prescaler based on the desired frequency
    TIM3->ARR = 1000; // Adjust the period based on the desired speed

    // Enable the update interrupt
    TIM3->DIER |= TIM_DIER_UIE;

    // Set the interrupt priority and enable the interrupt
    NVIC_SetPriority(TIM3_IRQn, 0);
    NVIC_EnableIRQ(TIM3_IRQn);

    // Start the timer
    TIM3->CR1 |= TIM_CR1_CEN;
}


void TIM3_IRQHandler(void)
{
    if (TIM3->SR & TIM_SR_UIF)
    {
			rotate();

			// Clear the update interrupt flag
			TIM3->SR &= ~TIM_SR_UIF;
    }
}

void setDire(int8_t direction) {
	dire = direction;
}

uint8_t getStep(void)
{
	return step;
}

void rotate(void)
{
		// Activate the pins based on the current step
		uint32_t outputData = 0;
	
		if(0b0001 & HalfStep[step])
			outputData |= GPIO_ODR_ODR_5;
		
		if(0b0010 & HalfStep[step])
			outputData |= GPIO_ODR_ODR_6;
		
		if(0b0100 & HalfStep[step])
			outputData |= GPIO_ODR_ODR_8;
		
		if(0b1000 & HalfStep[step])
			outputData |= GPIO_ODR_ODR_9;
	
		GPIOC->ODR &= ~(GPIO_ODR_ODR_5 | GPIO_ODR_ODR_6 | GPIO_ODR_ODR_8 | GPIO_ODR_ODR_9); // Turn off all pins first
		GPIOC->ODR |= outputData; // Activate the required pins based on the step
	
    // Increment or decrement the current step based on the direction
    if (dire == 0)
    {
        step++;
        if (step >= 8)
            step = 0;
    }
    else
    {
        if (step == 0)
            step = 7;
        else
            step--;
    }
}
