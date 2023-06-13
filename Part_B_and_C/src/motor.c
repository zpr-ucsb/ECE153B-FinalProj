/*
 * ECE 153B - Spring 2023
 */

#include "stm32l476xx.h"
#include "motor.h"

static const uint32_t HalfStep[8] = {0b0001, 0b0011, 0b0010, 0b0110, 0b0100, 0b1100, 0b1000, 0b1001};

static volatile int8_t dire = 0;
static volatile uint8_t step = 0;

void Motor_Init(void) {	
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;
	GPIOC->MODER &= (GPIO_MODER_MODE5_0 | GPIO_MODER_MODE6_0 | GPIO_MODER_MODE8_0 | GPIO_MODER_MODE9_0);
	GPIOC->OSPEEDR |= (GPIO_OSPEEDR_OSPEED5 | GPIO_OSPEEDR_OSPEED6 | GPIO_OSPEEDR_OSPEED8 | GPIO_OSPEEDR_OSPEED9);
	GPIOC->PUPDR &= 0 << 31;
	GPIOC->OTYPER &= 0 << 31;
	GPIOC->ODR &= 0<<12;

	step = 0;
}

void setDire(int8_t direction) {
	dire = direction;
}

int8_t getDire(void){
	return dire;
}

uint8_t getStep(void)
{
	return step;
}

void rotate(void)
{
		if(dire == 2)
			return;
	
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
