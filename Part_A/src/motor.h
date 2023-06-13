#ifndef __STM32L476G_DISCOVERY_MOTOR_H
#define __STM32L476G_DISCOVERY_MOTOR_H

#include "stm32l476xx.h"

void Motor_Init(void);
void rotate(void);
void setDire(int8_t direction);
void Half_Stepping_Clockwise(void);
void Half_Stepping_CounterClockwise(void);
void configureTimer(void);
uint8_t getStep(void);

#define DELAY 6000	// delay between steps of the sequences

#endif /* __STM32L476G_DISCOVERY_MOTOR_H */
