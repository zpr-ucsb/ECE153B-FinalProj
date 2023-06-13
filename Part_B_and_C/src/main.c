/*
 * ECE 153B - Spring 2023
 *
 * Name(s):
 * Section:
 * Lab: 6C
 */

#include "stm32l476xx.h"
#include "SysClock.h"
#include "SysTimer.h"
#include "LED.h"
#include "DMA.h"
#include "UART.h"
#include "motor.h"
#include "SPI.h"
#include "I2C.h"
#include "accelerometer.h"
#include <stdio.h>
#include <stdbool.h>

static char buffer[IO_SIZE];

#define CLOSEDOOR_DIRE 1
#define OPENDOOR_DIRE 0

bool disableTempControlFor3Seconds = false;
bool doorIsOpened = false;
int threeSeocondCount = 0;
	
void UART_onInput(char* inputs, uint32_t size) {
	LED_Toggle();

	// Process the received input
	for (uint32_t i = 0; i < size; i++) {
		char ch = inputs[i];
		
		if (ch == '\n')
			return;
		
		// Process each character in the input
		if (ch == '1') {
			setDire(OPENDOOR_DIRE);
			sprintf(buffer, "Opening Door\r\n");
			UART_print(buffer);
			
			doorIsOpened = true;
			disableTempControlFor3Seconds = true;
			threeSeocondCount = 0;
	 	}
	 	else if (ch == '2') {
			sprintf(buffer, "Closing Door\r\n");
			UART_print(buffer);
			setDire(CLOSEDOOR_DIRE);
			
			doorIsOpened = false;
			disableTempControlFor3Seconds = true;
			threeSeocondCount = 0;
	 	}
	 	else {
			sprintf(buffer, "That command is not recognized. Please send a valid command\r\n");
			UART_print(buffer);
		}
	}
}

#define CLOSED_POSITION_THRESHOLD -8
#define OPENED_POSITION_THRESHOLD 8
#define TEMPERATURE_THRESHOLD_HIGH 27
#define TEMPERATURE_THRESHOLD_LOW 25



int main(void) {
	// Switch System Clock = 80 MHz
	System_Clock_Init(); 
	Motor_Init();
	SysTick_Init();
	LED_Init();	
	
	SPI1_GPIO_Init();
	SPI1_Init();
	initAcc();
	
	I2C_GPIO_Init();
	I2C_Initialization();
	
	UART2_Init();
	
	sprintf(buffer, "Program Starts.\r\n");
	UART_print(buffer);
	
	//CODE FOR ACCELEROMETER
	double x,y,z;
	
	//CODE FOR TEMPERATURE SENSOR
	uint8_t SlaveAddress;
	uint8_t Data_Receive;
	uint8_t sendingArray[30];
	uint8_t receivingArray[30];
	
	int i = 0;
	int displayValues = 100;
	
	bool accelOpened = false;
	
	int lastTemp = 0;
	
	//stopped
	setDire(2);
	//setDire(CLOSEDOOR_DIRE);
	

	int threeSeocondTimer = 2000;
	bool tempChangedDoorHigh = false;
	bool tempChangedDoorLow = false;
	
	while(1) {
		SlaveAddress = 0b1001000 << 1; // TMP102 Address: 1001000
		sendingArray[0] = 0;
		I2C_SendData(I2C1, SlaveAddress, sendingArray, 1);
		I2C_ReceiveData(I2C1, SlaveAddress, receivingArray, 1);
		Data_Receive = receivingArray[0];

		int temperature = (Data_Receive & 0x7F) - (((Data_Receive & 0x80) != 0) ? 128 : 0);
		
		if(!disableTempControlFor3Seconds)
		{
			if (temperature > TEMPERATURE_THRESHOLD_HIGH && !doorIsOpened) {
				doorIsOpened = true;
				tempChangedDoorHigh = true;
				setDire(OPENDOOR_DIRE);
			} else if (temperature < TEMPERATURE_THRESHOLD_LOW && doorIsOpened) {
				doorIsOpened = false;
				tempChangedDoorLow = true;
				setDire(CLOSEDOOR_DIRE);
			}
		}
		
		readACCValues(&x, &y, &z);
		int8_t currentDire = getDire();
		
		if (z < .1 && x < CLOSED_POSITION_THRESHOLD && currentDire == CLOSEDOOR_DIRE) {
				setDire(2);
		} else if (z > 2 && currentDire == OPENDOOR_DIRE) {
				setDire(2);
		}
		
		if(disableTempControlFor3Seconds)
		{
			if(threeSeocondCount < threeSeocondTimer)
				threeSeocondCount++;
			else
			{
				threeSeocondCount = 0;
				disableTempControlFor3Seconds = false;
			}
		}
		
		if(i >= displayValues)
		{
			if(tempChangedDoorHigh)
			{
					tempChangedDoorHigh = false;
					sprintf(buffer,"Temperature too high. Door opening.\n");
					UART_print(buffer);
			}
			else if(tempChangedDoorLow) {
				tempChangedDoorLow = false;
				sprintf(buffer,"Temperature dropped. Door closing.\n");
				UART_print(buffer);
			}
			else if(temperature != lastTemp)
			{
				lastTemp = temperature;
				sprintf(buffer, "TEMP CHANGED:%6d\r\n", temperature);
				UART_print(buffer);
			}
			else
			{
				//sprintf(buffer, "Acceleration: %.2f, %.2f, %.2f\r\n", x, y, z);
				//UART_print(buffer);
			}
			
			
			LED_Toggle();
			i = 0;
		}
		
		delay(10);
		i++;
	}
}


