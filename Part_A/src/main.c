/*
 * ECE 153B - Spring 2023
 *
 * Name(s):
 * Section:
 */

#include "stm32l476xx.h"
#include "SysClock.h"
#include "SysTimer.h"
#include "UART.h"
#include "motor.h"
#include <stdio.h>


// Initializes USARTx
// USART2: UART Communication with Termite
// USART1: Bluetooth Communication with Phone
void Init_USARTx(int x) {
    if(x == 1) {
        UART1_Init();
        UART1_GPIO_Init();
        USART_Init(USART1);
    } else if(x == 2) {
        UART2_Init();
        UART2_GPIO_Init();
        USART_Init(USART2);
    } else {
        // Do nothing...
    }
}

int main(void) {
	char ch;
	// Switch System Clock = 80 MHz
	System_Clock_Init(); 
	Motor_Init();
	SysTick_Init();
	Init_USARTx(2);
	
	printf("Program Starts.\r\n");
	//printf("Hello! Enter a command: \n");
	
	setDire(1);
	
	while(1) {
			// Wait for a response from the terminal. While there is no response,
			// scanf will load a 0 into rxByte as well as return 0.
			while (scanf("%c", &ch) == 0) {
			
			}
			// Receive the response from the terminal.
			// Use scanf() to receive data from Termite. Data can be sent from Termite by typing
			// a message into the terminal and pressing enter.
			if (ch == 'Y' || ch == 'y') {
				printf("Changed dir. 1\n");
				setDire(1);
			}
			else if (ch == 'N' || ch == 'n') {
				printf("Changed dir. 0\n");
				setDire(0);
			}
			else {
				printf("That command is not recognized. Please send a valid command.");
			}
	}
}


