/*
 *@author: J. Scotty Solomon
 *@version 28-Sep-23
 *
 * This project uses the FRDM board to trace simple lines. 
 * Each wheel has its own line sensor. This project requires a line 
 * for each sensor to follow. Once both sensors stop sensing a line,
 * the motor is stopped.
 * 
 * This should be tested with a black line on a white background. The
 * sensors used are not perfect, so the closer to white that's detected, 
 * generally the further the sensor is from the desired line.
 *
 * Pins used with the dual motor driver: 
 * 
 * Driver | FRDM Board
 * PWMA: B2
 * AI1:  B1
 * AI1:  B0
 *
 * PWMB: B3
 * BI1:  C1
 * PI2:  C2
 *
 * VCC and STBY: P3V3
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MKL46Z4.h"
#include "fsl_debug_console.h"

#define TOLERANCE 230
#define ERROR_MULTIPLIER 3

void init();
void start();
void changeRightSpeed();
void changeLeftSpeed();

int rightMod = 20000; int leftMod = 20000;

/*
 * Main entry point, which awaits switch 1 to be pressed before
 * beginning main loop. Each sensor detects its own line, and the 
 * speed for each wheel is adjusted based upon the detected error.
 */
int main(void) {
    init();
	int rightSensor, leftSensor;
	int leftError, rightError;

    while(1) {
    	//switch 1 pressed
    	if(!(GPIOC->PDIR & 0x8)) {
    		start();

			//Right sensor data
			ADC0->SC1[0] = 0x05; //pte17, channel AD5a
			while(!(ADC0->SC1[0] & 0x80)){	}
			rightSensor = ADC0->R[0];

			//left sensor data
			ADC0->SC1[0] = 0x01; //pte16, channel DAD1
			while(!(ADC0->SC1[0] & 0x80)){	} //busy wait, not the best
			leftSensor = ADC0->R[0];

			leftError = TOLERANCE - leftSensor;
			rightError = TOLERANCE - rightError;

			//Left wheel too far left
			if(leftError < 0 && rightError >= 0) {
				leftMod += abs(leftError) * ERROR_MULTIPLIER;
				changeLeftSpeed();
			//Right wheel too far right
			} else if(rightError < 0 && leftError >= 0) {
				rightMod += abs(rightError) * ERROR_MULTIPLIER;
				changeRightSpeed();
			} else if(rightError < 0 && leftError < 0) {
				stop();
			}
    	}
    }
    return 0;
}

/**
 * Initializes necessary ports, pins, and switches
*/
void init() {
    /* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();

	#ifndef BOARD_INIT_DEBUG_CONSOLE_PERIPHERAL
		BOARD_InitDebugConsole();
	#endif

    //Enabling Ports
    SIM->SCGC5 |= 0x200;	//Port B
    SIM->SCGC5 |= 0x400; 	//Port C

    //Enabling Pins
	//B0
	PORTB->PCR[0] &= ~0x700;
	PORTB->PCR[0] |= 0x800;

	//B1
	PORTB->PCR[1] &= ~0x700;
	PORTB->PCR[1] |= 0x800;

	//B2 (PWM)
	PORTB->PCR[2] &= ~0x700;
	PORTB->PCR[2] |= 0x300;

	//B3 (PMW)
	PORTB->PCR[3] &= ~0x700;
	PORTB->PCR[3] |= 0x300;

	//C1
	PORTC->PCR[1] &= ~0x700;
	PORTC->PCR[1] |= 0x800;

	//C2
	PORTC->PCR[2] &= ~0x700;
	PORTC->PCR[2] |= 0x800);

	//C3
	PORTC->PCR[3] &= ~0x703;
	PORTC->PCR[3] |= 0x800; //enable pull ups

	//C12
	PORTC->PCR[12] &= ~0x703;
	PORTC->PCR[12] |= 0x800; //enable pull ups


	//Setting up pins for GPIO
	GPIOB->PDDR |= 0x1; //B0
	GPIOC->PDDR |= 0x3;	//C1-C2

	//Setting input switch
	GPIOC->PDDR &= ~0x8; //C3
}

/**
 * Starts ADC conversion for both sensors and begins PWM for both 
 * wheels, starting both at equal speeds.
*/
void start() {
	//Calibrating Analog to Digital Converter
	cal_v = ADC0->CLP0 + ADC0->CLP1 + ADC0->CLP2 + ADC0->CLP3 + ADC0->CLP4 + ADC0->CLPS;
	cal_v = cal_v >> 1 | 0x8000;
	ADC0->PG = cal_v;

	cal_v = 0;
	cal_v = ADC0->CLM0 + ADC0->CLM1 + ADC0->CLM2 + ADC0->CLM3 + ADC0->CLM4 + ADC0->CLMS;
	cal_v = cal_v >> 1 | 0x8000;
	ADC0->MG = cal_v;

	//Clock selection for PWM
	SIM->SCGC6 |= (1 << 25); // Clock Enable TPM1
	SIM->SCGC6 |= (1 << 26); // Clock Enable TPM2
	SIM->SOPT2 |= (0x2 << 24); // Set TPMSRC to OSCERCLK

	//Right wheel PWM 
	TPM2->CONTROLS[0].CnSC |= (0x1 << 2) | (0x1 << 4);
	TPM2->MOD = rightMod; //Right
	TPM2->SC |= 0x7; 
	TPM2->SC |= 0x01 << 3; //Right wheel pwm

	//Left wheel PWM
	TPM1->CONTROLS[0].CnSC |= (0x1 << 2) | (0x1 << 4);
	TPM1->MOD = leftMod; //Right
	TPM1->SC |= 0x7; 
	TPM1->SC |= 0x01 << 3; //Right wheel pwm

	//Setting motor driver to move forward
	GPIOB->PDOR &= 0x1; //B1
	GPIOC->PDOR &= 0x1; //C1
}

/**
 * Signals to motor driver to stop all wheels
 */
void stop() {
	GPIOB->PDOR &= ~(0x1); //B1
	GPIOC->PDOR &= ~(0x1); //C1
}

/** 
 * Changes right wheel speed based on set Mod value.
*/
void changeRightSpeed() {
	TPM2->MOD = rightMod;
	TPM2->SC |= 0x01 << 3; //start clock
}

/**
 * Changes left wheel speed based on set Mod value.
*/
void changeLeftSpeed() {
	TPM1->MOD = leftMod;
	TPM1->SC |= 0x01 << 3; //start clock
}