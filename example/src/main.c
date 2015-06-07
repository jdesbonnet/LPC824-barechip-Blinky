/*
 * @brief Blinky example using SysTick and interrupt
 *
 * @note
 * Copyright(C) NXP Semiconductors, 2013
 * All rights reserved.
 *
 * @par
 * Software that is described herein is for illustrative purposes only
 * which provides customers with programming information regarding the
 * LPC products.  This software is supplied "AS IS" without any warranties of
 * any kind, and NXP Semiconductors and its licensor disclaim any and
 * all warranties, express or implied, including all implied warranties of
 * merchantability, fitness for a particular purpose and non-infringement of
 * intellectual property rights.  NXP Semiconductors assumes no responsibility
 * or liability for the use of the software, conveys no license or rights under any
 * patent, copyright, mask work right, or any other intellectual property rights in
 * or to any products. NXP Semiconductors reserves the right to make changes
 * in the software without notification. NXP Semiconductors also makes no
 * representation or warranty that such application will be suitable for the
 * specified use without further testing or modification.
 *
 * @par
 * Permission to use, copy, modify, and distribute this software and its
 * documentation is hereby granted, under NXP Semiconductors' and its
 * licensor's relevant copyrights in the software, without fee, provided that it
 * is used in conjunction with NXP Semiconductors microcontrollers.  This
 * copyright, permission, and disclaimer notice must appear in all copies of
 * this code.
 */

//#include "board.h"
#include "chip.h"
#include <stdio.h>

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

#define TICKRATE_HZ (10)	/* 10 ticks per second */

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/**
 * @brief	Handle interrupt from SysTick timer
 * @return	Nothing
 */
void SysTick_Handler(void)
{
	static int systick_counter;

	systick_counter++;

	Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0, 15, systick_counter%2==0 );

}

// Pulse rain in pairs of cycle length, duty
static uint32_t pulsetrain[] = {1000,500,
								1000,500,
								1010,505,
								1000,500,
								100000,1};

/**
 * @brief	Handle interrupt from State Configurable Timer
 * @return	Nothing
 */
void SCT_IRQHandler(void)
{
	static uint32_t pulse;

	Chip_SCT_SetMatchReload(LPC_SCT, SCT_MATCH_0, pulsetrain[pulse++ % 10]);
	Chip_SCT_SetMatchReload(LPC_SCT, SCT_MATCH_2, pulsetrain[pulse++ % 10]);

	/* Clear the SCT Event 0 Interrupt */
	Chip_SCT_ClearEventFlag(LPC_SCT, SCT_EVT_0);
}

#define SCT_PWM            LPC_SCT
#define SCT_PWM_PIN_OUT    1		/* COUT1 Generate square wave */
#define SCT_PWM_PIN_LED    0		/* COUT0 [index 2] Controls LED */
#define SCT_PWM_OUT        1		/* Index of OUT PWM */
#define SCT_PWM_LED        2		/* Index of LED PWM */
#define SCT_PWM_RATE   10000		/* PWM frequency 10 KHz */


/**
 * @brief	main routine for blinky example
 * @return	Function should not exit.
 */
int main(void)
{

	SystemCoreClockUpdate();

	// If we don't call SystemInit() will use default 12MHz internal
	// clock without any PLL. This is good for initial experiments.
	//SystemInit();

	/* Initialize GPIO */
	Chip_GPIO_Init(LPC_GPIO_PORT);

	Chip_GPIO_SetPinDIROutput(LPC_GPIO_PORT, 0, 15);
	Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0, 15, true);

	/* Initialize the SCT as PWM and set frequency */
	Chip_SCTPWM_Init(SCT_PWM);
	Chip_SCTPWM_SetRate(SCT_PWM, SCT_PWM_RATE);

	/* Enable SWM clock before altering SWM */
	Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_SWM);
	//Chip_SWM_MovablePinAssign(SWM_SCT_OUT1_O, 1);
	Chip_SWM_MovablePinAssign(SWM_SCT_OUT0_O, 15);
	Chip_Clock_DisablePeriphClock(SYSCTL_CLOCK_SWM);

	/* Use SCT0_OUT1 pin */
	//Chip_SCTPWM_SetOutPin(SCT_PWM, SCT_PWM_OUT, SCT_PWM_PIN_OUT);
	Chip_SCTPWM_SetOutPin(SCT_PWM, SCT_PWM_LED, SCT_PWM_PIN_LED);

	/* Start with 50% duty cycle */
	//Chip_SCTPWM_SetDutyCycle(SCT_PWM, SCT_PWM_OUT, Chip_SCTPWM_GetTicksPerCycle(SCT_PWM) / 2);
	Chip_SCTPWM_SetDutyCycle(SCT_PWM, SCT_PWM_LED, Chip_SCTPWM_GetTicksPerCycle(SCT_PWM) / 2);

	/* Enable flag to request an interrupt for Event 0 */
	Chip_SCT_EnableEventInt(LPC_SCT, SCT_EVT_0);

	/* Enable the interrupt for the SCT */
	NVIC_EnableIRQ(SCT_IRQn);


	Chip_SCTPWM_Start(SCT_PWM);


	/* Enable SysTick Timer */
	SysTick_Config(SystemCoreClock / TICKRATE_HZ);

	/* Loop forever */
	while (1) {
		__WFI();
	}
}
