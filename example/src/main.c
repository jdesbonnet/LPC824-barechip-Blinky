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

#define TICKRATE_HZ (1000)	/* 10 ticks per second */

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

	//Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0, 15, systick_counter%2==0 );
}

// Pulse rain in pairs of cycle length, duty
static uint32_t pulsetrain[] = {500,250,
								500,250,
								600,300,
								480,200,
								500,250,
								600,250,
								500,250,
								100000,1};

/**
 * @brief	Handle interrupt from State Configurable Timer
 * @return	Nothing
 */
void SCT_IRQHandler(void)
{
	static uint32_t pulse;

	Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0, 14, true);
	Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0, 14, false);


	// Setup next pulse in pulsetrain
	Chip_SCT_SetMatchReload(LPC_SCT, SCT_MATCH_0, pulsetrain[pulse++ % 16]);
	Chip_SCT_SetMatchReload(LPC_SCT, SCT_MATCH_2, pulsetrain[pulse++ % 16]);

	/* Clear the SCT Event 0 Interrupt */
	Chip_SCT_ClearEventFlag(LPC_SCT, SCT_EVT_0);
}


static bool sequenceComplete, thresholdCrossed;

#define BOARD_ADC_CH 0

/**
 * @brief	Handle interrupt from ADC sequencer A
 * @return	Nothing
 */
void ADC_SEQA_IRQHandler(void)
{
	uint32_t pending;

	Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0, 14, true);
	Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0, 14, false);


	/* Get pending interrupts */
	pending = Chip_ADC_GetFlags(LPC_ADC);

	/* Sequence A completion interrupt */
	if (pending & ADC_FLAGS_SEQA_INT_MASK) {
		sequenceComplete = true;
	}

	/* Threshold crossing interrupt on ADC input channel */
	if (pending & ADC_FLAGS_THCMP_MASK(BOARD_ADC_CH)) {
		thresholdCrossed = true;
	}

	/* Clear any pending interrupts */
	Chip_ADC_ClearFlags(LPC_ADC, pending);
}

void ADC_OVR_IRQHandler(void) {
	Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0, 14, true);
	Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0, 14, false);
	Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0, 14, true);
	Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0, 14, false);
}


/* Setup a timer for a periodic (repeat mode) rate */
static void setupMRT(uint8_t ch, MRT_MODE_T mode, uint32_t rate)
{
	LPC_MRT_CH_T *pMRT;

	/* Get pointer to timer selected by ch */
	pMRT = Chip_MRT_GetRegPtr(ch);

	/* Setup timer with rate based on MRT clock */
	Chip_MRT_SetInterval(pMRT, (Chip_Clock_GetSystemClockRate() / rate) |
						 MRT_INTVAL_LOAD);

	/* Timer mode */
	Chip_MRT_SetMode(pMRT, mode);

	/* Clear pending interrupt and enable timer */
	Chip_MRT_IntClear(pMRT);
	Chip_MRT_SetEnabled(pMRT);
}


#define SCT_PWM            LPC_SCT
#define SCT_PWM_PIN_LED    0		/* COUT0 [index 2] Controls LED */
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

	// For debugging
	Chip_GPIO_SetPinDIROutput(LPC_GPIO_PORT, 0, 14);
	Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0, 14, true);

	/* Initialize the SCT as PWM and set frequency */
	Chip_SCTPWM_Init(SCT_PWM);
	// User MATCH0 to determine PWM frequency
	Chip_SCTPWM_SetRate(SCT_PWM, SCT_PWM_RATE);

	// SwitchMatrix: Assign SCT_OUT0 to PIO0_15
	Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_SWM);
	Chip_SWM_MovablePinAssign(SWM_SCT_OUT0_O, 15);
	Chip_Clock_DisablePeriphClock(SYSCTL_CLOCK_SWM);

	Chip_SCTPWM_SetOutPin(SCT_PWM, 2, 0); // 2=PWM ch 2; 0 = OUT0?

	/* Start with 50% duty cycle */
	Chip_SCTPWM_SetDutyCycle(SCT_PWM,
			2,
			Chip_SCTPWM_GetTicksPerCycle(SCT_PWM) / 2);

	/* Enable flag to request an interrupt for Event 0 */
	Chip_SCT_EnableEventInt(LPC_SCT, SCT_EVT_0);

	/* Enable the interrupt for the SCT */
	NVIC_EnableIRQ(SCT_IRQn);


	//
	// ADC
	//

	Chip_ADC_Init(LPC_ADC, 0);

	/* Need to do a calibration after initialization and trim */
	Chip_ADC_StartCalibration(LPC_ADC);
	while (!(Chip_ADC_IsCalibrationDone(LPC_ADC))) {}

	// Sampling clock rate (not conversion rate). A fully accurate conversion
	// requires 25 ADC clock cycles.
	Chip_ADC_SetClockRate(LPC_ADC, 160000 * 25);


	/* Setup a sequencer to do the following:
	   Perform ADC conversion of ADC channels 0 only */
	Chip_ADC_SetupSequencer(LPC_ADC, ADC_SEQA_IDX,
							(ADC_SEQ_CTRL_CHANSEL(0)
							//| ADC_SEQ_CTRL_HWTRIG_SCT_OUT0
							//| ADC_SEQ_CTRL_HWTRIG_CT16B0_MAT0
							| (3<<12) // SCT0_OUT3
							//| ADC_SEQ_CTRL_HWTRIG_ARM_TXEV
							//| ADC_SEQ_CTRL_MODE_EOS
							)
									);

	/* Enable the clock to the Switch Matrix */
	Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_SWM);
	/* Configure the SWM for P0-6 as the input for the ADC1 */
	Chip_SWM_EnableFixedPin(SWM_FIXED_ADC1);
	/* Disable the clock to the Switch Matrix to save power */
	Chip_Clock_DisablePeriphClock(SYSCTL_CLOCK_SWM);

	/* Clear all pending interrupts */
	Chip_ADC_ClearFlags(LPC_ADC, Chip_ADC_GetFlags(LPC_ADC));

	/* Enable ADC overrun and sequence A completion interrupts */
	Chip_ADC_EnableInt(LPC_ADC, (ADC_INTEN_SEQA_ENABLE
								| ADC_INTEN_OVRRUN_ENABLE));


	/* Enable ADC NVIC interrupt */
	NVIC_EnableIRQ(ADC_SEQA_IRQn);
	NVIC_EnableIRQ(ADC_OVR_IRQn);

	/* Enable sequencer */
	//Chip_ADC_EnableSequencer(LPC_ADC, ADC_SEQA_IDX);



	//
	// Multi Rate Timer (MRT)
	//

	/* MRT Initialization and disable all timers */
	Chip_MRT_Init();
	int i;
	for (i = 0; i < 4; i++) {
		Chip_MRT_SetDisabled(Chip_MRT_GetRegPtr(i));
	}

	/* Enable the interrupt for the MRT */
	NVIC_EnableIRQ(MRT_IRQn);

	/* Enable timers 0 and 1 in repeat mode with different rates */
	setupMRT(0, MRT_MODE_REPEAT, 500);
	//setupMRT(1, MRT_MODE_REPEAT, 40000);/* 4Hz rate */



	// Start pulse train
	Chip_SCTPWM_Start(SCT_PWM);


	/* Enable SysTick Timer */
	SysTick_Config(SystemCoreClock / TICKRATE_HZ);

	/* Loop forever */
	while (1) {
		__WFI();
	}
}


/**
 * @brief	Handle interrupt from MRT
 * @return	Nothing
 */
void MRT_IRQHandler(void)
{
	uint32_t int_pend;

	//Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0, 14, true);
	//Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0, 14, false);

	/* Get interrupt pending status for all timers */
	int_pend = Chip_MRT_GetIntPending();
	Chip_MRT_ClearIntPending(int_pend);

	/* Channel 0 and 1 are periodic, toggle on either interrupt */
	if (int_pend & (MRTn_INTFLAG(0) | MRTn_INTFLAG(1))) {

	}

}
