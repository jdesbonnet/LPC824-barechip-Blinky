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

static volatile uint32_t systick_counter=0;

/**
 * @brief	Handle interrupt from SysTick timer
 * @return	Nothing
 */
void SysTick_Handler(void)
{
	systick_counter++;
}


// Barker-11: +1 +1 +1 -1 -1 -1 +1 -1 -1 +1 -1
// Encoded in binary: 111 0001 0010
//static uint32_t barker11 = 0x712;
static const int num_cycles = 16;
// This bit pattern determines on which cycle numbers a 180 degree phase
// shift is introduced.
// 0b0000 0000  1000 0000  1000 0000  1000 0000
//static const uint8_t phase_pattern = 0x000029a0;
//static const uint32_t phase_pattern = 0x00008208;
static const uint32_t phase_pattern = 0x00000808;
//static const uint32_t phase_pattern = 0x00005555;

static volatile int32_t cycle_number = -1;

#define ADC_BUFFER_SIZE 1024
static uint16_t adc_buffer[ADC_BUFFER_SIZE];
static volatile uint8_t *adc_buffer_ptr;
static volatile uint32_t adc_count;

// MIME64 encode
static char mime64_encoding_table[] = {'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H',
                                'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P',
                                'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X',
                                'Y', 'Z', 'a', 'b', 'c', 'd', 'e', 'f',
                                'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n',
                                'o', 'p', 'q', 'r', 's', 't', 'u', 'v',
                                'w', 'x', 'y', 'z', '0', '1', '2', '3',
                                '4', '5', '6', '7', '8', '9', '+', '/'};

/**
 * @brief	Handle interrupt from State Configurable Timer
 * @return	Nothing
 */
void SCT_IRQHandler(void)
{

	//Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0, 14, true);
	//Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0, 14, false);
	//Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0, 14, true);
	//Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0, 14, false);
#ifdef FALSE
	if (cycle_number<5) {
		int i;
		for (i = 1; i <= cycle_number; i++) {
			Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0, 14, true);
			Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0, 14, false);
		}
	}
#endif

	Chip_SCT_SetMatchReload(LPC_SCT, SCT_MATCH_2, 300);

	if (phase_pattern & (1<<cycle_number) ) {
		Chip_SCT_SetMatchReload(LPC_SCT, SCT_MATCH_0, 900);
	} else {
		Chip_SCT_SetMatchReload(LPC_SCT, SCT_MATCH_0, 600);
	}

	cycle_number++;

	// +2 because we are setting the RELOAD register, also because
	// the first entry into interrupt is due to starter pulse
	if (cycle_number == num_cycles+2) {

		// Pulse is finished. Now switch to using SCT to trigger
		// ADC samples.

		//Chip_SCTPWM_Stop(LPC_SCT);
		Chip_SCT_SetMatchReload(LPC_SCT, SCT_MATCH_2, 300/6);
		Chip_SCT_SetMatchReload(LPC_SCT, SCT_MATCH_0, 600/6);

		Chip_SCTPWM_SetOutPin(LPC_SCT,
				2, // PWM channel
				3 //  the output channel eg SCT_OUT3 (there are 6 in total)
			);

		// SwitchMatrix: Unassign SCT_OUT0
		Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_SWM);
		Chip_SWM_MovablePinAssign(SWM_SCT_OUT0_O, 0xff); // was OUT0_O
		Chip_Clock_DisablePeriphClock(SYSCTL_CLOCK_SWM);

		NVIC_DisableIRQ(SCT_IRQn);
		cycle_number=-1;
	}

	/* Clear the SCT Event 0 Interrupt */
	Chip_SCT_ClearEventFlag(LPC_SCT, SCT_EVT_0);
}


/**
 * @brief	Handle interrupt from ADC sequencer A
 * @return	Nothing
 */
void ADC_SEQA_IRQHandler(void)
{
	uint32_t pending;

	//Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0, 14, true);
	//Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0, 14, false);


	/* Get pending interrupts */
	pending = Chip_ADC_GetFlags(LPC_ADC);

	/* Sequence A completion interrupt */
	if (pending & ADC_FLAGS_SEQA_INT_MASK) {
		//sequenceComplete = true;
	}

	adc_buffer[adc_count++] = (Chip_ADC_GetDataReg(LPC_ADC,3)>>4) & 0xfff;

	if (adc_count == ADC_BUFFER_SIZE) {
		//NVIC_DisableIRQ(ADC_SEQA_IRQn);
		// Disable the ADC hardware trigger
		Chip_SCTPWM_Stop(LPC_SCT);
	}

	/* Clear any pending interrupts */
	Chip_ADC_ClearFlags(LPC_ADC, pending);
}

/**
 * @brief ADC overflow interrupt handler.
 */
void ADC_OVR_IRQHandler(void) {
	//Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0, 14, true);
	//Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0, 14, false);
	//Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0, 14, true);
	//Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0, 14, false);
}

#ifdef ENABLE_MRT
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
#endif

#define SCT_PWM        LPC_SCT
#define SCT_PWM_RATE   1000		/* PWM frequency 10 KHz */

int __sys_write(int fileh, char *buf, int len) {
	Chip_UART_SendBlocking(LPC_USART0, buf,len);
	return len;
}

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

	//
	// Initialize UART
	//

	// Assign pins: use same assignment as serial bootloader
	Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_SWM);
	Chip_SWM_MovablePinAssign(SWM_U0_TXD_O, 4);
	Chip_SWM_MovablePinAssign(SWM_U0_RXD_I, 0);
	Chip_Clock_DisablePeriphClock(SYSCTL_CLOCK_SWM);

	Chip_UART_Init(LPC_USART0);
	Chip_UART_ConfigData(LPC_USART0, UART_CFG_DATALEN_8 | UART_CFG_PARITY_NONE | UART_CFG_STOPLEN_1);
	//Chip_Clock_SetUSARTNBaseClockRate((115200 * 16), true);
	Chip_Clock_SetUSARTNBaseClockRate((230400 * 16), true);

	//Chip_UART_SetBaud(LPC_USART0, 115200);
	Chip_UART_SetBaud(LPC_USART0, 230400);

	Chip_UART_Enable(LPC_USART0);
	Chip_UART_TXEnable(LPC_USART0);

	Chip_UART_SendBlocking(LPC_USART0, "Hello!\r\n", 8);

	printf ("System clock rate: %d\r\n", Chip_Clock_GetSystemClockRate());

	/* Initialize GPIO */
	Chip_GPIO_Init(LPC_GPIO_PORT);
	Chip_GPIO_SetPinDIROutput(LPC_GPIO_PORT, 0, 15);
	Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0, 15, true);

	// For debugging
	Chip_GPIO_SetPinDIROutput(LPC_GPIO_PORT, 0, 14);
	Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0, 14, true);


//#define ENABLE_PWM

#ifdef ENABLE_PWM

	Chip_SCT_Init(LPC_SCT);

	// User MATCH0 to determine PWM frequency
	Chip_SCTPWM_SetRate(SCT_PWM, SCT_PWM_RATE);

	// SwitchMatrix: Assign SCT_OUT0 to PIO0_15
	Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_SWM);
	Chip_SWM_MovablePinAssign(SWM_SCT_OUT3_O, 15); // was OUT0_O
	Chip_Clock_DisablePeriphClock(SYSCTL_CLOCK_SWM);

	// SCT->EV[2] =
	Chip_SCTPWM_SetOutPin(SCT_PWM,
			2, // PWM channel
			3 //  the output channel eg SCT_OUT3 (there are 6 in total)
		);

	/* Start with 50% duty cycle */
	// MATCHREL[2] = rate
	Chip_SCTPWM_SetDutyCycle(SCT_PWM,
			2,
			Chip_SCTPWM_GetTicksPerCycle(SCT_PWM) / 2);

	/* Enable flag to request an interrupt for Event 0 */
	Chip_SCT_EnableEventInt(LPC_SCT, SCT_EVT_0);

	/* Enable the interrupt for the SCT */
	NVIC_EnableIRQ(SCT_IRQn);

	// Start pulse train
	//Chip_SCTPWM_Start(SCT_PWM);
#endif

#define ENABLE_ADC
#ifdef ENABLE_ADC
	//
	// ADC
	//

	Chip_ADC_Init(LPC_ADC, 0);

	/* Need to do a calibration after initialization and trim */
	Chip_ADC_StartCalibration(LPC_ADC);
	while (!(Chip_ADC_IsCalibrationDone(LPC_ADC))) {}

	// Sampling clock rate (not conversion rate). A fully accurate conversion
	// requires 25 ADC clock cycles.
	Chip_ADC_SetClockRate(LPC_ADC, 500000 * 25);
	//Chip_ADC_SetClockRate(LPC_ADC, ADC_MAX_SAMPLE_RATE);
	Chip_ADC_SetDivider(LPC_ADC,0);

	/* Setup a sequencer to do the following:
	   Perform ADC conversion of ADC channel 3 only */
	Chip_ADC_SetupSequencer(LPC_ADC, ADC_SEQA_IDX,
							(ADC_SEQ_CTRL_CHANSEL(3)
							| (3<<12) // SCT0_OUT3 see UM10800 §21.3.3
							| ADC_SEQ_CTRL_MODE_EOS
							)
									);

	/* Enable fixed pin ADC3 with SitchMatrix */
	Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_SWM);
	Chip_SWM_EnableFixedPin(SWM_FIXED_ADC3);
	Chip_SWM_EnableFixedPin(SWM_FIXED_ADC10);
	Chip_Clock_DisablePeriphClock(SYSCTL_CLOCK_SWM);

	/* Clear all pending interrupts */
	Chip_ADC_ClearFlags(LPC_ADC, Chip_ADC_GetFlags(LPC_ADC));

	/* Enable ADC overrun and sequence A completion interrupts */
	Chip_ADC_EnableInt(LPC_ADC, (ADC_INTEN_SEQA_ENABLE
								//| ADC_INTEN_OVRRUN_ENABLE
								));


	/* Enable ADC NVIC interrupt */
	NVIC_EnableIRQ(ADC_SEQA_IRQn);
	NVIC_EnableIRQ(ADC_OVR_IRQn);

	/* Enable sequencer */
	Chip_ADC_EnableSequencer(LPC_ADC, ADC_SEQA_IDX);

	printf ("IOCON->PIO0_23=%x\r\n", LPC_IOCON->PIO0[IOCON_PIO23]);


#endif


	//
	// Multi Rate Timer (MRT)
	//
#ifdef ENABLE_MRT
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
#endif


	/* Enable SysTick Timer */
	SysTick_Config(SystemCoreClock / TICKRATE_HZ);




	uint32_t start_time;
	uint32_t t;

	/* Loop forever */
	while (1) {
		__WFI();

		// Repeat pulse every 100ms
		t = systick_counter;
		if ( (cycle_number== -1) && ((t%200)==0) && (t!=start_time) ) {
			start_time = t;



			// Setup SCT
			Chip_SCT_Init(LPC_SCT);
			/* Stop the SCT before configuration */
			Chip_SCTPWM_Stop(LPC_SCT);
			/* Set MATCH0 for max limit */
			LPC_SCT->REGMODE_U = 0;

			cycle_number = 0;

			LPC_SCT->EV[0].CTRL = 1 << 12;
			LPC_SCT->EV[0].STATE = 1;

			// SCT->EV[2] =
			Chip_SCTPWM_SetOutPin(SCT_PWM,
					2, // PWM channel
					0 //  the output channel eg SCT_OUT3 (there are 6 in total)
				);

			/* Set SCT Counter to count 32-bits and reset to 0 after reaching MATCH0 */
			Chip_SCT_Config(LPC_SCT, SCT_CONFIG_32BIT_COUNTER | SCT_CONFIG_AUTOLIMIT_L);
			/* Enable flag to request an interrupt for Event 0 */
			Chip_SCT_EnableEventInt(LPC_SCT, SCT_EVT_0);
			/* Enable the interrupt for the SCT */
			NVIC_EnableIRQ(SCT_IRQn);


			// Use dummy starter pulse to enter ISR to start pulse train
			//Chip_SCT_SetMatchReload(LPC_SCT, SCT_MATCH_0, 16);
			//Chip_SCT_SetMatchReload(LPC_SCT, SCT_MATCH_2, 8);

			// Setup ADC stuff
			adc_count = 0;

			// Start SCT

			/*
			int i;
			for (i = 0; i < 3; i++) {
				Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0, 14, true);
				Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0, 14, false);
			}
			*/

			// SwitchMatrix: Assign SCT_OUT0 to PIO0_15
			Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_SWM);
			Chip_SWM_MovablePinAssign(SWM_SCT_OUT0_O, 15); // was OUT0_O
			Chip_Clock_DisablePeriphClock(SYSCTL_CLOCK_SWM);


			Chip_SCT_ClearControl(LPC_SCT, SCT_CTRL_HALT_L | SCT_CTRL_HALT_H);
		}

		if (adc_count == ADC_BUFFER_SIZE) {
#define DUMP_DATA
#ifdef DUMP_DATA
			int i;
			for (i = 0; i < ADC_BUFFER_SIZE; i++) {
				//printf ("%d ", adc_buffer[i]);
				printf ("%c%c",mime64_encoding_table[(adc_buffer[i]>>6)&0x3f],
								mime64_encoding_table[adc_buffer[i]&0x3f]);
			}
			printf ("\r\n");
#endif

			adc_count = 0;
		}
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
