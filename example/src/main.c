/*

 */

//#include "board.h"
#include "chip.h"
#include <stdio.h>

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

#define TICKRATE_HZ (100)

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
static const int num_cycles = 8;
// This bit pattern determines on which cycle numbers a 180 degree phase
// shift is introduced.
// 0b0000 0000  1000 0000  1000 0000  1000 0000
//static const uint8_t phase_pattern = 0x000029a0;
//static const uint32_t phase_pattern = 0x00008208;
//static const uint32_t phase_pattern = 0x00000808;
static uint32_t phase_pattern = 0x0020;

static volatile int32_t cycle_number = -1;

// The period (1/f) of the pulse
static uint16_t center_freq_period=600;

#define BAUD_RATE 460800

#define CAPTURE_ECHO_WAVEFORM
//#define CAPTURE_ECHO_ENVELOPE

#ifdef CAPTURE_ECHO_ENVELOPE
#define ADC_CHANNEL 9
#define ADC_SAMPLE_RATE 20000
#define ADC_BUFFER_SIZE 500
#endif

#ifdef CAPTURE_ECHO_WAVEFORM
#define ADC_CHANNEL 3
#define ADC_SAMPLE_RATE 240000
#define ADC_BUFFER_SIZE 2000
#endif

static uint16_t adc_buffer[ADC_BUFFER_SIZE];
static volatile uint32_t adc_count;

/* Size of the source and destination buffers in 32-bit words.
   Allowable values  = 128, 256, 512, or 1024 */
#define DMA_BUFFER_SIZE            (1024)


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
 * Pulse debugging pin to indicate an event on a oscilloscope trace.
 *
 */
static void debug_pin_pulse (int n)
{
	int i;
	for (i = 0; i < n; i++) {
		Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0, 14, true);
		Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0, 14, false);
	}
}

/**
 * @brief	Handle interrupt from State Configurable Timer
 * @return	Nothing
 */
void SCT_IRQHandler(void)
{

	debug_pin_pulse (2);

#define PSK
#ifdef PSK
	int half_period = center_freq_period/2;
	Chip_SCT_SetMatchReload(LPC_SCT, SCT_MATCH_2, half_period);
	if (phase_pattern & (1<<cycle_number) ) {
		Chip_SCT_SetMatchReload(LPC_SCT, SCT_MATCH_0, center_freq_period + half_period);
	} else {
		Chip_SCT_SetMatchReload(LPC_SCT, SCT_MATCH_0, center_freq_period);
	}
#endif

//#define CHIRP
#ifdef CHIRP
	int half_period = center_freq_period/2;
	Chip_SCT_SetMatchReload(LPC_SCT, SCT_MATCH_2, half_period - cycle_number*2);
	Chip_SCT_SetMatchReload(LPC_SCT, SCT_MATCH_0, center_freq_period - cycle_number*4);
#endif

	cycle_number++;

	// +2 because we are setting the RELOAD register, also because
	// the first entry into interrupt is due to starter pulse
	if (cycle_number == num_cycles+2) {

		// Pulse is finished. Now switch to using SCT to trigger
		// ADC samples.

		// Disable the toggling of the the TX driver pins SCT0_OUT0, SCT0_OUT1
		// on Event0 or Event2.
		// This causes slightly different pulse pattern (at start of pulse). Why?
		//LPC_SCT->OUT[0].SET = 0;
		//LPC_SCT->OUT[0].CLR = 0;
		//LPC_SCT->OUT[1].SET = 0;
		//LPC_SCT->OUT[1].CLR = 0;

		// SwitchMatrix: Unassign TX driver pins SCT_OUT0, SCT_OUT1
		Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_SWM);
		Chip_SWM_MovablePinAssign(SWM_SCT_OUT0_O, 0xff);
		Chip_SWM_MovablePinAssign(SWM_SCT_OUT1_O, 0xff);
		Chip_Clock_DisablePeriphClock(SYSCTL_CLOCK_SWM);

		// No need for SCT interrupt during ADC phase.
		NVIC_DisableIRQ(SCT_IRQn);

		// Signal to main loop that pulse is complete
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

	/* Get pending interrupts */
	uint32_t pending = Chip_ADC_GetFlags(LPC_ADC);

	//debug_pin_pulse(1);

	//adc_buffer[adc_count++] = (Chip_ADC_GetDataReg(LPC_ADC,3)>>4) & 0xfff;
	adc_buffer[adc_count++] = Chip_ADC_GetDataReg(LPC_ADC,ADC_CHANNEL);

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
	debug_pin_pulse(8);
	Chip_ADC_ClearFlags(LPC_ADC, 1<<24); // Clear SEQA_OVR
}


static volatile bool dmaDone;
/**
 * @brief	DMA Interrupt Handler
 * @return	None
 */
void DMA_IRQHandler(void)
{

	debug_pin_pulse (64);

	/* Clear DMA interrupt for the channel */
	Chip_DMA_ClearActiveIntAChannel(LPC_DMA, DMA_CH0);

	dmaDone = true;
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

/**
 * Send printf() to UART
 */
int __sys_write(int fileh, char *buf, int len) {
	Chip_UART_SendBlocking(LPC_USART0, buf,len);
	return len;
}
/**
 * Initialize Multirate Timer (MRT)
 */
void mrt_init () {
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
}

/**
 * Initialize ADC for use.
 */
void adc_init () {

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
	   Perform ADC conversion of ADC channel ADC_CHANNEL only */
	Chip_ADC_SetupSequencer(LPC_ADC,
							ADC_SEQA_IDX,
							(ADC_SEQ_CTRL_CHANSEL(ADC_CHANNEL)
							| (3<<12) // SCT0_OUT3 see UM10800 §21.3.3
							| ADC_SEQ_CTRL_MODE_EOS
							)
									);

	/* Enable fixed pin ADC3, ADC9 with SwitchMatrix */
	Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_SWM);
	Chip_SWM_EnableFixedPin(SWM_FIXED_ADC3);
	Chip_SWM_EnableFixedPin(SWM_FIXED_ADC9);
	Chip_Clock_DisablePeriphClock(SYSCTL_CLOCK_SWM);

	/* Clear all pending interrupts */
	Chip_ADC_ClearFlags(LPC_ADC, Chip_ADC_GetFlags(LPC_ADC));

	/* Enable ADC overrun and sequence A completion interrupts */

	// This has impact on DMA operation. Why?
	Chip_ADC_EnableInt(LPC_ADC, (ADC_INTEN_SEQA_ENABLE
								//| ADC_INTEN_OVRRUN_ENABLE
								));


	// ADC interrupt disabled for DMA
	//NVIC_EnableIRQ(ADC_SEQA_IRQn);
	//NVIC_EnableIRQ(ADC_OVR_IRQn);

	/* Enable sequencer */
	Chip_ADC_EnableSequencer(LPC_ADC, ADC_SEQA_IDX);
}

/**
 * Start ultrasonic TX pulse. SCT IRQ handler will set cycle_number to -1
 * when complete.
 */
void start_pulse (int freq) {

	center_freq_period = SystemCoreClock / freq;

	// Setup SCT
	Chip_SCT_Init(LPC_SCT);

	/* Stop the SCT before configuration */
	Chip_SCTPWM_Stop(LPC_SCT);

	/* Set MATCH0 for max limit */
	// Match/capture mode register. (ref UM10800 section 16.6.11, Table 232, page 273)
	// Determines if match/capture operate as match or capture. Want all match.
	LPC_SCT->REGMODE_U = 0;

	cycle_number = 0;


	// Event 0 control: (ref UM10800 section 16.6.25, Table 247, page 282).
	// set MATCHSEL (bits 3:0) = MATCH0 register(0)
	// set COMBMODE (bits 13:12)= MATCH only(1)
	// So Event0 is triggered on match of MATCH0
	LPC_SCT->EV[0].CTRL =   (0 << 0 )
							| 1 << 12;
	// Event enable register (ref UM10800 section 16.6.24, Table 246, page 281)
	// Enable Event0 in State0 (default state). We are not using states (I think),
	// so this enables Event0 in the default State0.
	// Set STATEMSK0=1
	LPC_SCT->EV[0].STATE = 1<<0;


	// Configure Event2 to be triggered on Match2.
	int ix = 2;
	LPC_SCT->EV[ix].CTRL = ix | (1 << 12);
	LPC_SCT->EV[ix].STATE = 1;
	LPC_SCT->OUT[0].SET = 1<<0; // Event 0 sets SCT0_OUT[pin]
	LPC_SCT->OUT[0].CLR = 1<<ix;  // Event ix clears SCT0_OUT[pin]

	// Experimental push-pull driver for ultrasound TX
	// Want SCT0_OUT1 to be logic NOT of SCT0_OUT0
	// This is working now (mostly). Start of pulse not
	// right. Also being triggered by ADC sampling.
	LPC_SCT->OUT[1].CLR = 1<<0; // Event 0 clears SCT0_OUT1
	LPC_SCT->OUT[1].SET = 1 << ix;  // Event ix sets SCT0_OUT1

	/* Clear the output in-case of conflict */
	int pin = 0;
	LPC_SCT->RES = (LPC_SCT->RES & ~(3 << (pin << 1))) | (0x01 << (pin << 1));

	/* Set and Clear do not depend on direction */
	LPC_SCT->OUTPUTDIRCTRL = (LPC_SCT->OUTPUTDIRCTRL & ~((3 << (pin << 1))|SCT_OUTPUTDIRCTRL_RESERVED));


	/* Set SCT Counter to count 32-bits and reset to 0 after reaching MATCH0 */
	Chip_SCT_Config(LPC_SCT, SCT_CONFIG_32BIT_COUNTER | SCT_CONFIG_AUTOLIMIT_L);
	/* Enable flag to request an interrupt for Event 0 */
	Chip_SCT_EnableEventInt(LPC_SCT, SCT_EVT_0);
	/* Enable the interrupt for the SCT */
	NVIC_EnableIRQ(SCT_IRQn);

	// Setup ADC stuff
	adc_count = 0;

	// Start SCT

	// SwitchMatrix: Assign SCT_OUT0 to PIO0_15
	Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_SWM);
	Chip_SWM_MovablePinAssign(SWM_SCT_OUT0_O, 15);
	Chip_SWM_MovablePinAssign(SWM_SCT_OUT1_O, 9);
	Chip_Clock_DisablePeriphClock(SYSCTL_CLOCK_SWM);


	Chip_SCT_ClearControl(LPC_SCT, SCT_CTRL_HALT_L | SCT_CTRL_HALT_H);

}

/**
 * Does not work :(
 */
void adc_dma_capture () {

	// Setup DMA for ADC

	/* DMA initialization - enable DMA clocking and reset DMA if needed */
	Chip_DMA_Init(LPC_DMA);
	/* Enable DMA controller and use driver provided DMA table for current descriptors */
	Chip_DMA_Enable(LPC_DMA);
	Chip_DMA_SetSRAMBase(LPC_DMA, DMA_ADDR(Chip_DMA_Table));

	/* Setup channel 0 for the following configuration:
	   - High channel priority
	   - Interrupt A fires on descriptor completion */
	Chip_DMA_EnableChannel(LPC_DMA, DMA_CH0);
	Chip_DMA_EnableIntChannel(LPC_DMA, DMA_CH0);
	Chip_DMA_SetupChannelConfig(LPC_DMA, DMA_CH0,
			(DMA_CFG_HWTRIGEN
					//| DMA_CFG_PERIPHREQEN  //?? what's this for???
					| DMA_CFG_TRIGTYPE_EDGE
					| DMA_CFG_TRIGPOL_HIGH
					| DMA_CFG_TRIGBURST_BURST
					| DMA_CFG_BURSTPOWER_1
					 | DMA_CFG_CHPRIORITY(0)
					 ));

	// Attempt to use ADC SEQA to trigger DMA xfer
	Chip_DMATRIGMUX_SetInputTrig(LPC_DMATRIGMUX, DMA_CH0, DMATRIG_ADC_SEQA_IRQ);

	DMA_CHDESC_T dmaDesc;

	/* DMA descriptor for memory to memory operation - note that addresses must
	   be the END address for src and destination, not the starting address.
	     DMA operations moves from end to start. */
	//dmaDesc.source = DMA_ADDR(&src[SIZE_BUFFERS - 1]) + 3;
	dmaDesc.source = DMA_ADDR ( (&LPC_ADC->DR[ADC_CHANNEL]) ); // ADC data register is source
	//dmaDesc.source = DMA_ADDR ( &systick_counter ); // works!
	//dmaDesc.source = DMA_ADDR ( & LPC_SCT->COUNT_U ); // ADC data register is source

	dmaDesc.dest = DMA_ADDR(&adc_buffer[DMA_BUFFER_SIZE - 1]) ;
	dmaDesc.next = DMA_ADDR(0);

	/* Enable DMA interrupt */
	NVIC_EnableIRQ(DMA_IRQn);

	/* Setup transfer descriptor and validate it */
	Chip_DMA_SetupTranChannel(LPC_DMA, DMA_CH0, &dmaDesc);
	Chip_DMA_SetValidChannel(LPC_DMA, DMA_CH0);

	/* Setup data transfer and software trigger in same call */
	// See "Transfer Configuration registers" table 173 §12.6.18 page 179
	Chip_DMA_SetupChannelTransfer(LPC_DMA, DMA_CH0,
			 (
				DMA_XFERCFG_CFGVALID  // Channel descriptor is considered valid
				| DMA_XFERCFG_SETINTA //
				//| DMA_XFERCFG_SWTRIG  // When written by software, the trigger for this channel is set immediately.
				| DMA_XFERCFG_WIDTH_16 // 8,16,32 bits allowed
				| DMA_XFERCFG_SRCINC_0 // do not increment source
				| DMA_XFERCFG_DSTINC_1 // increment dest by widthx1
				| DMA_XFERCFG_XFERCOUNT(DMA_BUFFER_SIZE)
				)
				);


	// Setup SCT for ADC/DMA sample timing. Additional config was setup in
	// adc_init()
	uint32_t clock_hz =  Chip_Clock_GetSystemClockRate();
	Chip_SCT_SetMatchReload(LPC_SCT, SCT_MATCH_2, (clock_hz/ADC_SAMPLE_RATE)/2 );
	Chip_SCT_SetMatchReload(LPC_SCT, SCT_MATCH_0,  clock_hz/ADC_SAMPLE_RATE);
	// Using SCT0_OUT3 to trigger ADC sampling
	// Set SCT0_OUT3 on Event0 (Event0 configured to occur on Match0)
	LPC_SCT->OUT[3].SET = 1;
	// Clear SCT0_OUT3 on Event 2 (Event2 configured to occur on Match2)
	LPC_SCT->OUT[3].CLR = 1 << 2;

	//LPC_SCT->DMAREQ0 = (1<<30) | 1;
	//DMATRIG_ADC_SEQA_IRQ;

}

/**
 * Capture echo data using tight poll loop.
 */
void adc_poll_loop_capture () {

	// Use SCT to time ADC samples.
	Chip_SCT_SetMatchReload(LPC_SCT, SCT_MATCH_2, 12000000/ADC_SAMPLE_RATE);
	Chip_SCT_SetMatchReload(LPC_SCT, SCT_MATCH_0, 24000000/ADC_SAMPLE_RATE);

	// Using SCT0_OUT3 to trigger ADC sampling
	Chip_SCTPWM_SetOutPin(LPC_SCT,
			2, // PWM channel
			3 //  the output channel eg SCT_OUT3 (there are 6 in total)
		);


	// Tight loop poll to get ADC samples. We sould disable interrupts.
	int i;
	for (i = 0; i < ADC_BUFFER_SIZE; i++) {
		while ( LPC_ADC->DR[3] & (1<<31) == 0 ) ;
		adc_buffer[i] = (uint16_t)LPC_ADC->DR[3];
		debug_pin_pulse(1);
	}

	// Shift ADC data. We do this outside of capture loop to keep the
	// capture loop as fast as possible.
	for (i = 0; i < ADC_BUFFER_SIZE; i++) {
		adc_buffer[i] >>= 4;
	}

	// For interrupt method
	adc_count = 0;
}
/**
 * Capture echo data using ADC interrupt. This works, but cannot capture at rates
 * much above 240ksps.
 */
void adc_interrupt_capture () {

	// Use SCT to time ADC samples.
	uint32_t clock_hz =  Chip_Clock_GetSystemClockRate();
	Chip_SCT_SetMatchReload(LPC_SCT, SCT_MATCH_2, (clock_hz/ADC_SAMPLE_RATE)/2 );
	Chip_SCT_SetMatchReload(LPC_SCT, SCT_MATCH_0,  clock_hz/ADC_SAMPLE_RATE);

	// Using SCT0_OUT3 to trigger ADC sampling
/*
	Chip_SCTPWM_SetOutPin(LPC_SCT,
			2, // PWM channel
			3 //  the output channel eg SCT_OUT3 (there are 6 in total)
		);
*/

	// Set SCT0_OUT3 on Event0 (Event0 configured to occur on Match0)
	LPC_SCT->OUT[3].SET = 1;
	// Clear SCT0_OUT3 on Event 2 (Event2 configured to occur on Match2)
	LPC_SCT->OUT[3].CLR = 1 << 2;



	adc_count = 0;
	NVIC_EnableIRQ(ADC_SEQA_IRQn);

	// Wait for buffer to fill. Sleep as much as possible in the mean time.
	while (adc_count < ADC_BUFFER_SIZE) {
		__WFI();
	}

	// Signal that ADC is over (for debugging)
	debug_pin_pulse(64);


	// Shift ADC data. We do this outside of the ISR to keep the ISR duration
	// as short as possible.
	int i;
	for (i = 0; i < ADC_BUFFER_SIZE; i++) {
		adc_buffer[i] >>= 4;
	}

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
	// This causes a hang. Why? Maybe it's trying to use (non existant)
	// external crystal?
	// DISABLE:
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
	Chip_Clock_SetUSARTNBaseClockRate((BAUD_RATE * 16), true);

	//Chip_UART_SetBaud(LPC_USART0, 115200);
	Chip_UART_SetBaud(LPC_USART0, BAUD_RATE);

	Chip_UART_Enable(LPC_USART0);
	Chip_UART_TXEnable(LPC_USART0);

	printf ("System clock rate: %d\r\n", Chip_Clock_GetSystemClockRate());

	/* Initialize GPIO */
	Chip_GPIO_Init(LPC_GPIO_PORT);
	Chip_GPIO_SetPinDIROutput(LPC_GPIO_PORT, 0, 15);
	Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0, 15, true);

	// For debugging
	Chip_GPIO_SetPinDIROutput(LPC_GPIO_PORT, 0, 14);
	Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0, 14, true);


#define ENABLE_ADC
#ifdef ENABLE_ADC
	adc_init();
#endif


	//
	// Multi Rate Timer (MRT)
	//
#ifdef ENABLE_MRT
	mrt_init();
#endif


	/* Enable SysTick Timer */
	SysTick_Config(SystemCoreClock / TICKRATE_HZ);

	// Call one time initialization of ADC
	adc_init();

	uint32_t start_time;
	uint32_t t;

	/* Loop forever */
	while (1) {
		__WFI();

		// Repeat pulse at
		t = systick_counter;
		if ( (cycle_number== -1) && ((t%(TICKRATE_HZ/5))==0) && (t!=start_time) ) {
			start_time = t;

			start_pulse(40000);

			// Wait for end of pulse
			while (cycle_number != -1) {
				__WFI();
			};

			// Capture ADC values using DMA
			dmaDone=false;
			adc_dma_capture();
			while (!dmaDone) {
				__WFI();
			}

			int i;

			for (i =0; i < DMA_BUFFER_SIZE; i++) {
				//printf ("%x ",(adc_buffer[i]>>4)&0xfff);
				adc_buffer[i] >>= 4;
				printf ("%c%c",mime64_encoding_table[(adc_buffer[i]>>6)&0x3f],
								mime64_encoding_table[adc_buffer[i]&0x3f]);
			}
			printf ("\r\n");
			continue;

			// Capture ADC values in tight loop
			//adc_poll_loop_capture();

			// Capture ADC values using ADC ISR (only timing reliable way so far!)
			//adc_interrupt_capture();




			uint16_t v;
			uint16_t adc_min = 0xfff;
			uint16_t adc_max = 0;
			uint16_t adc_max_index, adc_min_index;
			uint32_t sum=0;
			uint64_t sum2=0;
			uint16_t top_envelope;
			uint16_t peak_value=0;
			uint16_t peak_index=0;
			for (i = 0; i < ADC_BUFFER_SIZE; i++) {
				v = adc_buffer[i];
				sum += v;
				sum2 += v*v;
				if (v > adc_max) {
					adc_max = v;
					adc_max_index = i;
				}
				if (v < adc_min) {
					adc_min = v;
					adc_min_index = i;
				}

				// Envelope detector using leaky integrator
				if (v>top_envelope) {
					top_envelope = v;
				} else {
					top_envelope -= top_envelope/16;
				}

				if (top_envelope > peak_value) {
					peak_value = top_envelope;
					peak_index = i;
				}
			}
			uint16_t mean_value = sum / ADC_BUFFER_SIZE;
			uint32_t mean_power = sum2 / ADC_BUFFER_SIZE - mean_value*mean_value;

			// Max pulse width in sample periods (about 1ms)
			const int max_pulse_width = 200;
			int search_start = peak_index > max_pulse_width ?  peak_index - max_pulse_width : 0;
			int search_end = peak_index < ADC_BUFFER_SIZE-max_pulse_width ? peak_index+max_pulse_width : ADC_BUFFER_SIZE;

			// Find start of pulse
			int threshold = peak_value / 2;
			int echo_start, echo_end;
			for (i = search_start; i < peak_index; i++) {
				v = adc_buffer[i];
				if (v > threshold) {
					echo_start = i;
					break;
				}
			}
			for (i = search_end-1; i >= peak_index; i--) {
				v = adc_buffer[i];
				if (v > threshold) {
					echo_end = i;
					break;
				}
			}
			int pulse_width = echo_end - echo_start;

			/*
			printf ("%d %d %d %d %d %d %d %d",
					phase_pattern, mean_power,
					adc_min, adc_max, adc_max_index,
					pulse_width,
					echo_start, echo_end
					);
			*/

			for (i = 0; i < ADC_BUFFER_SIZE; i++) {
				printf ("%c%c",mime64_encoding_table[(adc_buffer[i]>>6)&0x3f],
								mime64_encoding_table[adc_buffer[i]&0x3f]);
			}

			printf ("\r\n");

			// Signal that UART dump is complete
			debug_pin_pulse(32);
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
