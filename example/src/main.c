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

// Number of pings per second.
#define PULSE_RATE_HZ 10


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

//#define BAUD_RATE 9600
#define BAUD_RATE 460800

// What to capture
#define CAPTURE_ECHO_WAVEFORM
//#define CAPTURE_ECHO_ENVELOPE

// How to capture
#define CAPTURE_WITH_DMA
//#define CAPTURE_WITH_LOOP
//#define CAPTURE_WITH_INTERRUPT


#ifdef CAPTURE_ECHO_ENVELOPE
#define ADC_CHANNEL 9
#define ADC_SAMPLE_RATE 20000
#define ADC_BUFFER_SIZE 500
#endif

#ifdef CAPTURE_ECHO_WAVEFORM
#define ADC_CHANNEL 3
// Tested with up to 720ksps with DMA
// 240ksps : 6 samples per 40kHz cycle.
#define ADC_SAMPLE_RATE 240000
#define ADC_BUFFER_SIZE 3072
#endif

// Pulse compression technique (PSK180 | CHIRP)
#define PULSE_COMPRESSION_PSK
//#define PULSE_COMPRESION_CHIRP


static uint16_t adc_buffer[ADC_BUFFER_SIZE];
static volatile uint32_t adc_count;

/* Size of the source and destination buffers in 32-bit words.
   Allowable values  = 128, 256, 512, or 1024 */
#define DMA_BUFFER_SIZE            (1024)


#define MODE_WAVEFORM_OUT (1<<0)
#define MODE_ENVELOPE_OUT (1<<1)
#define MODE_ENVELOPE_FIXPT_OUT (1<<2)

static volatile uint32_t mode_flags = MODE_ENVELOPE_OUT | MODE_ENVELOPE_FIXPT_OUT;

// Base64 encode table
static char base64_encoding_table[] = {'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H',
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


#ifdef PULSE_COMPRESSION_PSK
	int half_period = center_freq_period/2;
	Chip_SCT_SetMatchReload(LPC_SCT, SCT_MATCH_2, half_period);
	if (phase_pattern & (1<<cycle_number) ) {
		Chip_SCT_SetMatchReload(LPC_SCT, SCT_MATCH_0, center_freq_period + half_period);
	} else {
		Chip_SCT_SetMatchReload(LPC_SCT, SCT_MATCH_0, center_freq_period);
	}
#endif

//#define CHIRP
#ifdef PULSE_COMPRESSION_CHIRP
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

	debug_pin_pulse (8);

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

DMA_CHDESC_T dmaDescA;
DMA_CHDESC_T dmaDescB;
DMA_CHDESC_T dmaDescC;

/**
 * Capture ADC data using DMA.
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

	// DMA is performed in 3 separate chunks (as max allowed in one transfer
	// is 1024 words). First to go is dmaDescA followed by B and C.

	// DMA descriptor for ADC to memory - note that addresses must
	// be the END address for src and destination, not the starting address.
	// DMA operations moves from end to start.
	dmaDescC.xfercfg = 			 (
			DMA_XFERCFG_CFGVALID  // Channel descriptor is considered valid
			| DMA_XFERCFG_SETINTA // DMA Interrupt A (A vs B can be read in ISR)
			//| DMA_XFERCFG_SWTRIG  // When written by software, the trigger for this channel is set immediately.
			| DMA_XFERCFG_WIDTH_16 // 8,16,32 bits allowed
			| DMA_XFERCFG_SRCINC_0 // do not increment source
			| DMA_XFERCFG_DSTINC_1 // increment dst by widthx1
			| DMA_XFERCFG_XFERCOUNT(DMA_BUFFER_SIZE)
			);
	dmaDescC.source = DMA_ADDR ( (&LPC_ADC->DR[ADC_CHANNEL]) );
	dmaDescC.dest = DMA_ADDR(&adc_buffer[DMA_BUFFER_SIZE*3 - 1]) ;
	dmaDescC.next = DMA_ADDR(0);

	dmaDescB.xfercfg = 			 (
			DMA_XFERCFG_CFGVALID  // Channel descriptor is considered valid
			| DMA_XFERCFG_RELOAD  // Causes DMA to move to next descriptor when complete
			| DMA_XFERCFG_SETINTA // DMA Interrupt A (A vs B can be read in ISR)
			//| DMA_XFERCFG_SWTRIG  // When written by software, the trigger for this channel is set immediately.
			| DMA_XFERCFG_WIDTH_16 // 8,16,32 bits allowed
			| DMA_XFERCFG_SRCINC_0 // do not increment source
			| DMA_XFERCFG_DSTINC_1 // increment dst by widthx1
			| DMA_XFERCFG_XFERCOUNT(DMA_BUFFER_SIZE)
			);
	dmaDescB.source = DMA_ADDR ( (&LPC_ADC->DR[ADC_CHANNEL]) );
	dmaDescB.dest = DMA_ADDR(&adc_buffer[DMA_BUFFER_SIZE*2 - 1]) ;
	dmaDescB.next = &dmaDescC;

	// ADC data register is source of DMA
	dmaDescA.source = DMA_ADDR ( (&LPC_ADC->DR[ADC_CHANNEL]) );
	dmaDescA.dest = DMA_ADDR(&adc_buffer[DMA_BUFFER_SIZE - 1]) ;
	//dmaDesc.next = DMA_ADDR(0);
	dmaDescA.next = &dmaDescB;



	// Enable DMA interrupt. Will be invoked at end of DMA transfer.
	NVIC_EnableIRQ(DMA_IRQn);

	/* Setup transfer descriptor and validate it */
	Chip_DMA_SetupTranChannel(LPC_DMA, DMA_CH0, &dmaDescA);
	Chip_DMA_SetValidChannel(LPC_DMA, DMA_CH0);

	// Setup data transfer and hardware trigger
	// See "Transfer Configuration registers" UM10800, §12.6.18, Table 173, page 179
	Chip_DMA_SetupChannelTransfer(LPC_DMA, DMA_CH0,
			 (
				DMA_XFERCFG_CFGVALID  // Channel descriptor is considered valid
				| DMA_XFERCFG_RELOAD  // Causes DMA to move to next descriptor when complete
				| DMA_XFERCFG_SETINTA // DMA Interrupt A (A vs B can be read in ISR)
				//| DMA_XFERCFG_SWTRIG  // When written by software, the trigger for this channel is set immediately.
				| DMA_XFERCFG_WIDTH_16 // 8,16,32 bits allowed
				| DMA_XFERCFG_SRCINC_0 // do not increment source
				| DMA_XFERCFG_DSTINC_1 // increment dst by widthx1
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

}

/**
 * Capture echo data using tight poll loop. Not working right.
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


	// Tight loop poll to get ADC samples. We should disable interrupts.
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
 * @brief Soft floating point envelope detection.
 *
 * Using formula
 * y(n+1) = x(n) > y(n) ? x(n) : A * y(n)
 * formula. A = 0.95
 *
 * Note: not efficient. Observing about 200us of calculation time for each point.
 *
 */
void envelope_detect_softfloat(int adc_mean) {
	int i;
	float top_envelope_f = 0;
	float v;

	printf("E ");


	uint32_t ma[6];
	int ma_head = 0;
	int ma_sum = 0;
	for (i = 0; i < DMA_BUFFER_SIZE * 3; i++) {
		v = (float) (adc_buffer[i] - adc_mean);
		if (v > top_envelope_f) {
			top_envelope_f = v;
		} else {
			//top_envelope_f -= top_envelope_f * 0.05;
			top_envelope_f *= 0.95;
		}
		int x = (int) top_envelope_f;
		if (x < 0) {
			x = 0;
		}

		ma_sum -= ma[ma_head];
		ma_sum += x;

		ma[ma_head] = x;
		ma_head++;
		if (ma_head == 6)
			ma_head = 0;

		if (i % 6 == 0) {
			x = ma_sum / 6;
			x += adc_mean; // why does this needed?
			printf("%c%c", base64_encoding_table[(x >> 6) & 0x3f],
					base64_encoding_table[x & 0x3f]);
		}
	}
	printf("\r\n");
}

/**
 * @brief Envelope detection using peak detection.
 *
 * Relies on a reasonable number of samples per cycle
 *
 */
void envelope_detect_peakdetect(int adc_mean) {
	int i,j=0;
	uint32_t y=0;

	printf("e ");

	for (i = 1; i < DMA_BUFFER_SIZE * 3 -1; i++) {

		if (adc_buffer[i] > adc_mean) {
			if (adc_buffer[i-1] < adc_buffer[i] && adc_buffer[i] > adc_buffer[i+1]) {
				// Peak
				y = adc_buffer[i];
			}
		}

		if (++j == 6) {
			printf("%c%c", base64_encoding_table[(y >> 6) & 0x3f],
					base64_encoding_table[y & 0x3f]);
			j = 0;
		}
	}
	printf("\r\n");
}


/**
 * @brief Fixed point arithmetic envelope detection.
 *
 * Using formula
 * y(n+1) = x(n) > y(n) ? x(n) : A * y(n)
 * formula. A = 0.95
 *
 * Using signed fixed point arithmetic with 8 fractional bits (Q23.8)
 *
 * Note: big improvement on soft floating point, with about 100us of calculation time
 * for each output point.
 *
 * Note: currently does not work.
 *
 */
void envelope_detect_fixedpoint(int adc_mean) {
	int i;
	uint32_t x,y=0;

	printf("e ");


	uint32_t ma[6];
	int ma_head = 0;
	int ma_sum = 0;
	for (i = 0; i < DMA_BUFFER_SIZE * 3; i++) {
		// Subtract ADC mean
		x =  (adc_buffer[i] - adc_mean);
		if (x<0) {
			x= 0;
		}
		x *= 256;

		//y = x > y ? x : y-y>>4;
		if (x > y) {
			y = x;
		} else {
			// y -= y/16 or floating point equivalent: y *= 0.9375;
			y -= y>>4;
		}

		// Moving average of 6 points. Using 6 because there are 6 samples per cycle.
		// Using 8 would make math more efficient though.
		ma_sum -= ma[ma_head];
		ma_sum += y;

		ma[ma_head] = y;
		ma_head++;
		if (ma_head == 6) {
			ma_head = 0;
		}

		// Was using (i%6==0) but changed to (ma_head==0) which will have the same effect
		// and saved 30us off the 100us to 70us per envelope output point. And this means
		// we can range an 10Hz instead of 5Hz. Such a small change makes a big difference! :)
		if (ma_head == 0) {
			x = ma_sum / 6;
			// Convert back to regular integer from fixed point
			x /= 256;
			//x += adc_mean; // why does this needed?
			printf("%c%c", base64_encoding_table[(x >> 6) & 0x3f],
					base64_encoding_table[x & 0x3f]);
		}
	}
	printf("\r\n");
}


/**
 * @brief	Main routine for ultrasound experiment.
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
	Chip_UART_ConfigData(LPC_USART0,
			UART_CFG_DATALEN_8
			| UART_CFG_PARITY_NONE
			| UART_CFG_STOPLEN_1);

	Chip_Clock_SetUSARTNBaseClockRate((BAUD_RATE * 16), true);
	Chip_UART_SetBaud(LPC_USART0, BAUD_RATE);


	/* Enable receive data interrupt */
	Chip_UART_IntEnable(LPC_USART0, UART_INTEN_RXRDY);

	Chip_UART_TXEnable(LPC_USART0);
	Chip_UART_Enable(LPC_USART0);

	NVIC_EnableIRQ(UART0_IRQn);


	// Now that UART is initialized, print hello message
	printf ("System clock rate: %d\r\n", Chip_Clock_GetSystemClockRate());
	printf ("SysTick rate: %d\r\n", TICKRATE_HZ);

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

	int adc_mean, adc_min = 0xffff, adc_max = 0;


	/* Loop forever */
	while (1) {
		__WFI();

		// Repeat pulse at
		t = systick_counter;
		if ( (cycle_number== -1) && ((t%(TICKRATE_HZ/PULSE_RATE_HZ))==0) && (t!=start_time) ) {
			start_time = t;

			start_pulse(40000);

			// Wait for end of pulse
			while (cycle_number != -1) {
				__WFI();
			};


#ifdef CAPTURE_WITH_DMA

			// Capture ADC values using DMA
			dmaDone=false;
			adc_dma_capture();
			while (!dmaDone) {
				__WFI();
			}

			// TODO: dmaDone needs to be fixed. Delay instead.
			int waitUntil = systick_counter+2;
			while (systick_counter<waitUntil) {
				__WFI();
			}

#endif

			debug_pin_pulse(64);

			int i;
			for (i =0; i < DMA_BUFFER_SIZE*3; i++) {
				adc_buffer[i] >>= 4;
			}


			if (mode_flags & (MODE_ENVELOPE_OUT | MODE_ENVELOPE_FIXPT_OUT) ) {

				// Calculate mean, min, max

				long sum = 0;
				for (i = 0; i < DMA_BUFFER_SIZE * 3; i++) {
					if (adc_buffer[i] > adc_max)
						adc_max = adc_buffer[i];
					if (adc_buffer[i] < adc_min)
						adc_min = adc_buffer[i];
					sum += adc_buffer[i];
				}
				adc_mean = sum / (DMA_BUFFER_SIZE * 3);

			}
				// Envelope
				// TODO: Current implementation has 300us gap between
				// each data point. This must be optimised. Eg use integer
				// math. At 460800bps each data point (2 Base64 chars/bytes)
				// takes about 40us to transmit.

			if (mode_flags & MODE_ENVELOPE_OUT) {
				envelope_detect_softfloat(adc_mean);
			}
			if (mode_flags & MODE_ENVELOPE_FIXPT_OUT) {
				//envelope_detect_fixedpoint(adc_mean);
				envelope_detect_peakdetect(adc_mean);
			}

			// Waveform
			if (mode_flags & MODE_WAVEFORM_OUT) {
				printf ("W ");
				for (i =0; i < DMA_BUFFER_SIZE*3; i++) {
					printf ("%c%c",base64_encoding_table[(adc_buffer[i]>>6)&0x3f],
								base64_encoding_table[adc_buffer[i]&0x3f]);
				}
				printf ("\r\n");
			}

			debug_pin_pulse(64);




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

/**
 * @brief	Handle UART interrupt
 * @return	Nothing
 */
void UART0_IRQHandler(void)
{
	uint32_t uart_status = LPC_USART0->STAT;

	// LPC81x: UM10601 §15.6.3, Table 162, p181. USART Status Register.
	// LPC82x: UM10800 §13.6.3, Table 178, p193.
	// Bit 0 RXRDY: 1 = data is available to be read from RXDATA
	// Bit 2 TXRDY: 1 = data may be written to TXDATA
	if (uart_status & UART_STAT_RXRDY ) {
		uint8_t c = LPC_USART0->RXDATA;
		switch (c) {
		case 'E':
			mode_flags |= MODE_ENVELOPE_OUT;
			break;
		case 'e':
			mode_flags &= ~MODE_ENVELOPE_OUT;
			break;
		case 'W':
			mode_flags |= MODE_WAVEFORM_OUT;
			break;
		case 'w':
			mode_flags &= ~MODE_WAVEFORM_OUT;
			break;
		case 'I':
			mode_flags |= MODE_ENVELOPE_FIXPT_OUT;
			break;
		case 'i':
			mode_flags &= ~MODE_ENVELOPE_FIXPT_OUT;
			break;
		}
	} else if (uart_status & UART_STAT_TXRDY ){
		LPC_USART0->INTENCLR = 1<<2; // TXRDYCLR
	}
}

