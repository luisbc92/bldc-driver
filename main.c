#include <msp430.h> 

/*
 * Definitions
 */

// Drivers
#define	AH		BIT0
#define BH		BIT1
#define CH		BIT2
#define AL		BIT3
#define BL		BIT4
#define CL		BIT5
// Feedback
#define	AFC		INCH_0
#define BFC		INCH_1
#define CFC		INCH_2
#define AF		BIT0
#define BF		BIT1
#define CF		BIT2
#define VREF	BIT4
#define	POS		1		// Positive Slope
#define NEG		0		// Negative Slop
// LUTs
static const char phase_high[] = { BH, CH, CH, AH, AH, BH };
static const char phase_low[] =  { AL, AL, BL, BL, CL, CL };
static const int  bemf_phase[] = { CFC, BFC, AFC, CFC, BFC, AFC };
static const char bemf_pol[] =   { POS, NEG, POS, NEG, POS, NEG };
// Variables
unsigned int comm_step = 0;			// Commutation step
unsigned int comm_time = 0;			// Commutation half-time (filtered)

/*
 * Configuration
 */
#define ZC_DELAY		1000		// Zero-Cross Delay (Time to wait before checking ZC to eliminate HF noise at low speeds)
#define ZC_ACCEL		50			// Zero-Cross Delay Minimization Range (Allows delay to be decreased at high speeds)
#define ZC_TRESHOLD		512			// Zero-Cross ADC Treshold (Value at which ZC happens)
#define IIR_A			1			// Commutation Filter Coefficient for Current Reading
#define IIR_B			3			// Commutation Filter Coefficient for Accumulation



/*
 * main.c
 */
int main(void) {
	// Stop
    WDTCTL = WDTPW | WDTHOLD;

    // Clock @ 16 MHz
    DCOCTL = 0;
    BCSCTL1 = CALBC1_16MHZ;
    DCOCTL = CALDCO_16MHZ;
    BCSCTL2 = 0;
    BCSCTL3 = 0;

    // Port 2 Outputs
    P2OUT = 0;
    P2DIR = AH|BH|CH|AL|BL|CL;

    // Zero-Crossing - ADC10
    // Positive Reference - VREF
    // Sample and Hold - 8xCLKs
    ADC10AE0 = AF|BF|CF|VREF;
    ADC10CTL0 = SREF_3 | ADC10SHT_1 | MSC | ADC10IE | ADC10ON;

    // Commutation - Timer0
    // Up-Mode, 16MHz CLK
    // Max Period: 4.096 ms
    TA0CTL = TASSEL_2 | ID_0 | MC_1;
    TA0CCTL0 = CCIE;
    TA0CCR0 = 0x0FFF;

    // PWM - Timer1
    // Up-Mode, 16MHz CLK
    // Top at 0xFF -> 62.5KHz
    TA1CCR0 = 0xFF;
    TA1CCR1 = 0xFF;
    TA1CTL = TASSEL_2 | ID_0 | MC_1 | TAIE;
    TA1CCTL1 = CCIE;

    // Enable Interrupts
    __enable_interrupt();

    // Infinite Loop
    while (1) {
    }

	return 0;
}

/*
 * Commutation
 * Timer0 A0 Interrupt
 */
#pragma vector = TIMER0_A0_VECTOR
__interrupt void TIMER0_A0_ISR(void) {
	// Commutate
	comm_step++;
	if (comm_step == 6) comm_step = 0;

	// Configure ADC
	ADC10CTL1 = CONSEQ_2 | bemf_phase[comm_step] | ADC10SSEL_3;
	ADC10CTL0 |= ENC | ADC10SC;

	// Configure Timer
	TA0CCTL0 = 0;		// Disable Interrupt
	TA0CCR0 = 0xFFFF;	// Count to top
	TAR = 0;			// Reset Timer
}

/*
 * Zero-Crossing
 * ADC10 Interrupt
 */
#pragma vector = ADC10_VECTOR
__interrupt void ADC10_ISR(void) {
	unsigned int zc = 0;					// Zero-Cross Flag
	unsigned int time = TAR;				// Commutation Time (Unfiltered)
	static unsigned int last_time = 0;		// Last Commutation Time

	// Check for zero-cross
	switch (bemf_pol[comm_step]) {
	case POS:
		if (ADC10MEM > ZC_TRESHOLD) zc = 1;
		break;
	case NEG:
		if (ADC10MEM < ZC_TRESHOLD) zc = 1;
		break;
	}

	// If zero-cross detected and
	// happened after false ZC at low speeds or
	// happened within range of accelerating at high speeds
	if (zc && (time > ZC_DELAY || time > last_time - ZC_ACCEL)) {
		last_time = time;							// Save last commutation time only when accelerating
		comm_time = (IIR_A*time + IIR_B*comm_time) / (IIR_A + IIR_B);		// Update filtered commutation time
		TA0CCR0 = comm_time;						// Set next commutation time
		TAR = 0;									// Clear timer
		TA0CCTL0 = CCIE;							// Enable interrupt
		ADC10CTL0 &= ~ENC;							// Disable ADC
	}

	ADC10CTL0 &= ~ADC10IFG;		// Clear ADC Flag
}

/*
 * PWM
 * Timer1 A1 Interrupt
 */
#pragma vector = TIMER1_A1_VECTOR
__interrupt void TIMER1_A1_ISR(void) {

	switch (TA1IV) {
	case 2:				// CCR1
		P2OUT = phase_low[comm_step];
		break;
	case 10:			// OVF
		P2OUT = phase_high[comm_step] | phase_low[comm_step];
		break;
	}

}













