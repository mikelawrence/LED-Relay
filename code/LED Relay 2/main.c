/*
 * LED Relay 2.c
 *
 *  The any edge triggered input sense interrupt for ACC1 or ACC2 causes the de-bounce timer to be reset.
 *  Any time this interrupt occurs the input is considered ON which will allow a noisy ON to be recognized 
 *  as ON immediately. If the de-bounce timeout occurs then either the input has stabilized ON or OFF and
 *  and the de-bounce will evaluate the state.
 * Created: 8/12/2017 6:58:33 AM
 * Author : Mike Lawrence
 */ 

/*
 * Programming sequence (Only works within the first 60 seconds of ACC1 turning ON)
 * Flash sequence = ACC2 ON Pulse OFF each less than 3 seconds. The number of ACC2 pulses represents the 
 *   number of 10 minute increments to stay on after power off. The range is 1 to 25 increments or 10 - 250 minutes.
 * Program sequence = ACC2 OFF for 4 to 7 seconds after last flash ACC2 ON, ACC2 ON for 4 to 7 seconds, ACC2 OFF for 4 to 7 seconds, ACC2 ON.
 *
 * Example: Program 20 minute stay on after power OFF. 
 *
 *   ACC2 ON for 1 sec, ACC2 OFF for 1 sec, ACC2 ON for 1 sec, ACC2 OFF for 5 sec, ACC2 ON for 5 sec, ACC2 OFF for 5 sec, ACC2 ON.
 *   {    Flash 1     }                    {    Flash 2     }  {   1st Prog OFF  } {   1st Prog ON  } {   2nd Prog OFF  } { Prog Complete }
 *
 */
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include "math.h"

/*
 * Fuse definitions
 */
FUSES =
{
	// FuseByte 1 = Watchdog timeouts set to 1,000 clocks or 1 s
	.FUSEBYTE1 = 0xFF & (~NVM_FUSES_WDWP_gm | WDWP_1KCLK_gc) & (~NVM_FUSES_WDP_gm | WDP_1KCLK_gc),	
	// FuseByte 2 = Application reset vector enabled, BOD is sampled in Power Down Mode
	.FUSEBYTE2 = 0xFF & (~NVM_FUSES_BOOTRST_bm | BOOTRST_APPLICATION_gc) & (~NVM_FUSES_BODPD_gm | BODPD_SAMPLED_gc),		
	// FuseByte 4 = Ext Reset is not disabled, Start Up Time is 0 ms, Watchdog timer is not locked 
	.FUSEBYTE4 = 0xFF & /* FUSE_RSTDISBL & */ (~NVM_FUSES_SUT_gm | SUT_0MS_gc) /* FUSE_WDLOCK */,							
	// FuseByte 5 = BOD continuous in Active Modes, EEPROM saved during chip erase, BOD Level is 2.0V
	.FUSEBYTE5 = 0xFF & (~NVM_FUSES_BODACT_gm | BODACT_CONTINUOUS_gc) & FUSE_EESAVE & (~NVM_FUSES_BODLVL_gm | BODLVL_2V0_gc), 
	// FuseByte 5 = Timer fault and detection defaults
	.FUSEBYTE6 = 0xFF,							
};

/*
 * Application specific definitions
 */
#define FALSE							0
#define TRUE							1
#define OFF								FALSE
#define ON								TRUE
#define DEFAULT_WAIT_MINUTES			30
#define WAIT_SECONDS					4
#define DEBOUNCE_TIME					0.050
#define WATCHDOG_TO						WDTO_2S
#define V12EN_port						PORTD
#define V1EN_bp							PIN4_bp
#define V2EN_bp							PIN5_bp
#define ACC1_port						PORTD
#define ACC1_bp							PIN2_bp
#define ACC2_port						PORTA
#define ACC2_bp							PIN2_bp
/*
 * Inferred definitions
 */
#define V1EN_ON()						V12EN_port.OUTSET = _BV(V1EN_bp)
#define V1EN_OFF()						V12EN_port.OUTCLR = _BV(V1EN_bp)
#define V2EN_ON()						V12EN_port.OUTSET = _BV(V2EN_bp)
#define V2EN_OFF()						V12EN_port.OUTCLR = _BV(V2EN_bp)
#define V12EN_ON()						V12EN_port.OUTSET = (_BV(V1EN_bp) | _BV(V2EN_bp))
#define V12EN_OFF()						V12EN_port.OUTCLR = (_BV(V1EN_bp) | _BV(V2EN_bp))
#define IS_ACC1_ON()					(ACC1_port.IN & _BV(ACC1_bp))
#define IS_ACC2_ON()					(ACC2_port.IN & _BV(ACC2_bp))
#define MAIN_TCNT_FROM_SECONDS(sec)		(uint16_t) round(sec / 0.001)


/*
 * Enumerations
 */
enum POWER_SM  { SM_POWER_RESET = 0, SM_POWER_DOWN, SM_POWER_OUT_OFF, SM_POWER_OUT_ON, SM_POWER_OUT_STAY_ON, SM_POWER_TIMER };
enum STAYON_SM { SM_STAYON_RESET = 0, SM_STAYON_WAIT_ON, SM_STAYON_WAIT_OFF};
enum PROG_SM   { SM_PROG_RESET = 0, SM_PROG_FLASH_ON, SM_PROG_FLASH_OFF, SM_PROG_END_ON, SM_PROG_END_OFF, SM_PROG_IND_ON, SM_PROG_IND_OFF };

/* 
 * EEPROM variables
 */
uint8_t EEMEM eeprom_wait_minutes = DEFAULT_WAIT_MINUTES;

/*
 * Global variables (Note best code optimization (code size) requires all defaults to be 0)
 */
volatile uint8_t  seconds = 0;						// Seconds counter
volatile uint8_t  minutes = 0;						// minutes counter

volatile uint8_t  acc1_debounce = 0;
volatile uint8_t  acc1_last;						// Last ACC1 state
volatile uint16_t acc1_on_start_time = 0;			// The value of Main Timer Count when ACC1 ON started
volatile uint16_t acc1_off_start_time = 0;			// The value of Main Timer Count when ACC1 OFF started

volatile uint8_t  acc2_debounce = 0;
volatile uint8_t  acc2_last;						// Last ACC2 state
volatile uint16_t acc2_on_start_time = 0;			// The value of Main Timer Count when ACC2 ON started
volatile uint16_t acc2_off_start_time = 0;			// The value of Main Timer Count when ACC2 OFF started

int main(void)
{
	uint8_t  power_state = SM_POWER_RESET;			// Current power state
	uint8_t  stayon_state = SM_STAYON_RESET;		// Current sequence state
	uint8_t  prog_state = SM_PROG_RESET;			// Current programming state
	uint8_t  wait_minutes;							// Number of minutes to stay on
	uint16_t acc1_on_time = 0;						// ACC1 length of time on
	uint16_t acc1_off_time = 0;						// ACC1 length of time off
	uint16_t acc2_on_time = 0;						// ACC2 length of time on
	uint16_t acc2_off_time = 0;						// ACC2 length of time off
	uint16_t tick_cnt_ms;							// Current time in ms (max 65.535 seconds)
	uint8_t  flash_count = 0;						// The number of valid program flashes received on ACC2
	
	// Disable the Watchdog timer on start
	wdt_disable();
    // main loop forever
    while (TRUE) 
    {
		// Each loop of main reset the Watchdog timer
		wdt_reset();
		// Compute most recent OFF and ON times
		cli();													// Disable interrupts before grabbing TCC4 count
		tick_cnt_ms = TCC4.CNT;									// Get the current tick_cnt
		if (acc1_last)
		{
			// ACC1 is currently ON so compute ON time
			acc1_on_time = tick_cnt_ms - acc1_on_start_time;	// ACC1 ON time in ms
			if (acc1_on_time > MAIN_TCNT_FROM_SECONDS(65.0))
			{
				// ACC1 ON time is too long
				acc1_on_time = MAIN_TCNT_FROM_SECONDS(65.0);	// ON time is restricted to 65.000 seconds
				acc1_on_start_time = tick_cnt_ms - MAIN_TCNT_FROM_SECONDS(65.0); // Adjust acc1_on_start_time
			}
		}
		else
		{
			// ACC1 is currently OFF so compute OFF time
			acc1_off_time = tick_cnt_ms - acc1_off_start_time;	// ACC1 OFF time in ms
			if (acc1_off_time > MAIN_TCNT_FROM_SECONDS(65.0))
			{
				// ACC1 OFF time is too long
				acc1_off_time = MAIN_TCNT_FROM_SECONDS(65.0);	// OFF time is restricted to 65.000 seconds
				acc1_off_start_time = tick_cnt_ms - MAIN_TCNT_FROM_SECONDS(65.0); // Adjust acc1_off_start_time
			}
		}
		if (acc2_last)
		{
			// ACC2 is currently ON so compute ON time
			acc2_on_time = tick_cnt_ms - acc2_on_start_time;	// ACC2 ON time in ms
			if (acc2_on_time > MAIN_TCNT_FROM_SECONDS(65.0))
			{
				// ACC2 ON time is too long
				acc2_on_time = MAIN_TCNT_FROM_SECONDS(65.0);	// ON time is restricted to 65.000 seconds
				acc2_on_start_time = tick_cnt_ms - MAIN_TCNT_FROM_SECONDS(65.0); // Adjust acc2_on_start_time
			}
		}
		else
		{
			// ACC2 is currently OFF so compute OFF time
			acc2_off_time = tick_cnt_ms - acc2_off_start_time;	// ACC2 OFF time in ms
			if (acc2_off_time > MAIN_TCNT_FROM_SECONDS(65.0))
			{
				// ACC2 OFF time is too long
				acc2_off_time = MAIN_TCNT_FROM_SECONDS(65.0);	// OFF time is restricted to 65.000 seconds
				acc2_off_start_time = tick_cnt_ms - MAIN_TCNT_FROM_SECONDS(65.0); // Adjust acc2_off_start_time
			}
		}
		sei();													// Enable interrupts

		// Power State Machine
		//   Manages initialization and the Power Switches
		switch (power_state)
		{
		case SM_POWER_DOWN:							// Board is Powered Down, Output is OFF, waiting for ACC1 turn ON
			V12EN_OFF();							// The power switches are OFF
			// See if ACC1 has switched ON
			if (acc1_last)
			{
				// ACC1 is now ON, what state we goto is dependent on ACC2
				if (acc2_last)
				{
					// ACC2 is ON
					power_state = SM_POWER_OUT_ON;	// Switch to Output ON state
				}
				else
				{
					// ACC2 is OFF
					power_state = SM_POWER_OUT_OFF;	// Switch to Output OFF state
				}
			}
			else
			{
				// ACC1 is still OFF, so enter Power Down State
				wdt_disable();						// Disable the watchdog timer before going to sleep
				set_sleep_mode(SLEEP_SMODE_PDOWN_gc); // Set Power Down Mode when sleep is executed
				sleep_mode();						// Enter Power Down State now
				wdt_enable(WATCHDOG_TO);			// Enable the Watchdog timer
				continue;							// We were woken from Power Down state, restart the while(TRUE) loop
			}
			break;
		case SM_POWER_OUT_OFF:						// Board is ON, Output is OFF (ACC1 is ON, ACC2 is OFF)
			V12EN_OFF();							// The power switches are OFF
			// See if ACC1 has switched OFF
			if (!acc1_last)
			{
				// ACC1 is now OFF
				power_state = SM_POWER_DOWN;		// Switch to Power Down State
			}
			else
			{
				// ACC1 is still ON, See if ACC2 has switched ON
				if (acc2_last)
				{
					// ACC2 is now ON
					power_state = SM_POWER_OUT_ON;	// Switch to Output On State
				}
			}
			break;
		case SM_POWER_OUT_ON:						// Board is ON, Output is ON (ACC1 is ON, ACC2 is ON)
			if (prog_state == SM_PROG_IND_OFF)
			{
				// The power switches are always OFF when Programming State is in Programming Success Output OFF state
				//  Handy indicator of programming success
				V12EN_OFF();						// The power switches are OFF
			}
			else
			{
				// Normal operation of Power Switches
				V12EN_ON();							// The power switches are ON				
			}
			// See if ACC1 has switched OFF
			if (!acc1_last)
			{
				// ACC1 is now OFF
				power_state = SM_POWER_DOWN;		// We need to power down
			}
			else
			{
				// ACC1 is still ON, See if ACC2 has switched OFF
				if (!acc2_last)
				{
					// ACC2 is now OFF
					power_state = SM_POWER_OUT_OFF;	// Switch to Output Off State
				}
			}
			break;
		case SM_POWER_OUT_STAY_ON:					// Board is ON, Output is ON (ACC1 is ON, ACC2 is ON)
			if (prog_state == SM_PROG_IND_OFF)
			{
				// The power switches are always OFF when Programming State is in Programming Success Output OFF state
				//  Handy indicator of programming success
				V12EN_OFF();						// The power switches are OFF
			}
			else
			{
				// Normal operation of Power Switches
				V12EN_ON();							// The power switches are ON				
			}
			// See if ACC1 has switched OFF
			if (!acc1_last)
			{
				// ACC1 is now OFF
				power_state = SM_POWER_TIMER;		// We need to enter Timer State
				cli();								// Disable interrupts
				minutes = 0; seconds = 0;			// Clear minutes and seconds
				TCC4.CCC = TCC4.CNT + MAIN_TCNT_FROM_SECONDS(1.0); // TCC4-CCC interrupt 1 second from now
				sei();								// Enable interrupts
			}
			else
			{
				// ACC1 is still ON, See if ACC2 has switched OFF
				if (!acc2_last)
				{
					// ACC2 is now OFF
					//  Make sure ACC2 OFF time is longer than 0.5 seconds before switching states
					//  This will allow ACC2 to turn off up to 0.5 seconds before ACC1 and
					//   still be recognized as Power Stay ON
					if (acc2_off_time > MAIN_TCNT_FROM_SECONDS(0.5))
					{
						power_state = SM_POWER_OUT_OFF;	// Switch to Output Off State
					}
				}
			}
			break;
		case SM_POWER_TIMER:						// ACC1 is OFF, Output is ON and waiting for timeout to occur
			V12EN_ON();								// The power switches are ON
			// See if ACC1 has switched ON
			if (acc1_last)
			{
				// ACC1 is now ON, what state we goto is dependent on ACC2
				if (acc2_last)
				{
					// ACC2 is ON
					power_state = SM_POWER_OUT_ON;	// Switch to Output ON state
				}
				else
				{
					// ACC2 is OFF
					power_state = SM_POWER_OUT_OFF;	// Switch to Output OFF state
				}
			}
			else
			{
				// ACC1 is still OFF, see if timeout has occurred
				cli();								// Disable interrupts
				if (minutes >= wait_minutes)
				{
					// Timeout has occurred
					sei();							// Re-enable interrupts
					power_state = SM_POWER_DOWN;	// Switch to Power Down State
				}
				else
				{
					// no timeout so go back to sleep for a while (second timer TCC4 will wake us up)
					sei();							// Re-enable interrupts
					wdt_disable();					// Disable the watchdog timer before going to sleep
					set_sleep_mode(SLEEP_SMODE_IDLE_gc); // Set Idle Mode when sleep is executed
					sleep_mode();					// Enter Idle Mode now, allow all interrupts
					//wdt_enable(WATCHDOG_TO);		// Enable the Watchdog timer
					continue;						// We were woken from Idle state, restart the while(TRUE) loop
				}
			}
			break;
		default:
			// Anything else is considered to SM_POWER_RESET
			cli();									// Disable interrupts
			// Clock defaults to internal 2MHz clock which is fine, but make sure 2MHz clock is ready before continuing
			while (!(OSC.STATUS & OSC_RC2MRDY_bm));
			// Initialize IOs, default all pins to input and have pull-ups enabled
			PORTA.DIRCLR = 0xFF;											// PORTA is all inputs
			PORTCFG.MPCMASK = 0xFF;
			PORTA.PIN0CTRL = PORT_OPC_PULLDOWN_gc | PORT_ISC_BOTHEDGES_gc;	// PORTA is all pullups
			PORTC.DIRCLR = 0xFF;											// PORTC is all inputs
			PORTCFG.MPCMASK = 0xFF;
			PORTC.PIN0CTRL = PORT_OPC_PULLDOWN_gc | PORT_ISC_BOTHEDGES_gc;	// PORTC is all pullups
			PORTD.DIRCLR = 0xFF;											// PORTD is all inputs
			PORTCFG.MPCMASK = 0xFF;
			PORTD.PIN0CTRL = PORT_OPC_PULLDOWN_gc | PORT_ISC_BOTHEDGES_gc;	// PORTD is all pullups
			PORTR.DIRCLR = 0xFF;											// PORTR is all inputs
			PORTCFG.MPCMASK = 0xFF;
			PORTR.PIN0CTRL = PORT_OPC_PULLDOWN_gc | PORT_ISC_BOTHEDGES_gc;	// PORTR is all pullups
			// Configure ACC1
			PORTCFG.MPCMASK = _BV(ACC1_bp) | _BV(3);
			ACC1_port.PIN0CTRL = PORT_OPC_TOTEM_gc | PORT_ISC_BOTHEDGES_gc;	// Input NO pullup and interrupt on any edge
			ACC1_port.INTCTRL = PORT_INTLVL_HI_gc;							// Interrupt will be high level
			ACC1_port.INTMASK |= _BV(ACC1_bp);								// Port interrupt enabled
			// Configure ACC2
			PORTCFG.MPCMASK = _BV(ACC2_bp);
			ACC2_port.PIN0CTRL = PORT_OPC_TOTEM_gc | PORT_ISC_BOTHEDGES_gc;	// Input NO pullup and interrupt on any edge
			ACC2_port.INTCTRL = PORT_INTLVL_HI_gc;							// Interrupt will be high level
			ACC2_port.INTMASK |= _BV(ACC2_bp);								// Port interrupt enabled
			// Configure V1EN and V2EN as totem-pole outputs
			V12EN_port.OUTCLR = _BV(V1EN_bp) | _BV(V2EN_bp);				// V1EN and V2EN will be low when enabled
			PORTCFG.MPCMASK = _BV(V1EN_bp) | _BV(V2EN_bp);
			V12EN_port.PIN0CTRL = PORT_OPC_TOTEM_gc;						// V1EN and V2EN will be totem-pole outputs
			V12EN_port.DIRSET = _BV(V1EN_bp) | _BV(V2EN_bp);				// V1EN and V2EN are now outputs
			// Configure Power Reduction
			PR.PRGEN = 1 << PR_XCL_bp				// XCL power down: enabled
					 | 1 << PR_RTC_bp				// RTC power down: enabled
					 | 0 << PR_EVSYS_bp				// EVSYS power down: disabled
					 | 1 << PR_EDMA_bp;				// EDMA power down: enabled
			PR.PRPA = 1 << PR_DAC_bp				// DACA power down: enabled
					| 1 << PR_ADC_bp				// ADCA power down: enabled
					| 1 << PR_AC_bp;				// ACA power down: enabled
			PR.PRPC = 1 << PR_TWI_bp				// TWIC power down: enabled
					| 1 << PR_USART0_bp				// USART0C power down: enabled
					| 1 << PR_SPI_bp				// SPIC power down: enabled
					| 1 << PR_HIRES_bp				// HIRESC power down: enabled
					| 0 << PR_TC5_bp				// TCC5 power down: disabled
					| 0 << PR_TC4_bp;				// TCC4 power down: disabled
			PR.PRPD = 1 << PR_USART0_bp				// USART0D power down: enabled
					| 1 << PR_TC5_bp;				// TDC5 power down: enabled
			// Configure 1ms tick on TCC5
			TCC5.PER = 1999;						// Period 1999 is 1 ms overflow
			TCC5.CTRLA = TC_CLKSEL_DIV1_gc			// Source is System Clock
					   | 0 << TC5_UPSTOP_bp			// Stop on Next Update: disabled
					   | 0 << TC5_EVSTART_bp		// Start on Next Event: disabled
					   | 0 << TC5_SYNCHEN_bp;		// Synchronization Enabled: disabled
			// Configure Event Channel 0 for TCC5 overflow
			EVSYS.CH0MUX = EVSYS_CHMUX_TCC5_OVF_gc; // Timer/Counter C5 Overflow
			// Configure main timer
			TCC4.CTRLA = TC_CLKSEL_EVCH0_gc			// Event Channel 0
					   | 0 << TC4_UPSTOP_bp			// Stop on Next Update: disabled
					   | 0 << TC4_EVSTART_bp		// Start on Next Event: disabled
					   | 0 << TC4_SYNCHEN_bp;		// Synchronization Enabled: disabled
			// Set interrupt level to high for TCC4-CCA, TCC4-CCB and TCC4-CCC
			TCC4.INTCTRLB = TC_CCAINTLVL_HI_gc		// CCA High level interrupt priority
						  | TC_CCBINTLVL_HI_gc		// CCB High level interrupt priority
						  | TC_CCCINTLVL_HI_gc		// CCC High level interrupt priority
						  | TC_CCDINTLVL_OFF_gc;	// CCD interrupt disabled
			// TCC4-CCC interrupt 1 second from now
			TCC4.CCC = TCC4.CNT + MAIN_TCNT_FROM_SECONDS(1.0);
			// Enable high level interrupts
			PMIC.CTRL = 0 << PMIC_RREN_bp			// Round-Robin Priority Enable: disabled
					  | 0 << PMIC_IVSEL_bp			// Interrupt Vector Select: disabled
					  | 1 << PMIC_HILVLEN_bp		// High Level Enable: enabled
					  | 0 << PMIC_MEDLVLEN_bp		// Medium Level Enable: disabled
					  | 0 << PMIC_LOLVLEN_bp;		// Low Level Enable: disabled
			// Get ACC1 current state
			if IS_ACC1_ON()
			{
				// ACC1 is currently on
				acc1_last = ON;						// Last ACC1 state is ON
				if IS_ACC2_ON()
				{
					// ACC1 and ACC2 are on
					power_state = SM_POWER_OUT_ON;	// Switch to Power Active State
				}
				else
				{
					// ACC1 is ON, ACC2 is OFF
					power_state = SM_POWER_OUT_OFF;	// Switch to Power Active State
				}
			}
			else
			{
				// ACC1 is currently off
				acc1_last = OFF;					// Last ACC1 state is OFF
				power_state = SM_POWER_DOWN;		// Switch to Power Down State
			}
			// Initialize variables
			seconds = 0;
			minutes = 0;
			// Read the wait minutes from EEPROM
			wait_minutes = eeprom_read_byte(&eeprom_wait_minutes);
			// Enable the Watchdog timer
			wdt_enable(WATCHDOG_TO);
			// Enable global interrupts
			sei();
			continue;								//  restart the main forever loop
		}
		// The StayON State Machine and Programming State Machines do not run when ACC1 is OFF
		if (!acc1_last) 
		{
			// ACC1 is OFF
			stayon_state = SM_STAYON_RESET;			// Reset the StayON State Machine
			prog_state = SM_PROG_RESET;				// Reset the Programming State Machine
			continue;								// Restart the while loop (this disables State Machines below this point)
		}
		// StayON State Machine
		//   Looks for a short sequence to keep the Outputs ON after bike is turned off
		switch (stayon_state)
		{
		case SM_STAYON_WAIT_ON:						// Waiting for ACC2 to turn ON (1st ON)
			if (acc2_on_time > MAIN_TCNT_FROM_SECONDS(3.0))
			{
				// ON time is longer than 3 seconds
				stayon_state = SM_STAYON_RESET;		// Reset the StayON State Machine
			}
			else
			{
				// ON time is less than 3 seconds
				if (!acc2_last)
				{
					// ACC2 turned OFF, we had a valid ON time
					stayon_state = SM_STAYON_WAIT_OFF; // Goto wait for 1st OFF time
				}
			}
			break;
		case SM_STAYON_WAIT_OFF:					// Waiting for ACC2 to turn OFF (1st OFF)
			if (acc2_off_time > MAIN_TCNT_FROM_SECONDS(3.0))
			{
				// OFF time is longer than 3 seconds
				stayon_state = SM_STAYON_RESET;		// Reset the StayON State Machine
			}
			else
			{
				// OFF time is less than 3 seconds
				if (acc2_last)
				{
					// ACC2 turned ON, this correctly identifies the StayON sequence
					stayon_state = SM_STAYON_RESET;	// Reset the StayON State Machine
					power_state = SM_POWER_OUT_STAY_ON; // Force the Power State Machine to Output Stay ON state
				}
			}
			break;
		default:									// All other states are considered SM_POWER_RESET
			// Do nothing if ACC1 is not ON
			if (acc1_last) 
			{
				// ACC2 turning ON will start the entire process
				if (acc2_last)
				{
					stayon_state = SM_STAYON_WAIT_ON; // Goto wait for ACC2 to turn OFF state
				}
			}
			break;
		}
		// Programming State Machine
		//   Looks for programming state based on ACC2 input
		switch (prog_state)
		{
		case SM_PROG_FLASH_ON:						// Waiting for ACC2 to turn OFF to capture an ON press for the flash count
			if (acc2_on_time > MAIN_TCNT_FROM_SECONDS(3.0))
			{
				// ACC2 ON time is longer than 3 seconds, this aborts flash the sequence
				prog_state = SM_PROG_RESET;			// Reset the Programming State Machine
			}
			else if (!acc2_last)
			{
				// ACC2 ON time is less than 3 seconds, this is a flash pulse
				++flash_count;						// Increment flash count
				prog_state = SM_PROG_FLASH_OFF;		// Goto wait for ACC2 to turn ON for flash OFF time
			}
			break;
		case SM_PROG_FLASH_OFF:						// Waiting for ACC2 to turn ON to capture an OFF time between ON presses
			if (acc2_off_time > MAIN_TCNT_FROM_SECONDS(7.0))
			{
				// ACC2 OFF time is longer than 7 seconds, this aborts the flash sequence
				prog_state = SM_PROG_RESET;			// Reset the Programming State Machine
			}
			else if (acc2_last)
			{
				// ACC2 OFF time is less than 7 seconds and ACC2 turned ON
				if (acc2_off_time > MAIN_TCNT_FROM_SECONDS(4.0))
				{
					// OFF time is between 4 and 7 seconds which ends the flash sequence and starts the program sequence
					prog_state = SM_PROG_END_ON;
				}
				else if (acc2_off_time > MAIN_TCNT_FROM_SECONDS(3.0))
				{
					// ACC2 OFF time is between 3 and 4 seconds, this aborts the flash sequence
					prog_state = SM_PROG_RESET;			// Reset the Programming State Machine
				}
				else
				{
					// ACC2 OFF time is less than 3 seconds and is valid off time, we are still flashing
					prog_state = SM_PROG_FLASH_ON;
				}
			}
			break;
		case SM_PROG_END_ON:						// Waiting for ACC2 to turn OFF to capture an ON press for the end sequence
			if (acc2_on_time > MAIN_TCNT_FROM_SECONDS(7.0))
			{
				// ACC2 OFF time is longer than 7 seconds, this aborts the end sequence
				prog_state = SM_PROG_RESET;			// Reset the Programming State Machine			
			}
			else if (!acc2_last)
			{
				// ACC2 ON time is less than 7 seconds and ACC2 turned OFF
				if (acc2_on_time > MAIN_TCNT_FROM_SECONDS(4.0))
				{
					// ACC2 ON time is between 4 - 7 seconds, this is a correct ON time for end sequence
					prog_state = SM_PROG_END_OFF;
				}
				else
				{
					// ACC2 ON time is less than 4 seconds, this aborts the end sequence
					prog_state = SM_PROG_RESET;		// Reset the Programming State Machine
				}
			}
			break;
		case SM_PROG_END_OFF:						// Waiting for ACC2 to turn ON to capture 2nd OFF time for the end sequence
			if (acc2_off_time > MAIN_TCNT_FROM_SECONDS(7.0))
			{
				// ACC2 OFF time is longer than 7 seconds, this aborts the end sequence
				prog_state = SM_PROG_RESET;			// Reset the Programming State Machine
			}
			else if (acc2_last)
			{
				// ACC2 OFF time is less than 7 seconds and ACC2 turned ON
				if (acc2_off_time > MAIN_TCNT_FROM_SECONDS(4.0))
				{
					// ACC2 OFF time is between 4 and 7 seconds which is valid
					if (flash_count > 25) {
						flash_count = 25;
					}
					// Each flash is 10 minutes
					flash_count = flash_count * 10;
					//  Write the flash_count as wait time to EEPROM
					eeprom_write_byte(&eeprom_wait_minutes, flash_count);
					// Update wait time in RAM
					wait_minutes = flash_count;
					// Indicate successful programming sequence with an Output Flash
					prog_state = SM_PROG_IND_ON;
				}
				else
				{
					// ACC2 OFF time is less than 4 seconds which is INVALID
					prog_state = SM_PROG_RESET;		// Reset the Programming State Machine
				}
			}
			break;
		case SM_PROG_IND_ON:						// Programming Sequence Success, leave Output ON for 1 second
			if (!acc2_last)
			{
				// ACC2 turned OFF, which aborts the Programming Sequence Success Output Flash
				prog_state = SM_PROG_RESET;
			}
			else
			{
				// ACC2 is still ON
				if (acc2_on_time > MAIN_TCNT_FROM_SECONDS(2.0))
				{
					// Output ON for 1 second, time to flash Output OFF for 1 second
					prog_state = SM_PROG_IND_OFF;
				}
			}
			break;
		case SM_PROG_IND_OFF:						// Programming Sequence Success, flash output off for 1 second
			if (!acc2_last)
			{
				// ACC2 turned OFF, which aborts the Programming Sequence Success Output Flash
				prog_state = SM_PROG_RESET;			// Reset the Programming State Machine
			}
			else
			{
				// ACC2 is still ON
				if (acc2_on_time > MAIN_TCNT_FROM_SECONDS(3.0))
				{
					// Output OFF for 1 second, we are done
					prog_state = SM_PROG_RESET;		// Reset the Programming State Machine
				}
			}
			break;
		default:									// All other states are considered reset state SM_PROG_RESET
			// once ACC1 has been on for more than 60 seconds Programming State Machine is disabled)
			if (acc1_on_time <= MAIN_TCNT_FROM_SECONDS(60.0))
			{
				// Rising edge will start the entire process
				if (acc2_last)
				{
					flash_count = 0;				// Reset flash_count before using it
					prog_state = SM_PROG_FLASH_ON;	// Goto wait for falling edge state
				}
			}
			break;
		}
    }
}

/*
 * PORTD Port Interrupt. (ACC1 Input Sense Interrupt)
 *  Used to detect ACC1 changing, any edge will cause this interrupt.
 *  Handles ACC1 turning ON. Note ACC1 turning OFF is handled by ACC1 De-Bounce Timer.
 */
ISR(PORTD_INT_vect)
{
	// Set ACC1 de-bounce timer
	TCC4.CCA = TCC4.CNT + MAIN_TCNT_FROM_SECONDS(DEBOUNCE_TIME);
	// Enable ACC1 de-bounce interrupt
	TCC4.INTCTRLB = (TCC4.INTCTRLB & ~TC4_CCAINTLVL_gm) | TC_CCAINTLVL_HI_gc;
	// Clear the interrupt flag
	ACC1_port.INTFLAGS |= _BV(ACC1_bp);
	// Look for rising edge change
	if (!acc1_last)
	{
		// ACC1 was low before now it has gone high
		acc1_last = ON;
		acc1_on_start_time = TCC4.CNT;
	}
}

/*
 * PORTA Interrupt. (ACC2 Input Sense Interrupt)
 *  Used to detect ACC2 changing, any edge will cause this interrupt.
 *  Handles ACC2 turning ON. Note ACC2 turning OFF is handled by ACC2 De-Bounce Timer.
 */
ISR(PORTA_INT_vect)
{
	// Set ACC2 de-bounce timer
	TCC4.CCB = TCC4.CNT + MAIN_TCNT_FROM_SECONDS(DEBOUNCE_TIME);
	// Enable ACC2 de-bounce interrupt
	TCC4.INTCTRLB = (TCC4.INTCTRLB & ~TC4_CCBINTLVL_gm) | TC_CCBINTLVL_HI_gc;
	// Clear the interrupt flag
	ACC2_port.INTFLAGS |= _BV(ACC2_bp);
	// Look for rising edge change
	if (!acc2_last)
	{
		// ACC2 was low before now it has gone high
		acc2_last = ON;
		acc2_on_start_time = TCC4.CNT;
	}
}

/*
 * Timer C4 Compare A interrupt (ACC1 De-bounce Timer)
 *  Used to handle ACC1 input going stable. Should occur DEBOUNCE_TIME after last edge of ACC1.
 *  Handles ACC1 turning OFF. Note ACC1 turning ON is handled by ACC1 Input Sense Interrupt
 */
ISR(TCC4_CCA_vect)
{
	// ACC1 input has stabilized, determine the new state
	if (acc1_last) {
		// ACC1 was previously ON
		if (!IS_ACC1_ON())
		{
			// ACC1 is now OFF
			acc1_last = OFF;
			acc1_off_start_time = TCC4.CNT;
		}
	}
	// Disable ACC1 de-bounce interrupt
	TCC4.INTCTRLB &= ~TC4_CCAINTLVL_gm;
}

/*
 * Timer C4 Compare B interrupt (ACC2 De-bounce Timer)
 *  Used to handle ACC2 input going stable. Should occur DEBOUNCE_TIME after last edge of ACC2.
 *  Handles ACC2 turning OFF. Note ACC2 turning ON is handled by ACC2 Input Sense Interrupt
 */
ISR(TCC4_CCB_vect)
{
	// ACC2 input has stabilized, determine the new state
	if (acc2_last) {
		// ACC2 was previously ON
		if (!IS_ACC2_ON())
		{
			// ACC2 is now OFF
			acc2_last = OFF;
			acc2_off_start_time = TCC4.CNT;
		}
	}
	// Disable ACC2 de-bounce interrupt
	TCC4.INTCTRLB &= ~TC4_CCBINTLVL_gm;				
}

/*
 * Timer C4 Compare C interrupt
 *  Used to count seconds and minutes
 */
ISR(TCC4_CCC_vect)
{
	// Compare again 1 second from now
	TCC4.CCC = TCC4.CNT + MAIN_TCNT_FROM_SECONDS(1.0);
	// Increment seconds
	seconds++;
	// Overflow seconds into minutes
	if (seconds == 60)
	{
		// Wrap second back to 0
		seconds = 0;
		// Increment minutes
		minutes++;
		// Prevent minutes from getting larger than 60
		if (minutes > 60)
		{
			minutes = 60;
		}
	}
}

