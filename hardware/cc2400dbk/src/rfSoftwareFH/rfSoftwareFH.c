/*****************************************************************************
 *                                                                           *
 *        **********                                                         *
 *       ************                                                        *
 *      ***        ***                                                       *
 *      ***   +++   ***                                                      *
 *      ***   + +   ***                                                      *
 *      ***   +                            CHIPCON CC2400                    *
 *      ***   + +   ***                    Example - CC2400DBK	             *
 *      ***   +++   ***                    Frequency Hopping                 *
 *      ***       ***                                                        *
 *       ***********                                                         *
 *        *********                                                          *
 *                                                                           *
 *****************************************************************************
 * Device:      CC2400                                                       *
 * File:		rfSoftwareFH.c					     						 * 
 * Author:		MBR/JOL						     							 *
 * Target:		ATmega8 					     							 *
 * Tools:		WINAVRgcc                              		 				 *
 * Created:		2004-01-15					        						 *
 * Description:	RF frequency hopping example for CC2400. This example uses   *                      
 *				hop frequencies defined by the pFreq[FREQ_COUNT] table. The  *
 *				packet used is the same fixed 10 byte packet, as in the      *
 *				example rfBlinkLed. Each Slave is assigned a timeslot by	 *
 *				using the Joystick as a timeslot selector. Timeslot 0 is 	 *
 *				always Master and will transmit the beacon in this slot. 	 * 
 *				Master is selected by using the S2 push button.	As soon as	 *
 *				the master starts to transmit the yellow TX LED is set. When *
 *				a slave is found or is in synch with Master, the green RX    *
 *				LED will be set. As a test if you reset the CC2400DBK, you   *
 *				will off course see that the slave will loose synch and the  *
 *				greee RX LED will be turned off								 *		 *
 *															  				 *
 * Important:	The software has not been tested with respect to harmonics   *
 *				and FCC regulations. The only requirement is that we are 	 *
 *				using the miniumum requirement of 15 hopping freqeuencies	 *
 *				The FCC regulations for 2.4 GHz described in the regulation  *
 *				Section 15.7.247 ((a),(1),(iii). The maximum output power is *
 *				described in the same section ((b),(1)).					 *
 * Ref. FCC 	 															 *
 * Regulations:	"For frequency hopping systems in the 2400 - 2483.5 MHz band *
 *				employing at least 75 hopping channel, and all hopping 		 *
 *				systems in the 5725 - 5850 MHz band: 1 Watt. For all other	 *
 *			    frequency hopping systems in the 2400 - 2483.5 MHz band: 	 *
 *				0.125 Watt.													 *							  			
 *		                                                             		 *
 *****************************************************************************
 * Revision history:                                                         *
 *                                                                           *
 * $Log: rfSoftwareFH.c,v $
 * Revision 1.3  2004/01/28 09:08:05  mbr
 * Changed:
 * Library changes to ucInit()  and ucTimer2Init()
 *
 * Revision 1.2  2004/01/20 15:40:28  mbr
 * Cosmetic Changes
 *
 * Revision 1.1  2004/01/19 14:10:13  mbr
 * Initial version in CVS.
 *
 * 
 *								             								 *	
 * 									     									 *
 *									     									 *	
 *                                                                           *
 ****************************************************************************/


//----------------------------------------------------------------------------
//  Master Include File 
//----------------------------------------------------------------------------
#include "include.h" 
//----------------------------------------------------------------------------
// MAIN PROGRAM
//----------------------------------------------------------------------------
// Number of Timeslots
#define  SLOT_COUNT 5
// Number of Frequency
#define  FREQ_COUNT 15
// Beacon flag
#define	 PKT_BEACON 0x01
//----------------------------------------------------------------------------
//  Global Variables 
//----------------------------------------------------------------------------
// CC2400 Status byte, updated every address access to CC2400
extern volatile uint8 CC2400_Status;
// Frequency hopping counter 
volatile uint8 currentFreq;
// Timeslot counter
volatile uint8 currentSlot;
// Slot variable
volatile uint8 mySlot;
// Packet Protocol variable 
BYTE flags;
// Packet Protocol variable 
uint8 length;
// Beacon counter
volatile uint8 noBeaconCount;
// SW flag 
volatile BOOL noSync;
// TX buffer for fixed packet of 10 bytes
BYTE pTXBuffer[] = { 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39 };
// RX buffer 
BYTE pRXBuffer[20];
//----------------------------------------------------------------------------
// Timeslot timer counter variables 
//----------------------------------------------------------------------------
// Long timer timeout
#define LONG_RXTX_TIMEOUT	150
#define LONG_FSON_TIMEOUT	130
// Normal timer timeout
#define NORMAL_RXTX_TIMEOUT	50
#define NORMAL_FSON_TIMEOUT	30
// Timer offset 
#define TIMEOUT_OFFSET 20
// Beacon threshold 
#define NO_BEACON_THRESHOLD 20

//----------------------------------------------------------------------------
// Register settings given by SmartRF Studio  
//----------------------------------------------------------------------------
RF_SETTINGS rfs   = {
		 0x0010,		// 1.  FSCTRL, Frequency synthesiser main control and status
		 0x0980,		// 2.  FSDIV, Frequency synthesiser frequency divison control
		 0x0040, 		// 3.  MDMCTRL, Modem main control and status
		 0x000F,		// 4.  FREND, Analog front-end control
		 0x7FF2,		// 5.  RSSI, RSSI information
		 0x17E0, 		// 6.  IOCFG, I/O Configuration register
		 0x7A94,		// 7.  FSMTC, Finite State Machine time constants
		 0x7FFF,		// 8.  MANAND, Manual signal AND-override register	 	 
		 0x0803,		// 9.  PAMTST, PA and transmit mixers test register
		 0x2B22, 		// 10. LMTST, LNA and receive mixers test register	 
		 0x134B, 		// 11. MDMTST0, Modem test register 0
		 0x004B, 		// 12. MDMTST1, Modem test register 1
		 0x0000,		// 13. DACTST, DAC Test register	 
		 0xA210, 		// 14. FSTST0, Test register: VCO array results and override
		 0x1002, 		// 15. FSTST1, Test register: VC DAC manual control, VCO current constant
		 0x0F71,		// 16. GRMDM, Generic radio modem control 
		 0x0000 		// 17. GRDEC, Generic radio decimation control and status	 
		 }; 
//----------------------------------------------------------------------------
// Frequency Hopping Table, with example frequencies  
//----------------------------------------------------------------------------
WORD pFreqs[FREQ_COUNT] = { 
		0x0980,	// 1.  Frequency 2432 MHz 
		0x0981, // 2.  Frequency 2433 MHz 
		0x0982, // 3.  Frequency 2434 MHz 
		0x0983, // 4.  Frequency 2435 MHz 
		0x0984, // 5.  Frequency 2436 MHz 
		0x0985, // 6.  Frequency 2437 MHz 
		0x0986, // 7.  Frequency 2438 MHz
		0x0987, // 8.  Frequency 2439 MHz
		0x0988, // 9.  Frequency 2440 MHz
		0x0989, // 10. Frequency 2441 MHz
		0x098A, // 11. Frequency 2442 MHz
		0x098B, // 12. Frequency 2443 MHz
		0x098C, // 13. Frequency 2444 MHz
		0x098D, // 14. Frequency 2445 MHz
		0x098E  // 15. Frequency 2446 MHz
}; // Frequency Hopping Table
//----------------------------------------------------------------------------
// Frequency hopping function  
//----------------------------------------------------------------------------
void nextFreq(void) {
	// Turn off PLL and go to Idle mode
	FASTSPI_STROBE(CC2400_SRFOFF); 
	// Change Frequency 	     
	currentFreq += 1;	
	// Restart frequency counter
	if (currentFreq == FREQ_COUNT) currentFreq = 0;
	// Program frequency
	if (currentSlot == mySlot) {
		// 0 MHz IF TX = RX + 1
		FASTSPI_SETREG(CC2400_FSDIV, pFreqs[currentFreq]);
	} else {
		// 0 MHz IF, RX = TX - 1
		FASTSPI_SETREG(CC2400_FSDIV, pFreqs[currentFreq] - 1);
	}	
	// Turn on PLL, calibrate and go to 	
	FASTSPI_STROBE(CC2400_SFSON); 
		     
}// nextFreq
//----------------------------------------------------------------------------
// Timeslot Change function  
//---------------------------------------------------------------------------- 
void nextSlot(void) {
	//Change timeslot
	currentSlot += 1;
	// Reset timeslot
	if (currentSlot == SLOT_COUNT) currentSlot = 0;
}// nexSlot
//----------------------------------------------------------------------------
// Main 
//---------------------------------------------------------------------------- 
int main(void) { 
    
    // Initalize ATMega8 microcontroller with 
    // PKT interrupt from CC2400 enabled  
	ucInit(PKT_INT);
	
	// Flush variables
	currentFreq = 0;
	currentSlot = 0;
	flags = 0;
	noBeaconCount = 42;
	
	// Forever
	while(TRUE) {
		
		// Setup Master selecting a button
		// Beacons are  transmitted by Master or device selected to timeslot 0
		if(!(PINB & (1<<SW_PRESSED))){
			//Master timeslot
			mySlot = 0;
			// Start to transmit set yellow LED
			SET_YLED;
			noSync = FALSE;
			break;
		}
		
		// Setup a Slave for one timeslot by using the joystick 
		if(!(PIND & (1<< JOYSTICK_SW))) {
			// Set Slave timeslot number
			mySlot = 1;
			noSync = TRUE;
			break;
		}
    }
        // Disable PKT interrupt from CC2400
        DISABLE_PKT_INT();
                
		// Configure CC2400
		rfInit(&rfs, TRUE);
		
		// Turn on CC2400 PLL, calibrate and go to Idle mode
		rfOn();
		
		// Enable PKT interrupt from CC2400
		ENABLE_PKT_INT();
        
        // Configure frequency hopping timers
		//Frequency Hopping and Timeslot Timer
		ucTimer1Init(NORMAL_FSON_TIMEOUT);	
		// RX/TX Timer 
		ucTimer2Init(NORMAL_RXTX_TIMEOUT, CLR_T2_CMP|T2_DIV_256);
	    
	    // Forever 
		while (TRUE) {}
} // Main
//--------------------------------------------------------------
// Turn on RX/TX / Timer 2 Interrupt Service Routine
//--------------------------------------------------------------
SIGNAL(SIG_OUTPUT_COMPARE2) {	
	// Reset frequency hopping and timeslot timer 
	TCNT1 = 0;
	// Who turn is it to transmit or receive?
	if (currentSlot == mySlot) {
		// Timeslot number is transmitted in the packet as a flag
		flags = (1 << mySlot);
		// Turn on Tx mode and send packet
		rfSendPacket(pTXBuffer, 10, &flags);
	} else {
		// Turn on RX mode
		rfBeginReceive();
	}
	// Enable PKT interrupt from CC2400
	ENABLE_PKT_INT();
	// If not in Synchronization, and Master Timeslot, increment BEACON count
	if (!noSync && (currentSlot == 0)) noBeaconCount++;
	
}// RXTX_TIMER_ISR

//--------------------------------------------------------------
// CC2400 PKT interrupt, PKT Interrupt Service Routine
//--------------------------------------------------------------
SIGNAL(SIG_INTERRUPT1) {
	// If it is my turn, transmit the packet
	if (mySlot == currentSlot) {
		// Transmit, do nothing
	} else {
		// Fixed packet length of 10 bytes        
		length = 10;
		// Preamble and synch detect by CC2400, start reading out the packet from the FIF0
		if (rfEndReceive(pRXBuffer, &length, &flags)) {
			// Turn on RX if S	
			if (noSync && (flags != PKT_BEACON)) {                          
			// Turn on RX mode
				rfBeginReceive();
			} else {
				// If beacon is transmitted at timeslot 0 
				if ((currentSlot == 0) && (flags == PKT_BEACON)) {
					// Readjust timers  				
				    TCNT2 = TIMEOUT_OFFSET; // Timeout timer offset 
				    TCNT1 = TIMEOUT_OFFSET; // Timeout timer offset
				    // Reset timers to timeslot generation again  
				    OCR2 =  NORMAL_RXTX_TIMEOUT; 	// Timer 2
				    OCR1A =  NORMAL_FSON_TIMEOUT;	// Timer 1
				    // Reset beacon count
				    noBeaconCount = 0;
				    // In synch with master and timeslot 0 set green LED
				    SET_GLED; 
				}
			}
		}
	}	
} //PKT_ISR

//--------------------------------------------------------------
// Change timeslot and frequency "FSON" TIMER/ Timer 1 Interrupt Service Routine
//--------------------------------------------------------------
SIGNAL(SIG_OUTPUT_COMPARE1A) {
	// Disable packet interrupt from CC2400		
	DISABLE_PKT_INT();
	// Hop to next frequency	
	nextFreq();	
	// If Master and timeslot 0
	if (mySlot == 0) {
	// No Synch 		
		noSync = FALSE;
	// Change timeslot 	
		nextSlot();
	} else {
		// If beacon count
		if (noBeaconCount > NO_BEACON_THRESHOLD) {
			// No Synch 
			noSync = TRUE;
			// Reset timers to beacon count 
			OCR2 = LONG_RXTX_TIMEOUT; // Timer 2: 	RXTX timer
			OCR1A = LONG_FSON_TIMEOUT; // Timer 1: 	FSON timer
			// Clear green LED, if synch is lost
			CLR_GLED;
			// Reset currentslot, when synch is lost
			currentSlot = 0;
		} else {
			SET_GLED;
			noSync = FALSE;
			// Change timeslot again			
			nextSlot();
		}
	}	
} // FSON_TIMER_ISR 
