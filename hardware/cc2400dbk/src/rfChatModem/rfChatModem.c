/*****************************************************************************
 *                                                                           *
 *        **********                                                         *
 *       ************                                                        *
 *      ***        ***                                                       *
 *      ***   +++   ***                                                      *
 *      ***   + +   ***                                                      *
 *      ***   +                            CHIPCON CC2400                    *
 *      ***   + +   ***                    Example - CC2400DBK	             *
 *      ***   +++   ***                                                      *
 *      ***       ***                                                        *
 *       ***********                                                         *
 *        *********                                                          *
 *                                                                           *
 *****************************************************************************
 * Device:      CC2400                                                       *
 * File:		rfChatModem.c					                             *
 * Author:		MBR						                                     *
 * Target:		ATmega8 					                                 *
 * Tools:		WinAVRGCC                                   				 *
 * Created:		2004-01-21					                                 *
 * Description:	RF Chat Modem example for CC2400 transmitting and		 	 *         		 * 
 *		        receiving for a one channel link                             *
 *																			 *
 *																			 *
 * This program demonstrates the use of the library, timers, and the		 *
 * serial port. The program is mainly interrupt driven and use HyperTerminal *
 * to communicate using the serial port
 *                                                                           *
 * A pair of CC2400DBKs running this program will function as a wireless	 *
 * chat modem cable, between serial port on each board.                      *
 *                                                                           *
 *                                                                           *
 * +----+            +--------+                 +--------+            +----+ *
 * |    |  UART0 RX  |        |                 |        |  UART0 RX  |    | *
 * |    |----------->|        |      RF      	|		 |<-----------|    | *
 * | PC |            | CC2400 |<--------------->| CC2400 |            | PC | *
 * |    |  UART0 TX  |        |   			    |        |  UART0 TX  |    | *
 * |    |<-----------|        |                 |        |----------->|    | *
 * +----+            +--------+                 +--------+            +----+ *
 *                                                                           *
 *        19200 baud            RF @ 1Mbps            	   19200 baud        *
 *                                                                           *
 *                                                                           *
 * FEATURES:                                                                 *
 *     - RS232 baud rate: 19.2 kBaud, 8 data bits,(no parity bit)			 *
 *						  2 stop bits and no flow control 					 *                                	
 *     - Max RF baud rate: 1 Mbps              		                     	 * 
 *     - Serial input/output while the transceiver is active (through use of *
 *       interrupts)                                                         *
 *                                                   						 *
 * LED INDICATORS:                                                           *
 *     - Yellow: Toggeled when transmitting each packet                      *                          *
 *     - Green:  Toggled for each received packet                            *
 *                                                                           *
 * HOW IT WORKS:                                                             *
 * The timer 2 takes care of changing states of the state machine. The three * 
 * states are idle, Tx, and Rx. Timer2 together with the different ISR take	 * 
 * care of RF transmission and reception, and UART transmission and			 * 
 * reception.																 *
 * 													            			 *
 * RF->UART:                                                                 *
 *		When an RF packet has been received, the RF RX packet data is		 * 
 *		transmitted over the UART.											 *
 *                                                                           *
 * UART->RF:                                                                 *
 *     When the first byte is received, a start RF starts to transmit.       *
 *     Everything received on UART is transmitted in the RF packet     		 *                                                        *
 *																			 *
 *****************************************************************************
 * Revision history:                                                         *
 *                                                                           *
 * $Log: rfChatModem.c,v $
 * Revision 1.2  2004/01/28 09:01:24  mbr
 * Cosmetic changes;
 * Bug Fix: Enabling/Disabling of interrupts
 *
 * Revision 1.1  2004/01/21 08:54:26  mbr
 * Initial version in CVS.
 *
 * 
 *										     								 *	
 * 									     									 *
 *									     									 *	
 *                                                                           *
 ****************************************************************************/

// Includes 
#include "include.h" // Master Include File

// When using 0 MHz offset TX must be 1 MHz higher than RX
#define TXFREQ		0x0981                                                 
#define RXFREQ		TXFREQ	- 1

// State variables for Next State
typedef enum {IDLE_STATE = 0, TX_STATE = 1, RX_STATE = 2} State_Type;

// FSM State function
void Idle (void);
void Tx (void);
void Rx (void);
	
// Table that contains a pointer to the function to call in each state
void(*state_table[])() = {Idle, Tx, Rx};

// State machine states 
State_Type Next_State;
State_Type State;

// Global variables
extern volatile uint8 CC2400_Status;
BOOL rxPacketComplete, rxinit;

//----------------------------------------------------------------------------
// Important Note:
// The data rate settings are for 1Mbps. Regsister settings are taken   
// from SmartRF Studio.
//----------------------------------------------------------------------------
// Register settings for RX and TX for 2432 MHz
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
	 0x0DF0,		// 16. GRMDM, Generic radio modem control 
	 0x0000 		// 17. GRDEC, Generic radio decimation control and status	 
};

// Transmisson timeout
sint16 TxTimer;
#define TX_TIMEOUT      10

// Receive timeout
sint16 RxTimer;
#define RX_TIMEOUT      10

// Buffers for transmitted and received data
// These are put into different banks so that they can be as large as possible
// The TX buffer is a ring buffer filled up with data to be sent in the next data packet
// The RX buffer is a ring buffer in which received data is stored and 
// sent to the UART

BYTE pTXBuffer[TX_BUFFER_SIZE];
BYTE pRXBuffer[RX_BUFFER_SIZE];

// Global variables for RF Modem
uint8 n;
uint8 length; 
BYTE flags;

// Static data variables 
static uint8 RF_RxHead;
static uint8 RF_RxTail;
static uint8 RF_TxHead;
static uint8 RF_TxTail;

uint8 tmplength; 

//----------------------------------------------------------------------------
// RX_TX freq change   
//----------------------------------------------------------------------------
void RxTxFreq(void) {
	// Turn off PLL and go to Idle mode
	FASTSPI_STROBE(CC2400_SRFOFF); 
	// Program frequency
	if (State == TX_STATE) {
		// 0 MHz IF TX, TX = RX + 1
		FASTSPI_SETREG(CC2400_FSDIV, TXFREQ);
	} else {
		// 0 MHz IF RX, RX = TX - 1
		FASTSPI_SETREG(CC2400_FSDIV, RXFREQ);
	}	
	// Turn on PLL, calibrate and go to 	
	FASTSPI_STROBE(CC2400_SFSON);
	
	// Check lock
	while(!(PINC & (1<<GIO1))); 
     
}// RxTxFreq
//----------------------------------------------------------------------------
// MAIN PROGRAM
//---------------------------------------------------------------------------- 
int main(void) {
     
	// Initialize microcontroller with UART 	
	ucInit(UART_INIT);
	
	// Configure CC2400	
	rfInit(&rfs, TRUE);	
 
 	//Startup Message
	writeln_p(PSTR("RF Modem Chat Ready"));
	writeln_p(PSTR(__DATE__));
	writeln_p(PSTR(__TIME__));

	// Turn on CC2400 PLL, calibrate and go to Idle mode		
	rfOn();
		
	// Initialize timer counters
	TxTimer = TX_TIMEOUT;
	RxTimer = RX_TIMEOUT;
	
	// Flush buffer index counters 
  	RF_RxTail = 0;
  	RF_RxHead = 0;
  	RF_TxTail = 0;
  	RF_TxHead = 0;
	
	// Force State and Next State 
	State = IDLE_STATE;
	Next_State = IDLE_STATE;
	
 // 1. Turn on timer for 1 ms timeouts
 // 2. Set Waveform Genertion Mode to Clear Timer on Compare (CTC)
 // 3. Set prescaling to clk/1024
	ucTimer2Init(T1_MS, CLR_T2_CMP|T2_DIV_1024);
  
    // Forever 
    // FSM State Machine controlled by timer 2  			
	for(;;){}
                 			       	
} // main
//----------------------------------------------------------------------------
//  PKT ISR 
//---------------------------------------------------------------------------- 
SIGNAL(SIG_INTERRUPT1) {	  
	// Clear external interrupt 1 request flag 
       CLR_INT1_IRQ();
    //If State is TX state    
	if (State == TX_STATE) {
		// Transmit, do nothing 
	} else {
		  // Start to read out data from FIFO			                      
           rfEndReceiveBuf(pRXBuffer+RF_RxHead, &tmplength, &flags);              
           // Calculate incoming RF RX data index 
           RF_RxHead = (RF_RxHead + tmplength) % RX_BUFFER_SIZE;	
           // Set rxPacketComplete software flag 
           rxPacketComplete = TRUE;
           // Toggle RX LED 
           TOGGLE_GLED;    
	}  	
}// PKT_ISR
//----------------------------------------------------------------------------
//  UART RX ISR, UART RX -> RF 
//----------------------------------------------------------------------------
SIGNAL(SIG_UART_RECV) {
 // RF TX
      UART0_RX_CLR_IRQ();
      // Store received data in buffer 
      UART0_RX(pTXBuffer[RF_TxHead]);  
      RF_TxHead = (RF_TxHead + 1) % TX_BUFFER_SIZE;
      if (RF_TxHead == RF_TxTail) {
      }	 	         
} // USART_RXC_ISR
//----------------------------------------------------------------------------
// UART TX interrupt, RF RX -> UART TX
//----------------------------------------------------------------------------
SIGNAL(SIG_UART_DATA) {	
	UART0_TX_CLR_IRQ();        
  // RF RX
  // Check if all data is transmitted 
  if (RF_RxHead != RF_RxTail) { 
     UART0_TX(pRXBuffer[RF_RxTail]);  // Start transmission 
     // Calculate buffer index 
     RF_RxTail = (RF_RxTail + 1) % RX_BUFFER_SIZE;    
  } else {
      UART0_INT_TX_DISABLE();   // Disable UDRE interrupt 
  } 
} // USART_TX_ISR
//----------------------------------------------------------------------------
//  Timer 2 ISR, used to generated Next State for FSM
//----------------------------------------------------------------------------
SIGNAL(SIG_OUTPUT_COMPARE2) {
	if(Next_State == TX_STATE) {				    
            if (--TxTimer < 0) {
                  TxTimer = TX_TIMEOUT;	  // Reload TX Timer   
                  state_table[Next_State](); // Change to Next State
            } 
	} else if(Next_State == RX_STATE) {			   
            if (--RxTimer < 0) {
                 RxTimer = RX_TIMEOUT; // Reload Rx Timer
                 state_table[Next_State](); // Change to next state
            }
        } else {
            state_table[Next_State](); // No timeout go to next state            
        }  
} // TIMER2_ISR
//----------------------------------------------------------------------------
//  FSM State Idle 
//---------------------------------------------------------------------------- 
void Idle (void) {
	
	// Set Current State to IDLE state  
	State = IDLE_STATE;
	      
	// If received UART data, transmit it on RF 
	if(RF_TxHead != RF_TxTail) {  
	      Next_State = TX_STATE;	        	       
	      rxinit = FALSE;	
	} else if(!rxinit) {		
		// Change to RX frequency 
		RxTxFreq(); 								
		// Enable Packet interrupt
		ENABLE_PKT_INT();
		// Turn on RX 
		rfBeginReceive();       
		// PacketComplete flag
		rxPacketComplete = FALSE;        
		// Set RX mode initflag
		rxinit = TRUE;              
	} else {
	  if(rxPacketComplete) {
	    Next_State = RX_STATE;	
	  }
	}
	// Enable UART interrupts		
	UART0_INT_RX_ENABLE();			
} // Idle
//----------------------------------------------------------------------------
//  FSM State Tx
//---------------------------------------------------------------------------- 
void Tx (void) {

	// Toggle Transmit LED
	TOGGLE_YLED; 
   	
  	// Set current state to RF TX state
	State = TX_STATE;
	
	// Change to RX or TX frequency
	RxTxFreq();		
 
	// If new received UART data copy UART data into RF packet		 
	// Wait for data from UART		
	if (RF_TxHead != RF_TxTail) { 
	    if(rfSendPacketLength(pTXBuffer+RF_TxTail, (RF_TxHead - RF_TxTail + TX_BUFFER_SIZE) % TX_BUFFER_SIZE, &flags)) {		        				
               Next_State = RX_STATE;
               RF_TxTail = RF_TxHead; 
            }
        } else {
            // Go back to Idle State
            Next_State = IDLE_STATE;
        }

} // Tx
//----------------------------------------------------------------------------
//  FSM State Rx
//---------------------------------------------------------------------------- 
void Rx (void) {
	// Set current state to RF RX state	
    State = RX_STATE;
        
    // If received RF packet
    if(rxinit == TRUE){
    	// Enable UDRE interrupt  
  	   	UART0_INT_TX_ENABLE();   
  	   
        // Transmit (on UART) the first byte that was received on RF
 	    UART0_TX(pRXBuffer[RF_RxTail]);
 	    RF_RxTail = (RF_RxTail + 1) % RX_BUFFER_SIZE;  	           
    }
	rxinit = FALSE;
    
    // If UART done transmitting
 	if(RF_RxTail == RF_RxHead){
	    // Set Next State to Idle state
        Next_State = IDLE_STATE;
     }        
} // Rx
