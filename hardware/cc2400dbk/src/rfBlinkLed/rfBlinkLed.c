/*****************************************************************************
 *                                                                           *
 *        **********                                                         *
 *       ************                                                        *
 *      ***        ***                                                       *
 *      ***   +++   ***                                                      *
 *      ***   + +   ***                                                      *
 *      ***   +                            CHIPCON CC2400                    *
 *      ***   + +   ***                    Example - CC2400	                 *
 *      ***   +++   ***                                                      *
 *      ***       ***                                                        *
 *       ***********                                                         *
 *        *********                                                          *
 *                                                                           *
 *****************************************************************************
 * Device:      CC2400                                                       *
 * File:		rfBlinkLed.c					                             *
 * Author:		MBR						                                 	 *
 * Target:		ATmega8 					                                 *
 * Tools:		WINAVRgcc                               					 *
 * Created:		2004-01-05					     							 * 
 * Description:	Blinking RF example for CC2400 when transmitting and         * 
 *				receiving for a one channel link                             *
 *****************************************************************************
 * Revision history:                                                         *
 *                                                                           *
 * $Log: rfBlinkLed.c,v $
 * Revision 1.3  2004/01/28 08:58:11  mbr
 * Changed: ucInit() to ucInit(CAL_INIT)
 *
 * Revision 1.2  2004/01/20 15:22:48  mbr
 * Change register settings: 250 kbps
 * IOCFG:  0x6030 -> 0x17E0; LMTST: 0x2B22 -> 0x2922
 *
 * Revision 1.1  2004/01/07 13:01:39  tos
 * Initial version in CVS.
 *													 						 *
 * 																			 *
 *									     									 *	
 * 									     									 *
 *									      									 *	
 *                                                                           *
 ****************************************************************************/

// Includes 
#include "include.h" // Master Include File

/*****************************************************************************
    MAIN PROGRAM
*****************************************************************************/

extern volatile uint8 CC2400_Status;

//************************************************************************** 
// Important Note:
// The data rate is configured at compile time by setting in the devboard.h 
// file. The default for CC2400DB is RANGE_1MBPS. This is what is programmed   
// into the application section of the processor flash.
//   
//**************************************************************************
	#if RANGE_1MBPS
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
	#endif
	
	#if RANGE_250KBPS
	// Register settings for RX and TX for 2432 MHz
	RF_SETTINGS rfs   = {
		 0x0010,		// 1.  FSCTRL, Frequency synthesiser main control and status
		 0x0980, 		// 2.  FSDIV, Frequency synthesiser frequency divison control
		 0x0040, 		// 3.  MDMCTRL, Modem main control and status
		 0x000F,		// 4.  FREND, Analog front-end control
		 0x7FF2,		// 5.  RSSI, RSSI information
		 0x17E0, 		// 6.  IOCFG, I/O Configuration register
		 0x7AAF,		// 7.  FSMTC, Finite State Machine time constants
		 0x7FFF,		// 8.  MANAND, Manual signal AND-override register	 	 
		 0x0803,		// 9.  PAMTST, PA and transmit mixers test register
		 0x2922, 		// 10. LMTST, LNA and receive mixers test register	 
		 0x1328, 		// 11. MDMTST0, Modem test register 0
		 0x0028, 		// 12. MDMTST1, Modem test register 1
		 0x0000,		// 13. DACTST, DAC Test register	 
		 0xA210, 		// 14. FSTST0, Test register: VCO array results and override
		 0x1002, 		// 15. FSTST1, Test register: VC DAC manual control, VCO current constant
		 0x0DF0,		// 16. GRMDM, Generic radio modem control 
		 0x0003 		// 17. GRDEC, Generic radio decimation control and status	 	
		 }; 
	#endif

//************************************************************************** 
// Important Note:
// For use of the 10 kbps settings please implement the software according  
// to CC2400 Errata Note 001  
//**************************************************************************	
	#if RANGE_10KBPS
	// Register settings for RX and TX for 2432 MHz
	RF_SETTINGS rfs   = {
		 0x0010,		// 1.  FSCTRL, Frequency synthesiser main control and status
		 0x0980, 		// 2.  FSDIV, Frequency synthesiser frequency divison control
		 0x0020, 		// 3.  MDMCTRL, Modem main control and status
		 0x000F,		// 4.  FREND, Analog front-end control
		 0x7FF2,		// 5.  RSSI, RSSI information
		 0x17E0, 		// 6.  IOCFG, I/O Configuration register
		 0x7AAF,		// 7.  FSMTC, Finite State Machine time constants
		 0x7FFF,		// 8.  MANAND, Manual signal AND-override register	 	 
		 0x0803,		// 9.  PAMTST, PA and transmit mixers test register
		 0x2B22, 		// 10. LMTST, LNA and receive mixers test register	 
		 0x131E, 		// 11. MDMTST0, Modem test register 0
		 0x001E, 		// 12. MDMTST1, Modem test register 1
		 0x0000,		// 13. DACTST, DAC Test register	 
		 0xA210, 		// 14. FSTST0, Test register: VCO array results and override
		 0x1002, 		// 15. FSTST1, Test register: VC DAC manual control, VCO current constant
		 0x0DE0,		// 16. GRMDM, Generic radio modem control 
		 0x0131 		// 17. GRDEC, Generic radio decimation control and status	 	
		 }; 
	#endif
	
// Main - a simple test program
int main(void) {
	
	uint8 length;
	BYTE pTXBuffer[] = {0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39}; 
	BYTE pRXBuffer[20];
	BYTE flags;
	
	// Initialize microcontroller
	ucInit(CAL_INIT);	
		
	// Configure CC2400		
	rfInit(&rfs, TRUE);	
	// Turn on CC2400 PLL, calibrate and go to Idle mode		
	rfOn();
	
	// Forever	
	while(TRUE) {		
					          
        while (!(PINB & (1<<SW_PRESSED))) { // if Push Button is pressed
        	flags = PKT_ACK_REQ;        	
        	//Set TX 1 MHz higher than RX 
	       	FASTSPI_SETREG(CC2400_FSDIV,TX_FREQ);
	       	// Wait 1 millisecond
	        ucWait(1,CLK_FREQ); 
	
    	    // If packet is send toggle transmit LED
    		if (rfSendPacketStateAck(pTXBuffer, 10, &flags)) {
				TOGGLE_YLED;
			} else {
				TOGGLE_RLED;
			}				
			// Wait 10 milliseconds 	
    		ucWait(10,CLK_FREQ);
    	}	        
        // Fixed packet length maximum 20 bytes
        length = 20;
        
        //Set RX 1 MHz lower than RX 
	     FASTSPI_SETREG(CC2400_FSDIV,RX_FREQ);
        	
        // Wait 1 millisecond
        ucWait(1,CLK_FREQ); 
        
        // If received packet toggle RX LED;
	    if (rfReceivePacket(pRXBuffer, &length, &flags)) {
			TOGGLE_GLED;
	    }    
    }
}

