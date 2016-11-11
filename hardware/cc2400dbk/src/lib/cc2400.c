/*****************************************************************************
 *                                                                           *
 *        **********                                                         *
 *       ************                                                        *
 *      ***        ***                                                       *
 *      ***   +++   ***                                                      *
 *      ***   + +   ***                                                      *
 *      ***   +                            CHIPCON CC2400                    *
 *      ***   + +   ***                    CC2400 function file	             *
 *      ***   +++   ***                                                      *
 *      ***       ***                                                        *
 *       ***********                                                         *
 *        *********                                                          *
 *                                                                           *
 *****************************************************************************
 * Device:      CC2400                                               		 *
 * File:		CC2400.c					                                 *
 * Author:		MBR						                                     *
 * Target:		ATmega8 					                                 *
 * Tools:		WINAVRgcc                                       			 *
 * Created:		2003-07-31					                                 *
 * Description:	Low level function for CC2400                                * 
 *		                                                                     *
 *****************************************************************************
 * Revision history:                                                         *
 *                                                                           *
 * $Log: cc2400.c,v $
 * Revision 1.3  2004/01/28 09:30:28  mbr
 * 1. Cosmetic changes
 * 2. Removed rfBeginReceiveInit() function
 * 3. Changed PLL LOCK check for the rfOn() function
 *
 * Revision 1.2  2004/01/20 15:19:01  mbr
 * Cosmetic Changes
 *
 * Revision 1.1  2004/01/07 13:09:43  tos
 * Initial version in CVS.
 *
 * 
 *									                                         *	
 * 									                                         *
 *									                                         *	
 *                                                                           *
 ****************************************************************************/

// Includes 
#include "include.h" // Master Include File

// Global example variables
volatile uint8 CC2400_Status = 0;
BOOL rxPacketComplete, rxinit;

//----------------------------------------------------------------------------
//  void rfInit(...)
//
//  Description:
//      A RF initialization function, that configure 
//      the CC2400. Used to configure CC2400 for RX and TX
//
//  Arguments:
//      byte *pRFS 
//		Data table that entails register settings given by SmartRF Studio 
//          
//		BOOL reset
//			To set if reset is desired or not
//  Return value:
//      void
//----------------------------------------------------------------------------
void rfInit(RF_SETTINGS *pRFS, BOOL reset) {
	
	// Optional reset
	if (reset) {
		FASTSPI_SETREG(CC2400_MAIN,0x0000);
		FASTSPI_SETREG(CC2400_MAIN,0x8000);
	}
	
	// Register settings
	FASTSPI_SETREG(CC2400_FSCTRL, pRFS->FSCTRL);	// 1. 
	FASTSPI_SETREG(CC2400_FSDIV, pRFS->FSDIV);		// 2.  
	FASTSPI_SETREG(CC2400_MDMCTRL, pRFS->MDMCTRL);	// 2.
	FASTSPI_SETREG(CC2400_FREND, pRFS->FREND);		// 4.
	FASTSPI_SETREG(CC2400_RSSI, pRFS->RSSI);		// 5.
	FASTSPI_SETREG(CC2400_IOCFG, pRFS->IOCFG);		// 6.
	FASTSPI_SETREG(CC2400_FSMTC, pRFS->FSMTC);		// 7.
	FASTSPI_SETREG(CC2400_MANAND,pRFS->MANAND);		// 8.		
	FASTSPI_SETREG(CC2400_PAMTST, pRFS->PAMTST);	// 9. 
	FASTSPI_SETREG(CC2400_LMTST, pRFS->LMTST);		// 10.
	FASTSPI_SETREG(CC2400_MDMTST0, pRFS->MDMTST0);	// 11.
	FASTSPI_SETREG(CC2400_MDMTST1, pRFS->MDMTST1);	// 12.
	FASTSPI_SETREG(CC2400_DACTST,pRFS->DACTST);	    // 13.
	FASTSPI_SETREG(CC2400_FSTST0, pRFS->FSTST0);	// 14.	
	FASTSPI_SETREG(CC2400_FSTST1, pRFS->FSTST1);	// 15.
	FASTSPI_SETREG(CC2400_GRMDM, pRFS->GRMDM);		// 16. 
	FASTSPI_SETREG(CC2400_GRDEC, pRFS->GRDEC);		// 17.
	 
} // rfInit
//----------------------------------------------------------------------------
//  void rfOn(...)
//
//  Description: Used turn on crystal oscillator and check if it is  
//				 stabilized by using the status byte. This function also turns
//				 the frequency synthesizer and use the status byte to check for
//				 lock. 					
//  Arguments:
//		None
//  Return value:
//      None
//----------------------------------------------------------------------------	
void rfOn(void) {
	
	// Turn on XOSC
	FASTSPI_STROBE(CC2400_SXOSCON);	
	// Wait for the crystal oscillator to be stable
	
	do { 
		FASTSPI_UPD_STATUS();         
	} while (!(CC2400_Status & (1 << XOSC16M_STABLE)));   
			
	// Turn on FS
	FASTSPI_STROBE(CC2400_SFSON);	

	// Wait for the synthesizer to lock   
	do { 
		FASTSPI_UPD_STATUS();	 
	} while (!(CC2400_Status & (1 << FS_LOCK)));		
} // rfOn
//----------------------------------------------------------------------------
//  void rfBeginReceive(...)
//
//  Description:  Used to turn off PLL, and go to Idle mode 
//				
//  Arguments:
//		None
//  Return value:
//      None
//----------------------------------------------------------------------------
void rfOff(void) {
	FASTSPI_STROBE(CC2400_SRFOFF);
} // rfOff

//----------------------------------------------------------------------------
//  BOOL rfSendPacketStateAck(...)
//
//  Description:
//      Used to send a packet using the current RF configuration. This function
//		also checks if we are in the correct state before we transmit.
//		The first bytes are inserted by CC2400 such as the preamble followed 
//		by a synchronization byte, the _length_ byte, the flags pointed to by 
//		the _pFlags, and the data pointed to by _pData_, a
//
//  Arguments:
//      BYTE *pData
//          A pointer to the actual data to transmit.
//      uint8 length
//          The number of bytes to transmit. 
//	 	BYTE *pFlags
//          A pointer to the packet flags to transmit.
//
//  Return value:
//      TRUE or FALSE
//----------------------------------------------------------------------------
BOOL rfSendPacketStateAck(BYTE *pData, uint8 length, BYTE *pFlags) {
	uint8 n, temp,dummy;
	uint16 ackWaitCnt = 2000;

	// Make sure that we're ready to transmit (FSM = 15)
	FASTSPI_GETREG(CC2400_FSMSTATE, n);
	n &= 0x1F;
	if (n != 15) return FALSE;
		
	// Fill FIFO and begin transfer
	// The packet length is inserted as the first byte of the packet
    FASTSPI_WRITE_FIFO(&length,1,n);
    FASTSPI_WRITE_FIFO(pFlags,1,n);
    FASTSPI_WRITE_FIFO(pData,length,n);
	FASTSPI_STROBE(CC2400_STX);	
	// Wait for transmission to end
	while (PIND & (1<<PKT));
    
	// Wait for acknowledge?
	if (*pFlags & PKT_ACK_REQ) {
		
		//Set RX 1 MHz lower than TX 
	    FASTSPI_SETREG(CC2400_FSDIV,RX_FREQ);
        	
        // Wait 1 millisecond
        ucWait(1,CLK_FREQ); 
        
        
        // Turn on RX again and timeout if no acknowledgement received 			
		FASTSPI_STROBE(CC2400_SRX);
		// Wait until packet signal goes low and indicates synch received
		while ((--ackWaitCnt) && (PIND & (1<<PKT)));		
		if (!ackWaitCnt) {
			// Turn on synthesizer
			FASTSPI_STROBE(CC2400_SFSON);
			return FALSE;
		}
		// Wait for packet to go high
		while(!(PIND & (1<<PKT)));
		//Readout length
		FASTSPI_READ_FIFO(&temp,1,n);
		// Readout flags 
		FASTSPI_READ_FIFO(pFlags,1,n);
		//Update CC2400 global Status Byte
		FASTSPI_UPD_STATUS();
		//Check ackknowledgement and CRC
		return ((*pFlags & PKT_ACK) && (CC2400_Status & (1 << STATUS_CRC_OK)));
	} else {
		return TRUE;
	}
} 

//----------------------------------------------------------------------------
//  void rfSendPacket(...)
//
//  Description:
//      Used to send a packet using the current RF configuration.
//		The first bytes are inserted by CC2400 such as the preamble followed 
//		by a synchronization byte, the _length_ byte, the flags pointed to by 
//		the _pFlags, and the data pointed to by _pData_, a
//
//  Arguments:
//      BYTE *pData
//          A pointer to the actual data to transmit.
//      uint8 length
//          The number of bytes to transmit. 
//	 	BYTE *pFlags
//          A pointer to the packet flags to transmit.
//
//  Return value:
//      void
//----------------------------------------------------------------------------
void rfSendPacket(BYTE *pData, uint8 length, BYTE *pFlags) {
	uint8 n;
	// Turn on RF TX mode 
	FASTSPI_STROBE(CC2400_STX);
	
    // Write length of packet to CC2400 FIFO
    FASTSPI_WRITE_FIFO(&length,1,n);
    
    // Write packet flags to CC2400 FIFO  
    FASTSPI_WRITE_FIFO(pFlags,1,n);
    
    // Write packet payload data to CC2400 FIFO  
    FASTSPI_WRITE_FIFO(pData,length,n);
      
    // Wait for packet to be transmitted.         
	while (PIND & (1<<PKT));

} //rfSendPacket

//----------------------------------------------------------------------------
//  BOOL rfSendPacketLength(...)
//
//  Description:
//      Used to send a packet using the current RF configuration. This function
//		also checks if we are in the correct state before we transmit.
//		The first bytes are inserted by CC2400 such as the preamble followed 
//		by a synchronization byte, the _length_, the flags pointed to by 
//		the _pFlags, and the data pointed to by _pData_, a
//
//  Arguments:
//      BYTE *pData
//          A pointer to the actual data to transmit.
//      uint8 length
//          The number of bytes to transmit. 
//	 	BYTE *pFlags
//          A pointer to the packet flags to transmit.
//
//  Return value:
//      Length of packet
//----------------------------------------------------------------------------
BOOL rfSendPacketLength(BYTE *pData, uint8 length, BYTE *pFlags) {
    uint8 n;
 
     // Turn on RF TX mode 
     FASTSPI_STROBE(CC2400_STX);
    
    // Write length of packet to CC2400 FIFO
      FASTSPI_WRITE_FIFO_BYTE(length);
       
    // Write packet flags to CC2400 FIFO    
      FASTSPI_WRITE_FIFO_BYTE(*pFlags); 
    
    // Write packet payload data to CC2400 FIFO with BUF limit 
     FASTSPI_WRITE_FIFO_BUF(pData,length,n);

    // Wait for packet to be transmitted.    
    while (PIND & (1<<PKT));
    
    // Return length of packet
    return length;
} // rfSendPacketLength
//----------------------------------------------------------------------------
//  void rfBeginReceive(...)
//
//  Description: Used only to turn on RX mode 
//				
//  Arguments:
//		None
//  Return value:
//      None
//----------------------------------------------------------------------------	
void rfBeginReceive(void) {
	FASTSPI_STROBE(CC2400_SRX);
}// rfBeginReceive	

//------------------------------------------------------------------------------
//  BOOL rfEndReceiveBuf(...)
//
//  Description:
//      Used to receive a packet using the current RF configuration. Uses the 
//		SPI macros with a given buffer size. 
//		The first byte that is read from CC2400 the length pointed to by _length, 
//		the flags pointed to by the _pFlags, and the data pointed to by _pData_
//
//  Arguments:
//      BYTE *pData
//          A pointer to the actual data to transmit.
//      uint8 *length
//          A pointer to the length of packet data. 
//	 	BYTE *pFlags
//          A pointer to the packet flags to transmit.
//
//  Return value:
//      TRUE or FALSE, If CRC is OK or not
//------------------------------------------------------------------------------
BOOL rfEndReceiveBuf(BYTE *pData, uint8 *length, BYTE *pFlags) {

	uint8 n;
	
    while (!(PIND & (1<<PKT)));
	
	// Read received packet length
	FASTSPI_READ_FIFO_BUF(length,1,n);
	
        // Read flags from received packet	 
	FASTSPI_READ_FIFO_BUF(pFlags,1,n);

        // Read out data from packet
	FASTSPI_READ_FIFO_BUF(pData,*length,n);
	
	// Check CRC
	FASTSPI_UPD_STATUS();
	return !!(CC2400_Status & (1 << STATUS_CRC_OK));	

} //rfEndReceiveBuf
//----------------------------------------------------------------------------
//  BOOL rfEndReceive(...)
//
//  Description:
//      Used to receive a packet using the current RF configuration. Uses the SPI macros
//      with a given buffer size. 
//		The first byte that is read from CC2400 the length pointed to by _length, 
//		the flags pointed to by the _pFlags, and the data pointed to by _pData_
//
//  Arguments:
//      BYTE *pData
//          A pointer to the actual data to transmit.
//      uint8 *length
//          A pointer to the length of packet data. 
//	 	BYTE *pFlags
//          A pointer to the packet flags to transmit.
//
//  Return value:
//      TRUE or FALSE, If CRC is OK or not
//----------------------------------------------------------------------------
BOOL rfEndReceive(BYTE *pData, uint8 *length, BYTE *pFlags) {
	uint8 n, actualLength;
	while (!(PIND & (1<<PKT)));
	
	FASTSPI_READ_FIFO(&actualLength,1,n);
    if (actualLength > *length) return FALSE;
	 
	FASTSPI_READ_FIFO(pFlags,1,n);
		
	FASTSPI_READ_FIFO(pData,actualLength,n);
	*length = actualLength;
	
	// Check CRC
	FASTSPI_UPD_STATUS();
	return !!(CC2400_Status & (1 << STATUS_CRC_OK));	

}//rfEndReceive 

//----------------------------------------------------------------------------
//  BOOL rfReceivePacket(...)
//
//  Description:
//      Used to receive a packet using the current RF configuration. This function
//		also checks if we are in the correct state before we transmit.
//		The first byte that is read from CC2400 the length pointed to by _length, 
//		the flags pointed to by the _pFlags, and the data pointed to by _pData_
//
//  Arguments:
//      BYTE *pData
//          A pointer to the actual data to transmit.
//      uint8 *length
//          A pointer to the length of packet data. 
//	 	BYTE *pFlags
//          A pointer to the packet flags to transmit.
//
//  Return value:
//      TRUE or FALSE, If CRC is OK or not
//----------------------------------------------------------------------------
BOOL rfReceivePacket(BYTE *pData, uint8 *pLength, BYTE *pFlags) {
	uint8 n, temp, dummy; 
	uint8 actualLength;
	BOOL result;
		
	// Make sure that we're ready to receive (FSM = 15)
	FASTSPI_GETREG(CC2400_FSMSTATE, n);
	n &= 0x1F;
	if (n != 15) return FALSE;
	
	// Begin reception
	FASTSPI_STROBE(CC2400_SRX);
	
	// Wait for incoming packet (can be aborted by SW2)
	while(PIND & (1<<PKT)) {
		if (!(PINB & (1<<SW_PRESSED))) {
			FASTSPI_STROBE(CC2400_SFSON);
			return FALSE;
		}
	}
	while(!(PIND & (1<<PKT)));
	
	// Read packet length, then data
	FASTSPI_READ_FIFO(&actualLength,1,n);
	FASTSPI_READ_FIFO(pFlags,1,n);
	
	// Check length
	if (actualLength > *pLength) {
		FASTSPI_STROBE(CC2400_SFSON);
		return FALSE;
	}
	// Length OK 
	FASTSPI_READ_FIFO(pData,actualLength,n);
	*pLength = actualLength;

	// Check CRC
	FASTSPI_UPD_STATUS();
	result = !!(CC2400_Status & (1 << STATUS_CRC_OK));	

	// Return acknowledge?
	if (result && (*pFlags & PKT_ACK_REQ)) {
		// Set flag bit
		*pFlags |= PKT_ACK;		
		temp = 0;
		//Set TX 1 MHz higher than RX 
	    FASTSPI_SETREG(CC2400_FSDIV,0x0981);       	
        // Wait 1 millisecond
        ucWait(1,CLK_FREQ); 
        // Send length			
	    FASTSPI_WRITE_FIFO(&temp,1,n);
	    // Send flags
	    FASTSPI_WRITE_FIFO(pFlags,1,n);
	    // Turn on TX mode
		FASTSPI_STROBE(CC2400_STX);
		// Wait until packet is transmitted
		while (PIND & (1<<PKT));
	}
	// Return CRC OK
	return result;
} // rfReceivePacket
