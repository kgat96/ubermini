/*******************************************************************************************************
 *                                                                                                     *
 *        **********                                                                                   *
 *       ************                                                                                  *
 *      ***        ***                                                                                 *
 *      ***   +++   ***                                                                                *
 *      ***   + +   ***                                                                                *
 *      ***   +                                   CHIPCON CC2400DBK BOOTLOADER                         *
 *      ***   + +   ***                          Demonstration board bootloader                        *
 *      ***   +++   ***                                                                                *
 *      ***        ***                                                                                 *
 *       ************                                                                                  *
 *        **********                                                                                   *
 *                                                                                                     *
 *******************************************************************************************************
 * This program allows an AVR with bootloader capabilities to read and write to its own Flash/EEPROM.  * 
 * It contains the AVR C and EC++ startup routine and must be tailored to suit customers's hardware.   * 
 *                                                                                                     *
 * Please that this code is based on Atmel Corporation source code from their application note AVR109, *
 * Self Programming                                                                                    *
 *                                                                                                     *
 * INSTRUCTIONS:                                                                                       *
 * To enter Programming mode the programming port pin PE5 is checked. If this pin is pulled low by     *
 * holding the joystick push button down after reset, programming mode is entered. If not,             *
 * normal execution is done from address 0000 "reset" vector in Application area. The PE5 pin should   *
 * be pulled HIGH by an external pull-up resistor.                                                     *
 *                                                                                                     *
 * The defines.h file must be set up for individual devices using the excel sheet preprocessor.xls. In *
 * general the linker file should be always be verified top match the used part's boot address and     *
 * size. Note that memory size is specified in bytes in the linker file                                *
 *                                                                                                     *
 *******************************************************************************************************
 * Compiler: IAR EW                                                                                   *
 * Target platform: CC2400DB (can easily be ported to other platforms)                                 *
 *******************************************************************************************************
 * Revision history:                                                                                   *
 * $Log: bootloader.c,v $
 * Revision 1.1  2004/07/20 12:22:19  mbr
 * Bootloader according application note AVR109,
 * Self Programming with only C - code
 *
 * 
 * 
 *                                                                              *
 *                                                                                                     *
 *******************************************************************************************************/
#include <inavr.h>
#include "defines.h"
#include "serial.h"


// Uncomment the following to save code space 
//#define REMOVE_AVRPROG_SUPPORT
//#define REMOVE_FUSE_AND_LOCK_BIT_SUPPORT
#define REMOVE_BLOCK_SUPPORT
//#define REMOVE_EEPROM_BYTE_SUPPORT
//#define REMOVE_FLASH_BYTE_SUPPORT

#ifndef REMOVE_BLOCK_SUPPORT
  UINT8 BlockLoad(UINT16 size, UINT8 mem,  long *address);
  void  BlockRead(UINT16 size, UINT8 mem,  long *address);

  // BLOCKSIZE should be chosen so that the following holds: BLOCKSIZE*n = PAGESIZE,  where n=1,2,3... 
  #define BLOCKSIZE PAGESIZE
#endif

//-------------------------------------------------------------------------------------------------------
//	void main (void)
//
//	DESCRIPTION:
//		Bootloader routine and main loop
//-------------------------------------------------------------------------------------------------------
__C_task void main(void) {
    
    UINT8 val;
    UINT16 temp_int;
    long address;
  
    // Set up function pointer to RESET vector
    void (*funcptr)(void) = 0x0000;       
     // initialize uart
    initBootUart();
    // Enable pull-up on PROGCTRL line on PROGPORT                       
    PROGPORT |= PROGCTRL();                 	
    // If PROGPIN is pulled low: programmingmode. 
    if(PROGMODE()) {  
      for(;;) {
        // Wait for command character.    
        val=recChar();
        // Check autoincrement status.
        if(val == 'a') {
          // Yes, we do autoincrement.  
          sendChar('Y');		   
          // Set address.  
        } else if(val == 'A') {
          // NOTE: Flash addresses are given in words, not bytes.   
          // read address 8 MSB 
          // address = recChar();                
          address = (recChar() << 8)| recChar();
          // Send OK back.
          sendChar('\r'); 
         // Chip erase.
        } else if(val == 'e') {              
              for(address = 0; address < APP_END; address += PAGESIZE) {
                // NOTE: Here we use address as a byte-address, not word-address, for convenience.
                _WAIT_FOR_SPM();
                _PAGE_ERASE(address); 
              }
              // Send OK back.
              sendChar('\r');
       }
#ifndef REMOVE_BLOCK_SUPPORT
        // Check block load support.
         else if(val == 'b') {
              // Report block load supported.
              sendChar('Y');
              // MSB first. 
              sendChar((BLOCKSIZE >> 8) & 0xFF);
               // Report BLOCKSIZE (bytes). 
              sendChar(BLOCKSIZE & 0xFF);
        // Start block load.
        } else if(val == 'B') {
              // Get block size.
              temp_int = (recChar() << 8) | recChar();
              // Get memtype. 
              val = recChar();
              // Block load. 
              sendChar(BlockLoad(temp_int, val , &address));
         // Start block read.       
        } else if(val == 'g') {
              // Get block size.
              temp_int = (recChar() << 8) | recChar();
              // Get memtype 
              val = recChar();
              // Block read 
              BlockRead(temp_int, val, &address);
       } 
#endif 
#ifndef REMOVE_FLASH_BYTE_SUPPORT            
       // Read program memory.
       else if(val=='R') {
            // Send high byte, then low byte of flash word.
            _WAIT_FOR_SPM();        
            _ENABLE_RWW_SECTION();
             sendChar( _LOAD_PROGRAM_MEMORY( (address << 1)+1 ) );
             sendChar( _LOAD_PROGRAM_MEMORY( (address << 1)+0 ) );
             // Auto-advance to next Flash word.
             address++;
       // Write program memory, low byte.
       // NOTE: Always use this command before sending high byte.       
       } else if(val == 'c') {
            // Get low byte for later _FILL_TEMP_WORD.
            temp_int = recChar();
            // Send OK back. 
            sendChar('\r');
        // Write program memory, high byte.     
       } else if(val == 'C') {
            // Get and insert high byte.
            temp_int |= (recChar() << 8); 
            _WAIT_FOR_SPM();
            // Convert word-address to byte-address and fill.
            _FILL_TEMP_WORD( (address << 1), temp_int );
            // Auto-advance to next Flash word. 
            address++;
            // Send OK back. 
            sendChar('\r');
       // Write page.       
       } else if(val == 'm') {
            // Protect bootloader area.
            if(address >= (APP_END >> 1)) {
                sendChar('?');
            } else {
                  _WAIT_FOR_SPM();
                  // Convert word-address to byte-address and write.
                  _PAGE_WRITE( address << 1 );  
            }
            // Send OK back.
             sendChar('\r');
       }      
#endif            
#ifndef REMOVE_EEPROM_BYTE_SUPPORT
       // Write EEPROM memory.
       else if (val == 'D') {
            _WAIT_FOR_SPM();
            // Setup EEPROM address.        
            EEARL = address; 
            EEARH = (address >> 8);
            // Get byte.
            EEDR = recChar();
            // Write byte. 
            EECR |= (1 << EEMWE); 
            EECR |= (1 << EEWE );
             // Wait for write operation to finish.
            while (EECR & (1 << EEWE));           
            // Auto-advance to next EEPROM byte.
            address++;
            // Send OK back. 
            sendChar('\r');
       // Read EEPROM memory.     
       }  else if (val == 'd') {
              // Setup EEPROM address.
              EEARL = address; 
              EEARH = (address >> 8);
              // Read byte...
              EECR |= (1<<EERE);
              // ...and send it back. 
              sendChar(EEDR);
              // Auto-advance to next EEPROM byte. 
              address++; 
       }
#endif
               
#ifndef REMOVE_FUSE_AND_LOCK_BIT_SUPPORT
       // Write lockbits.
       else if(val == 'l') {
            _WAIT_FOR_SPM();
            // Read and set lock bits.        
            _SET_LOCK_BITS(recChar());
            // Send OK back. 
            sendChar('\r');
       // Read lock bits.
       }  else if(val == 'r') {
              _WAIT_FOR_SPM();        
              sendChar(_GET_LOCK_BITS());       
        // Read fuse bits.
       }  else if(val == 'F') {
              _WAIT_FOR_SPM();        
              sendChar(_GET_LOW_FUSES()); 
        // Read high fuse bits.
       } else if(val == 'N') {
              _WAIT_FOR_SPM();        
              sendChar(_GET_HIGH_FUSES());
       // Read extended fuse bits.
       } else if(val == 'Q') {
              _WAIT_FOR_SPM();        
              sendChar( _GET_EXTENDED_FUSES() );
       }
#endif
          
#ifndef REMOVE_AVRPROG_SUPPORT         
       // Enter and leave programming mode.
       else if((val == 'P')||(val == 'L')) {
            // Nothing special to do, just answer OK.
            sendChar('\r');
        // Exit bootloader.
       } else if(val == 'E') {
            _WAIT_FOR_SPM();        
            _ENABLE_RWW_SECTION();
            sendChar('\r');
            // Jump to Reset vector 0x0000 in Application Section.
            funcptr(); 
       // Get programmer type.  
       } else if (val == 'p') {
            // Answer 'SERIAL'.     
            sendChar('S');
       // Return supported device codes.
       }  else if(val=='t') {
            // Supports only this device, of course.
            sendChar(PARTCODE);
            // Send list terminator. 
            sendChar(0);
        // Set LED, clear LED and set device type.  
       }  else if((val == 'x')||(val == 'y')||(val == 'T')) {
            // Ignore the command and it's parameter.
            recChar();
            // Send OK back. 
            sendChar('\r');
       }     
#endif       
        // Return software identifier     
        else if (val == 'S') {
            // Return 'AVRBOOT'.
            sendChar('A');
            // Software identifier (aka programmer signature) is always 7 characters.
            sendChar('V');
            sendChar('R');
            sendChar('B');
            sendChar('O');
            sendChar('O');
            sendChar('T');
        // Return Software Version    
        } else if (val == 'V') {
            sendChar('1');
            sendChar('5');
        // Return Signature Byte    
        } else if (val == 's') {							
            sendChar(SIGNATURE_BYTE_3);
            sendChar(SIGNATURE_BYTE_2);
            sendChar(SIGNATURE_BYTE_1);
        // The last command to accept is ESC (synchronization). 
        } else if(val != 0x1b) {
             // If not ESC, then it is unrecognized...
            sendChar('?');
        }
      } // end: for(;;)
    } else {
        _WAIT_FOR_SPM();        
        _ENABLE_RWW_SECTION();      
      // Jump to Reset vector 0x0000 in Application Section  
      funcptr();        					
    }
} // end: main

#ifndef REMOVE_BLOCK_SUPPORT

UINT8 BlockLoad(UINT16 size, UINT8 mem, long *address) {

        UINT8 buffer[BLOCKSIZE];
	UINT16 data, tempaddress;
		
	// EEPROM memory type.
    if(mem == 'E') {
        /* Fill buffer first, as EEPROM is too slow to copy with UART speed */
        for(tempaddress=0;tempaddress<size;tempaddress++)
            buffer[tempaddress] = recChar();
        
        /* Then program the EEPROM */
        _WAIT_FOR_SPM();
    	for( tempaddress = 0; tempaddress < size; tempaddress++) {
    	    // Setup EEPROM address
	    EEARL = *address;
            EEARH = ((*address) >> 8);
            // Get byte.
            EEDR = buffer[tempaddress]; 
            // Write byte.
            EECR |= (1 << EEMWE); 
            EECR |= (1 << EEWE);
            // Wait for write operation to finish.
            while (EECR & (1 << EEWE)); 
                  (*address)++; // Select next EEPROM byte
        }

        return ('\r'); // Report programming OK
    // Flash memory type.    
    } else if(mem=='F') { 
        // NOTE: For flash programming, 'address' is given in words.
        // Convert address to bytes temporarily.
        (*address) <<= 1;
        // Store address in page. 
        tempaddress = (*address);  
	
        do {
            data = recChar();
            data |= (recChar() << 8);
            _FILL_TEMP_WORD(*address,data);
            // Select next word in memory.
            (*address)+= 2;
            // Reduce number of bytes to write by two. 
            size -= 2;
        } while(size); // Loop until all bytes written.
        _PAGE_WRITE(tempaddress);
        _WAIT_FOR_SPM();
        _ENABLE_RWW_SECTION();
        // Convert address back to Flash words again.
        (*address) >>= 1;
        // Report programming OK 
        return ('\r');
    // Invalid memory type?     
    } else {
        return ('?');
    }
} // UINT8 BlockLoad(....)


void BlockRead(UINT16 size, UINT8 mem, long *address)
{
    // EEPROM memory type.
    // Read EEPROM
    if (mem == 'E') {
        do {
            // Setup EEPROM address
            EEARL = *address; 
            EEARH = ((*address) >> 8);
            // Select next EEPROM byte
            (*address)++;
            // Read EEPROM 
            EECR |= (1 << EERE);
            // Transmit EEPROM dat ato PC
            sendChar(EEDR); 
            // Decrease number of bytes to read
            size--; 
        } while (size); // Repeat until all block has been read
     // Flash memory type.
    } else if(mem == 'F') {
         // Convert address to bytes temporarily.
        (*address) <<= 1;	
        do {
            sendChar( _LOAD_PROGRAM_MEMORY(*address));
            sendChar( _LOAD_PROGRAM_MEMORY((*address) + 1));
            // Select next word in memory.
            (*address) += 2;
            // Subtract two bytes from number of bytes to read 
            size -= 2;
        } while (size); // Repeat until all block has been read
        // Convert address back to Flash words again.
        (*address) >>= 1; 
    }
}
#endif
/* end of file */
