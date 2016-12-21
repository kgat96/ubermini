/*****************************************************************************
 *                                                                           *
 *        **********                                                         *
 *       ************                                                        *
 *      ***        ***                                                       *
 *      ***   +++   ***                                                      *
 *      ***   + +   ***                                                      *
 *      ***   +                            CHIPCON CC2400                    *
 *      ***   + +   ***             	CC2400 INCLUDE FILE 		         *
 *      ***   +++   ***                                                      *
 *      ***       ***                                                        *
 *       ***********                                                         *
 *        *********                                                          *
 *                                                                           *
 *****************************************************************************
 *       																     *
 * A number of helpful constants and macros are also included which can      *
 * be used to increase the legibility of your code.                          *
 *****************************************************************************
 * Device:      CC2400                                                       *
 * File:		CC2400.h					                                 *
 * Author:		MBR						                                     *
 * Target:		ATmega8						                                 *
 * Created:		2004-01-05					                                 *
 *				    			                                             *
 *****************************************************************************
 * Revision history:                                                         *
 *                                                                           *
 * $Log: cc2400.h,v $
 * Revision 1.3  2004/01/28 09:20:35  mbr
 * Removed function rfBeginReceiveInit()
 *
 * Revision 1.2  2004/01/20 15:01:18  mbr
 * Added - extern rxPacketComplete, rxinit
 *
 * Revision 1.1  2004/01/07 13:06:01  tos
 * Initial version in CVS.
 *														 *
 *																			 *
 *																			 *
 *                                                                           *
 ****************************************************************************/

#ifndef CC2400_H
#define CC2400_H                   // Only include this header file once

/***************************************************************************************************
 * 			CC2400 STROBE, CONTROL AND STATUS REGSITER                                             *
 ***************************************************************************************************
*/ 
#define	CC2400_MAIN	      0x00	//	Main control register
#define	CC2400_FSCTRL	  0x01	//	Frequency synthesiser main control and status
#define	CC2400_FSDIV	  0x02	//	Frequency synthesiser frequency division control
#define	CC2400_MDMCTRL	  0x03	//	Modem main control and status
#define	CC2400_AGCCTRL	  0x04	//	AGC main control and status
#define	CC2400_FREND	  0x05	//	Analog front-end control
#define	CC2400_RSSI	      0x06	//	RSSI information
#define	CC2400_FREQEST	  0x07	//	Received signal frequency offset estimation
#define	CC2400_IOCFG	  0x08	//	I/O configuration register
#define	CC2400_FSMTC	  0x0B	//	Finite state machine time constants
#define	CC2400_MANAND	  0x0D	//	Manual signal AND-override register
#define	CC2400_FSMSTATE   0x0E	//	Finite state machine information and breakpoint
#define	CC2400_ADCTST	  0x0F	//	ADC test register
#define	CC2400_RXBPFTST   0x10	//	Receiver bandpass filters test register
#define	CC2400_PAMTST	  0x11	//	PA and transmit mixers test register
#define	CC2400_LMTST	  0x12	//	LNA and receive mixers test register
#define	CC2400_MANOR	  0x13	//	Manual signal OR-override register
#define	CC2400_MDMTST0	  0x14	//	Modem test register 0
#define	CC2400_MDMTST1	  0x15	//	Modem test register 1
#define	CC2400_DACTST	  0x16	//	DAC test register
#define	CC2400_AGCTST0	  0x17	//	AGC test register: various control and status
#define	CC2400_AGCTST1	  0x18	//	AGC test register: AGC timeout
#define	CC2400_AGCTST2	  0x19	//	AGC test register: AGC various parameters
#define	CC2400_FSTST0	  0x1A	//	Test register: VCO array results and override
#define	CC2400_FSTST1	  0x1B	//	Test register: VC DAC manual control VCO current constant
#define	CC2400_FSTST2	  0x1C	//	Test register:VCO current result and override
#define	CC2400_FSTST3	  0x1D	//	Test register: Charge pump current etc
#define	CC2400_MANFIDL	  0x1E	//	Manufacturer ID, lower 16 bit
#define	CC2400_MANFIDH	  0x1F	//	Manufacturer ID, upper 16 bit
#define	CC2400_GRMDM	  0x20	//	Generic radio modem control
#define	CC2400_GRDEC	  0x21	//	Generic radio decimation control and status
#define	CC2400_PKTSTATUS  0x22	//	Packet mode status
#define	CC2400_INT        0x23	//	Interrupt register
#define	CC2400_SYNCL	  0x2C	//	Synchronisation word, lower 16 bit
#define	CC2400_SYNCH	  0x2D	//	Synchronisation word, upper 16 bit
#define	CC2400_SXOSCON	  0x60	//	Command strobe register: Turn on XOSC
#define	CC2400_SFSON	  0x61	//	Command strobe register: Start and calibrate FS and 
#define	CC2400_SRX	      0x62	//	Command strobe register: Start RX
#define	CC2400_STX	      0x63	//	Command strobe register: Start TX (turn on PA)
#define	CC2400_SRFOFF	  0x64	//	Command strobe register: Turn off RX/TX and FS
#define	CC2400_SXOSCOFF   0x65	//	Command strobe register: Turn off XOSC
#define	CC2400_FIFOREG	  0x70	//	Write and read data to and from the 32 byte FIFO

/* Bit definitions 
   The Register Bit names are represented by their bit number (0-15)
*/

// The status byte of CC2400
#define	XOSC16M_STABLE       6 // Indicates whether the 16MHz oscillator is running ('1') or not
#define	CS_ABOVE_THRESHOLD_N 5 // Carrier sense flag.                               
#define	SYNC_RECEIVED	     4 // Indicates whether a sync word has been received or not
#define	STATUS_CRC_OK        3 // Indicates whether the next two bytes in the FIFO will 
                               // make the CRC calculation successful or not                                                             
#define	FS_LOCK              2 // Indicates whether the freq. synth. is in lock('1') or not
#define	FH_EVENT	         0 // Last event signalled on the FH pin
 
// Main control register
#define	RESET	          15
#define	FS_FORCE_EN	       9
#define	RXN_TX	           8
#define	XOSC32K_BYPASS	   3
#define	XOSC32K_EN	       2
#define	XOSC16M_BYPASS	   1
#define	XOSC16M_FORCE_EN   0

//	FSCTRL (0x01) - Frequency Synthesiser Control and Status										
#define		LOCK_THRESHOLD	4	//	Number of consecutive reference clock periods with successful sync windows required to indicate lock						
#define		CAL_DONE	    3	//	Calibration has been performed since the last time the FS was turned on.						
#define		CAL_RUNNING	    2	//	Calibration status, '1' when calibration in progress.						
#define		LOCK_LENGTH	    1	//	LOCK_WINDOW pulse width:0: 2 CLK_PRE periods1: 4 CLK_PRE periods						
#define		PLL_LOCK_STATUS	0	//	'1' when PLL is in lock, otherwise '0'.						
											
//	FSDIV (0x02) - Frequency Synthesiser Frequency Division Control										
#define		FREQ_R	        10	//	Read only. Directly gives the right frequency when reading.						
#define		FREQ  	         0	//	Frequency control word used in Bluetooth mode and in GR mode with manual frequency-hopping enabled.  						
											
//	MDMCTRL (0x03) - Modem Control and Status										
#define		MOD_OFFSET	    7	//	Modulator centre frequency in 15.625 kHz steps relative to 1 MHz. Two's complement signed value. I.e MOD_OFFSET=0x3F è centre frequency=1.48 MHz; MOD_OFFSET=0x40 è centre frequency=0.50 MHz.						
#define		MOD_DEV	        0	//	Modulator frequency deviation in 3.9062 kHz steps (0-500 kHz). Unsigned value. Reset value gives a deviation of 160 kHz.						
											
//	AGCCTRL (0x04) - AGC Control and Status										
#define		VGA_GAIN 	    8	//	When written, VGA manual gain override value; when read, the currently used VGA gain setting.
#define		AGC_LOCKED	    3	//	AGC lock status						
#define		AGC_LOCK	    2	//	Lock gain after maximum # of attempts.						
#define		AGC_SYNC_LOCK	1	//	Lock gain after sync word received and maximum # of attempts. (As configured in AGCTST0.AGC_ATTEMPTS. Attempts may be 0)
#define		VGA_GAIN_OE	    0	//	Use the VGA_GAIN value during RX instead of the AGC value.						
											
//	FREND (0x05) - Frontend Control Register										
#define		PA_DIFF	        3	//	Indicates whether PA output is differential (1) or singel-ended (0).						
#define		PA_LEVEL	    0	//	Output PA level. (~0 dBm)						
											
//	RSSI (0x06) - RSSI Status and Control Register										
#define		RSSI_VAL	    8	//	(Filtered) RSSI estimate on a logarithmic scale. Unit is 1 dB, offset is TBD [depends on the absolute gain of the RX chain and should be measured].						
#define		RSSI_CS_THRES	2	//	Carrier sense signal threshold value. Unit is 4 dB, offset is TBD [depends on the absolute gain of the RX chain and should be measured].  The CS_ABOVE_THRESHOLD_N signal goes low when the received signal is above this value. The CS_ABOVE_THRESHOLD_N signal is available on the GIO1 pin or in the status word returned on SPI transactions.The reset value corresponds to appr. -60 dBm.						
#define		RSSI_FILT	    0	//	RSSI filter length:0: 0 symbols (no filtering)1: 1 symbol2: 4 symbols3: 8 symbols						
											
//	FREQEST (0x07) - Received frequency offset estimation										
#define		RX_FREQ_OFFSET	8	//	Estimate of the received signals centre frequency in steps of TBD [function of decimation rate] kHz in comparison to the ideal 1 MHZ centre frequency. Two's complement signed value.						
											
//	IOCFG (0x08) - I/O configuration register										
#define		GIO6_CFG	    9	//	How to use the GIO6/LPO_CLK pin. See Table 10 for options. The reset value outputs the signal CRC_OK on pin GIO6.						
#define		GIO1_CFG	    3	//	How to use the GIO1/BLOCK pin. See Table 10 for options. The reset value outputs the signal LOCK_N on pin GIO1.						
#define		HSSD_SRC	    0	//	The HSSD module is used as follows:0: Off.1: Output AGC status (gain setting / peak detector status / accumulator value)2: Output ADC I and Q values.3: Output I/Q after digital downmix and channel filtering.4: Output RX signal magnitude / frequency unfiltered. (from demodulator.)5: Output RX signal magnitude / frequency filtered. (from demodulator.)6: Output RSSI / RX frequency offset estimation7: Input DAC values.The HSSD module requires that the FS is up and running as it uses CLK_PRE (~150 MHZ) to produce its ~37.5 MHz data clock and serialize its output words. Also, in order for HSSD to function properly PIN_MODE must be set for HSSD (and thus be in generic radio mode.)						
											
//	FSMTC (0x0B) - Finite state machine time constants										
#define		TC_RXON2AGCEN	13	//	The time in 5 us steps from RX is turned on until the AGC is enabled. This time constant must be large enough to allow the RX chain to settle so that the AGC algorithm starts working on a proper signal. The default value corresponds to 15 us.						
#define		TC_PAON2SWITCH	10	//	The time in us from the PA is turned on until the TX/RX switch allows the TX signal to pass.						
#define		TC_PAON2TX	     6	//	The time in us from the PA is turned on until the first TX bit is sent to the modulator.						
#define		TC_TXEND2SWITCH	 3	//	The time in us from the last bit is sent to the modulator until the RX/TX switch breaks the TX output.						
#define		TC_TXEND2PAOFF	 0	//	The time in us from the last bit is sent to the modulator until the PA is turned off.						
											
//	MANAND (0x0D) - Manual signal AND override register 										
#define		VGA_RESET_N	    15	//	The VGA_RESET_N signal is used to reset the peak detectors in the VGA in the RX chain.						
#define		VCO_LOCK_STATUS 14	//	The LOCK_STATUS signal is the top-level signal that indicates whether VCO lock is achieved or not.						
#define		BALUN_CTRL	    13	//	The BALUN_CTRL signal controls whether the PA should receive its required external biasing (1) or not (0) by controlling the RX/TX output switch.						
#define		RXTX	        12	//	RXTX signal: controls whether the LO buffers (0) or PA buffers (1) should be used.						
#define		PRE_PD	        11	//	Powerdown of prescaler.						
#define		PA_N_PD	        10	//	Powerdown of PA (negative path).						
#define		PA_P_PD	        9	//	Powerdown of PA (positive path). When PA_N_PD=1 and PA_P_PD=1 the upconversion mixers are in powerdown.						
#define		DAC_LPF_PD	    8	//	Powerdown of TX DACs.						
#define		BIAS_PD	        7	//	Powerdown control of global bias generator + XOSC clock buffer.						
#define		XOSC16M_PD	    6	//	Powerdown control of 16 MHz XOSC core.						
#define		CHP_PD	        5	//	Powerdown control of charge pump.						
#define		FS_PD	        4	//	Powerdown control of VCO, I/Q generator, LO buffers.						
#define		ADC_PD	        3	//	Powerdown control of the ADCs.						
#define		VGA_PD	        2	//	Powerdown control of the VGA.						
#define		RXBPF_PD	    1	//	Powerdown control of complex bandpass receive filter.						
#define		LNAMIX_PD	    0	//	Powerdown control of LNA, downconversion mixers and frontend bias.						
											
//	FSMSTATE (0x0E) - Finite state machine information and breakpoint										
#define		FSM_STATE_BKPT	8	//	FSM breakpoint state. State=0 means that breakpoints are disabled.						
#define		FSM_CUR_STATE	0	//	Gives the current state of the Control sequencer finite state machine.						
											
//	ADCTST (0x0F) - ADC Test Register										
#define		ADC_I	        8	//	Read the current ADC I-branch value.						
#define		ADC_Q	        0	//	Read the current ADC Q-branch value.						
											
//	RXBPFTST (0x10) - Receiver Bandpass Filters Test Register										
#define		RXBPF_CAP_OE	14	//	RX bandpass filter capacitance calibration override enable.						
#define		RXBPF_CAP_O	    7	//	RX bandpass filter capacitance calibration override value.						
#define		RXBPF_CAP_RES	0	//	RX bandpass filter capacitance calibration result.0 Minimum capasitance in the feedback.1: Second smallest capasitance setting.…127: Maximum capasitance in the feedback.						
											
//	PAMTST (0x11) - PA and Transmit Mixers Test Register										
#define		VC_IN_TEST_EN	12	//	When ATESTMOD_MODE=7 this controls whether the ATEST1 in is used to output the VC node voltage (0) or to control the VC node voltage (1).						
#define		ATESTMOD_PD	    11	//	Powerdown of analog test module.						
#define		ATESTMOD_MODE	8	//	When ATESTMOD_PD=0, the function of the analog test module is as follows:0: Outputs "I" (ATEST2) and "Q" (ATEST1) from RxMIX.1: Inputs "I" (ATEST2) and "Q" (ATEST1) to BPF.2: Outputs "I" (ATEST2) and "Q" (ATEST1) from VGA.3: Inputs "I" (ATEST2) and "Q" (ATEST1) to ADC.4: Outputs "I" (ATEST2) and "Q" (ATEST1) from LPF.5: Inputs "I" (ATEST2) and "Q" (ATEST1) to TxMIX.6: Outputs "P" (ATEST2) and "N" (ATEST1) from Prescaler.7: Connects TX IF to RX IF and simultaneously the ATEST1 pin to the internal VC node (see VC_IN_TEST_EN.						
#define		TXMIX_CAP_ARRAY	5	//	Selects varactor array settings in the transmit mixers.						
#define		TXMIX_CURRENT	3	//	Transmit mixers current:0: 1.72 mA1: 1.88 mA2: 2.05 mA3 2.21 mA						
#define		PA_CURRENT	    0	//	Current programming of the PA0: -3 current adjustment1: -2 current adjustment2: -1 current adjustment3: Nominal setting4: +1 current adjustment5: +2 current adjustment6: +3 current adjustment7: +4 current adjustment						
											
//	LMTST (0x12) - LNA and receive mixers test register										
#define		RXMIX_HGM	    13	//	Receiver mixers high gain mode enable.						
#define		RXMIX_TAIL	    11	//	Control of the receiver mixers output current.0: TBD1: TBD2: TBD3:TBD						
#define		RXMIX_VCM	    9	//	Controls VCM level in the mixer feedback loop0: 8 µA mixer current1: 12 µA mixer current (Nominal)2: 16 µA mixer current3: 20 µA mixer current						
#define		RXMIX_CURRENT	7	//	Controls current in the mixer0: 360 µA mixer current (x2)1: 720 µA mixer current (x2) (Nominal)2: 900 µA mixer current (x2) 3: 1260 µA mixer current (x2)						
#define		LNA_CAP_ARRAY	5	//	Selects varactor array setting in the LNA0: OFF1: 0.1pF (x2) (Nominal)2: 0.2pF (x2)3: 0.3pF (x2)						
#define		LNA_LOWGAIN	    4	//	Selects low gain mode of the LNA0: 19 dB (Nominal)1:   7 dB						
#define		LNA_GAIN	    2	//	Controls current in the LNA gain compensation branch0: OFF1: 100 µA LNA current2: 300 µA LNA current (Nominal)3: 1000 µA LNA current						
#define		LNA_CURRENT	    0	//	Controls main current in the LNA0: 240 µA LNA current (x2)1: 480 µA LNA current (x2) (Nominal)2: 640 µA LNA current (x2)3: 1280 µA LNA current (x2)						
				
//	MDMTST0 (0x14) - Modem Test Register 0										
#define		TX_PRNG	            13	//	When set, the transmitted data is taken from a 10-bit PRNG instead of from the BDATA1 pin in BlueRF mode, DIO pin in unbuffered GR mode or from the FIFO in buffered GR mode.						
#define		TX_1MHZ_OFFSET_N    12	//	Determines TX IF frequency:0: 1 MHz1: 0 MHz						
#define		INVERT_DATA	        11	//	When this bit is set the data sent is inverted before transmission in TX, and inverted after reception in RX.						
#define		AFC_ADJUST_ON_PACKET 10	//	When this bit is set to '1', modem parameters are adjusted for slow tracking of the received signal as opposed to quick acquisition, when a packet is received in RX.						
#define		AFC_SETTLING	    8	//	Controls how many max-min pairs that are used to compute the output.00: 1 pair01: 2 pairs10: 4 pairs11: 8 pairs						
#define		AFC_DELTA	        0	//	Programmable level used in AFC-algorithm which indicates the expected frequency deviation of the received signal.						
											
//	MDMTST1 (0x15) - Modem Test Register 1										
#define		BSYNC_THRESHOLD	0	//	Threshold value used in clock recovery algorithm.						
											
//	DACTST (0x16) - DAC Test Register										
#define		DAC_SRC	        12	//	The TX DACs data source is selected by DAC_SRC according to:0: Normal operation (from modulator).1: The DAC_I_O and DAC_Q_O override values below.2: From ADC3: I/Q after digital downmix and channel filtering.4: Full-spectrum White Noise (from PRNG.)5: RX signal magnitude / frequency filtered. (from demodulator.)6: RSSI / RX frequency offset estimation7: HSSD module.This feature will often require the DACs to be manually turned on in MANOVR and PAMTST.ATESTMOD_MODE=4.						
#define		DAC_I_O	        6	//	I-branch DAC override value.						
#define		DAC_Q_O	        0	//	Q-branch DAC override value.						
											
//	AGCTST0 (0x17) - AGC Test Register 0										
#define		AGC_SETTLE_BLANK_DN	13	//	AGC blanking enable/limit for negative gain changes.0: Disabled1-7: Duration of blanking signal in 8 MHz clock cycles.						
#define		AGC_WIN_SIZE	    11	//	AGC window size.						
#define		AGC_SETTLE_PEAK	    7	//	AGC settling period peak detectors.						
#define		AGC_SETTLE_ADC	    3	//	AGC settling period ADC.						
#define		AGC_ATTEMPTS	    0	//	The maximum number of attempts to set gain.						
											
//	AGCTST1 (0x18) - AGC Test Register 1										
#define		AGC_VAR_GAIN_SAT	14	//	Chooses the gain reduction upon saturation of the variable gain stage:0: -1/-3 gain steps1: -3/-5 gain steps						
#define		AGC_SETTLE_BLANK_UP	11	//	AGC blanking enable/limit for positive gain changes.0: Disabled1-7: Duration of blanking signal in 8 MHz clock cycles.						
#define		PEAKDET_CUR_BOOST	10	//	Doubles the bias current in the peak-detectors  in between the VGA stages when set.						
#define		AGC_MULT_SLOW	    6	//	AGC timing multiplier, slow mode.						
#define		AGC_SETTLE_FIXED	2	//	AGC settling period, fixed gain step.						
#define		AGC_SETTLE_VAR	    0	//	AGC settling period, variable gain step.						
											
//	AGCTST2 (0x19) - AGC Test Register 1										
#define		AGC_BACKEND_BLANKING  12	//	AGC blanking makes sure that the modem locks its bit synchronization and centre frequency estimator when the AGC changes gain.0: Disabled1-3: Fixed/variable enable						
#define		AGC_ADJUST_M3DB	      9	//	AGC parameter -3 dB.						
#define		AGC_ADJUST_M1DB	      6	//	AGC parameter -1 dB.						
#define		AGC_ADJUST_P3DB	      3	//	AGC parameter +3 dB.						
#define		AGC_ADJUST_P1DB	      0	//	AGC parameter +1 dB.						
											
//	FSTST0 (0x1A) - Frequency Synthesiser Test Register 0										
#define		RXMIXBUF_CUR	      14	//	RX mixer buffer bias current.0: 690uA 1: 980uA (nominal)2: 1.16mA 3: 1.44mA						
#define		TXMIXBUF_CUR	      12	//	TX mixer buffer bias current.0: 690uA 1: 980uA (nominal)2: 1.16mA 3: 1.44mA						
#define		VCO_ARRAY_SETTLE_LONG 11	//	When '1' this control bit doubles the time allowed for VCO settling during FS calibration.						
#define		VCO_ARRAY_OE	      10	//	VCO array manual override enable.						
#define		VCO_ARRAY_O	          5	    //	VCO array override value.						
#define		VCO_ARRAY_RES	      0	    //	The resulting VCO array setting from the last calibration.						
											
//	FSTST1 (0x1B) - Frequency Synthesiser Test Register 1										
#define		RXBPF_LOCUR	    15	//	Controls reference bias current to RX bandpass filters:0: 4 uA (Default)1: 3 uA						
#define		RXBPF_MIDCUR	14	//	Controls reference bias current to RX bandpass filters:0: 4 uA (Default)1: 3.5 uA						
#define		VCO_CURRENT_REF	10	//	The value of the reference current calibrated against during VCO calibration.						
#define		VCO_CURRENT_K	4	//	VCO current calibration constant. (current B override value when FSTST2.VCO_CURRENT_OE=1.)						
#define		VC_DAC_EN	    3	//	Controls the source of the VCO VC node in normal operation (PAMTST.VC_IN_TEST_EN=0):0: Loop filter (closed loop PLL)1: VC DAC (open loop PLL)						
#define		VC_DAC_VAL	    0	//	VC DAC output value						
											
//	FSTST2 (0x1C) - Frequency Synthesiser Test Register 2										
#define		VCO_CURCAL_SPEED    13	//	VCO current calibration speed:0: Normal1: Double speed2: Half speed3: Undefined.						
#define		VCO_CURRENT_OE	    12	//	VCO current manual override enable.						
#define		VCO_CURRENT_O	    6	//	VCO current override value (current A).						
#define		VCO_CURRENT_RES	    0	//	The resulting VCO current setting from last calibration.						
											
//	FSTST3 (0x1D) - Frequency Synthesiser Test Register 3										
#define		CHP_TEST_UP	    13	//	When CHP_DISABLE=1 forces the CHP to output "up" current.						
#define		CHP_TEST_DN	    12	//	When CHP_DISABLE=1 forces the CHP to output "down" current.						
#define		CHP_DISABLE	    11	//	Set to disable charge pump during VCO calibration.						
#define		PD_DELAY	    10	//	Selects short or long reset delay in phase detector:0: Short reset delay1: Long reset delay						
#define		CHP_STEP_PERIOD	8	//	The charge pump current value step period:0: 0.25 us1: 0.5 us2: 1 us3: 4 us						
#define		STOP_CHP_CURRENT  4	//	The charge pump current to stop at after the current is stepped down from START_CHP_CURRENT after VCO calibration is complete. The current is stepped down periodically with intervals as defined  in CHP_STEP_PERIOD.						
#define		START_CHP_CURRENT 0	//	The charge pump current to start with after VCO calibration is complete. The current is then stepped down periodically to the value STOP_CHP_CURRENT with intervals as defined  in CHP_STEP_PERIOD.						
											
//	MANFIDL (0x1E) - Manufacturer ID, Lower 16 Bit										
#define		PARTNUML      12	//	The device part number. CC2400 has part number 0x001.						
#define		MANFID	      0	//	Gives the JEDEC manufacturer ID. The actual manufacturer ID can be found in MANIFID[7:1], the number of continuation bytes in MANFID[11:8] and MANFID[0]=1. Chipcon's JEDEC manufacturer ID is 0x7F 0x7F 0x7F 0x9E (0x1E preceeded by three continuation bytes.)						
											
//	MANFIDH (0x1F) - Manufacturer ID, Upper 16 Bit										
#define		VERSION	      12	//	Version number. Current number is 0.						
#define		PARTNUMH      0	//	The device part number. CC2400 has part number 0x001.						
											
//	GRMDM (0x20) - Generic Radio Modem Control and Status										
#define		SYNC_ERRBITS_ALLOWED  13  //	Sync detection occurs when the number of bits in the sync word correlator different from that specified by the SYNC registers is equal to or lower than SYNC_ERRBITS_ALLOWED.						
#define		PIN_MODE	          11  //	Chooses the pin configuration in Generic Radio Interface mode (see Table 8 for details):0: Generic Radio unbuffered mode 1: Generic Radio buffered mode 2: HSSD test mode3: Unused						
#define		PACKET_MODE	          10  //	When this bit is set the packet mode is enabled in GR.In TX, this enables preamble generation in the buffered modes, sync word insertion in buffered modes, and CRC appending (if enabled by CRC_ON) in buffered modes.In RX, this enables sync word detection in buffered and unbuffered modes, and CRC verification (if enabled by CRC_ON) in buffered mode.						
#define		PRE_BYTES	          7	  //	The number of preamble bytes ("01010101") to be sent in packet mode:000: 0	001: 1	010: 2	011: 4100: 8	101:16:00	110:32:00	111: Infinitely on
#define		SYNC_WORD_SIZE	      5	  //	The size of the packet mode sync word sent in TX and correlated against in RX:00: The 8 MSB bits of  SYNC_WORD.01: The 16 MSB bits of  SYNC_WORD.10: The 24 MSB bits of  SYNC_WORD.11: The 32 MSB bits of  SYNC_WORD.						
#define		CRC_ON	              4   //	In packet mode a CRC-16 (CCITT) is calculated and is transmitted after the data in TX, and a CRC-16 is calculated during reception in RX.						
#define		DATA_FORMAT	          2	  //	Select line coding format used during RX and TX operations. 00: NRZ01: Manchester10:	8/10 Line Coding (Is not applied to preambles or sync words)11: Reserved					
#define		MODULATION_FORMAT	  1	  //	Modulation format of modem:0: (G)FSK1: OOK						
#define		TX_GAUSSIAN_FILTER	  0   //	When this bit is set the data sent in TX is Gaussian filtered before transmission.						
											
//	GRDEC (0x21) - Generic Radio Decimation Control and Status										
#define		IND_SATURATION	12	//	Signal indicates whether the accumulate and dump decimation filters have saturated at some point since the last read. Status flag is cleared by reading the GRDEC register.						
#define		DEC_SHIFT	    10	//	Controls extra shifts in decimation, for possible extra precision. Decimation value shift value:2: -23: -10: 01: 1						
#define		CHANNEL_DEC	    8	//	Control signal for channel filter bandwidth control00:  1MHz 01:  500kHz01:  250kHz11:  125kHz						
#define		DEC_VAL	        0	//	Controls decimation between demodulation and data-filter. 						
											
//	PKTSTATUS (0x22) - Packet Mode Status										
#define		SYNC_WORD_RECEIVED	10	//	The currently configured sync word has been received since RX was turned on.						
#define		CRC_OK	            9	//	Indicates that the two next bytes available to be read from the FIFO equal the CRC16 calculated over the bytes already read from the FIFO.						
#define		ERROR_8_10	        8	//	Indicates that 8/10 coding errors were detected in the last received packet.						
#define		ERRCNT_8_10	        0	//	Indicates the number of 8/10 coding errors that were detected in the last received packet.						
											
//	INT (0x23) - Interrupt Register										
#define		FH_POLARITY	    7	//	Polarity of the output signal FH. See XXX [Waveforms must be made] for waveforms.						
#define		PKT_POLARITY	6	//	Polarity of the output signal PKT. See XXX [Waveforms must be made] for waveforms.						
#define		FIFO_POLARITY	5	//	Polarity of the FIFO signal. See XXX [Waveforms must be made] for waveforms.						
#define		FIFO_THRESHOLD	0	//	The FIFO pin signals that the 32 byte data FIFO is near empty in TX or near full in RX. The threshold upon which to signal with the FIFO pin is:# bytes in FIFO >= FIFO_THRESHOLD in RX# bytes in FIFO <= 32 - FIFO_THRESHOLD in TX.						
											
//	SYNCL (0x2C) - Sync Word, Lower 16 Bit										
#define		SYNCWORDL	    0	//	Synchronisation word, lower 16 bit.  The default synchronization word of 0xD391DA26 has very good DC, autocorrelation, and bit-run properties for all synchronization word lengths.						
											
//	SYNCH (0x2D) - Sync Word, Upper 16 Bit										
#define		SYNCWORDH	    0	//	Synchronisation word, upper 16 bit.						

//----------------------------------------------------------------------------
// RF_SETTINGS is a data structure which contains all the settings
// required for a given pair of RX/TX channels.
// These values should be passed to the configuration function rfInit(...)
// before the RX/TX channel pair can be used. The configuration function writes
// the setup to the register in CC2400.
// The RX/TX channel pair is later used by passing the correpsonding
// RF_SETTINGS data structures to the function
// rfInit(..). 					
//------------------------------------------------------------------------------
typedef struct {
	WORD FSCTRL;	//	Frequency synthesiser main control and status  
	WORD FSDIV;		//	Frequency synthesiser frequency division control
	WORD MDMCTRL;  	//	Modem main control and status
	WORD FREND;		//	Analog Front-End Control
	WORD RSSI;		//	RSSI information
    WORD IOCFG;		//	I/O configuration register
    WORD FSMTC;		//  Finite State Machine time constants
	WORD MANAND;	//	Manual signal AND-override register 
	WORD PAMTST;	//	PA and transmit mixers test register
	WORD LMTST;		//	LNA and receive mixers test register
	WORD MDMTST0;  	//	Modem test register 0
	WORD MDMTST1;   //	Modem test register 1
	WORD DACTST;	//	DAC Test register
	WORD FSTST0;	//	Test register: VCO array results and override
	WORD FSTST1;	//	Test register: VC DAC manual control VCO current constant
    WORD GRMDM;		//	Generic radio modem control
    WORD GRDEC;		//	Generic radio decimation control and status
   
} RF_SETTINGS;

/***************************************************************************************************
 * 			LOCAL FUNCTION PROTOYPES
 ***************************************************************************************************
*/ 

#define PKT_ACK_REQ	0x01
#define PKT_ACK		0x02


//Function to configure and setup CC2400 
void rfInit(RF_SETTINGS *pRFS, BOOL reset);

// Function to turn on oscillator and PLL
void rfOn(void);

// Function to turn of PLL, go to Idle mode 
void rfOff(void);

// Transmit function to send a packet with return of length
BOOL rfSendPacketLength(BYTE *pData, uint8 length, BYTE *pFlags);

// Transmit function to send packets without return of length
void rfSendPacket(BYTE *pData, uint8 length, BYTE *pFlags);

// Transmit function with acknowledge check and CC2400 state machine check
BOOL rfSendPacketStateAck(BYTE *pData, uint8 length, BYTE *pFlags);

// Receive function to turn on RX only
void rfBeginReceive(void);

// Receive function to receive a packet with CRC check and timeout for packet
BOOL rfEndReceiveBuf(BYTE *pData, uint8 *pLength, BYTE *pFlags);

// Receive function to receive a packet with CRC check and length check
BOOL rfEndReceive(BYTE *pData, uint8 *length, BYTE *pFlags);

// Receive with acknowledge check
BOOL rfReceivePacket(BYTE *pData, uint8 *pLength, BYTE *pFlags);

// Global CC2400 SPI status byte 
extern volatile uint8 CC2400_Status;

// Global variable for rfChatModem example
extern BOOL rxPacketComplete, rxinit;

#endif // CC2400_H
