;--------------------------------------------------------------------------------------------------
; Project:  Multi-IO Board A Master PIC
; Date:     3/30/15
; Revision: See Revision History notes below.
;
; Overview:
;
; This program runs on the Master PIC on a Multi-IO Board Configuration A Board. It is designed to
; operate for a Longitudinal 4 Channel (two shoes combined), Longitudinal Shoe 1, Longitudinal
; Shoe 2, Transverse Ring 1, Transverse Ring 2, or Wall system.
;
; Longitudinal 4 Channel System
;
; This board handles four channels of longitudinal information. The 16 channels from the two
; shoes are combined into four channels before reaching the Muli-IO board. This board monitors the
; rotating head TDC sensor to create a clock position signal for the Slave PICs so that they can
; tag each peak value with the circumferential location at which it was recorded.
;
; Longitudinal Shoe 1 / Longitudinal Shoe 2 System
;
; As an alternative to the Longitudinal 4 Channel System, some systems use a separate Multi-IO
; board for each shoe of 8 channels each. The Shoe 1 board handles the TDC sensor as described
; above for the Longitudinal 4 Channel System.
;
; Transverse Ring 1 / Transverse Ring 2 System
;
; The transverse head is divided into two rings of shoes. One board handles Ring 1 and other
; handles Ring 2. The Rabbit software is slightl  different for each ring, but all the Master &
; Slave PIC code is identical for either.
;
; Wall System
;
; A Multi-IO board is used to collect minimum (lowest voltage) peak data on input signals created
; by a wall measurement system.
;
; Communications
;
; The Master PIC communicates with a Rabbit RCM4200 via serial port and with 8 Slave PICs via the
; I2C bus. It also communicates with digital gain and offset potentiometers via that same I2C bus.
;
; Digital Pots on I2C Bus
;
; Each Multi-IO board has four digital pot chips, each with four pots. Each of the eight channels
; uses two of the pots from a chip, so adjacent channels share a chip. Due to an early design
; miscalculation regarding the amount of I2C addressing space available, the circuit is designed so
; that each pot chip can be selected or deselected by an output pin of a Slave PIC. This is done
; by having A0 and A1 of each pot tied permanently to 0 while a PIC chip output controls A2, which
; is normally held at 0 as well. These three address lines comprise the low three bits of the
; chip's seven bit slave address. The upper four bits for this particular chip model is always
; 1010b. Thus, in the normal state with the PIC select line set to 0, all chips will have address
; 1010000b. When a chip is selected by taking the select line to 1, that chip will have an address
; of 1010100b. The Master can then address the selected chip at that address and it will be the
; only chip to respond. Of course, care must be taken not to select more than one chip at a time.
;
; deselected digital pot chip address: 1010000b
; selected digital pot chip address:   1010100b
;
; Slave PIC on I2C Bus
;
; Unlike the digital pot chips, the Slave PICs have programmable upper address nibbles. On the
; Multi-IO board, this is set to 1111 to differentiate it from the digital pot chips. The lower
; three bits are set to match the values on three of the Slave PIC's input pins. For each slave,
; these three bits are tied differently to give values of 0-7. The PIC with address input value of
; 0 handles analog input channel 1, while the slave with address 7 handles channel 8.
;
; Slave PIC addressing: 1111000b, 1111001b,...,1111111b
;
; System Clock
;
; The system clock is configured to run at 16 MHz. The CLKOUT pin outputs a clock at Fosc/4 which
; drives the input clock of one of the Slave PICs. The Slave PIC uses the PLL to generate a 16 MHz
; from the 4 Mhz input. The CLKOUT pin of that Slave PIC drives the next Slave PIC and so forth.
; Each PIC's CLKOUT output is FOSC/4 into the next PIC, so each must use its PLL to run its system
; clock at 16 MHz.
;
; The 16 MHz is used since the Slave PICs run their A/D convertors at Fosc/16 to achieve a sample
; period of 1 us. This is the fastest specified A/D conversion rate.
;
; The Ring 1 board also handles the encoder inputs and the tube detection photo eye. The code in
; the Ring 2 board, being identical, performs the same processing but the results are ignored and
; never read by the Rabbit on the Ring 2 board as the encoder/eye data is useless as there are no
; such devices connected to the Ring 2 board.
;
; Options Configurable at Runtime
;
; The Master PIC code is identical in the Longitudinal, Transverse, and Wall boards. Each of those
; boards do require unique processing. The Rabbit is the only device on the board which is
; programmed to know what type of board it inhabits and the host can thus determine the type as
; well by communication with the Rabbit. The Rabbit or the host can then set flags in the Master
; PIC and Slave PICs to activate different types of processing.
;
; Encoder Tracking on Transverse Ring 1 Board
;
; The Transverse Ring 1 board Master PIC monitors two quadrature encoder inputs via the RC3/RC4 and
; RC6/RC7 input ports. It also monitors a photo-eye to detect entry of the tube into the system via
; RA1 or RC5 input ports. The Master PIC sends an interrupt to the Rabbit which in turn signals the
; host when the encoders have moved a preselected distance and when the photo-eye has become
; blocked or unblocked.
;
; Photo Eye Tracking on Longitudinal Board(s)
;
; Some systems have two Longitudinal boards, one for each shoe. For systems with 4 channel muxes,
; only one board is required. Both boards monitor the TDC rotating head photo-eye so that they
; can calculate clock position information and transmit it to the Slave PICs so they can in turn
; tag each peak with its associated circumferential location.
;
; On the Longitudinal boards, Opto1 and R156 are not installed so they do not drive the Sync line.
; This allows the Master PIC to drive the Sync line to send clock signals to the Slave PICs. The
; TDC input on Sync Reset occurs once per head revolution; the Master PIC measures this period and
; divides the the count down into clock positions (12, 24, etc. depending on the desired
; resolution). The Master PIC then uses this divided period to generate a clock position signal
; using an internal timer to signal the Slave PICs via the Sync line. Each time the Sync line
; pulses it indicates a new clock position.
;
; The Slave PICs track the incoming Sync pulse with a counter connected to the input which they
; reset each time they receive a pulse from the Sync Reset line. Thus, the value in the counter is
; always equal to the clock position. This allows the Slaves to quickly tag the peaks with the value
; in the counter at the time the peak is detected without requiring any extra monitoring of the Sync
; line or incrementing of the counter...it's all handled automatically.
;
; Sync & Sync Reset I/O Configuration
;
; NOTE: The sync input is defined as SYNCT in this program as SYNC is already a pre-defined term.
;
;   Master PIC
;
;    Sync Reset connected to RA0 and RA5, either pin can be used to monitor the signal.
;    RA0 input allows Interrupt on Change (IOC) of the signal.
;    RA5 input allows both IOC and tracking by counter Timer1.
;
;    As RA5 also provides the IOC option, the connection to RA0 is only to maintain design
;    consistency with the Slave PICs which can only use RA0 as they must use RA5 as the system
;    clock input.
;
;    Normally, the Master PIC monitors the Sync Reset RA0 using the IOC option. The Sync line is
;    driven as an output on RC5 to the Slave PICs while RA1 is set to an input to avoid conflict.
;
;   Slave PICs
;
;    Sync Reset is monitored via RA0 using the IOC option. The Sync line is monitored via RC5
;    configured to count pulses with Timer 0. The Sync Reset pulse triggers a reset of the counter.
;
;--------------------------------------------------------------------------------------------------
; Notes on PCLATH
;
; The program counter (PC) is 13 bits. The lower 8 bits can be read and written as register PCL.
; The upper bits cannot be directly read or written.
;
; When the PCL register is written, PCLATH<4:0> is copied at the same time to the upper 5 bits of
; PC.
;
; When a goto is executed, 11 bits embedded into the goto instruction are loaded into the PC<10:0>
; while bits PCLATH<4:3> are copied to bits PC<12:11>
;
; Changing PCLATH does NOT instantly change the PC register. The PCLATH will be used the next time
; a goto is executed (or similar opcode) or the PCL register is written to. Thus, to jump farther
; than the 11 bits (2047 bytes) in the goto opcode will allow, the PCLATH register is adjusted
; first and then the goto executed.
;
;--------------------------------------------------------------------------------------------------
;
; Revision History:
;
; 1.0   Initial code. 
;
;--------------------------------------------------------------------------------------------------
; Miscellaneous Notes
;
; incf vs decf rollover
;
; When incrementing multi-byte values, incf can be used because it sets the Z flag - then the Z
; flag is set, the next byte up should then be incremented.
; When decrementing multi-byte values, decf CANNOT be used because it sets the Z flag but NOT the
; C flag.  The next byte up is not decremented when the lower byte reaches zero, but when it rolls
; under zero.  This can be caught by loading w with 1 and then using subwf and catching the C flag
; cleared. (C flag is set for a roll-over with addwf, cleared for roll-under for subwf.
; For a quickie but not perfect count down of a two byte variable, decf and the Z flag can be used
; but the upper byte will be decremented one count too early.
;
;--------------------------------------------------------------------------------------------------
; Operational Notes
;
;
;
;--------------------------------------------------------------------------------------------------
; Hardware Control Description
;
; Function by Pin
;
; Port A        Pin/Options/Selected Option/Description  (only the most common options are listed)
;
; RA0   I/*,IOC,USB-D+                  ~ I ~ Sync Reset
; RA1   I/*,IOC,USB-D-                  ~ I ~ not used, tied to Sync on RC5
; RA2   not implemented in PIC16f1459
; RA3   I/*,IOC,T1G,MSSP-SS,Vpp,MCLR    ~ Vpp
; RA4   I/O,IOC,T1G,CLKOUT,CLKR, AN3    ~ CLKOUT
; RA5   I/O,IOC,T1CKI,CLKIN             ~ I ~ not used, tied to Sync Reset on RA0
; RA6   not implemented in PIC16f1459
; RA7   not implemented in PIC16f1459
;
; Port B        Pin/Options/Selected Option/Description  (only the most common options are listed)
;
; RB0   not implemented in PIC16f1459
; RB1   not implemented in PIC16f1459
; RB2   not implemented in PIC16f1459
; RB3   not implemented in PIC16f1459
; RB4   I/O,IOC,MSSP-SDA/SDI,AN10       ~ I ~ I2CSDA, I2C bus data line to slaves
; RB5   I/O,IOC,EUSART-RX/DX,AN11       ~ I ~ EUSART-RX, serial port data from Rabbit
; RB6   I/O,IOC,MSSP-SCL/SCK            ~ I ~ I2CSCL, I2C bus clock line to slaves
; RB7   I/O,IOC,EUSART-TX/CK            ~ O ~ EUSART-TX, serial port data to Rabbit
;
; Port C        Pin/Options/Selected Option/Description  (only the most common options are listed)
;
; RC0   I/O,AN4,C1/2IN+,ICSPDAT,Vref    ~ ICSPDAT ~ in circuit programming data line
; RC1   I/O,AN5,C1/2IN1-,ICSPCLK,INT    ~ ICSPCLK ~ in circuit programming clock line
; RC2   I/O,AN6,C1/2IN2-,DACOUT1        ~ O ~ signal to Rabbit Interrupt pin
; RC3   I/O,AN7,C1/2IN3-,DACOUT2,CLKR   ~ I ~ encoder 1, A input
; RC4   I/O,C1/2OUT                     ~ I ~ encoder 1, B input
; RC5   I/O,T0CKI,PWM1                  ~ O ~ Sync output to slaves
; RC6   I/O,AN8,PWM2,MSSP-SS            ~ I ~ encoder 2, A input
; RC7   I/O,AN9,MSSP-SDO                ~ I ~ encoder 2, B input
;
;end of Hardware Control Description
;--------------------------------------------------------------------------------------------------
;
; User Inputs
;
; There are no direct user inputs.
;
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; Defines
;

; COMMENT OUT "#define debug" line before using code in system.
; Defining debug will insert code which simplifies simulation by skipping code which waits on
; stimulus and performing various other actions which make the simulation run properly.
; Search for "ifdef debug" to find all examples of such code.

;#define debug 1     ; set debug testing "on"


; Rabbit to Master PIC Commands -- sent by Rabbit to trigger actions

RBT_NO_ACTION                    EQU 0x00
RBT_GET_ALL_STATUS               EQU 0x01
RBT_GET_SLAVE_PEAK_DATA          EQU 0x02
;RABBIT_RESET_ENCODERS           EQU 0x01
;RABBIT_GET_ENCODERS             EQU 0x02
;RABBIT_GET_PEAK_DATA            EQU 0x03
;RABBIT_SET_GAIN                 EQU 0x04
;RABBIT_SET_OFFSET               EQU 0x05
;RABBIT_RESET                    EQU 0xff

; Master PIC to Slave PIC Commands -- sent by Master to Slaves via I2C to trigger actions

PIC_NO_ACTION_CMD               EQU 0x00
PIC_GET_ALL_STATUS_CMD          EQU 0x01
PIC_START_CMD                   EQU 0x02
PIC_GET_PEAK_PKT_CMD            EQU 0X03
PIC_ENABLE_POT_CMD              EQU 0x04
PIC_DISABLE_POT_CMD             EQU 0x05


; end of Defines
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; Configurations, etc. for the Assembler Tools and the PIC

	LIST p = PIC16F1459	;select the processor

    errorlevel  -306 ; Suppresses Message[306] Crossing page boundary -- ensure page bits are set.

    errorLevel  -302 ; Suppresses Message[302] Register in operand not in bank 0.

	errorLevel	-202 ; Suppresses Message[205] Argument out of range. Least significant bits used.
					 ;	(this is displayed when a RAM address above bank 1 is used -- it is
					 ;	 expected that the lower bits will be used as the lower address bits)

#INCLUDE <p16f1459.inc> 		; Microchip Device Header File


;#include <xc.h>

; Specify Device Configuration Bits

; CONFIG1
; __config 0xF9E4
 __CONFIG _CONFIG1, _FOSC_INTOSC & _WDTE_OFF & _PWRTE_OFF & _MCLRE_ON & _CP_OFF & _BOREN_OFF & _CLKOUTEN_ON & _IESO_OFF & _FCMEN_OFF
; CONFIG2
; __config 0xFFFF
 __CONFIG _CONFIG2, _WRT_ALL & _CPUDIV_NOCLKDIV & _USBLSCLK_48MHz & _PLLMULT_4x & _PLLEN_DISABLED & _STVREN_ON & _BORV_LO & _LPBOR_OFF & _LVP_OFF

; _FOSC_INTOSC -> internal oscillator, I/O function on OSC1/CLKIN pin
; _WDTE_OFF -> watch dog timer disabled
; _PWRTE_OFF -> Power Up Timer disabled
; _MCLRE_ON -> MCLR/VPP pin is Master Clear with weak pull-up automatically enabled
; _CP_OFF -> Flash Program Memory Code Protection off
; _BOREN_OFF -> Power Brown-out Reset off
; _CLKOUTEN_ON -> CLKOUT function on, Fosc/4 -> CLKOUT pin
; _IESO_OFF -> Internal/External Oscillator Switchover off
;   (not used for this application since there is no external clock)
; _FCMEN_OFF -> Fail-Safe Clock Monitor off
;   (not used for this application since there is no external clock)
; _WRT_ALL -> Flash Memory Self-Write Protection on -- no writing to flash
;
; _CPUDIV_NOCLKDIV -> CPU clock not divided
; _USBLSCLK_48MHz -> only used for USB operation
; _PLLMULT_4x -> sets PLL (if enabled) multiplier -- 4x allows software override
; _PLLEN_DISABLED -> the clock frequency multiplier is not used
;
; _STVREN_ON -> Stack Overflow/Underflow Reset on
; _BORV_LO -> Brown-out Reset Voltage Selection -- low trip point
; _LPBOR_OFF -> Low-Power Brown-out Reset Off
; _LVP_OFF -> Low Voltage Programming off
;
; end of configurations
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; Hardware Definitions
;
; Note regarding Port Latches vs Pins
;
; Writing to individial port pins can cause problems because that uses a read-modify-write
; operation. If the pins are read, some outputs could read differently than they are programmed in
; the port latch if the external hardware is causing the outputs to change state slowly. Thus,
; when the read-modify-write operation writes back to the latch, the wrong value for some pins
; might be permanently written.
;
; To avoid this, such operations should be done to the port latches (LATA, LATB, LATC) instead of
; the port (PORTA, PORTB, PORTC). This ensures that the values read are what have actually been
; programmed.
;
; For convenience, the port defines below are set to the Port when the associated signal is an
; input and to the Latch when the associated signal is an output so that the proper register will
; be used for each case.
;

; Port A defines

SYNC_RESET      EQU 0           ; input on RA0
SYNCT_RA1       EQU 1           ; not used, must be input, same signal as RC5
RA2             EQU 2           ; not implemented in PIC16f1459
RA3             EQU 3           ; Vpp ~ not used as I/O
RA4             EQU 4           ; CLKOUT ~ not used as I/O
SYNC_RESET_RA5  EQU 5           ; not used, same signal as RA0

SYNC_RESET_RD   EQU PORTA

; Port B defines

RB0             EQU 0           ; not implemented in PIC16f1459
RB1             EQU 1           ; not implemented in PIC16f1459
RB2             EQU 2           ; not implemented in PIC16f1459
RB3             EQU 3           ; not implemented in PIC16f1459
I2CSDA          EQU 4           ; I2C bus SDA line
SERIAL_IN       EQU 5           ; serial port in from Rabbit
I2CSCL          EQU 6           ; I2C bus SCL line
SERIAL_OUT      EQU 7           ; serial port out to Rabbit

; Port C defines

RC0             EQU 0           ; ICSPDAT ~ not used as I/O
RC1             EQU 1           ; ICSPCLK ~ not used as I/O
RBBT_INT        EQU 2           ; out to Rabbit Interrupt pin
ENC1A           EQU 3           ; encoder 1, A input
ENC1B           EQU 4           ; encoder 1, B input
SYNCT           EQU 5           ; output to slaves for circumfential clock position
ENC2A           EQU 6           ; encoder 2, A input
ENC2B           EQU 7           ; encoder 2, B input

SYNCT_WR        EQU LATC
ENCODERS_RD     EQU PORTC

; I2C bus ID byte for writing to the currently selected digital pot (see notes at top of page)
; upper nibble = 1010 (bits 7-4)
; chip A2-A0 inputs = 100 (bits 3-1)
; address of the currently selected chip: b'1010100'
; the lsb is R/W bit - set to 0 for write (bit 0)

DIG_POT_WR_CODE EQU     b'10101000'

; address of each pot inside the chip

POT1_ADDR           EQU     0x0
POT2_ADDR           EQU     0x1
POT3_ADDR           EQU     0x2
POT4_ADDR           EQU     0x3

; I2C bus ID bytes for writing and reading to the Slave PICs (see notes at top of page)
; upper nibble = 1111 (bits 7-4)
; chip A2-A0 inputs = 000 - 111 (bits 3-1) (set these to choose the desired slave)
; address of the slaves: b'1111xxx' where xxx is set to select slave 0-7
; the lsb is R/W bit:
;  R/W bit set to 0 (bit 0) for writing
;  R/W bit set to 1 (bit 0) for reading

SLAVE_PIC_WR_CODE   EQU     b'11110000'
SLAVE_PIC_RD_CODE   EQU     b'11110001'

; end of Hardware Definitions
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; Software Definitions

; bits in flags variable

UNUSED     EQU     0x0

; bits in flags2 variable

HEADER_BYTE_1_RCVD  EQU     0x00
HEADER_BYTE_2_RCVD  EQU     0x01
LENGTH_BYTE_VALID   EQU     0x02
SERIAL_PACKET_READY EQU     0x03

SERIAL_RCV_BUF_LEN      EQU .20     ; these should always match with one preceded by a period
SERIAL_RCV_BUF_LEN_RES  EQU 20      ; one is used in the variable definition, one used in code

I2C_RCV_BUF_LEN      EQU .5     ; these should always match with one preceded by a period
I2C_RCV_BUF_LEN_RES  EQU 5      ; one is used in the variable definition, one used in code

I2C_XMT_BUF_LEN      EQU .20     ; these should always match with one preceded by a period
I2C_XMT_BUF_LEN_RES  EQU 20      ; one is used in the variable definition, one used in code

; end of Software Definitions
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; Variables in RAM
;
; Note that you cannot use a lot of the data definition directives for RAM space (such as DB)
; unless you are compiling object files and using a linker command file.  The cblock directive is
; commonly used to reserve RAM space or Code space when producing "absolute" code, as is done here.
; 

; Assign variables in RAM - Bank 0 - must set BSR to 0 to access
; Bank 0 has 80 bytes of free space

 cblock 0x20                ; starting address

    flags                   ; bit 0: 0 = 
                            ; bit 1: 0 = 
                            ; bit 2: 0 = 
                            ; bit 3: 0 = 
                            ; bit 4: 0 = 
                            ; bit 5: 0 = 
							; bit 6: 0 = 
							; bit 7: 0 = 

    flags2                  ; bit 0: 1 = first serial port header byte received
                            ; bit 1: 1 = second serial port header byte received
                            ; bit 2: 1 = serial port packet length byte received and validated
                            ; bit 3: 1 = data packet ready for processing
                            ; bit 4: 0 =
                            ; bit 5: 0 =
							; bit 6: 0 =
							; bit 7: 0 =

    serialIntScratch0
    serialRcvPktLen
    serialRcvPktCnt
    serialRcvBufPtr
    serialRcvBufLen
    serialPortErrorCnt

    serialRcvBuf:SERIAL_RCV_BUF_LEN_RES

    i2cXmtBufNumBytes
    i2cXmtBufPtrH
    i2cXmtBufPtrL
    i2cXmtBuf:I2C_XMT_BUF_LEN_RES

    i2cRcvBufPtrH
    i2cRcvBufPtrL
    i2cRcvBuf:I2C_RCV_BUF_LEN_RES

    hiCurrentLimitPot       ; value for digital pot which sets the high current limit value
    loCurrentLimitPot       ; value for digital pot which sets the high current limit value
    powerLevel              ; store the power level of the high/low current values in use

    scratch0                ; these can be used by any function
    scratch1
    scratch2
    scratch3
    scratch4
    scratch5
    scratch6
    scratch7
    scratch8
    scratch9
    scratch10

	debug0					; debug mks - use a scratch variable instead?
	debug1					; debug mks - use a scratch variable instead?


 endc

;-----------------

; Assign variables in RAM - Bank 1 - must set BSR to 1 to access
; Bank 1 has 80 bytes of free space

 cblock 0xa0                ; starting address

    Flags2                  ; bit 0: 0 = LCD buffer not busy, 1 = buffer busy
                            ; bit 1: 0 = start bit not due, 1 = transmit start bit next
                            ; bit 2: 0 = stop bit not due,  1 = transmit stop bit next
                            ; bit 3: 0 = not buffer end,  1 = buffer end reached
                            ; bit 4: 0 = not delaying, 1 = delaying

 endc

;-----------------

; Assign variables in RAM - Bank 3 - must set BSR to 3 to access
; Bank 2 has 80 bytes of free space

 cblock 0x120                ; starting address

	block1PlaceHolder

 endc

;-----------------

; NOTE: use of this block by interrupts is not necessary with the PIC16f1459 as it pushes critical
; registers onto a stack.
;
; Define variables in the memory which is mirrored in all 4 RAM banks.  This area is usually used
; by the interrupt routine for saving register states because there is no need to worry about
; which bank is current when the interrupt is invoked.
; On the PIC16F628A, 0x70 thru 0x7f is mirrored in all 4 RAM banks.

; NOTE:
; This block cannot be used in ANY bank other than by the interrupt routine.
; The mirrored sections:
;
;	Bank 0		Bank 1		Bank 2		Bank3
;	70h-7fh		f0h-ffh		170h-17fh	1f0h-1ffh
;

 cblock	0x70
    W_TEMP
    FSR0L_TEMP
    FSR0H_TEMP
    STATUS_TEMP
    PCLATH_TEMP	
 endc

; end of Variables in RAM
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; Power On and Reset Vectors
;

	org 0x00                ; Start of Program Memory

	goto start              ; jump to main code section
	nop			            ; Pad out so interrupt
	nop			            ; service routine gets
	nop			            ; put at address 0x0004.


; interrupt vector at 0x0004
; NOTE: You must clear PCLATH before jumping to the interrupt routine - if PCLATH has bits set it
; will cause a jump into an unexpected program memory bank.

	clrf	STATUS          ; set to known state
    clrf    PCLATH          ; set to bank 0 where the ISR is located
    goto 	handleInterrupt	; points to interrupt service routine

; end of Reset Vectors
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; start
;

start:

    call    setup           ; preset variables and configure hardware

mainLoop:

;debug mks

 ;   call    handleAllStatusRbtCmd

 ;   call    handleSerialPacket

;    call    handleSerialPortReceiveInt

;    banksel flags2
;    goto    rsl2    ;debug mks -- remove this
;debug mks end

    banksel flags2                          ; handle packet in serial receive buffer if ready
    btfsc   flags2, SERIAL_PACKET_READY
    call    handleSerialPacket

    goto    mainLoop
    
; end of start
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; handleSerialPacket
;
; Processes a packet in the serial receive buffer.
;

handleSerialPacket:

    ;verify the checksum

    banksel flags2

    movf    serialRcvPktLen, W          ; copy number of bytes to variable for counting
    movwf   serialRcvPktCnt

    clrf    FSR0H                       ; point FSR0 at start of receive buffer
    movlw   serialRcvBuf
    movwf   FSR0L

    clrw                                ; preload W with zero

hspSumLoop:

    addwf   INDF0, W                    ; sum each data byte and the checksum byte at the end
    incf    FSR0L, F
    decfsz  serialRcvPktCnt, F
    goto    hspSumLoop

    movf    WREG, F                         ; test for zero
    btfsc   STATUS, Z                       ; error if not zero
    goto    parseCommandFromSerialPacket    ; checksum good so handle command

hspError:

    incf    serialPortErrorCnt, F           ; track errors
    goto    resetSerialPortReceiveBuffer

; end of handleSerialPacket
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; parseCommandFromSerialPacket
;
; Parses the command byte in a serial packet and performs the appropriate action.
;

parseCommandFromSerialPacket:

    banksel flags2

; parse the command byte by comparing with each command

    movf    serialRcvBuf, W
    sublw   RBT_GET_ALL_STATUS
    btfsc   STATUS,Z
    goto    handleAllStatusRbtCmd

    movf    serialRcvBuf, W
    sublw   RBT_GET_SLAVE_PEAK_DATA
    btfsc   STATUS,Z
    goto    handleGetSlavePeakDataRbtCmd

    goto    resetSerialPortReceiveBuffer

; end of parseCommandFromSerialPacket
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; handleAllStatusRbtCmd
;
; Handles the PIC_GET_ALL_STATUS command, returning the status byte, the error count for data from
; the Rabbit, the error count for data from the Slave PICS, and the status and error count for
; data from the Master as retrieved from each of the Slaves.
;

handleAllStatusRbtCmd:

    banksel scratch0

    movlw   0x00                    ; debug mks -- need to scan through all Slaves
    movwf   scratch0

    ;movlw   PIC_GET_ALL_STATUS_CMD ; debug mks
    movlw   PIC_ENABLE_POT_CMD  ; debug mks
    movwf   scratch1

    call    sendCommandToSlavePIC

    ;debug mks
    movlw   0x69
    banksel TXREG
    movwf   TXREG
    ;debug mks end

    goto    resetSerialPortReceiveBuffer

; end of handleAllStatusRbtCmd
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; handleGetSlavePeakDataRbtCmd
;
; Handles the PIC_GET_ALL_STATUS command, returning the status byte, the error count for data from
; the Rabbit, the error count for data from the Slave PICS, and the status and error count for
; data from the Master as retrieved from each of the Slaves.
;

handleGetSlavePeakDataRbtCmd:

    banksel flags2

    movlw   0x69
    banksel TXREG
    movwf   TXREG

    goto    resetSerialPortReceiveBuffer

; end of handleGetSlavePeakDataRbtCmd
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; setup
;
; Presets variables and configures hardware.
;
; NOTE: The system does not use the internal comparators even though it appears there are analog
; inputs to some of the comparator pins.  Those inputs are the output of external op-amps
; acting as comparators - their outputs are read as digital inputs on RA3 and RA4
;

setup:

    call    setupClock      ; set system clock source and frequency

    call    setupPortA      ; prepare Port A for I/O

    call    setupPortB      ; prepare Port B for I/O

    call    setupPortC      ; prepare Port C  for I/O

    call    initializeOutputs

    call    setupSerialPort

    call    setupI2CMaster7BitMode ; prepare the I2C serial bus for use

;    call    setDigitalPots  ; set digital pot values to stored values

;start of hardware configuration

    clrf    FSR0H            ;high byte of indirect addressing pointers -> 0
    clrf    FSR1H

    clrf    INTCON          ; disable all interrupts

    banksel OPTION_REG
    movlw   0x58
    movwf   OPTION_REG      ; Option Register = 0x58   0101 1000 b
                            ; bit 7 = 0 : weak pull-ups are enabled by individual port latch values
                            ; bit 6 = 1 : interrupt on rising edge
                            ; bit 5 = 0 : TOCS ~ Timer 0 run by internal instruction cycle clock (CLKOUT ~ Fosc/4)
                            ; bit 4 = 1 : TOSE ~ Timer 0 increment on high-to-low transition on RA4/T0CKI/CMP2 pin (not used here)
							; bit 3 = 1 : PSA ~ Prescaler disabled; Timer0 will be 1:1 with Fosc/4
                            ; bit 2 = 0 : Bits 2:0 control prescaler:
                            ; bit 1 = 0 :    000 = 1:2 scaling for Timer0 (if prescaler enabled)
                            ; bit 0 = 0 :
    
;end of hardware configuration

	
; enable the interrupts

	bsf	    INTCON,PEIE	    ; enable peripheral interrupts (Timer0 and serial port are peripherals)
;    bsf     INTCON,T0IE     ; enable TMR0 interrupts
    bsf     INTCON,GIE      ; enable all interrupts

    return

; end of setup
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; initializeOutputs
;
; Initializes all outputs to known values.
;
; Write to port latches to avoid problems with read-modify-write when changing multiple outputs
; in quick succession.
;

initializeOutputs:


    return

; end of initializeOutpus
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; setupClock
;
; Sets up the system clock source and frequency.
;
; Assumes clock related configuration bits are set as follows:
;
;   _FOSC_INTOSC,  _CPUDIV_NOCLKDIV, _PLLMULT_4x, _PLLEN_DISABLED
;
; Assumes all programmable clock related options are at Reset default values.
;
; NOTE: Adjust I2C baud rate generator value when Fosc is changed.
;

setupClock:

    ; choose internal clock frequency of 16 Mhz

    banksel OSCCON

    bsf     OSCCON, IRCF0
    bsf     OSCCON, IRCF1
    bsf     OSCCON, IRCF2
    bsf     OSCCON, IRCF3

    return

; end of setupClock
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; setupPortA
;
; Sets up Port A for I/O operation.
;
; NOTE: Writing to PORTA is same as writing to LATA for PIC16f1459. The code example from the
; data manual writes to both -- probably to be compatible with other PIC chips.
;
; NOTE: RA0, RA1 and RA3 can only be inputs on the PIC16f1459 device. 
;       RA2, RA6, RA7 are not implemented.
;

setupPortA:

    ; start with known safe configuration

    banksel PORTA                       ; init port value
    clrf    PORTA

    banksel LATA                        ; init port data latch (same as writing to PORTA)
    clrf    LATA

    banksel WPUA                        ; disable all weak pullups
    clrf    WPUA

    banksel ANSELA                      ; all pins digital I/O
    clrf    ANSELA

    banksel TRISA                       ; all pins digital inputs
    movlw   b'11111111'                 
    movwf   TRISA

    ; customize I/O directions
    ; some pins configured further by other functions for setup of serial port, I2C, ADC, etc.

    bsf     TRISA, SYNC_RESET           ; input

    return

; end of setupPortA
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; setupPortB
;
; Sets up Port B for I/O operation.
;
; NOTE: Writing to PORTB is same as writing to LATB for PIC16f1459. The code example from the
; data manual writes to both -- probably to be compatible with other PIC chips.
;
; NOTE: RB0, RB1, RB2, RB3 are not implemented on the PIC16f1459 device.
;

setupPortB:

    ; start with known safe configuration

    banksel PORTB                       ; init port value
    clrf    PORTB

    banksel LATB                        ; init port data latch (same as writing to PORTA)
    clrf    LATB

    banksel WPUB                        ; disable all weak pullups
    clrf    WPUB

    banksel ANSELB                      ; all pins digital I/O
    clrf    ANSELB

    banksel TRISB                       ; all pins digital inputs
    movlw   b'11111111'
    movwf   TRISB

    bsf     LATB,SERIAL_OUT             ; initialize SERIAL_OUT high before changing pin to output
                                        ; so a start bit won't be transmitted

    ; customize I/O directions
    ; some pins configured further by other functions for setup of serial port, I2C, ADC, etc.

    ; no configurations here

    return

; end of setupPortB
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; setupPortC
;
; Sets up Port C for I/O operation.
;
; NOTE: Writing to PORTC is same as writing to LATC for PIC16f1459. The code example from the
; data manual writes to both -- probably to be compatible with other PIC chips.
;

setupPortC:

    ; start with known safe configuration
    ; Port C does not have a weak pull-up register

    banksel PORTC                       ; init port value
    clrf    PORTC

    banksel LATC                        ; init port data latch (same as writing to PORTA)
    clrf    LATC

    banksel ANSELC                      ; all pins digital I/O
    clrf    ANSELC

    banksel TRISC                       ; all pins digital inputs
    movlw   b'11111111'
    movwf   TRISC

    ; customize I/O directions
    ; some pins configured further by other functions for setup of serial port, I2C, ADC, etc.

    bcf     TRISC, RBBT_INT             ; output to Rabbit Interrupt pin
    bsf     TRISC, ENC1A                ; encoder 1, A input
    bsf     TRISC, ENC1B                ; encoder 1, B input
    bcf     TRISC, SYNCT                ; output to slaves for circumferential clock position
    bsf     TRISC, ENC2A                ; encoder 2, A input
    bsf     TRISC, ENC2B                ; encoder 2, B input

    return

; end of setupPortC
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; setupSerialPort
;
; Sets up the serial port for communication with the Rabbit micro-controller.
; Also prepares the receive and transmit buffers for use.
;

setupSerialPort:

    call    resetSerialPortReceiveBuffer

    banksel serialRcvBufLen     ;store buffer length constant in a variable for easier maths
    movlw   SERIAL_RCV_BUF_LEN
    movwf   serialRcvBufLen

    clrf    serialPortErrorCnt

    ;set the baud rate to 57,600 (will actually be 57.97K with 0.64% error)
    ;for Fosc of 16 Mhz: SYNC = 0, BRGH = 1, BRG16 = 1, SPBRG = 68

    banksel TXSTA
    bsf     TXSTA, BRGH
    banksel BAUDCON
    bsf     BAUDCON, BRG16
    banksel SPBRGH
    clrf    SPBRGH
    banksel SPBRGL
    movlw   .68
    movwf   SPBRGL

    ;set UART mode and enable receiver and transmitter

    banksel ANSELB          ; RB5/RB7 digital I/O as RX/TX
    clrf    ANSELB

    banksel TRISB
    bsf     TRISB, TRISB5   ; set RB5/RX to input
    bcf     TRISB, TRISB7   ; set RB7/TX to output

    banksel TXSTA
    bcf     TXSTA, SYNC     ; clear bit for asynchronous mode
    bsf     TXSTA, TXEN     ; enable the transmitter
    bsf     RCSTA, CREN     ; enable the receiver
    bsf     RCSTA, SPEN     ; enable EUSART, configure TX/CK I/O pin as an output

    ; enable the receive interrupt; the transmit interrupt (PIE1/TXIE) is not enabled until data is
    ; ready to be sent
    ; for interrupts to occur, INTCON/PEIE and INTCON/GIE must be enabled also

    banksel PIE1
    bsf     PIE1, RCIE      ; enable receive interrupts

    return

; end of setupSerialPort
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; resetSerialPortReceiveBuffer
;
; Resets all flags and variables associated with the serial port receive buffer.
;

resetSerialPortReceiveBuffer:

    banksel flags2

    bcf     flags2, HEADER_BYTE_1_RCVD
    bcf     flags2, HEADER_BYTE_2_RCVD
    bcf     flags2, LENGTH_BYTE_VALID
    bcf     flags2, SERIAL_PACKET_READY

    clrf    serialRcvPktLen
    clrf    serialRcvPktCnt
    movlw   serialRcvBuf
    movwf   serialRcvBufPtr

    banksel RCSTA           ; check for overrun error - must be cleared to receive more data
    btfss   RCSTA, OERR
    goto    RSPRBnoOERRError

    bcf     RCSTA, CREN     ; clear error by disabling/enabling receiver
    bsf     RCSTA, CREN

RSPRBnoOERRError:

    return

; end of resetSerialPortReceiveBuffer
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; sendCommandToSlavePIC
;
; Sends a command byte to a Slave PIC via the I2C bus.
;
; scratch0 should contain the slave's ID number (3 lsb's of the slave's IC2 address)
; scratch1 should contain the command byte.
;

sendCommandToSlavePIC:

    banksel scratch1                    ; copy the command to the first byte in xmt buffer
    movf    scratch1, W
    banksel i2cXmtBufNumBytes
    movwf   i2cXmtBuf

    movlw   .1                          ; send one byte - the command byte
    movwf   i2cXmtBufNumBytes

    clrf    FSR0H                        ; point FSR0 at start of xmt buffer
    movlw   i2cXmtBuf
    movwf   FSR0L

    clrf    FSR1H                        ; point FSR1 at number of bytes to transmit variable
    movlw   i2cXmtBufNumBytes
    movwf   FSR1L

    banksel scratch0
    movf    scratch0, W

    call    sendBytesToSlavePICViaI2C

    return

; end of sendCommandToSlavePIC
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; setDigitalPots
;
; Sets the digital pot values to their stored settings.
;

setDigitalPots:


    banksel scratch0
    movlw   POT1_ADDR
    movwf   scratch0
    ;debug mks -- movlw   VOLTAGE_MONITOR_POT
    movwf   scratch1
    call    setDigitalPotInChip

    return

; end of setDigitalPots
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; handleInterrupt
;
; All interrupts call this function.  The interrupt flags must be polled to determine which
; interrupts actually need servicing.
;
; Note that after each interrupt type is handled, the interrupt handler returns without checking
; for other types.  If another type has been set, then it will immediately force a new call
; to the interrupt handler so that it will be handled.
;
; NOTE NOTE NOTE
; It is important to use no (or very few) subroutine calls.  The stack is only 16 deep and
; it is very bad for the interrupt routine to use it.
;

handleInterrupt:

                                    ; INTCON is a core register, no need to banksel
	btfsc 	INTCON, T0IF     		; Timer0 overflow interrupt?
	goto 	handleTimer0Int

    banksel PIR1

    btfsc   PIR1, RCIF              ; serial port receive interrupt?
    goto    handleSerialPortReceiveInt

    btfsc   PIR1, TXIF              ; serial port transmit interrupt?
    goto    handleSerialPortTransmitInt


; Not used at this time to make interrupt handler as small as possible.
;	btfsc 	INTCON, RBIF      		; NO, Change on PORTB interrupt?
;	goto 	portB_interrupt       	; YES, Do PortB Change thing

INT_ERROR_LP1:		        		; NO, do error recovery
	;GOTO INT_ERROR_LP1      		; This is the trap if you enter the ISR
                               		; but there were no expected interrupts

endISR:

	retfie                  	; Return and enable interrupts

; end of handleInterrupt
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; handleTimer0Int
;
; This function is called when the Timer0 register overflows.
;
; NOTE NOTE NOTE
; It is important to use no (or very few) subroutine calls.  The stack is only 16 deep and
; it is very bad for the interrupt routine to use it.
;

handleTimer0Int:

	bcf 	INTCON,T0IF     ; clear the Timer0 overflow interrupt flag

    ; do stuff here
    
    goto    endISR    

; end of handleTimer0Int
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; handleSerialPortReceiveInt
;
; This function is called when a byte(s) has been received by the serial port. The byte(s) will be
; checked to see if it is a header byte, a packet length byte, or a data byte. Data bytes will be
; stored in a buffer. If an error occurs in receiving a packet, the function will ignore data
; received before the error and begin watching for the next packet signature. Upon receiving a
; complete packet, a flag will be set to notify the main loop.
;
; The receive register is a two byte fifo, so two bytes could be ready. This function will process
; all bytes available.
;
; The RCIF flag is cleared by reading all data from the two byte receive FIFO.
;
; This code check each byte sequence to see if it starts with a header prefix (0xaa,0x55) followed
; by a valid length byte. If these are found, the bytes after the length byte are stored in a
; buffer. If the sequence is not matched or the supposed length byte is larger than the buffer,
; all flags are reset and the search for the first header byte starts over.
;
; Packet format:
;   0xaa, 0x55, length, data1, data2, data3,...checksum.
;
; This interrupt function does not verify the checksum; the main loop should do that if required.
; Once a packet has been received, a flag is set to alert the main loop that it is ready for
; processing. All further data will be ignored until the main loop clears that flag. If an error
; occurs, the data received to that point will be discarded and the search for the next packet
; begun anew.
;
; The packet length byte is the number of data bytes plus one for the checksum byte. It does not
; include the two header bytes or the length byte itself. If the length byte value is 0 or is
; greater than the buffer size, the packet will be ignored. If the length byte value is greater
; than the actual number of bytes sent (but still less than the buffer size), the current packet
; AND the next packet(s) will be discarded as the interrupt routine will wait until enough bytes
; are received from subsequent packets to equal the erroneously large length byte value.
;
; Thus, only one packet at a time can be handled. The processing required is typically minimal, so
; the main loop should be able to process each packet before another is received. Some care should
; be taken by the receiver to not flood the line with packets.
;
; The main loop does all the actual processing in order to minimize the overhead of the interrupt.
;
; NOTE NOTE NOTE
; It is important to use no (or very few) subroutine calls.  The stack is only 16 deep and
; it is very bad for the interrupt routine to use it.
;

handleSerialPortReceiveInt:

    ; if the packet ready flag is set, ignore all data until main loop clears it

    banksel flags2
    btfsc   flags2, SERIAL_PACKET_READY
    goto    rslExit

    ;RCREG is a two byte FIFO and may contain two bytes; read until RCIF flag is clear

readSerialLoop:

    banksel RCREG
    movf    RCREG, W        ; get byte from receive fifo

    banksel flags2

    btfsc   flags2, HEADER_BYTE_1_RCVD      ; header byte 1 already received?
    goto    rsl1                            ; if so, check for header byte 2

    bsf     flags2, HEADER_BYTE_1_RCVD      ; preset the flag, will be cleared on fail

    sublw   0xaa                            ; check for first header byte of 0xaa
    btfsc   STATUS, Z                       ; equal?
    goto    rsllp                           ; continue on, leaving flag set

    goto    rslError                        ; not header byte 1, reset all to restart search

rsl1:
    btfsc   flags2, HEADER_BYTE_2_RCVD      ; header byte 2 already received?
    goto    rsl2                            ; if so, check for length byte

    bsf     flags2, HEADER_BYTE_2_RCVD      ; preset the flag, will be cleared on fail

    sublw   0x55                            ; check for second header byte of 0x55
    btfsc   STATUS, Z                       ; equal?
    goto    rsllp                           ; continue on, leaving flag set

    goto    rslError                        ; not header byte 2, reset all to restart search

rsl2:
    btfsc   flags2, LENGTH_BYTE_VALID       ; packet length byte already received and validated?
    goto    rsl3                            ; if so, jump to store data byte

    movwf   serialRcvPktLen                 ; store the packet length
    movwf   serialRcvPktCnt                 ; store it again to count down number of bytes stored

    bsf     flags2, LENGTH_BYTE_VALID       ; preset the flag, will be cleared on fail

    movf    serialRcvPktLen, F              ; check for invalid packet size of 0
    btfsc   STATUS, Z
    goto    rslError

    subwf   serialRcvBufLen, W              ; check if packet length < buffer length
    btfsc   STATUS, C                       ; carry cleared if borrow was required
    goto    rsllp                           ; continue on, leaving flag set
                                            ; if invalid length, reset all to restart search

rslError:

    incf    serialPortErrorCnt, F           ; track errors
    call    resetSerialPortReceiveBuffer    
    goto    rsllp

rsl3:

    movwf   serialIntScratch0               ; store the new character
    clrf    FSR0H                           ; load FSR0 with buffer pointer
    movf    serialRcvBufPtr, W
    movwf   FSR0L
    incf    serialRcvBufPtr, F              ; advance the buffer pointer (FSR0L not affected)

    movf    serialIntScratch0, W            ; retrieve the new character
    movwf   INDF0                           ; store in buffer

    decfsz  serialRcvPktCnt, F              ; count down number of bytes stored
    goto    rsllp                           ; continue collecting until counter reaches 0

rsl4:

    bsf     flags2, SERIAL_PACKET_READY     ; flag main loop that a data packet is ready
    goto    rslExit

rsllp:

    banksel PIR1                            ; loop until receive fifo is empty
    btfsc   PIR1, RCIF
    goto    readSerialLoop

rslExit:

    banksel RCSTA           ; check for overrun error - must be cleared to receive more data
    btfss   RCSTA, OERR
    goto    noOERRError

    bcf     RCSTA, CREN     ; clear error by disabling/enabling receiver
    bsf     RCSTA, CREN

noOERRError:

    goto    endISR

; end of handleSerialPortReceiveInt
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; handleSerialPortTransmitInt
;
; This function is called when a byte is to be transmitted to the host via serial port.
;
; NOTE NOTE NOTE
; It is important to use no (or very few) subroutine calls.  The stack is only 16 deep and
; it is very bad for the interrupt routine to use it.
;
; The TXIF flag is cleared in the second instruction cycle after writing data to TXREG.
;

handleSerialPortTransmitInt:


    goto    endISR

; end of handleSerialPortTransmitInt
;--------------------------------------------------------------------------------------------------
   
;--------------------------------------------------------------------------------------------------
; SetBank0ClrWDT        
;
; Set Bank 0, Clear high byte of FSR pointers, Clear WatchDog timer
; 

SetBank0ClrWDT:

    clrf   FSR0H            ;high byte of indirect addressing pointers -> 0
    clrf   FSR1H

    banksel flags

    clrwdt                  ;keep watchdog from triggering

    return

;end of SetBank0ClrWDT
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; bigDelay
;
; Delays while periodically clearing WDT to prevent watch dog timer trigger.
;
; On entry at bigDelay, W holds LSB of delay value, MSB will be 0.
; On entry at bigDelayA, scratch1:W holds delay value.
;
; Notes on code for decrementing to 0:
;  subtract 1 from LSB by adding 0xff (two's comp for -1)
;  C bit will be set until LSB goes from 0 to 255; 0 is only value added to 0xff that won't carry
;  When C bit not set, subtract 1 from MSB until it reaches 0
;
; Uses W, scratch0, scratch1, scratch2, scratch3
;

bigDelay:
    clrf    scratch1
bigDelayA:
    movwf   scratch0        ; store W

LoopBD1:

	; call inner delay for each count of outer delay

    movlw   0x1
    movwf   scratch3
    movlw   0x6a            ; scratch3:W = delay value
    call    smallDelayA

    movlw   0xff            ; decrement LSByte by adding -1
    addwf   scratch0,F
    btfss   STATUS,C		; did LSByte roll under (0->255)?
	decf	scratch1,F		; decrement MSByte after LSByte roll under
	movf	scratch0,W		; check MSB:LSB for zero
	iorwf	scratch1,W
	btfsc	STATUS,Z
	goto    SetBank0ClrWDT  ; counter = 0, reset stuff and return

    goto    LoopBD1         ; loop until outer counter is zero

; end of bigDelay
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; smallDelay
;
; Delays while periodically clearing WDT to prevent watch dog timer trigger.
;
; On entry at smallDelay, W holds LSB of delay value, MSB will be 0.
; On entry at smallDelayA, scratch3:W holds delay value.
;
; Uses scratch2, scratch3
;

smallDelay:
    clrf    scratch3
smallDelayA:
    movwf   scratch2        ; store W

LoopSD1:

    clrwdt                  ; keep watch dog timer from triggering

    movlw   0xff            ; decrement LSByte by adding -1
    addwf   scratch2,F
    btfss   STATUS,C		; did LSByte roll under (0->255)?
	decf	scratch3,F		; decrement MSByte after LSByte roll under
	movf	scratch2,W		; check MSB:LSB for zero
	iorwf	scratch3,W
	btfsc	STATUS,Z
	return

    goto    LoopSD1         ; loop until outer counter is zero

; end of smallDelay
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; setDigitalPotInChip
;
; Sets to the value in scratch1 the pot specified by scratch0 in whichever digital pot chip is
; currently enabled.
;
; The desired chip is enabled by sending the PIC_ENABLE_POT_CMD to the PIC connected to that chip.
;

setDigitalPotInChip:

    call    clearSSP1IF             ; make sure flag is cleared before starting

    call    generateI2CStart

    movlw   DIG_POT_WR_CODE         ; send proper ID code to write to digital pot chip 1
    call    sendI2CByte             ; send code on I2C bus after SP1IF goes high

    banksel scratch0                ; send the address of the pot in the chip to access
    movf    scratch0,W
    call    sendI2CByte

    banksel scratch1                ; send the pot value
    movf    scratch1,W
    call    sendI2CByte

    call    waitForSSP1IFHighThenClearIt    ; wait for high flag upon transmission completion

    call    generateI2CStop

    call    waitForSSP1IFHighThenClearIt    ; wait for high flag upon stop condition finished

    goto    cleanUpI2CAndReturn             ; reset all status and error flags

; end of setDigitalPotInChip
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; sendBytesToSlavePICViaI2C
;
; Sends bytes to a Slave PIC via the I2C bus.
;
; WREG should contain the slave's ID number (3 lsb's of the slave's IC2 address)
; FSR0 should point to start of buffer.
; FSR1 should point to variable containing number of bytes to be transmitted. (must be > 0)
;

sendBytesToSlavePICViaI2C:

    call    clearSSP1IF             ; make sure flag is cleared before starting

    call    generateI2CStart

    lslf    WREG, W                 ; shift the address lsb bits up to merge with the write code
    iorlw   SLAVE_PIC_WR_CODE       ; merge slave address lsbs with upper nibble and rd/wr flag

    call    sendI2CByte             ; send byte in W register on I2C bus after SSP1IF goes high

loopSBLP1:

    moviw   FSR0++                  ; load next byte to be sent
    call    sendI2CByte

	decfsz	INDF1, F                ; count down number of bytes transferred
	goto	loopSBLP1               ; not zero yet - transfer more bytes

    call    waitForSSP1IFHighThenClearIt    ; wait for high flag upon transmission completion

    call    generateI2CStop

    call    waitForSSP1IFHighThenClearIt    ; wait for high flag upon stop condition finished

    call    cleanUpI2CAndReturn             ; reset all status and error flags

    return

; end of sendBytesToSlavePICViaI2C
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; generateI2CStart
;
; Generates a start condition on the I2C bus.
;

generateI2CStart:

    banksel SSP1CON2
    bsf     SSP1CON2,SEN

    return

; end of generateI2CStart
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; generateI2CRestart
;
; Generates a restart condition on the I2C bus.
;

generateI2CRestart:

    banksel SSP1CON2
    bsf     SSP1CON2,RSEN

    return

; end of generateI2CRestart
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; generateI2CStop
;
; Generates a stop condition on the I2C bus.
;

generateI2CStop:

    banksel SSP1CON2
    bsf     SSP1CON2,PEN

    return

; end of generateI2CStop
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; clearSSP1IF
;
; Sets the SSP1IF bit in register PIR1 to 0.
;

clearSSP1IF:

    banksel PIR1
    bcf     PIR1, SSP1IF

    return

; end of clearSSP1IF
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; cleanUpI2CAndReturn
;
; Clears all I2C status and error flags to ensure the bus is ready for further use.
;

cleanUpI2CAndReturn:

    call    clearSSP1IF         ; clear the I2C interrupt flag
    call    setCKP              ; release the I2C clock line so master can send next byte
    call    clearSSP1OV         ; clears the overflow bit to allow new data to be read
    call    clearWCOL           ; clears the write collision bit to allow new data to be written

    return

; end of cleanUpI2CAndReturn
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; setCKP
;
; Sets the CKP bit in register SSP1CON1 to 1.
;
; This will release the I2C bus clock so the master can transmit the next byte.
;

setCKP:

    banksel SSP1CON1
    bsf     SSP1CON1, CKP

    return

; end of setCKP
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; clearSSP1OV
;
; Clears the MSSP overflow bit to allow new bytes to be read.
;
; The bit is set if a byte was received before the previous byte was read from the buffer.
;

clearSSP1OV:

    banksel SSP1CON1
    bcf     SSP1CON1,SSPOV

    return

; end of clearSSP1OV
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; clearWCOL
;
; Clears the MSSP write collision bit to allow new bytes to be written
;
; The bit is set if a byte was placed in SSP1BUF at an improper time.
;

clearWCOL:

    banksel SSP1CON1
    bcf     SSP1CON1, WCOL

    return

; end of clearWCOL
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; waitForSSP1IFHigh
;
; Waits in a loop for SSP1IF bit in register PIR1 to go high.
;

waitForSSP1IFHigh:

    ifdef debug       ; if debugging, don't wait for interrupt to be set high as the MSSP is not
    return            ; simulated by the IDE
    endif

    banksel PIR1

wfsh1:
    btfss   PIR1, SSP1IF
    goto    wfsh1

    return

; end of waitForSSP1IFHigh
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; waitForSSP1IFHighThenClearIt
;
; Waits in a loop for SSP1IF bit in register PIR1 to go high and then clears that bit.
;
; The bit must be cleared at some point after it is set before performing most actions with the I2C
; but. In most cases, it can be cleared immediately for which this function is useful.
;
; If the bit is not cleared before an operation, then checking the bit immediately after the
; operation will make it appear that the operation completed immediately and the code will not
; wait until the MSSP module sets the bit after actual completion.
;

waitForSSP1IFHighThenClearIt:

    call    waitForSSP1IFHigh

    call    clearSSP1IF

    return

; end of waitForSSP1IFHighThenClearIt
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; sendI2CByte
;
; Waits until the SSP1IF bit in register PIR1 goes high and then transmits the byte in the W
; register on the I2C bus.
;

sendI2CByte:

    ; wait for SSP1IF to go high

    call    waitForSSP1IFHigh

    ; put byte in transmit buffer

    banksel SSP1BUF
    movwf   SSP1BUF

    ; clear interrupt flag

    call    clearSSP1IF

    return

; end of sendI2CByte
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; setupI2CMaster7BitMode
;
; Sets the MASTER SYNCHRONOUS SERIAL PORT (MSSP) MODULE to the I2C Master mode using the 7 bit
; address mode.
;

setupI2CMaster7BitMode:

    banksel TRISB
    bsf TRISB, TRISB4       ; set RB4/I2CSDA to input
    bsf TRISB, TRISB6       ; set RB6/I2CSCL to input

    ; set baud rate
    ; for Fosc at 16 Mhz, use 0x27 ~ for Fosc at 32 Mhz, use 0x4f

    movlw   0x27			; set baud rate at 100kHz
    banksel SSP1ADD
    movwf   SSP1ADD

    banksel SSP1CON1
    bcf	SSP1CON1,SSP1M0		; SSPM = b1000 ~ I2C Master mode, clock = FOSC / (4 * (SSPADD+1))(4)
    bcf	SSP1CON1,SSP1M1
    bcf	SSP1CON1,SSP1M2
    bsf	SSP1CON1,SSP1M3

    bsf	SSP1CON1,SSPEN		;enables the MSSP module

    return

; end setupI2CMaster7BitMode
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; reallyBigDelay
;
; Delays for a second or two.
;

reallyBigDelay:

    banksel scratch6

    movlw   .50
    movwf   scratch6

rbd1:

    movlw   .255
    movwf   scratch7

rbd2:

    movlw   .255
    movwf   scratch8

rbd3:

    decfsz  scratch8,F
    goto    rbd3

    decfsz  scratch7,F
    goto    rbd2

    decfsz  scratch6,F
    goto    rbd1

    return

; end of reallyBigDelay
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; debugFunc1
;
; Performs functions for debug testing such as stuffing values into variables, etc.
;

debugFunc1:


    return

; end of debugFunc1
;--------------------------------------------------------------------------------------------------

    END
