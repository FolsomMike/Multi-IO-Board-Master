;--------------------------------------------------------------------------------------------------
; Project:  Multi-IO Board A Transverse Master PIC
; Date:     3/30/15
; Revision: See Revision History notes below.
;
; Overview:
;
; This program runs on the Master PIC on a Multi-IO Board Configuration A Transverse Ring 1 or
; Multi-IO Board Configuration A Transverse Ring 2. The transverse head is divided into two rings
; of shoes. One board handles Ring 1 and other handles Ring 2. The Rabbit software is slightly
; different for each ring, but all the Master & Slave PIC code is identical for either.
;
; The Master PIC communicates with a Rabbit RCM4200 via serial port and with 8 Slave PICs via the
; I2C bus. It also communicates with digital gain and offset potentiometers via that same I2C bus.
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
; Port A
;
; RA0   In  - ???
; RA1   In  - ???
; RA2   xxx - not implemented in PIC16f1459
; RA3   In  - ???
; RA4   In  - ???
; RA5   Out - ???
; RA6   xxx - not implemented in PIC16f1459
; RA7   xxx - not implemented in PIC16f1459
;
; Port B
;
; RB0   xxx - not implemented in PIC16f1459
; RB1   xxx - not implemented in PIC16f1459
; RB2   xxx - not implemented in PIC16f1459
; RB3   xxx - not implemented in PIC16f1459
; RB4   I/O - I2CSDA
; RB5   In  - ???
; RB6   Out - I2CSCL
; RB7   Out - ???
;
; Port C
;
; RC0   Out - ???
; RC1   In  - ???
; RC2   In  - ???
; RC3   Out - ???
; RC4   Out - ???
; RC5   In  - ???
; RC6   Out - ???
; RC7   In  - ???
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


; Rabbit Commands -- sent by Rabbit to trigger various actions

RABBIT_GET_STATUS               EQU 0x00
RABBIT_RESET_ENCODERS           EQU 0x01
RABBIT_GET_ENCODERS             EQU 0x02
RABBIT_GET_PEAK_DATA            EQU 0x03
RABBIT_SET_GAIN                 EQU 0x04
RABBIT_SET_OFFSET               EQU 0x05
RABBIT_RESET                    EQU 0xff

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
 __CONFIG _CONFIG1, _FOSC_INTOSC & _WDTE_OFF & _PWRTE_OFF & _MCLRE_OFF & _CP_OFF & _BOREN_OFF & _CLKOUTEN_ON & _IESO_OFF & _FCMEN_OFF
; CONFIG2
; __config 0xFFFF
 __CONFIG _CONFIG2, _WRT_ALL & _CPUDIV_NOCLKDIV & _USBLSCLK_48MHz & _PLLMULT_4x & _PLLEN_DISABLED & _STVREN_ON & _BORV_LO & _LPBOR_OFF & _LVP_OFF

; _FOSC_INTOSC -> internal oscillator, I/O function on OSC1/CLKIN pin
; _WDTE_OFF -> watch dog timer disabled
; _PWRTE_OFF -> Power Up Timer disabled
; _MCLRE_OFF -> MCLR/VPP pin is digital input
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
; NOTE: All ports for outputs are defined as the latches for the port. Writing to the latches
; avoids problems with the read-modify-write system of the PIC.
;
; For inputs, the port must be read.
;

; Port A

SERIAL_IN_P     EQU     PORTA
SERIAL_IN       EQU     RA0         ; input ~ RA0 can only be input on PIC16f1459
UNUSED_RA1_P    EQU     PORTA
UNUSED_RA1      EQU     RA1         ; input ~ RA1 can only be input on PIC16f1459
;NA_RA2         EQU     RA2         ; RA2 not implemented on PIC16f1459
SHORT_DETECT_P  EQU     PORTA
SHORT_DETECT    EQU     RA3         ; input ~ RA3 can only be input on PIC16f1459
HI_LIMIT_P      EQU     PORTA
HI_LIMIT        EQU     RA4         ; input ~ cutting current hight limit
POWER_ON_P      EQU     PORTA
POWER_ON        EQU     RA5         ; output
;NA_RA6         EQU     RA6         ; RA6 not implemented on PIC16f1459
;NA_RA7         EQU     RA7         ; RA7 not implemented on PIC16f1459

; Port B

I2CSDA_LINE     EQU     RB4
JOG_DWN_SW_P    EQU     PORTB
JOG_DWN_SW      EQU     RB5         ; input
I2CSCL_LINE     EQU     RB6
SERIAL_OUT_P    EQU     PORTB
SERIAL_OUT      EQU     RB7         ; output ~ serial data out to other devices

; Port C

MOTOR_ENABLE_P  EQU     PORTC
MOTOR_ENABLE    EQU     RC0         ; output
MODE_SW_P       EQU     PORTC
MODE_SW         EQU     RC1         ; input
JOG_UP_SW_P     EQU     PORTC
JOG_UP_SW       EQU     RC2         ; input
MOTOR_DIR_P     EQU     PORTC
MOTOR_DIR       EQU     RC3         ; output
MOTOR_STEP_P    EQU     PORTC
MOTOR_STEP      EQU     RC4         ; output
LO_LIMIT_P      EQU     PORTC
LO_LIMIT		EQU     RC5         ; input ~ cutting current low limit
MOTOR_MODE_P    EQU     PORTC
MOTOR_MODE      EQU     RC6         ; output ~ motor step size selection
SELECT_SW_P     EQU     PORTC
SELECT_SW       EQU     RC7         ; input ~ select switch

; I2C bus ID byte for writing to digital pot 1
; upper nibble = 1010 (bits 7-4)
; chip A2-A0 inputs = 001 (bits 3-1)
; R/W bit set to 0 (bit 0)

DIGITAL_POT1_WRITE_ID       EQU     b'10100010'

POT1_ADDR           EQU     0x0
POT2_ADDR           EQU     0x1
POT3_ADDR           EQU     0x2
POT4_ADDR           EQU     0x3

; I2C bus ID bytes for writing and reading to the LED PIC
; upper nibble = 1010 (bits 7-4)
; chip A2-A0 inputs = 010 (bits 3-1)
; R/W bit set to 0 (bit 0) for writing
; R/W bit set to 1 (bit 0) for reading

SLAVE_PIC_WRITE_ID            EQU     b'10100100'
SLAVE_PIC_READ_ID             EQU     b'10100101'


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

;    call    handleSerialPortReceiveInt      ;debug mks -- remove this

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


;debug mks

    banksel flags2
    movf    serialRcvBuf, W
    banksel TXREG
    movwf   TXREG

;debug mks end

    call    resetSerialPortReceiveBuffer

    return

; end of handleSerialPacket
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

;    call    setupPortA      ; prepare Port A for I/O

;    call    setupPortB      ; prepare Port B for I/O

;    call    setupPortC      ; prepare Port C  for I/O

;    call    initializeOutputs

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

    banksel LATB
    bsf     LATB,SERIAL_OUT

    banksel LATA
    bcf     LATA, POWER_ON

    banksel LATC
    bsf     LATC, MOTOR_STEP

    banksel LATC
    bsf     LATC, MOTOR_DIR

    banksel LATC              ; disable the motor
    bsf     LATC, MOTOR_ENABLE

    banksel LATC
	bcf     LATC, MOTOR_MODE    ; choose full step if J8-1 (MS1) = Off and J8-2 (MS2) = On
                                        ; see notes at top of page for more info

    return

; end of initializeOutpus
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; setupClock
;
; Saves the flags value to eeprom.
;
; Sets up the system clock source and frequency.
;
; Assumes clock related configuration bits are set as follows:
;
;   _FOSC_INTOSC,  _CPUDIV_NOCLKDIV, _PLLMULT_4x, _PLLEN_DISABLED
;
; Assumes all programmable clock related options are at Reset default values.
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

    banksel WPUA
    movlw   b'00000000'                 ; disable weak pull-ups
    movwf   WPUA

    banksel PORTA
    clrf    PORTA                       ; init port value

    banksel LATA                        ; init port data latch
    clrf    LATA

    banksel ANSELA
    clrf    ANSELA                      ; setup port for all digital I/O

    ; set I/O directions

    banksel TRISA
    movlw   b'11111111'                 ; first set all to inputs
    movwf   TRISA

    ; set direction for each pin used

    bsf     TRISA, SERIAL_IN            ; input
    bsf     TRISA, SHORT_DETECT         ; input
    bsf     TRISA, HI_LIMIT             ; input
    bcf     TRISA, POWER_ON             ; output

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

    banksel WPUB
    movlw   b'00000000'                 ; disable weak pull-ups
    movwf   WPUB

    banksel PORTB
    clrf    PORTB                       ; init port value

    banksel LATB                        ; init port data latch
    clrf    LATB

    bsf     LATB,SERIAL_OUT             ; initialize SERIAL_OUT high before changing pin to output
                                        ; so a start bit won't be transmitted
    banksel ANSELB
    clrf    ANSELB                      ; setup port for all digital I/O

    ; set I/O directions

    banksel TRISB
    movlw   b'11111111'                 ; first set all to inputs
    movwf   TRISB

    bsf     TRISB, JOG_DWN_SW           ; input
    bcf     TRISB, SERIAL_OUT           ; output

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

    ; Port C does not have a weak pull-up register

    banksel PORTC
    clrf    PORTC                       ; init port value

    banksel LATC                        ; init port data latch
    clrf    LATC

    banksel ANSELC
    clrf    ANSELC                      ; setup port for all digital I/O

    ; set I/O directions

    banksel TRISC
    movlw   b'11111111'                 ; first set all to inputs
    movwf   TRISC

    bcf     TRISC, MOTOR_ENABLE         ; output
    bsf     TRISC, MODE_SW              ; input
    bsf     TRISC, JOG_UP_SW            ; input
    bcf     TRISC, MOTOR_DIR            ; output
    bcf     TRISC, MOTOR_STEP           ; output
    bsf     TRISC, LO_LIMIT             ; input
    bcf     TRISC, MOTOR_MODE           ; output
    bsf     TRISC, SELECT_SW            ; input

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
; Sends a command and/or data to a Slave PIC via the I2C bus.
;

sendCommandToSlavePIC:

    banksel scratch0

    movlw   .3                      ; send command byte and two values
    movwf   scratch0
    ; debug mks load command here -- movlw   LEDPIC_SET_LEDS         ; put command byte in scratch1
    movwf   scratch1
    movlw   scratch1                ; point to first byte to be sent
    movwf   FSR0L

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
    call    setDigitalPotInChip1

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
; NOTE NOTE NOTE
; It is important to use no (or very few) subroutine calls.  The stack is only 16 deep and
; it is very bad for the interrupt routine to use it.
;
; The RCIF flag is cleared by reading all data from the two byte receive FIFO.
;
; This code check each byte sequence to see if it starts with a header prefix (0x55,0xaa) followed
; by a valid length byte. If these are found, the bytes after the length byte are stored in a
; buffer. If the sequence is not matched or the supposed length byte is larger than the buffer,
; all flags are reset and the search for the first header byte starts over.
;
; Packet format:
;   0x55, 0xaa, length, data1, data2, data3,...checksum.
;
; This interrupt function does not verify the checksum; the main loop should do that if required.
; Once a packet has been received, a flag is set to alert the main loop that it is ready for
; processing. All further data will be ignored until the main loop clears that flag. If an error
; occurs, the data received to that point will be discarded and the search for the next packet
; begun anew.
;
; The packet length byte is the number of data bytes plus one for the checksum byte. It does not
; include the two header bytes or the length byte itself.
;
; Thus, only one packet at a time can be handled. The processing required is typically minimal, so
; the main loop should be able to process each packet before another is received. Some care should
; be taken by the receiver to not flood the line with packets.
;
; The main loop does all the actual processing in order to minimize the overhead of the interrupt.
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

    sublw   0x55                            ; check for first header byte of 0x55
    btfsc   STATUS, Z                       ; equal?
    goto    rsllp                           ; continue on, leaving flag set

    goto    rslError                        ; not header byte 1, reset all to restart search

rsl1:
    btfsc   flags2, HEADER_BYTE_2_RCVD      ; header byte 2 already received?
    goto    rsl2                            ; if so, check for length byte

    bsf     flags2, HEADER_BYTE_2_RCVD      ; preset the flag, will be cleared on fail

    sublw   0xaa                            ; check for second header byte of 0xaa
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

rslError:

    incf    serialPortErrorCnt, F           ; track errors
    call    resetSerialPortReceiveBuffer    ; invalid length, reset all to restart search
    goto    rsllp

rsl3:

    movwf   serialIntScratch0               ; store the new character
    clrf    FSR0H                           ; load FSR0 with buffer pointer
    movf    serialRcvBufPtr, W
    movwf   FSR0L
    incf    serialRcvBufPtr, F              ; advance the buffer pointer

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
; This function is called when a byte has been received by the serial port.
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
; setDigitalPotInChip1
;
; Sets the pot specified by scratch0 in digital pot chip 1 to the value in scratch1.
;

setDigitalPotInChip1:

    call    clearSSP1IF             ; make sure flag is cleared before starting

    call    generateI2CStart

    movlw   DIGITAL_POT1_WRITE_ID   ; send proper ID to write to digital pot chip 1
    call    sendI2CByte             ; send byte in W register on I2C bus after SP1IF goes high

    banksel scratch0                ; send the address of the pot in the chip to access
    movf    scratch0,W
    call    sendI2CByte

    banksel scratch1                ; send the pot value
    movf    scratch1,W
    call    sendI2CByte

    call    waitForSSP1IFHighThenClearIt    ; wait for high flag upon transmission completion

    call    generateI2CStop

    call    waitForSSP1IFHighThenClearIt    ; wait for high flag upon stop condition finished

    return

; end of setDigitalPotInChip1
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; sendBytesToSlavePICViaI2C
;
; Sends byte to a Slave PIC via the I2C bus.
;
; The number of bytes to be written should be in scratch0.
; Indirect register FSR0 should point to first byte in RAM to be written.
;

sendBytesToSlavePICViaI2C:

    call    clearSSP1IF             ; make sure flag is cleared before starting

    call    generateI2CStart

    movlw   SLAVE_PIC_WRITE_ID      ; send proper ID to write to LED PIC
    call    sendI2CByte             ; send byte in W register on I2C bus after SSP1IF goes high

loopSBLP1:

    moviw   FSR0++                  ; load next byte to be sent
    call    sendI2CByte

    banksel scratch0
	decfsz	scratch0,F              ; count down number of bytes transferred
	goto	loopSBLP1               ; not zero yet - trasfer more bytes

    call    waitForSSP1IFHighThenClearIt    ; wait for high flag upon transmission completion

    call    generateI2CStop

    call    waitForSSP1IFHighThenClearIt    ; wait for high flag upon stop condition finished

    return

; end of sendBytesToSlavePICViaI2C
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; generateI2CStart
;
; Generates a start condition on the I2C bus.
;

generateI2CStart:

    banksel SSPCON2
    bsf     SSPCON2,SEN

    return

; end of generateI2CStart
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; generateI2CRestart
;
; Generates a restart condition on the I2C bus.
;

generateI2CRestart:

    banksel SSPCON2
    bsf     SSPCON2,RSEN

    return

; end of generateI2CRestart
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; generateI2CStop
;
; Generates a stop condition on the I2C bus.
;

generateI2CStop:

    banksel SSPCON2
    bsf     SSPCON2,PEN

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

    banksel SSPBUF
    movwf   SSPBUF

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
; NOTE: RB4 and RB6 must have been configured elswhere as inputs for this mode.
;

setupI2CMaster7BitMode:

    movlw   0x27			; set baud rate at 100kHz for oscillator frequency of 16 Mhz
    banksel SSPADD
    movwf   SSPADD

    banksel SSPCON1
    bcf	SSPCON1,SSP1M0		; SSPM = b1000 ~ I2C Master mode, clock = FOSC / (4 * (SSPADD+1))(4)
    bcf	SSPCON1,SSP1M1
    bcf	SSPCON1,SSP1M2
    bsf	SSPCON1,SSP1M3

    bsf	SSPCON1,SSPEN		;enables the MSSP module

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
