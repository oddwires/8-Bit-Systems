;  MonZ80.ASM - Z80 Debug monitor for use with NoICEZ80
;
;  Copyright (c) 2005 by John Hartman
;
;  Modification History:
;		17-Oct-95 JLH port MONZ80.S to PseudoSam assembler, straight Z80 target
;		21-Jul-00 JLH change FN_MIN from F7 to F0
;		12-Mar-01 JLH V3.0: improve text about paging, formerly called "mapping"
;		27-Mar-02 JLH Replace bad equates FN_READ_REG and FN_WRITE_REG
;		11-Jan-05 JLH Correct (commented out) bug in Z180 reset/illegal op-code trap
;		09-Jul-25 TP  1) Various format changes to work with Telemark Assembler (TASM) Ver 3.2
;					  2) Replaced GETCHAR and PUTCHAR routines to work with 6850 UART
;					  3) Modified RESET routine to inintialise target hardware at startup
;					  4) Modified Memory Map to match target hardware
;					  5) Paged memory support HAS NOT been implemented
;  				  The registered distribution disk for the  Telemark Assembler (TASM) Ver 3.2
;				  can be found at https://github.com/spotco/TI-asm/tree/master/tasm
;		09-Jul-25 TP  Cosmetic changes to code layout + set unused EEPROM bytes to zero
;
;============================================================================
;
;  The original monitor used the Zilog ASM800 assembler, and used
;  conditional assembly to support various target processors.
;  PseudoSam also has no conditional assembly, so this version has been
;  stripped to just the basic Z80 code.
;
;  If you are using PseudoSam with a Z180 or Z84C15 processor, you should
;  look at the Z180 and Z84C15 specific code (initialization etc.) in
;  MONZ80.S, and port that which you require to this file.
;============================================================================
;
;  To customize for a given target, you must change code in the
;  hardware equates, the string TSTG, and the routines RESET and REWDT.
;  You may or may not need to change GETCHAR, PUTCHAR, depending on
;  how peculiar your UART is.
;
;  For more information, refer to the NoICE help file monitor.htm
;
;  To add banked or paged memory support:
;  1) Define page latch port PAGELATCH here
;  2) If PAGELATCH is write only, define or import the latch port's RAM
;	  image PAGEIMAGE here (The application code must update PAGEIMAGE
;	  before outputing to PAGELATCH)
;  3) Search for and modify PAGELATCH, PAGEIMAGE, and REG_PAGE usage below
;  4) In TSTG below edit "LOW AND HIGH LIMIT OF PAGED MEM"
;	  to appropriate range (typically 4000H to 07FFFH for two-bit MMU)
;
;  For more information, refer to the NoICE help file 2bitmmu.htm
;
;============================================================================
;
;  Hardware definitions
ROM_START   	.equ	00000h				; Start of monitor code
RAM_START		.equ	01f4eh				; Start of monitor ram
USER_CODE		.equ	08000h				; Start of user's ints/code
;
; Equates for I/O mapped 6850 serial port and DRAM page control register...
COM0_CR			.equ	070h                ; Note: Hardcoded to use COM0
COM0_DR			.equ	071h
COM1_CR			.equ	072h				; Com1 is initialised at startup, but not used.
COM1_DR			.equ	073h
DRAM_CR     	.equ    0ffh                ; 00h = Page 0, 01h = Page 1
;
;  Define monitor serial port
RXRDY			.equ	00h			        ; Bit number (not mask) for RX Buffer full
TXRDY			.equ	01h			        ; Bit number (not mask) for tx buffer empty
;
; define other hardware I/O ports...
D_PORT0			.equ	000h
D_PORT1			.equ	001h
D_PORT2			.equ	002h
D_PORT3			.equ	003h
CON_REG			.equ	004h
PAGELATCH		.equ	0ffh				; 00h = Page 0, 01h = Page 1
;
;============================================================================
;  RAM definitions:	 top 1K (or less)
		.org    RAM_START					; Monitor RAM
;
;  Initial user stack
;  (Size and location is user option)
				.block  64
INITSTACK:
;
;  Monitor stack
;  (Calculated use is at most 6 bytes.	Leave plenty of spare)
				.block  16
MONSTACK:
;
;  Target registers:  order must match that in TRGZ80.C
TASK_REGS:
REG_STATE:  	.block  1
REG_PAGE:		.block  1
REG_SP:			.block  2
REG_IX:			.block  2
REG_IY:			.block  2
REG_HL:			.block  2
REG_BC:			.block  2
REG_DE:			.block  2
REG_AF:										; Label on FLAGS, A as a WORD
REG_FLAGS:		.block  1
REG_A:			.block  1
REG_PC:			.block  2
REG_I:			.block  1
REG_IFF:		.block  1
 ;
REG_HLX:		.block  2					; Alternate register set
REG_BCX:		.block  2
REG_DEX:		.block  2
REG_AFX:									; Label on FLAGS, A as a WORD
REG_FLGX:		.block  1
REG_AX:			.block  1
T_REGS_SIZE		.equ    $-TASK_REGS
; !!! Caution:	don't put parenthesis around the above in ASM180:
; !!! The parenthesis in (*-TASK_REGS) are "remembered", such that
; !!! LD BC,T_REGS_SIZE is the same as LD BC,(T_REGS_SIZE)
; !!! It is OK to use parenthesis around the difference if the difference
; !!! is to be divided - just not around the entire expression!!!!!
;
;  Communications buffer
;  (Must be at least as long as TASK_REG_SIZE.	Larger values may improve
;  speed of NoICE memory load and dump commands)
COMBUF_SIZE     .equ    67				    ; Data size for COMM buffer
COMBUF:			.block  2+COMBUF_SIZE+1		; Buffer also has fn, len, and check
;
RAM_END         .equ	$		            ; Address of top+1 of ram
;
;===========================================================================
;  8080 mode Interrupt vectors
;
;  Reset, RST 0,  or trap vector
				.org	0
R0:				di
				jp		RESET
				nop
				nop
				nop
				nop
;
;  Interrupt RST 08.  Used for breakpoint.	Any other RST
;  may be used instead by changing the code below and the value of the
;  breakpoint instruction in the status string TSTG.  If RST NN cannot
;  be used, then CALL may be used instead.	However, this will restrict
;  the placement of breakpoints, since CALL is a three byte instruciton.
				push	af
				ld		a,1					; State = 1 (breakpoint)
				jp		INT_ENTRY
				nop
				nop
;
;  Interrupt RST 10
				jp		USER_CODE + 010h
				nop
				nop
				nop
				nop
				nop
;
;  Interrupt RST 18
				jp		USER_CODE + 018h
				nop
				nop
				nop
				nop
				nop
;
;  Interrupt RST 20
				jp		USER_CODE + 020h
				nop
				nop
				nop
				nop
				nop
;
;  Interrupt RST 28
				jp		USER_CODE + 028h
				nop
				nop
				nop
				nop
				nop
;
;  Interrupt RST 30
				jp		USER_CODE + 030h
				nop
				nop
				nop
				nop
				nop
;
;  Interrupt RST 38
				jp		USER_CODE + 038h
				nop
				nop
				nop
				nop
				nop
;
;===========================================================================
;
;  Non-maskable interrupt:	bash button
;  PC is stacked, interrupts disabled, and IFF2 has pre-NMI interrupt state
;
;  At the user's option, this may vector thru user RAM at USER_CODE+66H,
;  or enter the monitor directly.  This will depend on whether or not
;  the user wishes to use NMI in the application, or to use it with
;  a push button to break into running code.
				.fill	(066h-$),0h         ; Fill remaining bytes to NMI vector
NMI_ENTRY:		push	af
				ld		a,2
				jp		INT_ENTRY
;
;  Or, if user wants control of NMI:
;;;				jp      USER_CODE + H'66	; Jump thru vector in RAM
;;	(and enable NMI handler in DUMMY_INTS below)
;
;===========================================================================
;
;  Dummy handlers for RST and NMI.	This code is moved to the beginning
;  of USER_RAM, where the RST and NMI interrupts jump to it.  The code
;  then enters the monitor, specifying a STATE value which identifies the
;  interrupt which occurred.  This facilitates identification of
;  unexpected interrupts.  If the user desires, s/he may overwrite the
;  beginning of USER_RAM with appropriate handler code.	 Spacing of
;  this code is designed such that the user may re-ORG to 0 to run
;  the code from ROM.
DUMMY_INTS:
;
;  RST 0
				push	af
				ld		a,0					; State = 0 (interrupt 0)
				jp		INT_ENTRY
				nop
				nop
;
;  RST 8
				push	af
				ld		a,3					; State = 3 (interrupt 8)
				jp		INT_ENTRY
				nop
				nop
;
;  RST 10h
				push	af
				ld		a,4					; State = 4 (interrupt 10)
				jp		INT_ENTRY
				nop
				nop
;
;  RST 18h
				push	af
				ld		a,5					; State = 5 (interrupt 18)
				jp		INT_ENTRY
				nop
				nop
;
;  RST 20h
				push	af
				ld		a,6					; State = 6 (interrupt 20)
				jp		INT_ENTRY
				nop
				nop
;
;  RST 28h
				push	af
				ld		a,7					; State = 7 (interrupt 28)
				jp		INT_ENTRY
				nop
				nop
;
;  RST 30h
				push	af
				ld		a,8					; State = 8 (interrupt 30)
				jp		INT_ENTRY
				nop
				nop
;
;  RST 38h
				push	af
				ld		a,1					; State = 1 (breakpoint)
				ld		a,9					; State = 9 (interrupt 38)
				jp		INT_ENTRY
;
;  Use this if NMI is to be vectored through RAM.  Else comment it out
;;;				.org	DUMMY_INTS+H'66
;;;				push	af
;;;				ld		a,2
;;;				jp		INT_ENTRY
;
DUMMY_SIZE      .equ    *-DUMMY_INTS

;===========================================================================
;  Power on reset or trap
RESET:
;
;----------------------------------------------------------------------------
;;	if Z180, enable this code to tell illegal op-code from a reset
;;
;;;	 See if this is an illegal op-code trap or a reset
;;				ld	  	(REG_SP),sp			; Save user's stack pointer (or zero after reset)
;;				ld	  	sp,MONSTACK			; And get a guaranteed stack
;;				push	af					; Save a and flags
;;				in0	  	a,(itc)				; Check trap status (flags destroyed!!)
;;				bit	  	7,a					; If this bit is one, there was trap!
;;				jr	  	z,INIT				; JIF reset
;;;
;;;	 Illegal instruction trap:
;;;	 Back up the stacked PC by either 1 or 2 bytes, depending on the state
;;;	 of the UFO bit in the itc
;;				ld		(REG_HL),hl
;;				pop		hl					; Get stacked af
;;				ld		(REG_AF),hl			; Save AF
;;				ld		sp,(REG_SP)			; Restore SP after trap
;;				pop		hl					; Get stacked pc
;;				dec		hl					; Back up one byte
;;				bit		6,a
;;				jr		z,TR20			    ; JIF 1 byte op-code
;;              dec	hl						; ELSE back up second op-code byte
;;;
;;;	 Reset the trap bit
;;TR20:	        and		H'7F				; Clear the trap bit
;;              out0	(itc),a
;;;
;;;	 Get IFF2
;;;	 It is not clear that we can determine the pre-trap state of the
;;;	 interrupt enable:	the databook says nothing about IFF2 vis a vis
;;;	 trap.	We presume that interrupts are disabled by the trap.
;;;	 However, we proceed as if IFF2 contained the pre-trap state
;;				ld      a,i					; Get p flag = IFF2 (side effect)
;;				di							; Be sure ints are disabled
;;				ld		(REG_I),a			; Save int reg
;;				ld		a,0
;;				jp		po,tr30			    ; JIF parity odd (flag=0)
;;				inc		a					; ELSE set a = flag = 1 (enabled)
;;tr30:         ld		(REG_IFF),a			; Save interrupt flag
;;;
;;;	 Save registers in reg block for return to master
;;              ld	  	a,10
;;              ld	  	(REG_STATE),a		; Set state to "TRAP"
;;              jp	  	ENTER_MON			; HL = offending PC
;;
;-------------------------------------------------------------------------
;  Initialize monitor
INIT:	        ld		sp,MONSTACK
;
;; Enable the following code it you have an initialization table
;; at INIOUT.  See MONZ80.S
;;
;  Initialize target hardware
;;				ld		hl,INIOUT			; Put adress of initialization data table into hl
;;				ld		d,OUTCNT			; Put number of data and addr. pairs into reg. b
;
;  Caution:	OUT and OUTI place the 8 bit address from C on A7-A0, but
;  the contents of the B register on A15-A7.  The Z180's on-chip peripherals
;  decode 16 bits of I/O address, for reasons known only to ZIlog.
;  Thus, either be sure B=0 or use the Z180 OTIM
;  We do the former, so as to operate the same code on Z80, Z84C15 or Z80
;;				ld		b,0					; So a15-a8 will be 0
;;rst10:        ld		c,(hl)				; Load address from table
;;				inc		hl
;;				ld		a,(hl)				; Load data from table
;;				inc		hl
;;				out		(c),a				; Output a to i/o address (A15-A8 = 0)
;;				dec		d
;;				jr		nz,rst10			; Loop for d (address, data) pairs
;
;===========================================================================
;  Perform user hardware initilaization here

;===========================================================================
;
				sub		a
				out		(DRAM_CR),a			; Select DRAM page 0
				ld		a,03fh				; Interrupts are disabled...
				out		(D_PORT0),a			; ...so set all LED's off to protect the driver circuits...
				out		(D_PORT1),a
				out		(D_PORT2),a
				out		(D_PORT3),a
				ld		a,0ffh
				out		(CON_REG),a			; Shut the beeper off...
				ld		a,03h				; ACIA master reset
				out		(COM0_CR),a			; Initialise both serial ports
				out		(COM1_CR),a
				ld		a,055h				; Crystal frequency = 2.4576 MHz
											; 74HCT4060 divides by 16 => 153,600 Hz clock signal
											; 6850 UART Control Register = 055h
				out		(COM0_CR),a			;       Bit 0 & 1:	 Divide by 16 => 9,600 Hz Baud
				out		(COM1_CR),a			;       Bit 2,3,& 4:	 8 Bits + 1 Stop bit
											;       Bit 5 & 6:	 RTS = low, Transmitting Interrupt Enabled
											;       Bit 7:		 Receive interrupt disabled
;
;===========================================================================
;  Initialize user interrupt vectors to point to monitor
				ld		hl,DUMMY_INTS		; Dummy handler code
				ld		de,USER_CODE		; Start of user codespace
				ld		bc,DUMMY_SIZE		; Number of bytes
				ldir						; Copy code

;===========================================================================
;
;  Initialize user registers
				ld		hl,INITSTACK
				ld		(REG_SP),hl			; Init user's stack pointer
				ld		hl,0
				ld		a,l
				ld		(REG_PC),hl			; Init all regs to 0
				ld		(REG_HL),hl
				ld		(REG_BC),hl
				ld		(REG_DE),hl
				ld		(REG_IX),hl
				ld		(REG_IY),hl
				ld		(REG_AF),hl
				ld		(REG_HLX),hl
				ld		(REG_BCX),hl
				ld		(REG_DEX),hl
				ld		(REG_AFX),hl
				ld		(REG_I),a
				ld		(REG_STATE),a		; Set state as "RESET"
				ld		(REG_IFF),a			; No interrupts
;
;  Initialize memory paging variables and hardware (if any)
				ld		(REG_PAGE),a		; Page 0
;;;				ld		(PAGEIMAGE),a
;;;				out		(PAGELATCH),a		; Set hardware page
;
;  Set function code for "GO".	Then if we reset after being told to
;  GO, we will come back with registers so user can see the crash
				ld		a,FN_RUN_TARGET
				ld		(COMBUF),a
				jp		RETURN_REGS			; Dump regs, enter monitor
;
;===========================================================================
;  Get a character to A
;
;  Return A=char, CY=0 if data received
;		  CY=1 if timeout (0.5 seconds)
;
;  Uses 6 bytes of stack including return address
;
GETCHAR:		push	bc
				push	de
;
				ld		de,08000h			; Long timeout
				ld		a,COM0_CR			; Get selected 6850 Control reg for loop
				ld		c,a					; Control reg in C
gc10:	        dec		de
				ld		a,d
				or		e
				jr		z,gc90				; Exit if timeout
				in		a,(c)				; Read device status
				bit		RXRDY,a				; Bit 0
				jr		z,gc10				; Not ready yet.
;
;  Data received:  return CY=0. Data in A
				xor		a					; CY=0
				inc		c					; Selected 6850 Data reg in C
				in		a,(c)				; Read data
				pop		de
				pop		bc
				ret
;
;  Timeout:	 return CY=1
gc90:	        scf							; CY=1
				pop		de
				pop		bc
				ret
;
;===========================================================================
;  Output character in A
;
;  Uses 6 bytes of stack including return address
;
PUTCHAR:		push    bc					; Save:	 used for I/O address
				push	af					; Save byte to output
				ld		a,0ffh				; Bit of a pause
pc02:	        dec		a
				jr		nz,pc02
				ld		a,COM0_CR			; Get selected 6850 Control reg
				ld		c,a					; Selected ACIA Control reg in C
pc10:	        in		a,(c)				; Read device status
				bit		TXRDY,a				; Tx ready ?
				jr		nz,pc10
;
				inc		c					; Selected 6850 Data reg in C
				pop		af
				out		(c),a				; Transmit char
;
				pop		bc
				ret
;
;===========================================================================
;  Response string for GET TARGET STATUS request
;  Reply describes target:
TSTG:			.db		0					;   2: Processor type = Z80
				.db		COMBUF_SIZE			;   3: Size of communications buffer
				.db		0					;   4: No options
				.dw		08000h				; 5,6: Bottom of paged mem (none)
				.dw		0ffffh				; 7,8: Top of paged mem (none)
				.db		b1-b0				;   9: Breakpoint instruction length
b0:				rst		08h					;  10+ Breakpoint instruction
b1:				.db		"NoICE Z80 monitor v3.0",0
TSTG_SIZE		.equ	$-TSTG				; Size of NULL terminated string
;
;===========================================================================
;  HARDWARE PLATFORM INDEPENDENT EQUATES AND CODE
;
;  Communications function codes.
FN_GET_STATUS	.equ	0ffh				; Reply with device info
FN_READ_MEM		.equ	0feh				; Reply with data
FN_WRITE_MEM	.equ	0fdh				; Reply with status (+/-)
FN_RD_REGS		.equ	0fch				; Reply with registers
FN_WR_REGS		.equ	0fbh				; Reply with status
FN_RUN_TARGET	.equ	0fah				; Reply (delayed) with registers
FN_SET_BYTES	.equ	0f9h				; Reply with data (truncate if error)
FN_IN			.equ	0f8h				; Input from port
FN_OUT			.equ	0f7h				; Output to port
;
FN_MIN			.equ	0f0h				; Minimum recognized function code
FN_ERROR		.equ	0f0h				; Error reply to unknown op-code
;
;===========================================================================
;  Enter here via RST nn for breakpoint:  AF, PC are stacked.
;  Enter with A=interrupt code = processor state
;  Interrupt status is not changed from user program and IFF2==IFF1
INT_ENTRY:
;
;  Interrupts may be on:  get IFF as quickly as possible, so we can di
				ld		(REG_STATE),a		; Save entry state
				ld		(REG_HL),hl			; Save HL
				ld		a,i					; Get P FLAG = IFF2 (side effect)
				di							; No interrupts allowed
;
				ld		(REG_I),a			; Save int reg
				ld		a,0
				jp		po,BREAK10			; JIF Parity odd (FLAG=0)
				inc		a					; ELSE Set a = FLAG = 1 (enabled)
BREAK10:        ld      (REG_IFF),a			; Save interrupt FLAG
;
;  Save registers in reg block for return to master
				pop		hl					; Get FLAGS in L, accum in H
				ld		(REG_AF),hl			; Save A and FLAGS
;
;  If entry here was by breakpoint (state=1), then back up the program
;  counter to point at the breakpoint/RST instruction.	Else leave PC alone.
;  (If CALL is used for breakpoint, then back up by 3 bytes)
				pop		hl					; Get PC of breakpoint/interrupt
				ld		a,(REG_STATE)
				cp		1
				jr		nz,NOTBP			; JIF not a breakpoint
				dec		hl					; Back up PC to point at breakpoint
NOTBP:	        jp		ENTER_MON			; HL points at breakpoint opcode
;
;===========================================================================
;  Main loop:  wait for command frame from master
MAIN:	        ld		sp,MONSTACK			; Clean stack is happy stack
				ld		hl,COMBUF			; Build message here
;
;  First byte is a function code
				call	GETCHAR				; Get a function (uses 6 bytes of stack)
				jr		c,MAIN				; JIF timeout: resync
				cp		FN_MIN
				jr		c,MAIN				; JIF below min: illegal function
				ld		(hl),a				; Save function code
				inc		hl
;
;  Second byte is data byte count (may be zero)
				call	GETCHAR				; Get a length byte
				jr		c,MAIN				; JIF timeout: resync
				cp		COMBUF_SIZE+1
				jr		nc,MAIN				; JIF too long: illegal length
				ld		(hl),a				; Save length
				inc		hl
				or		a
				jr		z,MA80				; Skip data loop if length = 0
;
;  Loop for data
				ld		b,a					; Save length for loop
MA10:	        call	GETCHAR				; Get a data byte
				jr		c,MAIN				; JIF timeout: resync
				ld		(hl),a				; Save data byte
				inc		hl
				djnz	MA10
;
;  Get the checksum
MA80:	        call	GETCHAR				; Get the checksum
				jr		c,MAIN				; JIF timeout: resync
				ld		c,a					; Save checksum
;
;  Compare received checksum to that calculated on received buffer
;  (Sum should be 0)
				call	CHECKSUM
				add		a,c
				jr		nz,MAIN				; JIF bad checksum
;
;  Process the message.
				ld		a,(COMBUF+0)		; Get the function code
				cp		FN_GET_STATUS
				jp		z,TARGET_STATUS
				cp		FN_READ_MEM
				jp		z,READ_MEM
				cp		FN_WRITE_MEM
				jp		z,WRITE_MEM
				cp		FN_RD_REGS
				jp		z,READ_REGS
				cp		FN_WR_REGS
				jp		z,WRITE_REGS
				cp		FN_RUN_TARGET
				jp		z,RUN_TARGET
				cp		FN_SET_BYTES
				jp		z,SET_BYTES
				cp		FN_IN
				jp		z,IN_PORT
				cp		FN_OUT
				jp		z,OUT_PORT
;
;  Error: unknown function.	 Complain
				ld		a,FN_ERROR
				ld		(COMBUF+0),a		; Set function as "Error"
				ld		a,1
				jp		SEND_STATUS			; Value is "Error"

;===========================================================================
;
;  Target Status:  FN, len
;
TARGET_STATUS:  ld		hl,TSTG				; Data for reply
				ld		de,COMBUF+1			; Return buffer
				ld		bc,TSTG_SIZE		; Length of reply
				ld		a,c
				ld		(de),a				; Set size in reply buffer
				inc		de
				ldir						; Move reply data to buffer
;
;  Compute checksum on buffer, and send to master, then return
				jp		SEND

;===========================================================================
;
;  Read Memory:	 FN, len, page, Alo, Ahi, Nbytes
;
READ_MEM:
;
;  Set page
;;				ld		a,(COMBUF+2)
;;				ld		(PAGEIMAGE),a
;;				ld		bc,PAGELATCH
;;				out		(bc),a
;
;  Get address
				ld		hl,(COMBUF+3)
				ld		a,(COMBUF+5)		; Number of bytes to get
;
;  Prepare return buffer: FN (unchanged), LEN, DATA
				ld		de,COMBUF+1			; Pointer to LEN, DATA
				ld		(de),a				; Return length = Requested data
				inc		de
				or		a
				jr		z,GLP90				; JIF no bytes to get
;
;  Read the requested bytes from local memory
				ld		b,a
GLP:	        ld		a,(hl)				; Get byte to A
				ld		(de),a				; Store to return buffer
				inc		hl
				inc		de
				djnz	GLP
;
;  Compute checksum on buffer, and send to master, then return
GLP90:	        jp		SEND

;===========================================================================
;
;  Write Memory:  FN, len, page, Alo, Ahi, (len-3 bytes of Data)
;
;  Uses 2 bytes of stack
;
WRITE_MEM:
;
;  Set page
;;				ld		a,(COMBUF+2)
;;				ld		(PAGEIMAGE),a
;;				ld		bc,PAGELATCH
;;				out		(bc),a
;
				ld		hl,COMBUF+5			; Pointer to source data in message
				ld		de,(COMBUF+3)		; Pointer to destination
				ld		a,(COMBUF+1)		; Number of bytes in message
				sub		3					; Less page, ADDRLO, ADDRHI
				jr		z,WLP50				; Exit if none requested
;
;  Write the specified bytes to local memory
				ld		b,a
				push	bc					; Save byte counter
WLP10:	        ld		a,(hl)				; Byte from host
				ld		(de),a				; Write to target ram
				inc		hl
				inc		de
				djnz	WLP10
;
;  Compare to see if the write worked
				ld		hl,COMBUF+5			; Pointer to source data in message
				ld		de,(COMBUF+3)		; Pointer to destination
				pop		bc					; Size again
;
;  Compare the specified bytes to local memory
WLP20:	        ld		a,(de)				; Read back what we wrote
				cp		(hl)				; Compare to host data
				jr		nz,WLP80			; JIF write failed
				inc		hl
				inc		de
				djnz	WLP20
;
;  Write succeeded:	 return status = 0
WLP50:	        xor		a					; Return status = 0
				jr		WLP90
;
;  Write failed:  return status = 1
WLP80:	        ld		a,1
;
;  Return OK status
WLP90:	        jp		SEND_STATUS

;===========================================================================
;
;  Read registers:	FN, len=0
;
READ_REGS:
;
;  Enter here from int after "RUN" and "STEP" to return task registers
RETURN_REGS:	ld		hl,TASK_REGS		; Register live here
				ld		a,T_REGS_SIZE		; Number of bytes
;
;  Prepare return buffer: FN (unchanged), LEN, DATA
				ld		de,COMBUF+1			; Pointer to LEN, DATA
				ld		(de),a				; Save data length
				inc		de
;
;  Copy the registers
				ld		b,a
GRLP:	        ld		a,(hl)				; Get byte to a
				ld		(de),a				; Store to return buffer
				inc		hl
				inc		de
				djnz	GRLP
;
;  Compute checksum on buffer, and send to master, then return
				jp		SEND

;===========================================================================
;
;  Write registers:	 FN, len, (register image)
;
WRITE_REGS:     ld		hl,COMBUF+2			; Pointer to data
				ld		a,(COMBUF+1)		; Number of bytes
				or		a
				jr		z,WRR80				; JIF no registers
;
;  Copy the registers
				ld		de,TASK_REGS		; Our registers live here
				ld		b,a
WRRLP:	        ld		a,(hl)				; Get byte to a
				ld		(de),a				; Store to register ram
				inc		hl
				inc		de
				djnz	WRRLP
;
;  Return OK status
WRR80:	        xor		a
				jp		SEND_STATUS

;===========================================================================
;
;  Run Target:	FN, len
;
;  Uses 4 bytes of stack
;
RUN_TARGET:
;
;  Restore user's page
;;				ld		a,(REG_PAGE)
;;				ld		(PAGEIMAGE),a
;;				ld		bc,PAGELATCH
;;				out		(bc),a
;
;  Restore alternate registers
				ld		hl,(REG_AFX)
				push	hl
				pop		af
				ex		af,af'				; 'Load alternate AF
		;
				ld		hl,(REG_HLX)
				ld		bc,(REG_BCX)
				ld		de,(REG_DEX)
				exx							; Load alternate regs
;
;  Restore main registers
				ld		bc,(REG_BC)
				ld		de,(REG_DE)
				ld		ix,(REG_IX)
				ld		iy,(REG_IY)
				ld		a,(REG_I)
				ld		i,a
;
;  Switch to user stack
				ld		hl,(REG_PC)			; User PC
				ld		sp,(REG_SP)			; Back to user stack
				push	hl					; Save user PC for RET
				ld		hl,(REG_AF)
				push	hl					; Save user A and FLAGS for POP
				ld		hl,(REG_HL)			; User HL
;
;  Restore user's interrupt state
				ld		a,(REG_IFF)
				or		a
				jr		z,RUTT10			; JIF ints off: leave off
;
;  Return to user with interrupts enabled
				pop		af
				ei							; ELSE enable them now
				ret
;
;  Return to user with interrupts disabled
RUTT10:         pop		af
				ret
;
;===========================================================================
;
;  Common continue point for all monitor entrances
;  HL = user PC, SP = user stack
;  REG_STATE has current state, REG_HL, REG_I, REG_IFF, REG_AF set
;
;  Uses 2 bytes of stack
;
ENTER_MON:		ld		(REG_SP),sp			; Save user's stack pointer
				ld		sp,MONSTACK			; And use ours instead
;
				ld		(REG_PC),hl
				ld		(REG_BC),bc
				ld		(REG_DE),de
				ld		(REG_IX),ix
				ld		(REG_IY),iy
;
;  Get alternate register set
				exx
				ld		(REG_HLX),hl
				ld		(REG_BCX),bc
				ld		(REG_DEX),de
				ex		af,af'              ; 'Load alternate AF
				push	af
				pop		hl
				ld		(REG_AFX),hl
;
;;				ld		a,(PAGEIMAGE)		; Get current user page
				xor		a					; ...or none if unpaged target
				ld		(REG_PAGE),a		; Save user page
;
;  Return registers to master
				jp		RETURN_REGS

;===========================================================================
;
;  Set target byte(s):	FN, len { (page, alow, ahigh, data), (...)... }
;
;  Return has FN, len, (data from memory locations)
;
;  If error in insert (memory not writable), abort to return short data
;
;  This function is used primarily to set and clear breakpoints
;
;  Uses 2 bytes of stack
;
SET_BYTES:		ld		hl,COMBUF+1
				ld		b,(hl)				; Length = 4*nbytes
				inc		hl
				inc		b
				dec		b
				ld		c,0					; C gets count of inserted bytes
				jr		z,SB90				; JIF no bytes (C=0)
				push	hl
				pop	ix						; IX points to return buffer
;
;  Loop on inserting bytes
SB10:	        ld		a,(hl)				; Memory page
				inc		hl
;;				ld		(PAGEIMAGE),a
;;				push	bc
;;				ld		bc,PAGELATCH
;;				out		(bc),a				; Set page
;;				pop		bc
				ld		e,(hl)				; Address to de
				inc		hl
				ld		d,(hl)
				inc		hl
;
;  Read current data at byte location
				ld		a,(de)			    ; Read current data
				ld		(ix+0),a		    ; Save in return buffer
				inc		ix
;
;  Insert new data at byte location
				ld		a,(hl)
				ld		(de),a				; Set byte
				ld		a,(de)				; Read it back
				cp		(hl)				; Compare to desired value
				jr		nz,SB90				; BR IF insert failed: ABORT
				inc		hl
				inc		c					; Else count one byte to return
;
				dec		b
				dec		b
				dec		b
				djnz	SB10				; Loop for all bytes
;
;  Return buffer with data from byte locations
SB90:	        ld		a,c
				ld		(COMBUF+1),a		; Set count of return bytes
;
;  Compute checksum on buffer, and send to master, then return
				jp		SEND

;===========================================================================
;
;  Input from port:	 FN, len, PortAddressLo, PAhi (=0)
;
IN_PORT:
;
;  Get port address
				ld		bc,(COMBUF+2)
;
;  Read port value
				in		a,(c)				; IN with A15-A8 = B; A7-A0 = C
;
;  Return byte read as "status"
				jp		SEND_STATUS

;===========================================================================
;
;  Output to port:	FN, len, PortAddressLo, PAhi (=0), data
;
OUT_PORT:
;
;  Get port address
				ld		bc,(COMBUF+2)
;
;  Get data
				ld		a,(COMBUF+4)
;
;  Write value to port
				out		(c),a				; OUT with A15-A8 = B; A7-A0 = C
;
;  Return status of OK
				xor		a
				jp		SEND_STATUS
;
;===========================================================================
;  Build status return with value from "A"
;
SEND_STATUS:	ld		(COMBUF+2),a		; Set status
				ld		a,1
				ld		(COMBUF+1),a		; Set length
				jr		SEND

;===========================================================================
;  Append checksum to COMBUF and send to master
;
;  Uses 6 bytes of stack (not including return address: jumped, not called)
;
SEND:	        call	CHECKSUM			; Get A=CHECKSUM, HL->checksum location
				neg
				ld		(hl),a				; Store negative of checksum
;
;  Send buffer to master
				ld		hl,COMBUF			; Pointer to data
				ld		a,(COMBUF+1)		; Length of data
				add		a,3					; Plus FUNCTION, LENGTH, CHECKSUM
				ld		b,a					; Save count for loop
SND10:	        ld		a,(hl)
				call	PUTCHAR				; Send a byte (uses 6 bytes of stack)
				inc		hl
				djnz	SND10
				jp		MAIN				; Back to main loop

;===========================================================================
;  Compute checksum on COMBUF.	COMBUF+1 has length of data,
;  Also include function byte and length byte
;
;  Returns:
;		A = checksum
;		HL = pointer to next byte in buffer (checksum location)
;		B is scratched
;
;  Uses 2 bytes of stack including return address
;
CHECKSUM:		ld		hl,COMBUF			; Pointer to buffer
				ld		a,(COMBUF+1)		; Length of message
				add		a,2					; Plus function, length
				ld		b,a					; Save count for loop
				xor		a					; Init checksum to 0
CHK10:	        add		a,(hl)
				inc		hl
				djnz	CHK10				; Loop for all
				ret							; Return with checksum in A
;
;===========================================================================
;  Hardware initialization table
INIOUT:
OUTCNT          .equ    (*-INIOUT)/2		; Number of initializing pairs
;
; Fill remaining bytes to end of 2K block...
                .fill   (ROM_START+07ffh-$),0h

		.END	RESET
