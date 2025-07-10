CODE     SEGMENT     PARA     PUBLIC     'CODE'

;         ******************************************
;         *                                        *
;         *   ASSEMBLER LISTING OF TOUCH SCREEN    *
;         *   DRIVER SOFTWARE FOR USE WITH THE     *
;         *   IBM  P.C.                            *
;         *                                        *
;         *   This is a true Character Device      *
;         *   Driver, installed by the CONFIG.SYS  *
;         *   file when the P.C. is booted.        *
;         *                                        *
;         ******************************************

;EQUATES

;Define Static Request Header :-

  SRH         EQU  0           ;Static Request Header Start
  SRH_LEN     EQU  13          ;Static Request Header Length
  SRH_LEN_FLD EQU  SRH         ;Static Request Header Length Field
  SRH_UCD_FLD EQU  SRH+1       ;Static Request Header Unit Code Field
  SRH_CCD_FLD EQU  SRH+2       ;Static Request Header Command Code Field
  SRH_STA_FLD EQU  SRH+3       ;Static Request Header Status Field
  SRH_RES_FLD EQU  SRH+5       ;Static Request Header Reserved Area Field

;Define Data Field for INIT :-

  INI         EQU  SRH+SRH_LEN ;Init Data Start
  INI_LEN     EQU  9           ;Init Data Length
  INI_UTS_FLD EQU  INI         ;Init Units Field
  INI_ADD_FLD EQU  INI+1       ;Init End Address Field
  INI_BPB_FLD EQU  INI+5       ;Init Pointer to BPB Field

;Define Data Field for Input/Output :-

  IO          EQU  SRH+SRH_LEN ;Input/Output Data Start
  IO_LEN      EQU  9           ;Input/Output Data Length
  IO_MED_FLD  EQU  IO          ;Input/Output Media Descriptor Field
  IO_ADD_FLD  EQU  IO+1        ;Input/Output Buffer Address Field
  IO_BYT_FLD  EQU  IO+5        ;Input/Output Buffer Length Field

  LF          EQU  0AH         ;Define 'Line Feed' character
  CR          EQU  0DH         ;Define 'Cariage Return' character
  EOF         EQU  1AH         ;Define 'End of File' character
  BELL        EQU  07H         ;Define 'BELL' character

TCH_SCR4  PROC      FAR

          ASSUME    CS:CODE,ES:CODE,DS:CODE

BEGIN:

;  Start of executable code.

START     EQU       $  

;         Device Header 
;         =============

          NEXT_DEV  DD   -1             ;Pointer to next device
          ATTRIBUTE DW   8000H          ;Device attributes:-
                                        ; Character Device
                                        ; No IOCTL
                                        ; No output until busy
                                        ; Not current clock
                                        ; Not standard output
                                        ; Not standard input
          STRATEGY  DW   DEV_STRATEGY   ;Pointer to Strategy input
          INTERRUPT DW   DEV_INT        ;Pointer to Imterrupt input
          DEV_NAME  DB   'TCH_SCR4'     ;Name of character device

;MESSAGES

          MESSAGE1 DB   13,10,'**- Strategy -**',13,10
                   DB   '    Header segment - $'
          MESSAGE2 DB   13,10,'    Header offset  - $'
          MESSAGE3 DB   13,10,'**- Interrupt -**',13,10
                   DB   '    Header segment - $'
          MESSAGE4 DB   13,10,'**- Request Header -**',13,10,'$' 
          MESSAGE5 DB   13,10,'Length       - $'
          MESSAGE6 DB   13,10,'Unit code    - $'
          MESSAGE7 DB   13,10,'Command code - $'
          MESSAGE8 DB   13,10,'Status       - $'
          MESSAGE9 DB   13,10,'( DOS )      - $'
         MESSAGE10 DB   13,10,'Data         - $'
         MESSAGE11 DB   27,'[0;68;"TYPE TCH_SCR4";13p'        ;Define F10 key
                   DB   27,'[0;67;"COPY CON TCH_SCR4";13p'    ;Define F9  key
                   DB   "  -- Touch Screen Driver -- ",13,10
                   DB   "        Version X           ",13,10,"$"
         MESSAGE12 DB   "    Installed on COMM $"
         MESSAGE13 DB   "    No Touchscreen Found    ",BELL,13,10,"$"

         MESSAGE15 DB   "Unrecognised driver command :- ",BELL,"$"

         T_S_MESS  DB   "OOOO."           ;Message that should be
                                          ;returned by Touchscreen.
         SIXTEEN   DB   16                ;Constant used for multiplication.

DOS_FCT   LABEL     WORD

                   DW   INIT
                   DW   MEDIA_CHECK
                   DW   BUILD_BPB
                   DW   IOCTL_IN
                   DW   INPUT
                   DW   ND_INPUT
                   DW   IN_STAT
                   DW   IN_FLUSH
                   DW   OUTPUT
                   DW   OUT_VERIFY
                   DW   OUT_STAT
                   DW   OUT_FLUSH
                   DW   IOCTL_OUT

DRV_FCT    LABEL      BYTE

                   DB    'M0.  '          ;List of strings recognised
                   DB    'M1.  '          ;by the driver software.
                   DB    'M2.  '
                   DB    'M3.  '
                   DB    'M128.'
MODE129:           DB    'M129.'          ;( Also messages sent to Touchscreen
MODE130:           DB    'M130.'          ;when entering modes C0, C1 etc. )
MODE131:           DB    'M131.'
                   DB    'C0.  '
                   DB    'C1.  '
                   DB    'C2.  '

COM_TAB    LABEL       BYTE

                   DW     MESSAGE         ;Routine called for 'M0.'
                   DW     MESSAGE         ;                   'M1.'
                   DW     MESSAGE         ;                   'M2.'
                   DW     MESSAGE         ;                   'M3.'
                   DW     MESSAGE         ;                   'M128.'
                   DW     MESSAGE         ;                   'M129.'
                   DW     MESSAGE         ;                   'M130.'
                   DW     MESSAGE         ;                   'M131.'
                   DW     CO_ORD_0        ;                   'C0.'
                   DW     CO_ORD_1        ;                   'C1.'
                   DW     CO_ORD_2        ;                   'C2.'

         UARTREG   DW   ?                  ;Address of uart control registers
                                           ;used to differentiate between
                                           ;COMM1 and COMM2 during
                                           ;I/O operations

         BUFF_IN   DB  256 DUP (?)         ;Driver input buffer

         IPT_CT    DW  0                   ;Input buffer pointer

         BUFF_OUT  DB  'M129.',251 DUP (?) ;Driver output buffer
                                           ;( Contains string to send to
                                           ;Touchscreen during 'search' )

         OUT_CT    DW  0                   ;Output buffer pointer

         TIME_OUT  DW  0                   ;Communication timeout value

         MODE      DB  1                   ;Driver operating mode :-
                                           ; 1 = M0 M1 M2 M3 M128
                                           ;     M129  M130  M131
                                           ; 2 = C0
                                           ; 3 = C1
                                           ; 4 = C2

DEV_STRATEGY:

;   This is the strategy entry point from DOS. The driver software saves
;   the pointer to the DOS request header and returns.

          MOV    CS:RH_SEG,ES
          MOV    CS:RH_OFF,BX  

          RET

          RH_SEG    DW   ?             ;Storage space for pointer
          RH_OFF    DW   ?             ;to Request Header.

DEV_INT:

;   This is the interrupt entry point from DOS. The driver software saves
;   the machine state before decoding the request header to obtain the
;   required function.

          PUSH    AX
          PUSH    BX
          PUSH    CX
          PUSH    DX
          PUSH    DI
          PUSH    SI
          PUSH    DS
          PUSH    ES

          MOV     CS:RH_SEG,ES      ;Pointer to Request Header still
          MOV     CS:RH_OFF,BX      ;contained in ES:BX.

          MOV     DS,CS:[RH_SEG]    ;Point DS:BX to header
          MOV     BX,CS:[RH_OFF]

          MOV     AL,DS:BYTE PTR SRH_CCD_FLD[BX] ;Get command code from
                                                 ;the Request Header.
          ROL     AL,1
          LEA     DI,DOS_FCT        ;Point to table of function addresses
          XOR     AH,AH
          ADD     DI,AX
          CALL    WORD PTR CS:[DI]  ;Jump to selected function

;Program execution will return to this point after the required
;function has been executed.

          POP     ES
          POP     DS
          POP     SI
          POP     DI
          POP     DX
          POP     CX
          POP     BX
          POP     AX

          RET                       ;Return to DOS.

TCH_SCR4  ENDP

UNDFND    PROC     NEAR

;The following functions are not supported by this device.

MEDIA_CHECK:
BUILD_BPB:
IOCTL_IN:
ND_INPUT:
IN_STAT:
IN_FLUSH:
OUT_VERIFY:
OUT_STAT:
OUT_FLUSH:
IOCTL_OUT:

         MOV     DS,CS:[RH_SEG]    ;Point DS:BX to Request Header.
         MOV     BX,CS:[RH_OFF]

         OR      DS:WORD PTR SRH_STA_FLD[BX],8103H   ;Set status done
                                                     ;error 'BAD COMMAND'
         RET         

UNDFND   ENDP

INIT     PROC     NEAR

;Routine to initialize the communication channel and to return the
;driver end address to DOS.

          MOV     DS,CS:[RH_SEG]    ;Point DS:BX to Request Header
          MOV     BX,CS:[RH_OFF]

          MOV     AX,OFFSET DRV_END                ;Get offset of
                                                   ;end of driver
          MOV     DS:WORD PTR INI_ADD_FLD[BX],AX   ;Place in Request Header
          MOV     AX,CS                            ;Get end segment
          MOV     DS:WORD PTR INI_ADD_FLD+2[BX],AX ;Place in header

          PUSH    CS                    ;Restore Data addressability
          POP     DS

          MOV       DX,OFFSET MESSAGE11 ;DOS function call to display
          MOV       AH,09H              ;message eleven
          INT       21H

          MOV       DX,3FDH             ;COMM1 line status register
          IN        AL,DX               ;Get line status
          TEST      AL,80H              ;Bit seven will return zero if the
                                        ;communication channel is installed.
          JNZ       NOT_COM1

          MOV       CS:[UARTREG],3FFH
          CALL      UARTINIT            ;Initialize COMM1

          CALL      MESSAGE             ;Send initialization message
          JC        NOT_COM1            ;If timeout, no RS-232 connection

          CALL      GETDATA             ;Fetch Touchscreen reply
          JC        NOT_COM1            ;If timeout, no Touchscreen connected
          CALL      MESS_COMP           ;Check for correct response from
                                        ;Touchscreen
          JC        NOT_COM1            ;If response is incorrect, the
                                        ;Touchscreen is not connected.

          MOV       DX,OFFSET MESSAGE12 ;DOS function call to display
          MOV       AH,09H              ;message twelve
          INT       21H

          MOV       DL,'1'
          MOV       AH,2
          INT       21H

          MOV       DL,CR
          MOV       AH,2
          INT       21H

          JMP       CONT
NOT_COM1:
          MOV       DX,2FDH             ;COMM2 line status register
          IN        AL,DX               ;Get line status
          TEST      AL,80H              ;Bit seven will return zero if the
                                        ;communication channel is installed.
          JNZ       NOT_COM2

          MOV       CS:[UARTREG],2FFH
          CALL      UARTINIT            ;Initialize COMM1

          CALL      MESSAGE             ;Send initialization message
          JC        NOT_COM2

          CALL      GETDATA             ;Fetch Touchscreen reply
          JC        NOT_COM2            ;If timeout, no Touchscreen connected
          CALL      MESS_COMP           ;Check for correct response from
                                        ;Touchscreen
          JC        NOT_COM2            ;If response is incorrect, the
                                        ;Touchscreen is not connected.

          MOV       DX,OFFSET MESSAGE12
          MOV       AH,09H
          INT       21H

          MOV       DL,'2'
          MOV       AH,2
          INT       21H

          MOV       DL,CR
          MOV       AH,2
          INT       21H

          JMP       CONT

NOT_COM2: MOV       DX,OFFSET MESSAGE13
          MOV       AH,09H
          INT       21H
CONT:
          MOV     DS,CS:[RH_SEG]    ;Point DS:BX to Request Header
          MOV     BX,CS:[RH_OFF]

          OR      DS:WORD PTR SRH_STA_FLD[BX],0100H ;Set status done,
                                                    ;Device ready
          RET

INIT      ENDP

INPUT     PROC    NEAR

;Routine to input data from Touchscreen 

         CMP     CS:[IPT_CT],0    ;Test for input buffer empty
         JNE     BUFF_FUL

         CALL    GETDATA          ;Fetch data string from Touchscreen
                                  ;and place in input buffer
         JC      IPUT_ERR         ;If carry is set then the communication
                                  ;routines have timed out and there is a
                                  ;communications error.

BUFF_FUL:MOV     DS,CS:[RH_SEG]   ;Point DS:BX to Request Header.
         MOV     BX,CS:[RH_OFF]
         MOV     AX,DS:[BX+16]     ;GET DOS BUFFER SEGMENT
         MOV     ES,AX
         MOV     AX,DS:[BX+14]     ;GET DOS BUFFER OFFSET
         MOV     DI,AX             ;SET DESTINATION INDEX

         MOV     BX,CS:[IPT_CT]
         MOV     AL,CS:BYTE PTR BUFF_IN[BX]
         INC     CS:[IPT_CT]
         MOV     ES:[DI],AX
         CMP     AL,EOF
         JNE     IPT_JMP
         MOV     CS:[IPT_CT],0

IPT_JMP: 
         MOV     AX,CS:[RH_SEG]    ;POINT DS:BX TO HEADER
         MOV     DS,AX
         MOV     BX,CS:[RH_OFF]

         OR      DS:[BX+3],0100H   ;SET STATUS DONE
         PUSH    CS                ;RESTORE DATA ADDRESSABILITY
         POP     DS

         RET

IPUT_ERR:

         MOV     AX,CS:[RH_SEG]    ;POINT DS:BX TO HEADER
         MOV     DS,AX
         MOV     BX,CS:[RH_OFF]

         OR      DS:[BX+3],810BH   ;SET STATUS DONE READ ERROR
         PUSH    CS                ;RESTORE DATA ADDRESSABILITY
         POP     DS

         RET

INPUT    ENDP

OUTPUT   PROC    NEAR

              ;ROUTINE TO OUTPUT CHARACTERS TO TOUCHSCREEN
              ;( INPUT FROM DOS )

         MOV     AX,CS:[RH_SEG]    ;POINT DS:BX TO HEADER
         MOV     DS,AX
         MOV     BX,CS:[RH_OFF]

         MOV     AX,DS:[BX+16]     ;GET DOS BUFFER SEGMENT
         MOV     ES,AX
         MOV     AX,DS:[BX+14]     ;GET DOS BUFFER OFFSET
         MOV     DI,AX             ;SET DESTINATION INDEX

         MOV     BX,CS:[OUT_CT]
         MOV     AL,ES:[DI]        ;get character from DOS buffer
         MOV     CS:BYTE PTR BUFF_OUT[BX],AL
         INC     CS:[OUT_CT]
         CMP     AL,LF
         JNE     OUT_JMP

;The following section of program is executed after a 'LF' character
;is placed in the driver buffer. The 'LF' character terminates the string
;being passed to the driver and signals that the driver should decode the
;command and perform the required function.

         MOV     CS:[OUT_CT],0       ;reset buffer pointer
         INC     BX
         MOV     CS:BYTE PTR BUFF_OUT[BX],13
         INC     BX
         MOV     CS:BYTE PTR BUFF_OUT[BX],10
         INC     BX
         MOV     CS:BYTE PTR BUFF_OUT[BX],'$'
         PUSH    CS
         POP     DS
         CALL    STRING
         JC      OUT_ERR
         DEC     AL

         ROL     AL,1
         LEA     DI,COM_TAB        ;Point to table of function addresses
         XOR     AH,AH
         ADD     DI,AX
         CALL    WORD PTR CS:[DI]  ;Jump to selected function

;Program execution will return to this point after the required
;function has been executed.

OUT_JMP: MOV     AX,CS:[RH_SEG]    ;POINT DS:BX TO HEADER
         MOV     DS,AX
         MOV     BX,CS:[RH_OFF]

         OR      DS:[BX+3],0100H   ;SET STATUS DONE
         PUSH    CS                ;RESTORE DATA ADDRESSABILITY
         POP     DS

         RET

OUT_ERR: MOV     DX,OFFSET MESSAGE15
         MOV     AH,9
         INT     21H

         MOV     DX,OFFSET BUFF_OUT
         MOV     AH,9
         INT     21H

         JMP     OUT_JMP

OUTPUT    ENDP

CONVDATA  PROC      NEAR

          XOR     AH,AH
          MOV     AL,CS:[SI]
          SUB     AL,"@"
          MUL     CS:SIXTEEN
          MOV     CS:[DI],AL
          INC     SI
          MOV     AL,CS:[SI]
          SUB     AL,"@"
          ADD     CS:[DI],AL
;         ADD     CS:BYTE PTR[DI],EOF+2     ;Convert to prevent EOF clashes
;         CMP     CS:BYTE PTR[DI],EOF
;         JNE     C_DAT_END
;         DEC     CS:BYTE PTR[DI]
C_DAT_END:INC     SI
          INC     DI

          RET

CONVDATA  ENDP

STRING    PROC      NEAR

          MOV     DL,12            ;DL contains the number of the string
NXT_TEST: DEC     DL               ;being compared plus one.
          JNZ     STR_CONT
          STC                      ;indicate no match
          RET
STR_CONT:
          MOV     BH,0
          MOV     BL,DL
          CLC
          ROL     BL,1
          ROL     BL,1
          ADD     BL,DL           ;BL = string count * 5
          SUB     BL,5
          MOV     SI,0            ;initialize buffer pointer

STR_CMP:  MOV     AL,CS:BYTE PTR DRV_FCT[BX]

          CMP     CS:BYTE PTR BUFF_OUT[SI],AL
          JNE     NXT_TEST
          INC     SI
          INC     BX
          CMP     AL,'.'
          JNE     STR_CMP
          MOV     AL,DL            ;put number of string in AL
          CLC                      ;indicate match found
          RET

STRING    ENDP

MESSAGE   PROC      NEAR

;This sub_routine sends a string of ASCII characters terminated by a '.'
;from the driver buffer to the Touchscreen.

          MOV       CS:BYTE PTR MODE,1
          MOV       BX,0

NEXTCHAR: MOV       CX,0FFFH
PAUSE:    LOOP      PAUSE

          MOV       AL,CS:BYTE PTR BUFF_OUT[BX];GET CHARACTER POINTED TO BY BX

          PUSH      AX
          CALL      SENDCHAR                  ;SEND FIRST CHARACTER
          POP       AX
          JC        MESS_ERR

          CMP       AL,'.'
          JE        MESS_END

          INC       BX                        ;POINT TO NEXT CHARACTER

          JMP       NEXTCHAR          

MESS_END: CLC
          RET                           ;RETURN TO MAIN ROUTINE

MESS_ERR: STC
          RET

MESSAGE   ENDP

CO_ORD_0  PROC      NEAR

          MOV       CX,5
          MOV       BX,0
C0_LOOP:  MOV       AL,CS:BYTE PTR MODE129[BX]
          MOV       CS:BYTE PTR BUFF_OUT[BX],AL
          INC       BX
          LOOP      C0_LOOP

          CALL      MESSAGE             ;Send contents of output buffer
                                        ;to Touchscreen.

          MOV       CS:BYTE PTR MODE,2  ;Indicate driver now operating in
                                        ;Mode C0.
          MOV       CS:[IPT_CT],0
          MOV       CS:[OUT_CT],0

          RET

CO_ORD_0  ENDP

CO_ORD_1  PROC      NEAR

          MOV       CX,5
          MOV       BX,0
C1_LOOP:  MOV       AL,CS:BYTE PTR MODE131[BX]
          MOV       CS:BYTE PTR BUFF_OUT[BX],AL
          INC       BX
          LOOP      C1_LOOP

          CALL      MESSAGE             ;Send contents of output buffer
                                        ;to Touchscreen.

          MOV       CS:BYTE PTR MODE,3  ;Indicate driver now operating in
                                        ;Mode C1.
          MOV       CS:[IPT_CT],0
          MOV       CS:[OUT_CT],0

          RET

CO_ORD_1  ENDP

CO_ORD_2  PROC      NEAR

          MOV       CX,5
          MOV       BX,0
C2_LOOP:  MOV       AL,CS:BYTE PTR MODE130[BX]
          MOV       CS:BYTE PTR BUFF_OUT[BX],AL
          INC       BX
          LOOP      C2_LOOP

          CALL      MESSAGE             ;Send contents of output buffer
                                        ;to Touchscreen.

          MOV       CS:BYTE PTR MODE,4  ;Indicate driver now operating in
                                        ;Mode C2.
          MOV       CS:[IPT_CT],0
          MOV       CS:[OUT_CT],0

          RET

 CO_ORD_2  ENDP 

SENDCHAR  PROC      NEAR

;SUB-ROUTINE TO SEND A CHARACTER TO THE TOUCH SCREEN. THE CHARACTER TO
;BE SENT SHOULD BE CONTAINED IN THE AL REGISTER.

          PUSH      BX
          MOV       BX,AX               ;SAVE AX
          MOV       CS:[TIME_OUT],0

          CALL      RTSON               ;TURN RTS LINE ON

          MOV       DX,3FDH             ;POINT TO UART LINE STATUS REGISTER
          AND       DX,CS:[UARTREG]

REGFULL:  IN        AL,DX               ;GET LINE STATUS

          INC       CS:[TIME_OUT]
          CMP       CS:[TIME_OUT],0FFFFH
          JE        SEND_ERR

          TEST      AL,20H              ;TEST FOR TRANSMITTER HOLDING
                                        ;REGISTER EMPTY
          JE        REGFULL             ;IF NOT EMPTY THEN WAIT.

          MOV       DX,3FEH             ;POINT TO MODEM STATUS REGISTER
          AND       DX,CS:[UARTREG]

          MOV       CS:[TIME_OUT],0

NOCTS:    IN        AL,DX               ;GET MODEM STATUS

          INC       CS:[TIME_OUT]
          CMP       CS:[TIME_OUT],0FFFFH
          JE        SEND_ERR

          TEST      AL,10H              ;TEST FOR CLEAR TO SEND SIGNAL
          JE        NOCTS               ;IF TOUCH SCREEN NOT READY THEN WAIT

          MOV       DX,3F8H             ;POINT TO TRANSMITTER HOLDING
                                        ;REGISTER
          AND       DX,CS:[UARTREG]

          MOV       AX,BX
          OUT       DX,AL               ;SEND ASCII CHARACTER

          CALL      RTSOFF              ;TURN RTS OFF

          CLC
          POP       BX
          RET                           ;RETURN TO CALLING ROUTINE.

SEND_ERR:
          STC
          POP       BX
          RET

SENDCHAR  ENDP

GETDATA        PROC          NEAR

;SUB-ROUTINE TO FETCH ASCII DATA FROM THE TOUCH SCREEN. DATA BYTES ARE
;READ IN AND STORED IN CONSECUTIVE BYTES ( TSDATA0 TO TSDATA8 ) UNTILL
;A FULL STOP IS RECIEVED AS AN END OF MESSAGE MARKER FROM THE TOUCH
;SCREEN.

;THE SUB-ROUTINE PERFORMS A DOUBLE READ AND DISCARDS THE FIRST SET OF
;DATA TO FLUSH OLD DATA FROM THE TOUCH SCREEN.

          MOV       CS:[TIME_OUT],0

          MOV       CX,2H               ;NUMBER OF READS TO BE PERFORMED
DOAGAIN:  MOV       BX,0                ;POINT TO START OF TOUCH
                                        ;SCREEN DATA STORAGE AREA

NXTCHAR:  CALL      RTSON               ;TURN RTS ON

          MOV       DX,3FDH             ;ADDRESS OF LINE STATUS REGISTER
          AND       DX,CS:[UARTREG]

NODATA:   IN        AL,DX               ;LINE STATUS REGISTER IN AL

          INC       CS:[TIME_OUT]
          CMP       CS:[TIME_OUT],0FFFFH
          JE        G_D_ERR

          TEST      AL,1H
          JZ        NODATA              ;WAIT UNTILL CHARACTER RECEIVED

          CALL      RTSOFF              ;TURN RTS OFF

          MOV       DX,3F8H             ;ADDRESS OF RECEIVER DATA REGISTER
          AND       DX,CS:[UARTREG]

          IN        AL,DX               ;RECEIVED DATA IN AL
          MOV       CS:BYTE PTR BUFF_IN[BX],AL          ;STORE DATA

          INC       BX                  ;POINT TO NEXT STORAGE SPACE

          CMP       AL,"."              ;COMPARE TO END OF MESSAGE MARKER
          JNE       NXTCHAR             ;IF NOT, READ NEXT CHARACTER

          LOOP      DOAGAIN             ;REPEAT READ PROCESS.

          MOV       AL,EOF
          MOV       CS:BYTE PTR BUFF_IN[BX],AL

          CMP       CS:BYTE PTR MODE,1
          JE        G_D_END

          CMP       CS:BYTE PTR MODE,2
          JNE       G_D_M3

          MOV       SI,OFFSET BUFF_IN
          MOV       DI,OFFSET BUFF_IN

          CALL      CONVDATA

          CALL      CONVDATA

          MOV       CS:BYTE PTR[DI],EOF

          JMP       G_D_END

G_D_M3:   CMP       CS:BYTE PTR MODE,3
          JNE       G_D_M4

          MOV       SI,OFFSET BUFF_IN
          MOV       DI,OFFSET BUFF_IN

          MOV       CX,8

GD_LP0:   CALL      CONVDATA

          LOOP      GD_LP0

          MOV       CS:BYTE PTR[DI],EOF

          JMP       G_D_END
G_D_M4:
          MOV       SI,OFFSET BUFF_IN
          MOV       DI,OFFSET BUFF_IN

          MOV       CX,4

GD_LP1:   CALL      CONVDATA

          LOOP      GD_LP1

          MOV       CS:BYTE PTR[DI],EOF 

G_D_END:  CLC                           ;indicate no error
          RET                           ;RETURN TO MAIN ROUTINE.

G_D_ERR:  STC                           ;indicate error
          RET

GETDATA        ENDP

UARTINIT		PROC		NEAR

;THIS SUB-ROUTINE INITIALIZES THE UART TO THE CORRECT PARAMETERS
;FOR USE WITH THE TOUCH SCREEN.

          MOV       DX,3FBH             ;POINT TO LINE CONTROL REGISTER
          AND       DX,CS:[UARTREG]

          MOV       AL,80H              ;SELECT BAUD RATE DIVISOR REGISTERS
          OUT       DX,AL
          MOV       DX,3F8H             ;POINT TO BAUD RATE DIVISOR LSB
          AND       DX,CS:[UARTREG]

          MOV       AL,0CH              ;SET FOR 9600
          OUT       DX,AL
          MOV       DX,3F9H             ;POINT TO BAUD RATE DIVISOR MSB
          AND       DX,CS:[UARTREG]

          MOV       AL,0H               ;SET FOR 9600
          OUT       DX,AL
          MOV       DX,3FBH             ;POINT TO LINE CONTROL REGISTER
          AND       DX,CS:[UARTREG]

          MOV       AL,03H              ;SET FOR 8 DATA BITS
                                        ;        1 STOP BIT
                                        ;        NO PARITY
          OUT       DX,AL
          MOV       DX,3FCH             ;POINT TO MODEM CONTROL REGISTER
          AND       DX,CS:[UARTREG]

          MOV       AL,08H              ;DE-ACTIVATE DTR, RTS AND INTERUPTS
          OUT       DX,AL
          MOV       DX,3F9H             ;POINT TO INTERUPT ENABLE REGISTER
          AND       DX,CS:[UARTREG]

          MOV       AL,0                ;DISABLE ALL INTERUPTS
          OUT       DX,AL
          RET                           ;RETURN TO MAIN ROUTINE.

UARTINIT  ENDP

RTSON     PROC      NEAR

;THIS SUBROUTINE TURNS THE 'REQUEST TO SEND' LINE TO THE TOUCH SCREEN
;TO ITS ACTIVE STATE. ALL REGISTERS ARE RETURNED TO THEIR INITIAL VALUES
;UPON COMPLETION.

          PUSH      AX                  ;SAVE AX
          PUSH      DX                  ;SAVE DX
          MOV       DX,3FCH             ;POINT TO UART MODEM CONTROL REGISTER
          AND       DX,CS:[UARTREG]

          MOV       AL,3H               ;SET ACTIVE 'REQUEST TO SEND'
          OUT       DX,AL               ;AND ACTIVE 'DATA TERMINAL READY'
                                        ;CONTROL LINES.
          POP       DX                  ;RESTORE DX
          POP       AX                  ;RESTORE AX

          RET                           ;RETURN TO MAIN ROUTINE.

RTSON     ENDP

RTSOFF    PROC      NEAR

;THIS ROUTINE TURNS THE 'REQUEST TO SEND' LINE TO THE TOUCH SCREEN
;TO ITS INACTIVE STATE. ALL REGISTERS ARE RETURNED TO THEIR INITIAL VALUES
;UPON COMPLETION.

          PUSH      AX                  ;SAVE AX
          PUSH      DX                  ;SAVE DX
          MOV       DX,3FCH             ;POINT TO UART MODEM CONTROL REGISTER
          AND       DX,CS:[UARTREG]

          MOV       AL,1H               ;SET INACTIVE 'REQUEST TO SEND'
          OUT       DX,AL               ;AND ACTIVE 'DATA TERMINAL READY'
                                        ;CONTROL LINES.
          POP       DX                  ;RESTORE DX
          POP       AX                  ;RESTORE AX

          RET                           ;RETURN TO MAIN ROUTINE.

RTSOFF    ENDP

MESS_COMP PROC   NEAR

         MOV      CX,5
         MOV      SI,0
M_C_LOOP:
         MOV      AL,CS:BYTE PTR T_S_MESS[SI]
         CMP      CS:BYTE PTR BUFF_IN[SI],AL
         JNE      NOT_SAME
         INC      SI
         LOOP     M_C_LOOP
         CLC
         RET
NOT_SAME:
         STC
         RET

MESS_COMP ENDP

R_H      PROC    NEAR

         PUSH     CS
         POP      DS

         MOV      DX,OFFSET MESSAGE4
         MOV      AH,9
         INT      21H

         MOV      AX,CS:[RH_SEG]
         MOV      DS,AX

         MOV      CX,0
         MOV      BX,CS:[RH_OFF]
         MOV      CL,DS:[BX]        ;GET HEADER LENGTH
         INC      CL                ;ALLOW FOR DOS 3 VARIATION

         PUSH     CS
         POP      DS
         MOV      DX,OFFSET MESSAGE5
         MOV      AH,9
         INT      21H

         MOV      AX,CS:[RH_SEG]
         MOV      DS,AX
         MOV      AL,DS:[BX]
         CALL     W_BYTE
         INC      BX
         DEC      CX

         PUSH     CS
         POP      DS
         MOV      DX,OFFSET MESSAGE6
         MOV      AH,9
         INT      21H

         MOV      AX,CS:[RH_SEG]
         MOV      DS,AX
         MOV      AL,DS:[BX]
         CALL     W_BYTE
         INC      BX
         DEC      CX

         PUSH     CS
         POP      DS
         MOV      DX,OFFSET MESSAGE7
         MOV      AH,9
         INT      21H

         MOV      AX,CS:[RH_SEG]
         MOV      DS,AX
         MOV      AL,DS:[BX]
         CALL     W_BYTE
         INC      BX
         DEC      CX

         PUSH     CS
         POP      DS
         MOV      DX,OFFSET MESSAGE8
         MOV      AH,9
         INT      21H

         MOV      AX,CS:[RH_SEG]
         MOV      DS,AX
         MOV      AX,DS:[BX]
         CALL     W_WORD
         INC      BX
         INC      BX
         DEC      CX
         DEC      CX

         PUSH     CS
         POP      DS
         MOV      DX,OFFSET MESSAGE9
         MOV      AH,9
         INT      21H

         MOV      AX,CS:[RH_SEG]
         MOV      DS,AX

         PUSH     CX

         MOV      CX,8
R_H_LP1: MOV      AL,DS:[BX]

         CALL     W_BYTE

         INC      BX

         LOOP     R_H_LP1

         POP      CX
         CLC
         SBB      CX,8

         PUSH     CS
         POP      DS
         MOV      DX,OFFSET MESSAGE10
         MOV      AH,9
         INT      21H

         MOV      AX,CS:[RH_SEG]
         MOV      DS,AX
         MOV      AL,DS:[BX]
         CALL     W_BYTE
         INC      BX
         DEC      CX

R_H_LOOP:MOV      AL,DS:[BX]

         CALL     W_BYTE

         INC      BX
         LOOP     R_H_LOOP

         MOV      DL,0DH
         MOV      AH,2
         INT      21H
         MOV      DL,0AH
         INT      21H

         MOV     AX,CS:[RH_SEG]    ;POINT DS:BX TO HEADER
         MOV     DS,AX
         MOV     BX,CS:[RH_OFF]

         MOV     DX,DS:[BX+16]     ;GET DOS BUFFER SEGMENT
         MOV     AX,DX
         MOV     ES,AX
         CALL    W_WORD
         MOV     DL,':'
         MOV     AH,2
         INT     21H
         MOV     AX,DS:[BX+14]     ;GET DOS BUFFER OFFSET
         CALL    W_WORD
         MOV     DL,' '
         MOV     AH,2
         INT     21H
         PUSH    BX
         MOV     DX,DS:[BX+14]
         MOV     BX,DX
         MOV     DL,ES:[BX]
         MOV     AH,2
         INT     21H
         POP     BX

         RET

R_H      ENDP

W_WORD   PROC    NEAR

         MOV      CS:W_SPACE,AX

         PUSH     AX
         PUSH     BX
         PUSH     CX
         PUSH     DX

         MOV      BX,1
         MOV      CX,2

W_WORD_0:MOV      AL,CS:BYTE PTR W_SPACE[BX]

         PUSH     AX
         AND      AL,0F0H
         ROR      AL,1
         ROR      AL,1
         ROR      AL,1
         ROR      AL,1

         ADD      AL,'0'
         CMP      AL,'9'
         JLE      W_WORD_1
         ADD      AL,7
W_WORD_1:MOV      DL,AL

         MOV      AH,2
         INT      21H

         POP      AX
         AND      AL,0FH
         ADD      AL,'0'
         CMP      AL,'9'
         JLE      W_WORD_2
         ADD      AL,7
W_WORD_2:MOV      DL,AL

         MOV      AH,2
         INT      21H

         DEC      BX
         LOOP     W_WORD_0

         POP     DX
         POP     CX
         POP     BX
         POP     AX

         RET

W_SPACE  DW       ?

W_WORD   ENDP

W_BYTE   PROC    NEAR

         MOV      CS:W_SBYTE,AL

         PUSH     AX
         PUSH     BX
         PUSH     CX
         PUSH     DX

         MOV      BX,0

         MOV      AL,CS:W_SBYTE

         PUSH     AX
         AND      AL,0F0H
         ROR      AL,1
         ROR      AL,1
         ROR      AL,1
         ROR      AL,1

         ADD      AL,'0'
         CMP      AL,'9'
         JLE      W_BYTE_1
         ADD      AL,7
W_BYTE_1:MOV      DL,AL

         MOV      AH,2
         INT      21H

         POP      AX
         AND      AL,0FH
         ADD      AL,'0'
         CMP      AL,'9'
         JLE      W_BYTE_2
         ADD      AL,7
W_BYTE_2:MOV      DL,AL

         MOV      AH,2
         INT      21H

         MOV     DL,' '
         MOV     AH,2
         INT     21H

         POP     DX
         POP     CX
         POP     BX
         POP     AX

         RET

W_SBYTE   DB     ?

W_BYTE   ENDP

DRV_END:

;MACRO TO ALIGN THE DRIVER ON A PARAGRAPH BOUNDARY
if ($-START) MOD 16
ORG  ($-START)+16-(($-START) MOD 16)      
endif

CODE		ENDS
END		BEGIN
