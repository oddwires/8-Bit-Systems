CODE     SEGMENT     PARA     PUBLIC     'CODE'

;         ******************************************
;         *                                        *
;         *   ASSEMBLER LISTING OF TOUCH SCREEN    *
;         *   DRIVER SOFTWARE FOR USE WITH THE     *
;         *   IBM  P.C.                            *
;         *                                        *
;         ******************************************


          ASSUME    CS:CODE,ES:CODE,DS:CODE

          MODE0     DB   'M0.  '        ;CHARACTER STRINGS TO BE SENT
          MODE1     DB   'M1.  '        ;TO THE TOUCH SCREEN DURING
          MODE2     DB   'M2.  '        ;INITIALIZATION.
          MODE3     DB   'M3.  '
          MODE128   DB   'M128.'        ;IT IS IMPORTANT THAT THESE
          MODE129   DB   'M129.'        ;STRINGS BE SPACE FILLED UP TO
          MODE130   DB   'M130.'        ;FIVE CHARACTERS AND TERMINATED
          MODE131   DB   'M131.'        ;BY A FULL STOP.

          TWO       DB    2             ;CONSTANTS USED FOR MULTIPLICATION
          FIVE      DB    5
          SIXTEEN   DB   16
          EIGHTEEN  DB   18
          TWENTY4   DB   24
          SEVENTY9  DB   79
          EIGHTY    DB   80
          NINETY    DB   90
          ONE20     DB  120

;MESSAGES TO BE SENT TO THE SCREEN DURING SOFTWARE INSTALLATION.

     MESSAGE0  DB   13,10,13,10,"-- Touch Screen Driver "
               DB   "software installed. --",13,10,13,10,"$"
     MESSAGE1  DB   13,10," --   Comunication Channel COMM$"
     MESSAGE2  DB   "   Enter 1 for use on COMM1",13,10
               DB   "         2 for use on COMM2.",13,10,"$"
     MESSAGE3  DB   " initialized for use with TouchScreen  --"
               DB   13,10,13,10,"$"

INT_FCTAB		LABEL	BYTE

          DW        INITLIZE            ;START ADDRESSES OF THE ROUTINES
          DW        UNDFND              ;THAT FORM THE TOUCH SCREEN
          DW        UNDFND              ;DRIVER FUNCTIONS.
          DW        XYPRESS
          DW        UNDFND
          DW        XYPRESS
          DW        UNDFND
          DW        UNDFND
          DW        UNDFND
          DW        UNDFND
          DW        UNDFND
          DW        UNDFND
          DW        UNDFND
          DW        UNDFND
          DW        UNDFND
          DW        UNDFND
          DW        UNDFND
          DW        UNDFND
          DW        UNDFND
          DW        UNDFND

BAS_FCTAB		LABEL	BYTE

          DW        INITLIZE
          DW        XYPRESS
          DW        SIZE
          DW        UNDFND
          DW        UNDFND

          PARAM_A   DW   0              ;STORAGE SPACE FOR THE FOUR
          PARAM_B   DW   0              ;INPUT / OUTPUT PARAMETERS.
          PARAM_C   DW   0
          PARAM_D   DW   0

          COMMASK   DW   ?              ;MASK USED TO DIFFERENTIATE
                                        ;BETWEEN COMM1 AND COMM2
                                        ;DURING INPUT/OUTPUT OPERATIONS

          OLDXVAL   DW   ?              ;STORAGE SPACE USED IN FUNCTION
          OLDYVAL   DW   ?              ;FIVE TO KEEP CO-ORDINATES.

        
          THISPASS  DB   ?              ;VARIABLE USED BY FUNCTION
                                        ;FIVE TO INDICATE IF THE SCREEN
                                        ;IS BEING TOUCHED.

          LASTPASS  DB   0              ;VARIABLE USED BY FUNCTION
                                        ;FIVE TO INDICATE IF THE SCREEN
                                        ;WAS BEING TOUCHED THE LAST
                                        ;TIME THE FUNCTION WAS CALLED.

          DB 256 DUP(?)                  ;DRIVER STACK
STACKTOP: DB        ?

          STACKOFF  DW        ?         ;STORAGE SPACE FOR BASIC'S
          STACKSEG  DW        ?         ;STACK POINTER.

          TSDATA0   DB        ?         ;STORAGE SPACE FOR ASCII DATA
          TSDATA1   DB        ?         ;READ FROM TOUCH SCREEN.
          TSDATA2   DB        ?
          TSDATA3   DB        ?
          TSDATA4   DB        ?
          TSDATA5   DB        ?
          TSDATA6   DB        ?
          TSDATA7   DB        ?
          TSDATA8   DB        ?

BEGIN:

;THIS SECTION OF PROGRAM IS EXECUTED WHEN THE DRIVER SOFTWARE IS BEING
;INSTALLED. IT'S FUNCTION IS TO SET UP THE INTERUPT RESPONSE VECTOR
;TABLE AND TO INITIALIZE THE COMMUNICATION CHANNEL TO THE TOUCH SCREEN.

          MOV       AX,0H               ;POINT TO INTERUPT RESPONSE
          MOV       ES,AX               ;VECTOR TABLE.
          MOV       DI,204              ;POINT TO INTERUPT 51.
          MOV       AX,OFFSET FCN_STRT  ;FETCH START ADDRESS OF DRIVER.
          CLD                           ;WRITE START ADDRESS TO VECTOR
          STOSW                         ;TABLE.
          MOV       AX,CS               ;WRITE CODE SEGMENT TO VECTOR
          STOSW                         ;TABLE

          MOV       AX,CODE             ;RESTORE NORMAL DATA ADDRESSABILITY.
          MOV       DS,AX

          MOV       DX,OFFSET MESSAGE0  ;DOS FUNCTION CALL TO DISPLAY
          MOV       AH,09H              ;MESSAGE ZERO
          INT       21H

          CALL      UARTINIT            ;INITIALIZE THE COMMUNICATION
                                        ;CHANNEL

WRNGINP:  MOV       DX,OFFSET MESSAGE2  ;DOS FUNCTION CALL TO DISPLAY
          MOV       AH,09H              ;MESSAGE2 
          INT       21H

          MOV       AH,08H              ;DOS FUNCTION CALL TO GET
          INT       21H                 ;KEYBOARD INPUT.

          PUSH      AX                  ;SAVE INPUT ON STACK

          CMP       AL,"1"              ;TEST FOR COMM1 INPUT
          JNE       TEST2               ;IF NOT THEN TEST FOR COMM2
          MOV       CS:[COMMASK],3FFH   ;SET MASK FOR OPERATION
                                        ;ON COMM1
          JMP       CONT

TEST2:    CMP       AL,"2"
          JNE       WRNGINP             ;IF INCORRECT INPUT THEN
                                        ;RE-DISPLAY QUESTION.
          MOV       CS:[COMMASK],2FFH   ;SET MASK FOR OPERATION
                                        ;ON COMM2

CONT:     MOV       DX,OFFSET MESSAGE1  ;DOS FUNCTION CALL TO DISPLAY
          MOV       AH,09H              ;MESSAGE ONE
          INT       21H

          POP       DX                  ;PUT KEYBOARD INPUT INTO DX

          MOV       AH,2H               ;DOS FUNCTION CALL TO DISPLAY
          INT       21H                 ;ONE CHARACER STORED IN DL

          MOV       DX,OFFSET MESSAGE3  ;DOS FUNCTION CALL TO DISPLAY
          MOV       AH,09H              ;MESSAGE THREE
          INT       21H

          MOV       DX,400H             ;RETURN TO DOS LEAVING 400
          MOV       AH,31H              ;PROGRAM PAGES RESIDENT
          MOV       AL,0                ;IN MEMORY
          INT       21H


START     PROC		FAR

C_IN:
          PUSH       BP
          MOV        BP,SP
          PUSH       DI
          PUSH       SI

          MOV        SI,[BP]+6
          MOV        ES,[BP]+8
          MOV        AX,ES:[SI]
          MOV        CS:[PARAM_A],AX

          MOV        SI,[BP]+10
          MOV        ES,[BP]+12
          MOV        AX,ES:[SI]
          MOV        CS:[PARAM_B],AX

          MOV        SI,[BP]+14
          MOV        ES,[BP]+16
          MOV        AX,ES:[SI]
          MOV        CS:[PARAM_C],AX

          MOV        SI,[BP]+18
          MOV        ES,[BP]+20
          MOV        AX,ES:[SI]
          MOV        CS:[PARAM_D],AX


          CLI                           ;DISABLE ALL INTERUPTS
          MOV       CS:STACKOFF,SP      ;STORE BASIC'S STACK OFFSET
          MOV       CS:STACKSEG,SS      ;STORE BASIC'S STACK SEGMENT
          MOV       AX,CS
          MOV       SS,AX               ;SET DRIVER STACK SEGMENT
          MOV       SP,OFFSET STACKTOP  ;SET DRIVER STACK OFFSET
          STI                           ;ENABLE INTERUPTS

;ROUTINE TO SELECT TOUCH SCREEN DRIVER FUNCTION.

          MOV       AX,CS:[PARAM_A]     ;GET INPUT PARAMETER A
          ROL       AL,1                ;AX = AX * 2
          LEA       DI,BAS_FCTAB        ;POINT TO START OF BASIC
                                        ;FUNCTION TABLE
          XOR       AH,AH               ;CLEAR AH
          ADD       DI,AX               ;POINT TO ADDRESS OF REQUIRED
                                        ;FUNCTION
         CALL      WORD PTR CS:[DI]    ;EXECUTE SELECTED FUNCTION.

;AFTER THE TOUCH SCREEN DRIVER FUNCTION HAS BEEN EXECUTED,PROGRAM
;EXECUTION WILL RETURN TO THIS POINT.

;RESTORE THE STACK

          CLI                           ;DISABLE ALL INTERUPTS
          MOV       SS,CS:STACKSEG      ;RESTORE BASIC'S STACK SEGMENT
          MOV       SP,CS:STACKOFF      ;RESTORE BASIC'S STACK OFFSET
          STI                           ;ENABLE ALL INTERUPTS


          MOV        DI,[BP]+6
          MOV        ES,[BP]+8
          MOV        AX,CS:[PARAM_A]
          MOV        ES:[DI],AX

          MOV        DI,[BP]+10
          MOV        ES,[BP]+12
          MOV        AX,CS:[PARAM_B]
          MOV        ES:[DI],AX

          MOV        DI,[BP]+14
          MOV        ES,[BP]+16
          MOV        AX,CS:[PARAM_C]
          MOV        ES:[DI],AX

          MOV        DI,[BP]+18
          MOV        ES,[BP]+20
          MOV        AX,CS:[PARAM_D]
          MOV        ES:[DI],AX

          CLD                                   
          POP        SI
          POP        DI
          MOV        SP,BP
          POP        BP
          RET


;THIS SECTION OF PROGRAM CONTAINS THE THREE DIFFERENT ENTRY POINTS
;FOR THE TOUCH SCREEN DRIVER. IF THE DRIVER IS CALLED BY MEANS
;OF INTERUPT 51, PROGRAM EXECUTION IS TRANSFERED TO THE LABEL 'ASMBLYIN'.
;IF THE DRIVER IS CALLED THROUGH BASIC, THE ENTRY POINT IS TWO BYTES
;LATER ( LABEL BASICIN ). SINCE PARAMETERS ARE PASSED TO AND FROM
;THE DRIVER BY DIFFERENT MEANS, IT IS NECASSARY TO HANDLE THE TWO
;ENTRY POINTS SEPERATELY.

          JMP       C_IN

FCN_STRT:
          JMP       SHORT ASMBLYIN      ;ENTRY POINT FROM INTERUPT 51

;GET INPUT PARAMETERS FROM BASIC

BASICIN:  PUSH      BP                  ;ENTRY POINT FROM BASIC
          MOV       BP,SP
          MOV       SI,[BP]+12          ;GET ADDRESS OF PARAMETER A
          MOV       AX,[SI]             ;GET PARAMETER A
          MOV       CS:[PARAM_A],AX     ;STORE INPUT PARAMETER A
          MOV       SI,[BP]+10          ;GET ADDRESS OF PARAMETER B
          MOV       AX,[SI]             ;GET PARAMETER B
          MOV       CS:[PARAM_B],AX     ;STORE INPUT PARAMETER B
          MOV       SI,[BP]+8           ;GET ADDRESS OF PARAMETER C
          MOV       AX,[SI]             ;GET PARAMETER C
          MOV       CS:[PARAM_C],AX     ;STORE INPUT PARAMETER C
          MOV       SI,[BP]+6           ;GET ADDRESS OF PARAMETER D
          MOV       AX,[SI]             ;GET PARAMETER D
          MOV       CS:[PARAM_D],AX     ;STORE INPUT PARAMETER D

;THE BASIC STACK HAS A MAXIMUM SIZE OF 16 BYTES. 13 OF THESE BYTES
;HAVE ALREADY BEEN USED AT THIS POINT. IT IS NECASSARY TO SET UP
;A NEW STACK FOR USE WITHIN THE DRIVER IF SUB-ROUTINES ARE TO BE
;USED.

          CLI                           ;DISABLE ALL INTERUPTS
          MOV       CS:STACKOFF,SP      ;STORE BASIC'S STACK OFFSET
          MOV       CS:STACKSEG,SS      ;STORE BASIC'S STACK SEGMENT
          MOV       AX,CS
          MOV       SS,AX               ;SET DRIVER STACK SEGMENT
          MOV       SP,OFFSET STACKTOP  ;SET DRIVER STACK OFFSET
          STI                           ;ENABLE INTERUPTS

;ROUTINE TO SELECT TOUCH SCREEN DRIVER FUNCTION.

          MOV       AX,CS:[PARAM_A]     ;GET INPUT PARAMETER A
          ROL       AL,1                ;AX = AX * 2
          LEA       DI,BAS_FCTAB        ;POINT TO START OF BASIC
                                        ;FUNCTION TABLE
          XOR       AH,AH               ;CLEAR AH
          ADD       DI,AX               ;POINT TO ADDRESS OF REQUIRED
                                        ;FUNCTION
          CALL      WORD PTR CS:[DI]    ;EXECUTE SELECTED FUNCTION.

;AFTER THE TOUCH SCREEN DRIVER FUNCTION HAS BEEN EXECUTED,PROGRAM
;EXECUTION WILL RETURN TO THIS POINT.

;RESTORE THE BASIC STACK

          CLI                           ;DISABLE ALL INTERUPTS
          MOV       SS,CS:STACKSEG      ;RESTORE BASIC'S STACK SEGMENT
          MOV       SP,CS:STACKOFF      ;RESTORE BASIC'S STACK OFFSET
          STI                           ;ENABLE ALL INTERUPTS

;PASS OUTPUT PARAMETERS BACK TO BASIC.

          MOV       DI,[BP]+6           ;GET ADDRESS OF PARAMETER D
          MOV       AX,CS:[PARAM_D]     ;GET OUTPUT PARAMETER D
          MOV       [DI],AX             ;PASS PARAMETER
          MOV       DI,[BP]+8           ;GET ADDRESS OF PARAMETER C
          MOV       AX,CS:[PARAM_C]     ;GET OUTPUT PARAMETER C
          MOV       [DI],AX             ;PASS PARAMETER
          MOV       DI,[BP]+10          ;GET ADDRESS OF PARAMETER B
          MOV       AX,CS:[PARAM_B]     ;GET OUTPUT PARAMETER B
          MOV       [DI],AX             ;PASS PARAMETER
          MOV       DI,[BP]+12          ;GET ADDRESS OF PARAMETER A
          MOV       AX,CS:[PARAM_A]     ;GET OUTPUT PARAMETER A
          MOV       [DI],AX             ;PASS PARAMETER

          POP       BP                  ;RESTORE BP

          RET       8                   ;RETURN TO BASIC AND ADJUST
                                        ;STACK TO ALLOW FOR PARAMETERS.

;GET INPUT PARAMETERS FROM AX,BX,CX AND DX REGISTERS

ASMBLYIN: MOV       CS:[PARAM_A],AX     ;STORE INPUT PARAMETER A
          MOV       CS:[PARAM_B],BX     ;STORE INPUT PARAMETER B
          MOV       CS:[PARAM_C],CX     ;STORE INPUT PARAMETER C
          MOV       CS:[PARAM_D],DX     ;STORE INPUT PARAMETER D

;ROUTINE TO SELECT TOUCH SCREEN DRIVER FUNCTION.

          MOV       AX,CS:[PARAM_A]     ;GET INPUT PARAMETER A
          ROL       AL,1                ;AX = AX * 2
          LEA       DI,BAS_FCTAB        ;POINT TO START OF BASIC
                                        ;FUNCTION TABLE
          XOR       AH,AH               ;CLEAR AH
          ADD       DI,AX               ;POINT TO ADDRESS OF REQUIRED
                                        ;FUNCTION
         CALL      WORD PTR CS:[DI]    ;EXECUTE SELECTED FUNCTION.

;AFTER THE TOUCH SCREEN DRIVER FUNCTION HAS BEEN EXECUTED,PROGRAM
;EXECUTION WILL RETURN TO THIS POINT.


          MOV       DX,CS:[PARAM_D]     ;FETCH OUTPUT PARAMETER D
          MOV       CX,CS:[PARAM_C]     ;FETCH OUTPUT PARAMETER C
          MOV       BX,CS:[PARAM_B]     ;FETCH OUTPUT PARAMETER D
          MOV       AX,CS:[PARAM_A]     ;FETCH OUTPUT PARAMETER A

          IRET                          ;RETURN FROM INTERUPT.

START     ENDP

UNDFND    PROC      NEAR

;IF THIS FUNCTION IS CALLED, THEN A CALL TO A UNDEFINED TOUCH SCREEN
;DRIVER FUNCTION HAS BEEN MADE. NO FURTHER ACTION IS TAKEN AND CONTROL
;IS PASSED BACK TO THE PARAMETER PASSING ROUTINES.

          RET                           ;RETURN TO PARAMETER PASSING ROUTINES

UNDFND    ENDP

INITLIZE  PROC      NEAR

;INITLIZE IS USED TO INITIALIZE THE TOUCH SCREEN TO ONE OF THE EIGHT
;POSSIBLE MODES OF OPERATION. THE MODE OF OPERATION IS PASSED TO
;THE ROUTINE AS PARAMETER B, AND ACTUALY USED BY THE SUBROUTINE 'TSINIT'.
;PARAMETER A IS RETURNED WITH A VALUE OF -1 AND PARAMETER B IS RETURNED
;WITH A VALUE OF 2 TO INDICATE THAT THE HARDWARE AND SOFTWARE ARE INSTALLED.

;THIS FUNCTION CAN BE CALLED WHEN THE TOUCH SCREEN IS IN ANY MODE.

          CALL      UARTINIT            ;INITIALIZE UART

          CALL      TSINIT              ;INITIALIZE TOUCH SCREEN

          MOV       CS:[PARAM_A],-1     ;SET STATUS PARAMETER TO
                                        ;INDICATE HARDWARE AND
                                        ;SOFTWARE ARE INSTALLED

          MOV       CS:[PARAM_B],2      ;THIS IS A DUMMY PARAMETER
                                        ;USED TO MAKE THE TOUCH SCREEN
                                        ;FUNCTION ZERO SIMILAR TO
                                        ;THE MOUSE FUNCTION ZERO

          RET                           ;RETURN TO PARAMETER PASSING ROUTINES

INITLIZE  ENDP

XY        PROC      NEAR

;XY INDICATES IF THE TOUCH SCREEN IS BEING TOUCHED OR NOT.
;IF IT IS, PARAMETER B IS SET TO '1' OTHERWISE  '0' IS RETURNED.
;THE PARAMETERS C AND D REPRESENT THE X AND Y VALUES OF THE
;TOUCHED POINT. IF THE SCREEN IS NOT TOUCHED, THESE VALUES ARE
;BOTH RETURNED AS 255 ( DECIMAL ).

;THIS FUNCTION SHOULD ONLY BE CALLED WHEN THE TOUCH SCREEN IS IN MODE 'M129'.

          CALL      GETDATA             ;GET ASCII DATA FROM TOUCH SCREEN.

          MOV       DI,OFFSET PARAM_C   ;POINT TO PARAMETER C
          MOV       SI,OFFSET TSDATA0   ;POINT TO FIRST BYTE OF ASCII
                                        ;DATA RECIEVED FROM TOUCH SCREEN.

          CALL      CONVDATA            ;CONVERT TWO BYTES OF ASCII
                                        ;DATA INTO INTEGER FORM
                                        ;AND STORE AS PARAMETER C

          CALL      CONVDATA            ;CONVERT NEXT TWO BYTES OF ASCII
                                        ;DATA INTO INTEGER FORM
                                        ;AND STORE AS NEXT PARAMETER (D)

          MOV       CS:[PARAM_B],0H     ;SET PARAMETER B TO INDICATE
                                        ;SCREEN NOT TOUCHED
          MOV       AX,CS:[PARAM_C]     ;FETCH VALUE FOR X
          CMP       AX,255              ;TEST TO SEE IF SCREEN IS
          JE        NOTOUCH             ;TOUCHED
          MOV       AX,CS:[PARAM_D]     ;FETCH VALUE FOR Y
          CMP       AX,255              ;TEST TO SEE IF SCREEN IS
          JE        NOTOUCH             ;TOUCHED

;   IF THE TOUCH SCREEN IS BEING TOUCHED THEN THE FOLLOWING SECTION
;   IS EXECUTED.

          MOV       CS:[PARAM_B],0H     ;SET PARAMETER B TO INDICATE
                                        ;SCREEN IS TOUCHED
          MOV       AX,CS:[PARAM_C]     ;STORE PRESENT CO-ORDINATES
          MOV       CS:[OLDXVAL],AX     ;FOR FUTURE USE BY FUNCTION 5
          MOV       AX,CS:[PARAM_D]
          MOV       CS:[OLDYVAL],AX

NOTOUCH:  RET                           ;RETURN TO PARAMETER PASSING ROUTINES.

XY        ENDP

XYPRESS   PROC     NEAR

;XYPRESS INDICATES IF THE TOUCH SCREEN IS BEING TOUCHED OR NOT.
;IF IT IS, PARAMETER A IS SET TO '1' OTHERWISE  '0' IS RETURNED.
;THE PARAMETERS C AND D REPRESENT THE X AND Y VALUES OF THE
;POINT BEING TOUCHED. IF THE SCREEN IS NOT BEING TOUCHED, THE X AND
;Y VALUES FOR THE LAST POINT TOUCHED ARE RETURNED. PARAMETER B IS
;THE 'BUTTON PRESS' PARAMETER AND IS SET FROM A '0' TO A '1' ON THE
;FIRST CALL TO FUNCTION 5 AFTER THE FINGER IS REMOVED FROM THE SCREEN.

;THIS FUNCTION SHOULD ONLY BE CALLED WHEN THE TOUCH SCREEN IS IN MODE 'M129'.

          MOV       CS:[PARAM_A],0H     ;INITIALIZE PARAMETER A TO
                                        ;INDICATE SCREEN NOT TOUCHED
          MOV       CS:[PARAM_B],0H     ;INITIALIZE PARAMETER B TO
                                        ;INDICATE NO BUTTON PRESS
          MOV       CS:[THISPASS],0H    ;INITIALIZE VARIABLE TO
                                        ;INDICATE SCREEN NOT TOUCHED.

          CALL      GETDATA             ;GET ASCII DATA FROM TOUCH SCREEN.

          MOV       DI,OFFSET PARAM_C   ;POINT TO PARAMETER C
          MOV       SI,OFFSET TSDATA0   ;POINT TO FIRST BYTE OF ASCII
                                        ;DATA FROM TOUCH SCREEN.

          CALL      CONVDATA            ;CONVERT TWO BYTES OF ASCII
                                        ;DATA INTO INTEGER FORM AND
                                        ;STORE AS PARAMETER C.

          CALL      CONVDATA            ;CONVERT NEXT TWO BYTES ASCII
                                        ;DATA INTO INTEGER FORM AND
                                        ;STORE AS NEXT PARAMETER (D).

;THE FOLLOWING SECTION OF PROGRAM CHECKS THAT VALID DATA HAS BEEN
;RECIEVED FROM THE TOUCH SCREEN. IF THE VALUE FOR X IS NOT BETWEEN
;0 AND 120, OR THE VALUE FOR Y IS NOT BETWEEN 0 AND 90, THE LAST VALID
;DATA RECIEVED FROM THE TOUCH SCREEN IS SUBSTITUTED FOR THE PRESENT
;DATA.
          MOV       AX,CS:[PARAM_C]    ;GET VALUE OF X
          CMP       AX,120             ;X < 120 ?
          JLE       TESTD              ;IF YES THEN TEST VALIDITY OF Y 
          CMP       AX,255             ;IF X = 255 THEN THE DATA COULD
          JNE       DATANOK            ;BE INDICATING THAT THE SCREEN
                                       ;IS NOT BEING TOUCHED, THEREFORE
                                       ;CONTINUE WITH VALIDITY CHECK.

TESTD:    MOV       AX,CS:[PARAM_D]    ;GET VALUE OF Y
          CMP       AX,90              ;Y < 90 ?
          JLE       DATAOK             ;IF YES THEN THE DATA RECIEVED
                                       ;FROM THE TOUCH SCREEN IS VALID.
          CMP       AX,255             ;IF Y = 255 THEN THE TOUCH
          JE        DATAOK             ;SCREEN IS INDICATING THAT IT
                                       ;IS NOT BEING TOUCHED AND THE
                                       ;RECIEVED DATA IS STILL VALID.

;THE FOLLOWING SECTION OF PROGRAM IS EXECUTED IF INVALID DATA HAS
;BEEN RECIEVED FROM THE TOUCH SCREEN.

DATANOK:  MOV       AX,CS:[OLDXVAL]    ;FETCH LAST VALID VALUE OF X
          MOV       CS:[PARAM_C],AX    ;SUBSTITUTE FOR PRESENT VALUE
          MOV       AX,CS:[OLDYVAL]    ;FETCH LAST VALID VALUE OF Y
          MOV       CS:[PARAM_D],AX    ;SUBSTITUTE FOR PRESENT VALUE

;TEST TO SEE IF THE TOUCH SCREEN IS INDICATING THAT IT IS NOT BEING
;TOUCHED.

DATAOK:   MOV       AX,CS:[PARAM_C]    ;FETCH VALUE OF X
          CMP       AX,255             ;IF X = 255 THEN THE SCREEN
          JE        NTOUCH             ;IS NOT BEING TOUCHED.
          MOV       AX,CS:[PARAM_D]    ;FETCH VALUE FOR Y
          CMP       AX,255             ;IF Y = 255 THEN THE SCREEN
          JE        NTOUCH             ;IS NOT BEING TOUCHED.

          MOV       CS:[PARAM_A],1H    ;SET PARAMETER A TO INDICATE
                                       ;SCREEN IS TOUCHED.

;THE FOLLOWING SECTION IS EXECUTED IF THE SCREEN IS BEING TOUCHED AND
;VALID DATA HAS BEEN RECEIVED.

          MOV       CS:[THISPASS],1H   ;SET VARIABLE TO INDICATE SCREEN
                                       ;BEING TOUCHED.
          MOV       AX,CS:[PARAM_C]    ;FETCH VALUE OF X
          MOV       CS:[OLDXVAL],AX    ;STORE VALUE FOR NEXT CALL TO
                                       ;FUNCTION 5
          MOV       AX,CS:[PARAM_D]    ;FETCH VALUE OF Y
          MOV       CS:[OLDYVAL],AX    ;STORE VALUE FOR NEXT CALL TO
                                       ;FUNCTION 5
          JMP       MOVPARAM

;THE FOLLOWING SECTION IS EXECUTED IF THE SCREEN IS NOT BEING TOUCHED
;BUT VALID DATA IS STILL BEING RECIEVED.

NTOUCH:   MOV       AL,CS:[LASTPASS]   ;TEST TO SEE IF SCREEN WAS BEING
          CMP       AL,0               ;TOUCHED LAST TIME FUNCTION 5
          JE        MOVPARAM           ;WAS CALLED.

          MOV       CS:[PARAM_B],1H    ;IF YES THEN SET PARAMETER B
                                       ;TO INDICATE A 'BUTTON PRESS'.

MOVPARAM: MOV       AX,CS:[OLDXVAL]    ;FETCH LAST VALID VALUE FOR X
          MOV       CS:[PARAM_C],AX    ;STORE AS PARAMETER C
          MOV       AX,CS:[OLDYVAL]    ;FETCH LAST VALID VALUE FOR Y
          MOV       CS:[PARAM_D],AX    ;STORE AS PARAMETER D

          MOV       AL,CS:[THISPASS]   ;RECORD IF SCREEN WAS TOUCHED
          MOV       CS:[LASTPASS],AL   ;DURING THIS CALL TO FUNCTION 5
                                       ;FOR THE NEXT CALL TO FUNCTION 5

          RET                          ;RETURN TO PARAMETER PASSING ROUTINES.

XYPRESS   ENDP

SIZE      PROC      NEAR

;SIZE RETURNS THE CO-ORDINATE AND SIZE OF THE TOUCHED AREA ON
;THE SCREEN. PARAMETERS A AND B ARE USED TO RETURN THE X AND Y PARTS
;OF THE CO-ORDINATE. PARAMETER C RETURNS THE LENGTH, AND PARAMETER D
;RETURNS THE WIDTH OF THE TOUCHED AREA.

;THIS FUNCTION SHOULD ONLY BE CALLED WITH THE TOUCH SCREEN IN MODE 'M130'.

          CALL      GETDATA            ;GET ASCII DATA FROM TOUCH SCREEN

          MOV       SI,OFFSET TSDATA0  ;POINT TO FIRST BYTE OF ASCII
                                       ;DATA RECIEVED FROM TOUCH SCREEN
          MOV       DI,OFFSET PARAM_A  ;POINT TO PARAMETER A

          CALL      CONVDATA            ;CONVERT TWO BYTES OF ASCII
                                        ;DATA INTO INTEGER FORM AND
                                        ;STORE AS PARAMETER A.

          CALL      CONVDATA            ;CONVERT NEXT TWO BYTES ASCII
                                        ;DATA INTO INTEGER FORM AND
                                        ;STORE AS NEXT PARAMETER (B).


          CALL      CONVDATA            ;CONVERT NEXT TWO BYTES ASCII
                                        ;DATA INTO INTEGER FORM AND
                                        ;STORE AS NEXT PARAMETER (C).


          CALL      CONVDATA            ;CONVERT NEXT TWO BYTES ASCII
                                        ;DATA INTO INTEGER FORM AND
                                        ;STORE AS NEXT PARAMETER (D).

          RET                           ;RETURN TO PARAMETER PASSING ROUTINES.

SIZE      ENDP

MESSAGE   PROC      NEAR

;THIS SUB-ROUTINE SENDS A STRING OF ASCII CHARACTERS TERMINATED BY A
;FULL STOP TO THE TOUCH SCREEN. THE BX REGISTER SHOULD CONTAIN THE
;START ADDRESS OF THE STRING WHEN THE ROUTINE IS CALLED.

NEXTCHAR: MOV       CX,0FFFFH
PAUSE:    LOOP      PAUSE               ;DELAY TO PREVENT CHARACTERS
                                        ;BEING SENT TO THE TOUCH SCREEN
                                        ;TOO FAST.
          MOV       AL,CS:[BX]          ;GET CHARACTER POINTED TO BY BX
          PUSH      AX                  ;SAVE AX

          CALL      SENDCHAR            ;SEND FIRST CHARACTER

          POP       AX                  ;RESTORE AX
          INC       BX                  ;POINT TO NEXT CHARACTER
          CMP       AL,'.'              ;TEST FOR END OF MESSAGE
          JNE       NEXTCHAR            ;IF NOT SEND NEXT CHARACTER

          RET                           ;RETURN TO MAIN ROUTINE

MESSAGE   ENDP

SENDCHAR  PROC      NEAR

;SUB-ROUTINE TO SEND A CHARACTER TO THE TOUCH SCREEN. THE CHARACTER TO
;BE SENT SHOULD BE CONTAINED IN THE AL REGISTER.

          PUSH      AX                  ;SAVE AX

          CALL      RTSON               ;TURN RTS LINE ON

          MOV       DX,3FDH             ;POINT TO UART LINE STATUS REGISTER
          AND       DX,CS:[COMMASK]

REGFULL:  IN        AL,DX               ;GET LINE STATUS
          TEST      AL,20H              ;TEST FOR TRANSMITTER HOLDING
                                        ;REGISTER EMPTY
          JZ        REGFULL             ;IF NOT EMPTY THEN WAIT.

          MOV       DX,3FEH             ;POINT TO MODEM STATUS REGISTER
          AND       DX,CS:[COMMASK]

NOCTS:    IN        AL,DX               ;GET MODEM STATUS
          TEST      AL,10H              ;TEST FOR CLEAR TO SEND SIGNAL
          JZ        NOCTS               ;IF TOUCH SCREEN NOT READY THEN WAIT

          POP       AX                  ;RESTORE AX
          MOV       DX,3F8H             ;POINT TO TRANSMITTER HOLDING
                                        ;REGISTER
          AND       DX,CS:[COMMASK]

          OUT       DX,AL               ;SEND ASCII CHARACTER

          CALL      RTSOFF              ;TURN RTS OFF

          RET                           ;RETURN TO CALLING ROUTINE.

SENDCHAR  ENDP

GETDATA        PROC          NEAR

;SUB-ROUTINE TO FETCH ASCII DATA FROM THE TOUCH SCREEN. DATA BYTES ARE
;READ IN AND STORED IN CONSECUTIVE BYTES ( TSDATA0 TO TSDATA8 ) UNTILL
;A FULL STOP IS RECIEVED AS AN END OF MESSAGE MARKER FROM THE TOUCH
;SCREEN.

;THE SUB-ROUTINE PERFORMS A DOUBLE READ AND DISCARDS THE FIRST SET OF
;DATA TO FLUSH OLD DATA FROM THE TOUCH SCREEN.

          MOV       CX,2H               ;NUMBER OF READS TO BE PERFORMED
DOAGAIN:  MOV       BX,OFFSET TSDATA0   ;POINT TO START OF TOUCH
                                        ;SCREEN DATA STORAGE AREA

NXTCHAR:  CALL      RTSON               ;TURN RTS ON

          MOV       DX,3FDH             ;ADDRESS OF LINE STATUS REGISTER
          AND       DX,CS:[COMMASK]

NODATA:   IN        AL,DX               ;LINE STATUS REGISTER IN AL
          TEST      AL,1H               ;TEST FOR RECEIVED DATA READY BIT
          JZ        NODATA              ;WAIT UNTILL CHARACTER RECEIVED

          CALL      RTSOFF              ;TURN RTS OFF

          MOV       DX,3F8H             ;ADDRESS OF RECEIVER DATA REGISTER
          AND       DX,CS:[COMMASK]

          IN        AL,DX               ;RECEIVED DATA IN AL
          MOV       CS:[BX],AL          ;STORE DATA
          INC       BX                  ;POINT TO NEXT STORAGE SPACE

          CMP       AL,"."              ;COMPARE TO END OF MESSAGE MARKER
          JNE       NXTCHAR             ;IF NOT, READ NEXT CHARACTER

          LOOP      DOAGAIN             ;REPEAT READ PROCESS.

          RET                           ;RETURN TO MAIN ROUTINE.

GETDATA        ENDP

CONVDATA       PROC      NEAR

;THIS ROUTINE  CONVERTS  ASCII DATA RECIEVED FROM THE TOUCH
;SCREEN AND STORED IN TWO CONSECUTIVE BYTES POINTED TO BY THE 
;SI REGISTER AND STORES THE RESULT IN INTEGER FORM IN THE PARAMETER
;STORAGE SPACE POINTED TO BY THE DI REGISTER. 
;SUCCESIVE CALLS TO THE ROUTINE WILL CONVERT AND STORE SUCCESIVE DATA.

          XOR       AH,AH               ;CLEAR REGISTER
          MOV       AL,CS:[SI]          ;GET MSB OF DATA
          SUB       AL,64               ;CONVERT TO INTEGER
          MUL       CS:SIXTEEN          ;AX = AL * 16
          MOV       CS:[DI],AX          ;STORE AS PARAMETER
          INC       SI                  ;POINT TO LSB OF DATA
          MOV       AL,CS:[SI]          ;GET LSB OF DATA
          SUB       AL,64               ;CONVERT TO INTEGER
          ADD       CS:[DI],AX          ;ADD LSB TO MSB
                                        ;AND STORE AS PARAMETER.

          INC       SI                  ;POINT TO NEXT DATA
          INC       DI                  ;POINT TO NEXT PARAMETER.
          INC       DI

          RET                           ;RETURN TO MAIN ROUTINE.

CONVDATA       ENDP

UARTINIT		PROC		NEAR

;THIS SUB-ROUTINE INITIALIZES THE UART TO THE CORRECT PARAMETERS
;FOR USE WITH THE TOUCH SCREEN.

          MOV       DX,3FBH             ;POINT TO LINE CONTROL REGISTER
          AND       DX,CS:[COMMASK]

          MOV       AL,80H              ;SELECT BAUD RATE DIVISOR REGISTERS
          OUT       DX,AL
          MOV       DX,3F8H             ;POINT TO BAUD RATE DIVISOR LSB
          AND       DX,CS:[COMMASK]

          MOV       AL,0CH              ;SET FOR 9600
          OUT       DX,AL
          MOV       DX,3F9H             ;POINT TO BAUD RATE DIVISOR MSB
          AND       DX,CS:[COMMASK]

          MOV       AL,0H               ;SET FOR 9600
          OUT       DX,AL
          MOV       DX,3FBH             ;POINT TO LINE CONTROL REGISTER
          AND       DX,CS:[COMMASK]

          MOV       AL,03H              ;SET FOR 8 DATA BITS
                                        ;        1 STOP BIT
                                        ;        NO PARITY
          OUT       DX,AL
          MOV       DX,3FCH             ;POINT TO MODEM CONTROL REGISTER
          AND       DX,CS:[COMMASK]

          MOV       AL,08H              ;DE-ACTIVATE DTR, RTS AND INTERUPTS
          OUT       DX,AL
          MOV       DX,3F9H             ;POINT TO INTERUPT ENABLE REGISTER
          AND       DX,CS:[COMMASK]

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
          AND       DX,CS:[COMMASK]

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
          AND       DX,CS:[COMMASK]

          MOV       AL,1H               ;SET INACTIVE 'REQUEST TO SEND'
          OUT       DX,AL               ;AND ACTIVE 'DATA TERMINAL READY'
                                        ;CONTROL LINES.
          POP       DX                  ;RESTORE DX
          POP       AX                  ;RESTORE AX

          RET                           ;RETURN TO MAIN ROUTINE.

RTSOFF    ENDP

TSINIT    PROC       NEAR

;THIS SUB-ROUTINE READS PARAMETER B AND INITIALIZES THE TOUCH SCREEN
;TO THE CORRESPONDING MODE OF OPPERATION. PARAMETER B SHOULD BE IN
;THE RANGE 0 ( MODE 0 ) TO 7 ( MODE 131 ).

DATFAIL:  MOV        AX,CS:[PARAM_B]    ;GET PARAMETER B
          XOR        AH,AH              ;CLEAR AH
          MUL        CS:FIVE            ;AX = AL * 5
          MOV        BX,OFFSET MODE0    ;POINT TO START OF DATA STRINGS
          ADD        BX,AX              ;POINT TO REQUIRED DATA STRING

          CALL       MESSAGE            ;SEND MODE INFORMATION DATA STRING
                                        ;TO TOUCH SCREEN

NXTBYTE:  CALL      RTSON               ;TURN RTS ON

          MOV       DX,3FDH             ;POINT TO UART LINE STATUS REGISTER
          AND       DX,CS:[COMMASK]

NDATA:    IN        AL,DX               ;GET LINE STATUS
          TEST      AL,1H               ;TEST FOR RECEIVED DATA READY
          JZ        NDATA               ;IF NOT THEN WAIT
		
          CALL      RTSOFF              ;TURN RTS OFF

          MOV       DX,3F8H             ;POINT TO UART RECEIVER DATA REGISTER
          AND       DX,CS:[COMMASK]

          IN        AL,DX               ;GET RECEIVED DATA

          CMP       AL,"?"              ;IF THE TOUCH SCREEN RESPONDS WITH
          JE        DATFAIL             ;A '?' THEN THE DATA TRANSMITION
                                        ;HAS FAILED AND HAS TO BE REPEATED
                                        ;FROM THE START

          CMP       AL,"."              ;IF THE TOUCH SCREEN RESPONDS WITH
          JNE       NXTBYTE             ;A '.' THEN THE TOUCH SCREEN HAS BEEN
                                        ;INITIALIZED TO A NEW OPERATING
                                        ;MODE. IF ANY OTHER CHARACTER IS
                                        ;RECEIVED, IT IS PART OF A TOUCH
                                        ;DATA TRANSMITION AND SHOULD BE
                                        ;DISGARDED

          RET                           ;RETURN TO MAIN ROUTINE.

TSINIT    ENDP

CODE		ENDS
END		BEGIN
