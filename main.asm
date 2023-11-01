;
; Project_1_Team26.asm
;
; Created: 10/18/2023 3:06:34 PM
; Author : Team 26 - Seth Sievers, Carson Williams, Garrett Herman, Dawson Ploudre 
;

/* ----------- INTERRUPT VECTORS ----------- */
; this project utilizes interrupts, this populates the vector table
.LIST
.ORG 0x0000
	RJMP SETUP
.ORG 0x001A
	JMP DELAY_T1_ISR
.ORG 0x0020 
	JMP DELAY_T0_OCRA_ISR
.ORG 0x0024
	JMP DELAY_T0_OVF_ISR
.ORG 0x0040
	JMP BUZZER_T3_ISR
/* ----------------------------------------- */


; ------------------------------------------------------------------------------------------------------
; ----------------------------------------------- "MAIN" -----------------------------------------------
; ------------------------------------------------------------------------------------------------------

; setup code runs once at the beginning 
SETUP:
	; Move the Stack Pointer
	LDI R16, HIGH(RAMEND)
	OUT SPH, R16
	LDI R16, LOW(RAMEND)
	OUT SPL, R16

	; Initialize the Timers 
	CALL DELAY_T1_INIT
	CALL DISPLAY_T0_INIT
	CALL BUZZER_T3_INIT

	; Port Configuration 
	LDI R16, 0x00
	OUT DDRA, R16 ;setup all switch pins as inputs
	LDI R16, 0xFF
	OUT PORTD, R16 ;turn off all LEDs
	OUT DDRD, R16 ;LED pins are all outputs
	OUT PORTA, R16; enable all pullups
	SBI DDRE, 4 ;buzzer pin is output
	MOV R15, R16

	; Set values to loop from 0 to 30 or 30 to 0
	LDI R17, 31; R17 = 31, make sure < 31
	LDI R18, 255; R18 = 255, make sure not 255 

	LDI R16, 0x00; initialize counter
	LDI R20, 0; initialize fibo
	LDI R21, 1; initialize fibo

; loop code runs continuously 
LOOP: 
		COM R16; complement since LEDs are active low
		MOV R15, R16; output bit value to LEDs
		COM R16; complement again to make normal value
		SBIS PINA, 0; check if bit 0 inside PINA is 0, inc if so
		RJMP sw1_inc; 
		SBIS PINA, 1; check if bit 1 inside PINA is 0, dec if so
		RJMP sw2_dec;
		SBIS PINA, 2
		RJMP sw3_fibo
		SBIS PINA, 3
		RJMP sw4_reset
		RJMP LOOP;

sw1_inc:		INC R16; increment our counter
			CALL DELAY_100_MS; call delay function
			;SBI PINA, 0; reset switch back to 1
			CP R17, R16; compare registers to make sure R16 is not 31
			BREQ over; reset to 0 if >= 31
			RJMP LOOP; jump back to start

sw2_dec:		DEC R16; decrement our counter
			CALL DELAY_100_MS; call delay function
			;SBI PINA, 1; reset switch back to 1
			CP R18, R16; compare registers to make sure R16 is not 255
			BREQ under; set to 30 if = 255 (looped backwards)
			RJMP LOOP; jump back to start
sw3_fibo:
			ADD R20, R21;Add R20 and R21
			MOV R16, R20
			CALL DELAY_100_MS; call delay function
			CALL DELAY_100_MS; call delay function
			MOV R20, R21;store R21 to R20
			MOV R21, R16
			RJMP LOOP;jump back to start
sw4_reset:
		LDI R16, 0x00; reset R16
		CALL DELAY_100_MS; call delay function
		CALL DELAY_100_MS; call delay function
		RJMP LOOP; jump back to start


		  

over:	LDI R16, 0x9B; 1500Hz tone will be 0x029B
		PUSH R16; send low byte of value to stack
		LDI R16, 0x02
		PUSH R16; send high byte of value to stack
		CALL BUZZER_SET_FREQ
		CALL BUZZER_START
		LDI R16, 100
		PUSH R16
		CALL DELAY_10_MS
		CALL BUZZER_STOP
		LDI R16, 0
		LDI R20, 0
		LDI R21, 1
		RJMP LOOP

under:	LDI R16, 0xD0; 500Hz tone will be 0x07D0
		PUSH R16; send low byte of value to stack
		LDI R16, 0x07
		PUSH R16; send high byte of value to stack
		CALL BUZZER_SET_FREQ
		CALL BUZZER_START
		LDI R16, 100
		PUSH R16
		CALL DELAY_10_MS
		CALL BUZZER_STOP
		LDI R16, 30;
		RJMP LOOP;


;------------------------------------------------------------------------------------------------------------
; ----------------------------------------------- SUBROUTINES -----------------------------------------------
;------------------------------------------------------------------------------------------------------------

/* --------------------------------- DELAY SUBROUTINES --------------------------------- */
;Author: Seth Sievers
;Team Number: 26
;These subroutines will keep track of the current time and implement a basic variable 
;	delay subroutine. 

; DELAY_T1_INIT
;Purpose: This subroutine configures the 16 bit timer 1 hardware 
;Arguments: None 
;Returns: Nothing 
DELAY_T1_INIT:
	PUSH R16; using R16 so push to stack to ensure previous value is saved
	CLI
	LDI R16, 0b00000000
	MOV R11, R16; set the low byte of the 16 bit time value to 0 
	MOV R12, R16; set the high byte of the 16 bit time value to 0 
	STS TCCR1A, R16 ; disables compare output pins, and sets WGM[1:0] bits to CTC 
	LDI R16, 0b00001000
	STS TCCR1B, R16; disable noise canceler, set WGM[3:2] to CTC, disable timer clock
	CLR R16 
	STS TCNT1H, R16; clear high byte of timer value 
	STS TCNT1L, R16; clear low byte of timer value
	LDI R16, 0x4E
	STS OCR1AH, R16; write high byte of compare value 
	LDI R16, 0x20; 
	STS OCR1AL, R16; write low byte of compare value, final value should be 20,000
	LDI R16, 0b00000010
	STS TIMSK1, R16; enable only OC1 interrupt
	LDI R16, 0b00001010
	STS TCCR1B, R16; disable noise canceler, set WGM[3:2] to CTC, set 8x prescaler
	SEI; enable interrupts 
	POP R16; restore the previous value of R16
	RET 

;DELAY_T1_ISR
;Purpose: This ISR will increment the time counter, and dec input debounce timer
;Args: None
;Returns: Nothing 
DELAY_T1_ISR:
	IN R8, SREG; store the SREG so that it is not changed by ISR 
	;R11: low byte, R12: high byte 
	INC R11 ;if R11 is full, DOES NOT AFFECT C
	TST R11; if R11 overflowed then R11 is 0, thus Z = 1
	BRNE DELAY_TIME_NO_OVERFLOW
	INC R12 ;if c = 1, carry into R12 by incrementing 
	DELAY_TIME_NO_OVERFLOW:
		TST R14
		BREQ DELAY_DEBOUNCE_ALREADY_ZERO
		DEC R14; if R14 is not zero, dec it as 10ms has passed 
		DELAY_DEBOUNCE_ALREADY_ZERO: 
			OUT SREG, R8; restore SREG value 
			RETI ; return from interrupt 

;DELAY_10_MS
;Purpose: This subroutine is a delay function that will waste cycles until the specified time has elapsed
;	Arguments should be pushed onto the stack in the order they appear in the below list
;Arguments: DelayTime: Byte specifying how many 10's of ms to wait 
;Returns: Nothing
DELAY_10_MS:
	POP R9; store the first byte of return address
	POP R10; store the second byte of return address
	POP R13; store the argument in the temp register so it is not buried beneath saved GPRs
	PUSH R16; save value for later restoring
	PUSH R18; save value for later restoring 
	MOV R16, R11; move the initial low byte into R16 to compare with
	DELAY_10_MS_LOOP:
		MOV R18, R11; save low byte for subtraction 
		SUB R18, R16; subtract initial low byte from current low byte 
		CP R18, R13; if R13 is greater than R19, C is set
		BRCS DELAY_10_MS_LOOP; when C is cleared, delay has passed, if C set, branch back to loop 
	POP R18; restore R18
	POP R16; restore R16 previous value 
	PUSH R10; restore return address
	PUSH R9; restore return address
	RET

DELAY_100_MS: 
	LDI R19, 10
	PUSH R19
	CALL DELAY_10_MS
	RET

;CHECK_INPUT
;Purpose: This subroutine utilizes the timer 1 hardware to debounce PINA switch inputs without
;	blocking the main loop
;NOTE: Due to the implementation, all inputs share the same debounce counter R14, to support
;	multiple concurrent inputs, the subroutine will have to be adjusted
;	To pass Arguments push the value on the stack in the same order specified
;Arguments: BIT: Bit position number in PINA specified as a byte
;Returns: Nothing, Z IS SET IF THE VALUE IN PINA IS 0!!!, If Z is cleared, Read is invalid -> Take No Action 
CHECK_INPUT:
	POP R9; store the first byte of return address
	POP R10; store the second byte of return address
	POP R13; save the argument 
	TST R14
	BRNE CHECK_INPUT_END ; if R14 is not 0, debounce delay has not elapsed, since Z:0 means invalid, this will return with Z cleared 
	PUSH R16
	PUSH R17
	IN R16, PINA ; read in the pin values 
	TST R13 ;if bit 0, no shifting is necessary 
	BREQ CHECK_INPUT_NO_SHIFT
	CHECK_INPUT_LOOP: ;loop and shift the pin values until the rightmost bit is the one desired
		LSR R16
		DEC R13
		BRNE CHECK_INPUT_LOOP
	CHECK_INPUT_NO_SHIFT:  
		CBR R16, 0b11111110 ;since rightmost bit is bit of interest, this will clear all undesired bits
		TST R16 ;if pin is 0, this sets z flag 
		BRNE CHECK_INPUT_SWITCH_NOT_PRESSED
		LDI R17, 7
		MOV R14, R17; sets the debounce delay to 5*10ms = 50ms 
		CHECK_INPUT_SWITCH_NOT_PRESSED: 
			POP R17
			POP R16	
	CHECK_INPUT_END:
		PUSH R10; restore return address
		PUSH R9; restore return address
		RET
/* ------------------------------------------------------------------------------------- */

/* --------------------------------- DISPLAY SUBROUTINES --------------------------------- */
;Author: Seth Sievers
;Team Number: 26
;These Subroutines will be used for my individual contribution to the project which is 
;	a brightness control for the counter display 

;DISPLAY_T0_INIT
;Purpose: This subroutine configures the 8 bit timer 0 hardware 
;Arguments: None
;Returns: Nothing 
DISPLAY_T0_INIT:
	PUSH R16
	LDI R16, 0b00000000
	OUT TCCR0A, R16 ;normal mode, no output pins
	LDI R16, 0b00000000
	OUT TCCR0B, R16 ; Halt Prescaler
	LDI R16, 0b00000011
	STS TIMSK0, R16 ;enable output compare and overflow int
	CLR R16
	OUT TCNT0, R16 ;set current timer count to 0
	LDI R16, 0xFF
	MOV R7, R16 ;set the initial display brightness to 100%
	OUT OCR0A, R7 ;set the output compare value to the brightness value
	LDI R16, 0b00000100
	OUT TCCR0B, R16 ; prescaler of 256 -> 245Hz overflow (PWM FREQ)
	POP R16
	RET

;DELAY_T0_OVF_ISR
;Purpose: This ISR will set PORTD to the value stored in R15, This turns on the display
;Args: None
;Returns: Nothing
DELAY_T0_OVF_ISR:
	OUT PORTD, R15 ;SREG not changed so save cycles
	RETI

;Delay_T0_OCRA_ISR
;Purpose: This ISR will set all bits in PORTD, This disables the display 
;Args: None
;Returns: Nothing 
DELAY_T0_OCRA_ISR:
	IN R8, SREG ;save SREG
	CLR R7 ;cant SER into <16 REG
	COM R7
	OUT PORTD, R7;set PORTD to all 1's, disabling all LED
	OUT SREG, R8 ;Restore SREG
	RETI
/* --------------------------------------------------------------------------------------- */

/* --------------------------------- BUZZER SUBROUTINES --------------------------------- */
;Author: Seth Sievers
;Team Number: 26
;These Subroutines will configure and support the operation of Timer 3 which will be used for 
;	generating a specific frequency signal for the buzzer operation 


; BUZZER_T3_INIT
;Purpose: This subroutine configures the 16 bit timer 3 hardware 
;Arguments: None 
;Returns: Nothing 
BUZZER_T3_INIT:
	PUSH R16; using R16 so push to stack to ensure previous value is saved
	CLI ;disable interrupts
	LDI R16, 0b00000000
	STS TCCR3A, R16 ; disables compare output pins, and sets WGM[1:0] bits to CTC 
	LDI R16, 0b00001000
	STS TCCR3B, R16; disable noise canceler, set WGM[3:2] to CTC, disable timer clock
	CLR R16 
	STS TCNT3H, R16; clear high byte of timer value 
	STS TCNT3L, R16; clear low byte of timer value
	LDI R16, 0b00000010
	STS TIMSK3, R16; enable only OC1 interrupt
	CLR R16
	MOV R6, R16; set the buzzer state to off
	SEI; enable interrupts 
	POP R16; restore the previous value of R16
	RET 

; BUZZER_STOP
;Purpose: This subroutine will stop the buzzer from making noise and halt the timer
;Arguments: None
;Returns: Nothing 
BUZZER_STOP:
	PUSH R16
	LDI R16, 0b00001000
	STS TCCR3B, R16; disable noise canceler, set WGM[3:2] to CTC, disable timer clock
	POP R16
	RET

; BUZZER_START
;Purpose: This subroutine will start the timer and reenable the buzzer 
;Arguments: None
;Returns: Nothing 
BUZZER_START: 
	PUSH R16
	LDI R16, 0b00001010
	STS TCCR3B, R16; disable noise canceler, set WGM[3:2] to CTC, set 8x prescaler
	POP R16
	RET

; BUZZER_SET_FREQ
;Purpose: This subroutine changes the buzzer frequency (does not stop or start the buzzer)
;Arguments: 16 bit number pushed onto the stack specifying frequency PUSH LOW_BYTE then PUSH HIGH_BYTE
;	The number for a given frequency can be calculated with the following formula
;	NUMBER = (1/(FREQ*2))*(2*10^6)
BUZZER_SET_FREQ:
	POP R9 ;save return address 
	POP R10
	POP R13 ;save HIGH_BYTE of number
	STS OCR3AH, R13; write high byte of compare value 
	POP R13 ;save LOW_BYTE of freq number
	STS OCR3AL, R13; write low byte of compare value 
	PUSH R10 ;restore return address
	PUSH R9 
	RET

; BUZZER_T3_ISR
; Purpose: This ISR will toggle the buzzer on and off
; Arguments: None
; Returns: Nothing 
BUZZER_T3_ISR:
	IN R8, SREG ;save SREG
	TST R6 ;if z is not set, set PE4, else clear PE4
	BRNE BUZZER_CLEAR
	CBI PORTE, 4
	RJMP BUZZER_RETURN
	BUZZER_CLEAR:
		SBI PORTE, 4
	BUZZER_RETURN:
		COM R6 ;invert R6 so next time it is toggled it flips
		OUT SREG, R8 ;Restore SREG
		RETI
/* -------------------------------------------------------------------------------------- */