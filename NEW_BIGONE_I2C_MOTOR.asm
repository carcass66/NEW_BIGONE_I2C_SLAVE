				.include "m328Pdef.inc" 	; Use ATMega328P
				.def temp=R16
				.def del1=R17
				.def del2=R18			
				.def del3=R19
				.def sys=R20
;======================================== Start macro.inc==========================

				.macro    OUTI                          
				LDI    R16,@1        
				.if @0 < 0x40        
				OUT    @0,R16               
				.else        
				STS      @0,R16        
				.endif        
				.endm
				
				.macro	UOUT			; Universal OUT
				.if		@0 < 0x40
				OUT		@0,@1
				.else
				STS		@0,@1
				.endif
				.endm

				.macro	UIN				; Universal IN
				.if		@0 > 0x3f
				LDS		@0,@1
				.else
				IN		@0,@1
				.endif
				.endm

				.macro LDIL				; LDI low
				PUSH R17				; Save high register value in Stack,
				LDI R17,@1				; Load new value to R17,
				MOV @0,R17				; Move value from R17 to low register,
				POP R17					; Return old value to R17
				.endm

				.macro SETB				;SET BIT
				.if @0 < 0x20			; Low IO
				SBI @0,@1
				.else
				.if @0<0x40				; High IO
				IN @2,@0
				ORI @2,1<<@1
				OUT @0,@2
				.else					; Memory
				LDS @2,@0
				ORI @2,1<<@1
				STS @0,@2
				.endif
				.endif
				.endm

				.macro CLRB				;CLEAR BIT
				.if @0 < 0x20			; Low IO
				CBI @0,@1
				.else
				.if @0<0x40				; High IO
				IN @2,@0
				ANDI @2,~(1<<@1)
				OUT @0,@2
				.else					; Memory
				LDS @2,@0
				ANDI @2,~(1<<@1)
				STS @0,@2
				.endif
				.endif
				.endm

;==========================================End macro.inc============================

; RAM ==============================================================================
				.DSEG					; RAM segment
				.ORG SRAM_START+20
mainVar:		.byte	8				;set main variable
encoderVar:		.byte	8				;set variable for encoders counts 

; FLASH ============================================================================
				.CSEG
;==========================================INTERRUPTS VECTORS ======================
				.ORG $000				; (RESET)          
				RJMP	RESET			
				.ORG $002				; (INT0) External Interrupt Request 0   
				RETI					      
				.ORG $004				; (INT1) External Interrupt Request 1
				RETI                      
				.ORG $006				; (PCINT0) Pin Change Interrupt Request 0
				RETI                        
				.ORG $008				; (PCINT1) Pin Change Interrupt Request 1
				RETI                     
				.ORG $00A				; (PCINT2) Pin Change Interrupt Request 2
				RETI
				.ORG $00C				; (WDT) Watchdog Time-out Interrupt
				RETI
				.ORG $00E				; (TIMER2_COMPA) Timer/Counter2 Compare Match A
 				RETI
				.ORG $010				; (TIMER2_COMPB) Timer/Counter2 Compare Match B
				RETI
				.ORG $012				; (TIMER2_OVF) Timer/Counter2 Overflow
				RETI
				.ORG $014				; (TIMER1_CAPT) Timer/Counter1 Capture Event
				RETI
				.ORG $016				; (TIMER1_COMPA) Timer/Counter1 Compare Match A
				RETI
				.ORG $018				; (TIMER1_COMPB) Timer/Counter1 Compare Match B
				RETI
				.ORG $01A				; (TIMER1_OVF) Timer/Counter1 Overflow
				RETI
				.ORG $01C				; (TIMER0_COMPA) Timer/Counter0 Compare Match A
				RETI
				.ORG $01E				; (TIMER0_COMPB) Timer/Counter0 Compare Match B
				RETI
				.ORG $020				; (TIMER0_OVF) Timer/Counter0 Overflow
				RETI
				.ORG $022				; (SPI STC) SPI Serial Transfer Complete
				RETI
				.ORG $024				; (USART_RX) USART Rx Complete
				RETI
				.ORG $026				; (USART_UDRE) USART Data Register Empty
				RETI
				.ORG $028				; (USART_TX) USART Tx Complete
				RETI
				.ORG $02A				; (ADC) ADC Conversion Complete
				RETI
				.ORG $02C				; (EE READY) EEPROM Ready
				RETI
				.ORG $02E				; (ANALOG COMP) Analog Comparator
				RETI
				.ORG $030				; (TWI) 2-wire Serial Interface (I2C)
				RJMP	INT_I2C	
				.ORG $032				; (SPM READY) Store Program Memory Ready
				RETI
				.ORG INT_VECTORS_SIZE
;====================================================================================
RESET:			CLI
RAM_FLUSH:		LDI ZL, Low(SRAM_START)	;Clean RAM and all registers
				LDI ZH, High(SRAM_START)
				CLR temp
FLUSH:			ST Z+, temp
				CPI ZH, High(RAMEND+1)
				BRNE FLUSH
				CPI Zl, Low(RAMEND+1)
				BRNE FLUSH
				CLR ZL
				CLR ZH
				CLR R0
				CLR R1
				CLR R2
				CLR R3
				CLR R4
				CLR R5
				CLR R6
				CLR R7
				CLR R8
				CLR R9
				CLR R10
				CLR R11
				CLR R12
				CLR R13
				CLR R14
				CLR R15
				CLR R16
				CLR R17
				CLR R18
				CLR R19
				CLR R20
				CLR R21
				CLR R22
				CLR R23
				CLR R24
				CLR R25
				CLR R26
				CLR R27
				CLR R28
				CLR R29	
				LDI temp, Low(RAMEND)	;Stack init
				OUT SPL, temp
				LDI temp, High(RAMEND)        
				OUT SPH, temp
				LDI temp, 0x32			;Set I2C slave address
				STS TWAR, temp

;====================================================================================
STOP:			CLI						;Disable interrupts
				;STOP ALL MOTORS

				;==========================================
				SETB DDRB,0					;Set PB0 as output
				SETB DDRB,2					;Set PB2 as output
				SETB DDRB,3					;Set PB3 as output
				SETB DDRB,4					;Set PB4 as output
				CLRB PORTB,0				;PB0 is LOW
				CLRB PORTB,2				;PB2 is LOW
				CLRB PORTB,3				;PB3 is LOW
				CLRB PORTB,4				;PB4 is LOW
				RJMP IDLE				;Return to loop
;=====================================================================================================
INT_I2C:		LDS temp, TWSR			;Check and clean(cut two low bits) the status register
				ANDI temp, 0xF8			;
				CPI temp, 0x60			;If receive our address to write(we need to receive data from master)
				BREQ SLA_W				;Go to SLA_W lable
				CPI temp, 0x70			;If receive broadcast
				BREQ SLA_W				;Go to SLA_W lable
				CPI temp, 0x80			;If receive byte
				BREQ BYTE				;Go to BYTE lable			
				CPI temp, 0x90			;If receive broadcast byte
				BREQ BYTE				;Go to BYTE lable
				CPI temp, 0x88			;If receive last byte
				BREQ LAST_BYTE			;Go to LAST_BYTE lable
				CPI temp, 0x98			;If receive last broadcast byte
				BREQ LAST_BYTE			;Go to LAST_BYTE lable
				CPI temp, 0xA0			;If receive Restart
				BREQ SLA_W				;Go to SLA_W lable
				CPI temp, 0xA8			;If receive our address to read(we need to send data to master)
				BREQ SLA_R				;Go to SLA_R lable
				CPI temp, 0xB8
				BREQ S_BYTE_R_ACK		;Go to S_BYTE_R_ACK lable
				CPI temp, 0xC0
				BREQ S_LBYTE_R_NACK	;Go to S_LBYTE_R_NACK
				CPI temp, 0xC8
				BREQ S_LBYTE_R_ACK	;Go to S_LBYTE_R_ACK
				RETI
SLA_W:			LDI temp, 0b11000101	;Send ACK
				STS TWCR, temp
				RJMP Vix
BYTE:			LDS temp, TWDR			;Load byte to temp
				RCALL ByteReceive
				RJMP Vix
ByteReceive:	ST -Y, temp
				CPI sys,6
				BREQ ByteReceiveNack
				LDI temp, 0b11000101
				STS TWCR, temp
				INC sys
				RET
ByteReceiveNack:LDI temp, 0b10000101
				STS TWCR, temp
				RJMP Vix
LAST_BYTE:		LDS temp, TWDR
				ST -Y, temp
				LDI temp, 0b10001000		;RECHECK!!!!!!!
				STS TWCR, temp
				RJMP Vix
SLA_R:			NOP
				RJMP Vix
S_BYTE_R_ACK:	NOP
				RJMP Vix
S_LBYTE_R_NACK:	NOP
				RJMP Vix
S_LBYTE_R_ACK:	NOP
				RJMP Vix
Vix:			SEI
				RETI				
;======================================================================================================				
IDLE:			CLI
				LDI YH, high(mainVar)		;Load mamory address of mainVar to Y registers
				LDI YL, low(mainVar)
				LDI temp, 0x00
				ST Y+,temp
				ST Y+,temp
				ST Y+,temp
				ST Y+,temp
				ST Y+,temp
				ST Y+,temp
				ST Y+,temp
				ST Y+,temp
				LDI sys,0
				RCALL TWI_Init				;Enable I2C interrupts
				SEI							;Enable interrupts(global)
				LDI del1, 0x0F				;Delay
DELAY1:			dec del1
				CPI del1, 0x00
				BREQ DEL_OUT
				LDI del2, 0xFF
DELAY2:			dec del2
				CPI del2, 0x00
				BREQ DELAY1
				LDI del3, 0xFF
DELAY3:			dec del3
				CPI del3, 0x00
				BREQ DELAY2
				RJMP DELAY3
DEL_OUT:		CLI							;Disable interrupts(global)
				LDI YH, high(mainVar)		;Load mamory address of mainVar to Y registers
				LDI YL, low(mainVar)		;Disable interrupts (global)
				LD temp, Y+					;Check command(high bit of mainVar)
				CPI temp, 0b10000000		;If command is "10000000"
				BREQ FWARD_T				;Go to FWARD lable
				CPI temp, 0b00000001		;If command is '00000001' 
				BREQ BACKWARD_T				;Go to BACKWARD lable
				CPI temp, 0b00000000		;If command is '00000000'
				BREQ STOP_T					;Go to STOP lable
				CPI temp, 0b00010000		;If command is '00010000' 
				BREQ LEFT_T					;Go to LEFT lable
				CPI temp, 0b00001000		;If command is '00001000'
				BREQ RIGHT_T				;Go to RIGHT lable
				CPI temp, 0b10010000		;If comand is '10010000'
				BREQ FWARD_L_T				;Go to FWARD_L
				CPI temp, 0b10001000		;If comand is '10001000'
				BREQ FWARD_R_T				;Go to FWARD_R
				CPI temp, 0b00010001		;If comand is '00010001'
				BREQ BACKWARD_L_T			;Go to BACKWARD_L
				CPI temp, 0b00001001		;If comand is '00001001'
				BREQ BACKWARD_R_T			;Go to BACKWARD_R
				RJMP IDLE

FWARD_T:		RJMP FWARD
BACKWARD_T:		RJMP BACKWARD
STOP_T:			RJMP STOP
LEFT_T:			RJMP LEFT
RIGHT_T:		RJMP RIGHT
FWARD_L_T:		RJMP FWARD_L
FWARD_R_T:		RJMP FWARD_R
BACKWARD_L_T:	RJMP BACKWARD_L
BACKWARD_R_T:	RJMP BACKWARD_R


TWI_Init:		LDI temp, 0b01000101
				STS TWCR, temp
				RET
;====================================================================================
FWARD:			CLI							;Disable interrupts(global)
				;Set side of rotation of MOTORSC
				;Set PWM date from mainVar
				SETB DDRB,0					;Set PB0 as output
				SETB DDRB,2					;Set PB2 as output
				SETB DDRB,3					;Set PB3 as output
				SETB DDRB,4					;Set PB4 as output
				SETB PORTB,0				;PB0 is HIGH
				CLRB PORTB,2				;PB2 is LOW
				CLRB PORTB,3				;PB3 is LOW
				CLRB PORTB,4				;PB4 is LOW
				RJMP IDLE					;Go to IDLE
BACKWARD:		CLI							;Disable interrupts(global)
				;Set side of rotation of MOTORS
				;Set PWM date from mainVar
				SETB DDRB,0					;Set PB0 as output
				SETB DDRB,2					;Set PB2 as output
				SETB DDRB,3					;Set PB3 as output
				SETB DDRB,4					;Set PB4 as output
				CLRB PORTB,0				;PB0 is LOW
				SETB PORTB,2				;PB2 is HIGH
				CLRB PORTB,3				;PB3 is LOW
				CLRB PORTB,4				;PB4 is LOW
				RJMP IDLE					;Go to IDLE
LEFT:			CLI							;Disable interrupts(global)
				;Set side of rotation of right MOTORS
				;Set PWM date from mainVar
				SETB DDRB,0					;Set PB0 as output
				SETB DDRB,2					;Set PB2 as output
				SETB DDRB,3					;Set PB3 as output
				SETB DDRB,4					;Set PB4 as output
				CLRB PORTB,0				;PB0 is LOW
				CLRB PORTB,2				;PB2 is LOW
				SETB PORTB,3				;PB3 is HIGH
				CLRB PORTB,4				;PB4 is LOW
				RJMP IDLE					;Go to IDLE
RIGHT:			CLI							;Disable interrupts(global)
				;Set side of rotation of left MOTORS
				;Set PWM date from mainVar
				SETB DDRB,0					;Set PB0 as output
				SETB DDRB,2					;Set PB2 as output
				SETB DDRB,3					;Set PB3 as output
				SETB DDRB,4					;Set PB4 as output
				CLRB PORTB,0				;PB0 is LOW
				CLRB PORTB,2				;PB2 is LOW
				CLRB PORTB,3				;PB3 is LOW
				SETB PORTB,4				;PB4 is HIGH
				RJMP IDLE					;Go to IDLE

FWARD_L:		CLI							;Disable interrupts(global)
				;Set side of rotation of MOTORS
				;Set PWM date from mainVar
				SETB DDRB,0					;Set PB0 as output
				SETB DDRB,2					;Set PB2 as output
				SETB DDRB,3					;Set PB3 as output
				SETB DDRB,4					;Set PB4 as output
				SETB PORTB,0				;PB0 is HIGH
				CLRB PORTB,2				;PB2 is LOW
				SETB PORTB,3				;PB3 is HIGH
				CLRB PORTB,4				;PB4 is LOW
				RJMP IDLE					;Go to IDLE
FWARD_R:		CLI							;Disable interrupts(global)
				;Set side of rotation of MOTORS
				;Set PWM date from mainVar
				SETB DDRB,0					;Set PB0 as output
				SETB DDRB,2					;Set PB2 as output
				SETB DDRB,3					;Set PB3 as output
				SETB DDRB,4					;Set PB4 as output
				SETB PORTB,0				;PB0 is HIGH
				CLRB PORTB,2				;PB2 is LOW
				CLRB PORTB,3				;PB3 is LOW
				SETB PORTB,4				;PB4 is HIGH
				RJMP IDLE					;GO to IDLE
BACKWARD_L:		CLI							;Disable interrupts(global)
				;Set side of rotation of MOTORS
				;Set PWM date from mainVar
				SETB DDRB,0					;Set PB0 as output
				SETB DDRB,2					;Set PB2 as output
				SETB DDRB,3					;Set PB3 as output
				SETB DDRB,4					;Set PB4 as output
				CLRB PORTB,0				;PB0 is LOW
				SETB PORTB,2				;PB2 is HIGH
				SETB PORTB,3				;PB3 is HIGH
				CLRB PORTB,4				;PB4 is LOW
				RJMP IDLE					;Go to IDLE
BACKWARD_R:		CLI							;Disable interrupts(global)
				;Set side of rotation of MOTORS
				;Set PWM date from mainVar
				SETB DDRB,0					;Set PB0 as output
				SETB DDRB,2					;Set PB2 as output
				SETB DDRB,3					;Set PB3 as output
				SETB DDRB,4					;Set PB4 as output
				CLRB PORTB,0				;PB0 is LOW
				SETB PORTB,2				;PB2 is HIGH
				CLRB PORTB,3				;PB3 is LOW
				SETB PORTB,4				;PB4 is HIGH
				RJMP IDLE					;Go to IDLE

; EEPROM ===============================================
		.ESEG					; EEPROM segment
