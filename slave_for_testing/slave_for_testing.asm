		.include "m328Pdef.inc" 	; Use ATMega328P
		.def temp = R16
		.def end = R17
		.def err = R18
		.def sys = R19
		.def addr = R20

		
		.equ FREQ = 8000000
		.equ FreqSCL = 400000
		.equ FreqTWBR = ((FREQ/FreqSCL)-16)/2
;======================================== Start macro.inc==========================
.MACRO SETB							;Set bit
.if @0 < 0x20 ; Low IO
SBI @0,@1
.else
.if @0<0x40 ; High IO
IN @2,@0
ORI @2,1<<@1
OUT @0,@2
.else ; Memory
LDS @2,@0
ORI @2,1<<@1
STS @0,@2
.endif
.endif
.ENDM

.MACRO CLRB							;Clear bit
.if @0 < 0x20 ; Low IO
CBI @0,@1
.else
.if @0<0x40 ; High IO
IN @2,@0
ANDI @2,~(1<<@1)
OUT @0,@2
.else ; Memory
LDS @2,@0
ANDI @2,~(1<<@1)
STS @0,@2
.endif
.endif
.ENDM
	

;==========================================End macro.inc============================

; RAM ==============================================================================
		.DSEG
		.ORG SRAM_START+40					; RAM segment
		VAR_SLV_1:	.byte 8
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
		.ORG 0x0030				; (TWI) 2-wire Serial Interface (I2C)
		JMP INT_I2C	
		.ORG $032				; (SPM READY) Store Program Memory Ready
		RETI
		.ORG INT_VECTORS_SIZE
;====================================================================================
RESET:		LDI temp,Low(RAMEND)
		OUT SPL,temp
		LDI temp,High(RAMEND)        
		OUT SPH,temp
		
		LDI temp, FreqTWBR
		STS TWBR,temp
		
		LDI temp,0x00		
		OUT DDRB, temp
		;LDI temp,0b00001111
		;OUT PORTB,temp			
		NOP
		SETB PORTB,0,temp	;PULL-UP
		SETB PORTB,1,temp	;PULL-UP
		SETB PORTB,2,temp	;PULL-UP
		SETB PORTB,3,temp	;PULL-UP
		jmp MAIN
;====================================================================================
INT_I2C:	CLI
		IN temp, SREG
		PUSH temp
		LDS temp,TWSR
		ANDI temp, 0xF8
		CPI temp, 0x00
		BREQ BUS_FAIL
		CPI temp, 0x08
		BREQ SLAW_Adr
		CPI temp, 0x10
		BREQ SCND_START
		CPI temp, 0x18
		BREQ SLA_W_ACK
		CPI temp, 0x20
		BREQ SLA_W_NACK			
		CPI temp, 0x28
		BREQ BYTE_ACK
		CPI temp, 0x30
		BREQ BYTE_NACK
		CPI temp, 0x38
		BREQ COLLIS
		CPI temp, 0x40
		BREQ SLA_R_ACK
		CPI temp, 0x48
		BREQ SLA_R_NACK
		CPI temp, 0x50
		BREQ REC_BYTE
		CPI temp, 0x58
		BREQ REC_BYTE_NACK
		RJMP Vix
BUS_FAIL: RCALL TWI_Stop
		RJMP Vix
SCND_START:	RCALL TWI_Stop
		RJMP Vix			
SLAW_Adr:	MOV temp,addr
		RCALL TWI_SendByte
		RJMP Vix
SLA_W_ACK:	LD temp,-Y
		inc sys
		RCALL TWI_SendByte
		RJMP Vix
SLA_W_NACK:	RCALL TWI_Stop
		RJMP Vix
BYTE_ACK:	RJMP NextByteSend
BYTE_NACK:	RCALL TWI_Stop
		ldi end, 0x05			
		RJMP Vix
COLLIS:		RCALL TWI_Stop					
		RJMP Vix
SLA_R_ACK:	RCALL TWI_ReceiveAck
		RJMP Vix
SLA_R_NACK:	RCALL TWI_Stop					
		RJMP Vix
REC_BYTE:	LDS temp,TWDR
		ST Y+,temp 			
		inc sys
		CPI sys,7
		BREQ Byte_REC_NACK
		RCALL TWI_ReceiveAck
		RJMP Vix
REC_BYTE_NACK:	RJMP Stopp				
		RJMP Vix
Vix:	SEI	
		POP temp
		OUT SREG, temp
		RETI
Byte_REC_NACK:	LDI temp, 0b10000101
		STS TWCR, temp
		RJMP Vix
Stopp:	LDI temp, TWDR
		ST Y+,temp
		RCALL TWI_Stop
		RJMP Vix
TWI_SendByte:	STS TWDR,temp
		LDI temp, 0b10000101
		STS TWCR,temp
		RET
NextByteSend:	CPI sys,8
		BREQ ByteSendStop
		LD temp,-Y
		RCALL TWI_SendByte
		inc sys
		RJMP Vix			
ByteSendStop:	RCALL TWI_Stop
		RJMP Vix
TWI_ReceiveAck: LDI temp, 0b11000101
		STS TWCR, temp
		RET
TWI_Stop:	LDI temp,0b10010100
		STS TWCR,temp
		ldi err, 0x01
		RET
		
;==================================================================================
MAIN:		CLI
		sbis pinb, 0
		RJMP Front
		sbis pinb, 1
		RJMP Back
		sbis pinb, 2
		RJMP Right
		sbis pinb, 3
		RJMP Left
		RJMP Stop
		
I2C_SEND:ldi sys,0
		ldi end, 0x02
		ldi err, 0x00
		ldi YL, low(VAR_SLV_1)
		ldi YH, high(VAR_SLV_1)
		ST Y+,temp
		ldi temp,0b11111111
		ST Y+,temp
		ST Y+,temp
		ST Y+,temp
		ST Y+,temp
		ST Y+,temp
		ST Y+,temp
		ST Y+,temp
		ldi temp, 0b10100101		;TWINT=1, TWSTA=1, TWEN=1, TWIE=1
		STS TWCR,temp
		SEI
DELAY:	cpi end, 0x05
		breq DELAY_EXIT
;		cpi err, 0x01
;		breq I2C_SEND
		rjmp DELAY
DELAY_EXIT:	RET
Front:		CLI
		ldi addr, 0x32
		ldi temp,0b10000000
		RCALL I2C_SEND
		RJMP MAIN
Back:		CLI
		ldi addr, 0x32
		ldi temp,0b00000001
		RCALL I2C_SEND
		RJMP MAIN
Right:		CLI
		ldi addr, 0x32
		ldi temp,0b00001000
		RCALL I2C_SEND
		RJMP MAIN
Left:		CLI
		ldi addr, 0x32
		ldi temp,0b00010000
		RCALL I2C_SEND
		RJMP MAIN
Stop:		CLI
		ldi addr, 0x32
		ldi temp,0b00000000
		RCALL I2C_SEND
		RJMP MAIN
;====================================================================================
; EEPROM ===============================================
		.ESEG
							; EEPROM segment
