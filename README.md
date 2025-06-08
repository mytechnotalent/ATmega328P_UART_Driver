<img src="https://raw.githubusercontent.com/mytechnotalent/ATmega328P_UART_Driver/refs/heads/main/ATmega328P%20UART%20Driver.png">

## FREE Reverse Engineering Self-Study Course [HERE](https://github.com/mytechnotalent/Reverse-Engineering-Tutorial)

<br>

# ATmega328P UART Driver
An ATmega328P UART driver written entirely in Assembler.

<br>

# Code
```assembler
; ===================================================================
; Project: ATmega328P UART Driver
; ===================================================================
; Author: Kevin Thomas
; E-Mail: ket189@pitt.edu
; Version: 1.0
; Date: 12/29/24
; Target Device: ATmega328P (Arduino Nano)
; Clock Frequency: 16 MHz
; Toolchain: AVR-AS, AVRDUDE
; License: Apache License 2.0
; Description: This program is a simple I/O to which when a button
;              connected to PD2 is pressed, an external LED connected
;              to PD4 will illuminate, otherwise it will be off and
;              utilizes UART at 9600 baud and echos every char 
;              pressed.
; ===================================================================

; ===================================================================
; SYMBOLIC DEFINITIONS
; ===================================================================
.equ     DDRD, 0x0A               ; Data Direction Register for PORTD
.equ     PORTD, 0x0B              ; PORTD Data Register
.equ     PIND, 0x09               ; Input Pins Address for PORTD
.equ     PD2, 2                   ; Pin 2 of PORTD (D2 on Nano)
.equ     PD4, 4                   ; Pin 4 of PORTD (D4 on Nano)   
.equ     UCSR0A, 0xC0             ; USART0 Control & Status Reg A
.equ     UCSR0B, 0xC1             ; USART0 Control & Status Reg B
.equ     UCSR0C, 0xC2             ; USART0 Control & Status Reg C
.equ     UBRR0L, 0xC4             ; USART0 Baud Rate Register Low
.equ     UBRR0H, 0xC5             ; USART0 Baud Rate Register High
.equ     UDR0, 0xC6               ; USART0 I/O Data Register
.equ     RXEN0, 4                 ; USART0 Receiver Enable
.equ     TXEN0, 3                 ; USART0 Transmitter Enable
.equ     UDRE0, 5                 ; USART0 Data Register Empty
.equ     RXC0, 7                  ; USART0 Receive Complete
.equ     UCSZ00, 1                ; character size
.equ     UCSZ01, 2                ; character size

; ===================================================================
; PROGRAM ENTRY POINT
; ===================================================================
.global  program                  ; global label; make avail external
.section .text                    ; start of the .text (code) section

; ===================================================================
; PROGRAM LOOP
; ===================================================================
; Description: Main program loop which executes all subroutines and 
;              then repeads indefinately.
; -------------------------------------------------------------------
; Instructions: AVR Instruction Set Manual
;               6.87 RCALL – Relative Call to Subroutine
;               6.90 RJMP – Relative Jump
; ===================================================================
program:
  RCALL  Config_Pins              ; config pins
  RCALL  UART_Init                ; UART init
program_loop:
  RCALL  Check_Button             ; check button state; control LED
  RCALL  Poll_UART                ; non-blocking check; incoming char
  RJMP   program_loop             ; infinite loop

; ===================================================================
; SUBROUTINE: Config_Pins
; ===================================================================
; Description: Main configuration of pins on the ATmega128P Arduino 
;              Nano.
; -------------------------------------------------------------------
; Instructions: AVR Instruction Set Manual
;               6.33 CBI – Clear Bit in I/O Register
;               6.95 SBI – Set Bit in I/O Register
;               6.88 RET – Return from Subroutine
; ===================================================================
config_pins:
  CBI    DDRD, PD2                ; set PD2 as input (button pin)
  SBI    PORTD, PD2               ; enable pull-up resistor on PD2
  SBI    DDRD, PD4                ; set PD4 as output (LED pin)
  RET                             ; return from subroutine

; ===================================================================
; SUBROUTINE: UART_Init
; ===================================================================
; Description: Initializes UART0 for serial communication at 9600 baud, 
;              8 data bits, no parity, 1 stop bit.
; -------------------------------------------------------------------
; Instructions: AVR Instruction Set Manual
;               6.63 LDI – Load Immediate
;               6.117 STS – Store Direct to Data Space
;               6.83 ORI – Logical OR with Immediate
;               6.88 RET – Return from Subroutine
; ===================================================================
UART_Init:
  LDI    R16, 103                ; set 9600 baud rate low byte
  STS    UBRR0L, R16             ; store in UBRR0L
  LDI    R16, 0                  ; set 9600 baud rate high byte
  STS    UBRR0H, R16             ; store in UBRR0H
  LDI    R16, (1 << RXEN0)       ; enable RX (Receiver Enable)
  ORI    R16, (1 << TXEN0)       ; enable TX (Transmitter Enable)
  STS    UCSR0B, R16             ; store in UCSR0B
  LDI    R16, (1 << UCSZ00)      ; set UCSZ00 (Bit 2) for 8 data bits
  ORI    R16, (1 << UCSZ01)      ; set UCSZ01 (Bit 1) for 8 data bits
  STS    UCSR0C, R16             ; store in UCSR0C
  RET                            ; return from subroutine

; ===================================================================
; SUBROUTINE: Check_Button
; ===================================================================
; Description: Checks if PD2 is pressed and if so, drive LED high,
;              otherwise drive LED low.
; -------------------------------------------------------------------
; Instructions: AVR Instruction Set Manual
;               6.97 SBIS – Skip if Bit in I/O Register is Set
;               6.96 SBIC – Skip if Bit in I/O Register is Cleared
;               6.88 RET – Return from Subroutine
; ===================================================================
Check_Button:
  SBIS   PIND, PD2                ; skip next inst if PD2 is high
  RCALL  LED_On                   ; if PD2 is low; BTN pressed
  SBIS   PIND, PD2                ; skip next inst if PD2 is low
  RCALL  LED_Off                  ; if PD2 is high; BTN not pressed
  RET                             ; return from subroutine

; ===================================================================
; SUBROUTINE: LED_On
; ===================================================================
; Description: Sets PB5 high to turn on the LED.
; -------------------------------------------------------------------
; Instructions: AVR Instruction Set Manual
;               6.95 SBI – Set Bit in I/O Register
;               6.88 RET – Return from Subroutine
; ===================================================================
led_on:
  SBI    PORTD, PD4               ; set PD4 high
  RET                             ; return from subroutine

; ===================================================================
; SUBROUTINE: LED_Off
; ===================================================================
; Description: Clears PB5 to turn off the LED.
; -------------------------------------------------------------------
; Instructions: AVR Instruction Set Manual
;               6.33 CBI – Clear Bit in I/O Register
;               6.88 RET – Return from Subroutine
; ===================================================================
LED_Off:
  CBI    PORTD, PD4               ; set PD4 low
  RET                             ; return from subroutine

; ===================================================================
; SUBROUTINE: Poll_UART
; ===================================================================
; Description: Non-blocking UART polling. If a byte arrives, it is 
;              echoed back via UART.
; -------------------------------------------------------------------
; Instructions: AVR Instruction Set Manual
;               6.114 LDS – Load Direct from Data Space
;               6.101 SBRS – Skip if Bit in Register is Set             
;               6.88 RET – Return from Subroutine
;               6.115 STS – Store Direct to Data Space
; ===================================================================
Poll_UART:
  LDS    R16, UCSR0A             ; load UCSR0A
  SBRS   R16, RXC0               ; skip next if RXC0 is set; has data
  RET                            ; return from subroutine
  LDS    R16, UDR0               ; read received byte from UDR0
.Poll_TX_Wait:
  LDS    R17, UCSR0A             ; load UCSR0A
  SBRS   R17, UDRE0              ; skip next if UDRE0 is set; buf rdy
  RJMP   .Poll_TX_Wait           ; wait until buffer is ready
  STS    UDR0, R16               ; write byte to UDR0 (transmit)
  RET                            ; return from subroutine
```

<br>

## License
[Apache License 2.0](https://github.com/mytechnotalent/ATmega328P_UART_Driver/blob/main/LICENSE)

