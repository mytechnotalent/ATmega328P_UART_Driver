<img src="https://raw.githubusercontent.com/mytechnotalent/ATmega328P_IO_Driver/refs/heads/main/ATmega328P%20UART%20Driver.png">

## FREE Reverse Engineering Self-Study Course [HERE](https://github.com/mytechnotalent/Reverse-Engineering-Tutorial)

<br>

# ATmega328P UART Driver
An ATmega328P UART driver written entirely in Assembler.

<br>

# Code
```assembler
; ===================================================================
; Project: ATmega328P IO Driver
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
  rcall  config_pins              ; config pins
  rcall  uart_init                ; uart init
program_loop:
  rcall  check_button             ; check button state; control LED
  rcall  poll_uart                ; non-blocking check; incoming char
  rjmp   program_loop             ; infinite loop

; ===================================================================
; SUBROUTINE: config_pins
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
  cbi    DDRD, PD2                ; set PD2 as input (button pin)
  sbi    PORTD, PD2               ; enable pull-up resistor on PD2
  sbi    DDRD, PD4                ; set PD4 as output (LED pin)
  ret                             ; return from subroutine

; ===================================================================
; SUBROUTINE: uart_init
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
uart_init:
  ldi    r16, 103                ; set 9600 baud rate low byte
  sts    UBRR0L, r16             ; store in UBRR0L
  ldi    r16, 0                  ; set 9600 baud rate high byte
  sts    UBRR0H, r16             ; store in UBRR0H
  ldi    r16, (1 << RXEN0)       ; enable RX (Receiver Enable)
  ori    r16, (1 << TXEN0)       ; enable TX (Transmitter Enable)
  sts    UCSR0B, r16             ; store in UCSR0B
  ldi    r16, (1 << UCSZ00)      ; set UCSZ00 (Bit 2) for 8 data bits
  ori    r16, (1 << UCSZ01)      ; set UCSZ01 (Bit 1) for 8 data bits
  sts    UCSR0C, r16             ; store in UCSR0C
  ret                            ; return from subroutine

; ===================================================================
; SUBROUTINE: check_button
; ===================================================================
; Description: Checks if PD2 is pressed and if so, drive LED high,
;              otherwise drive LED low.
; -------------------------------------------------------------------
; Instructions: AVR Instruction Set Manual
;               6.97 SBIS – Skip if Bit in I/O Register is Set
;               6.96 SBIC – Skip if Bit in I/O Register is Cleared
;               6.88 RET – Return from Subroutine
; ===================================================================
check_button:
  sbis   PIND, PD2                ; skip next inst if PD2 is high
  rcall  led_on                   ; if PD2 is low; BTN pressed
  sbic   PIND, PD2                ; skip next inst if PD2 is low
  rcall  led_off                  ; if PD2 is high; BTN not pressed
  ret                             ; return from subroutine

; ===================================================================
; SUBROUTINE: led_on
; ===================================================================
; Description: Sets PB5 high to turn on the LED.
; -------------------------------------------------------------------
; Instructions: AVR Instruction Set Manual
;               6.95 SBI – Set Bit in I/O Register
;               6.88 RET – Return from Subroutine
; ===================================================================
led_on:
  sbi    PORTD, PD4               ; set PD4 high
  ret                             ; return from subroutine

; ===================================================================
; SUBROUTINE: led_off
; ===================================================================
; Description: Clears PB5 to turn off the LED.
; -------------------------------------------------------------------
; Instructions: AVR Instruction Set Manual
;               6.33 CBI – Clear Bit in I/O Register
;               6.88 RET – Return from Subroutine
; ===================================================================
led_off:
  cbi    PORTD, PD4               ; set PD4 low
  ret                             ; return from subroutine

; ===================================================================
; SUBROUTINE: poll_uart
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
poll_uart:
  lds    r16, UCSR0A             ; load UCSR0A
  sbrs   r16, RXC0               ; skip next if RXC0 is set; has data
  ret                            ; return from subroutine
  lds    r16, UDR0               ; read received byte from UDR0
.poll_tx_wait:
  lds    r17, UCSR0A             ; load UCSR0A
  sbrs   r17, UDRE0              ; skip next if UDRE0 is set; buf rdy
  rjmp   .poll_tx_wait           ; wait until buffer is ready
  sts    UDR0, r16               ; write byte to UDR0 (transmit)
  ret                            ; return from subroutine
```

<br>

## License
[Apache License 2.0](https://github.com/mytechnotalent/ATmega128P_UART_Driver/blob/main/LICENSE)

