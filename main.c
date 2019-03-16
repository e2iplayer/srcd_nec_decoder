// #############################################################################
// #              --- Infrared Remote Decoder (NEC Protocol) ---               #
// #############################################################################
// # main.c - Testcode for IR Library                                          #
// #############################################################################
// #              Version: 1.1 - Compiler: AVR-GCC 4.5.3 (Linux)               #
// #      (c) 2013 by Malte Pöggel - All rights reserved. - License: BSD       #
// #               www.MALTEPOEGGEL.de - malte@maltepoeggel.de                 #
// #############################################################################
// #   Redistribution and use in source and binary forms, with or without mo-  #
// # dification, are permitted provided that the following conditions are met: #
// #                                                                           #
// # * Redistributions of source code must retain the above copyright notice,  #
// #   this list of conditions and the following disclaimer.                   #
// # * Redistributions in binary form must reproduce the above copyright       #
// #   notice, this list of conditions and the following disclaimer in the     #
// #   documentation and/or other materials provided with the distribution.    #
// #                                                                           #
// #    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS    #
// # "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED #
// #      TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A      #
// #     PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT    # 
// #   HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,  #
// # SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED  #
// #    TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,    #
// #  OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY   #
// #  OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING  #
// #    NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS     #
// #      SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.         #
// #############################################################################

#ifndef F_CPU 
#warning "F_CPU undefined, set to 8MHz"
#define F_CPU 8000000UL
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <util/delay.h>

// disable WDT according to: https://www.nongnu.org/avr-libc/user-manual/group__avr__watchdog.html
uint8_t mcusr_mirror __attribute__ ((section (".noinit")));
void get_mcusr(void) __attribute__((naked)) __attribute__((section(".init3")));
void get_mcusr(void)
{
  mcusr_mirror = MCUSR;
  MCUSR = 0;
  wdt_disable();
}
// end

#include "libs/libnecdecoder.h"


#define BAUD 9600
#include <util/setbaud.h>

#if defined(IR_ORIGINAL_POWER_KEY) && defined(IR_ORIGINAL_ADDRES_L) && defined(IR_ORIGINAL_ADDRES_H)
#define PIN_POWER_ENABLE PORTD &= ~(1 << PD4);
#define PIN_POWER_DISABLE PORTD |= (1 << PD4);
#define PIN_POWER_PULSE_DURATION_US 563
static uint8_t power_raw_code[4] = {IR_ORIGINAL_ADDRES_L, IR_ORIGINAL_ADDRES_H, IR_ORIGINAL_POWER_KEY, ~IR_ORIGINAL_POWER_KEY};
#else
#define IR_POWER_SWITCH
#define PIN_POWER_ENABLE PORTD |= (1 << PD4);
#define PIN_POWER_DISABLE PORTD &= ~(1 << PD4);
#endif


 // Demo code description:
 // INT0   input for the IR receiver (i.e. TSOP 1736, TSOP 31236)
 // PORTB0 toggled on every valid packet
 // PORTB1 high if the current key is hold
 // UART   will output address and command at 9600 Baud 8N1

extern volatile uint8_t ir_rerror;
extern volatile uint8_t rc_timeout;
volatile uint8_t last_key;
volatile uint8_t last_key_valid;

///////////////////////////////////////////////////////////////////////////////
// ###### Send single character via UART ######
///////////////////////////////////////////////////////////////////////////////
static void uart_putchar(const char c) 
{
    /* Wait until data register empty. */
    loop_until_bit_is_set(UCSRA, UDRE);
    UDR = c;
    //loop_until_bit_is_set(UCSRA, TXC); /* Wait until transmission ready. */
}

///////////////////////////////////////////////////////////////////////////////
// ###### Send string via UART ######
///////////////////////////////////////////////////////////////////////////////
static void uart_putstring( const char* s )
{
    while (*s)
    {
        uart_putchar(*s);
        s++;
    }
}

///////////////////////////////////////////////////////////////////////////////
// put_key
///////////////////////////////////////////////////////////////////////////////
static void put_key(char type, uint8_t last_key)
{
    uart_putchar(type);
    uart_putchar(' ');
    uart_putchar('0'+((last_key / 100) % 10));
    uart_putchar('0'+((last_key / 10) % 10));
    uart_putchar('0'+(last_key % 10));
    uart_putchar('\n');
}

///////////////////////////////////////////////////////////////////////////////
// handle_power_key
///////////////////////////////////////////////////////////////////////////////
static void handle_power_key(uint8_t force)
{
    // handle only in deep standby or if forced

#ifdef IR_POWER_SWITCH
    if (0 == (PIND & (1 << PD5)))
    {
        cli();
        PIN_POWER_ENABLE
        _delay_ms(100);
        PIN_POWER_DISABLE
        sei();
    }
#else
    if (0 == (PIND & (1 << PD5)) || force)
    {
        cli();
        do
        {
            PIN_POWER_ENABLE
            _delay_us(9000);
            PIN_POWER_DISABLE
            _delay_us(4500);

            for (int i=0; i<4; ++i)
            {
                uint8_t byte = power_raw_code[i];
                for (int j=0; j<8; ++j)
                {
                    PIN_POWER_ENABLE
                    _delay_us(PIN_POWER_PULSE_DURATION_US);
                    PIN_POWER_DISABLE
                    _delay_us(PIN_POWER_PULSE_DURATION_US);
                    if (byte & 0x01)
                    {
                       _delay_us(2*PIN_POWER_PULSE_DURATION_US);
                    }
                    byte >>= 1;
                }
            }

            PIN_POWER_ENABLE
            _delay_us(PIN_POWER_PULSE_DURATION_US);
            PIN_POWER_DISABLE

            _delay_ms(50);
            /* We need to resend the power key code because during our transmision the main ir receive diode 
             * can also receive signal directly from remote.
             * This is simple workaround which does not requre addional hardware and for my usage it is enough. 
             * With this workaround STB will be waked up when user release the power button.
             */
        } while (0 == (PIND & (1 << PD5)));
        sei();
    }
#endif

}

int main( void )
{
    const char REBOOT_CMD[] = {'R', 's', 'T'};
    uint8_t reboot_idx = 0;
    
    // output pin
    DDRD |= (1 << PD4);     // send power on (PD4)
    PIN_POWER_DISABLE       // switch off

    // input pin
    DDRD &= ~(1 << PD5);    // is deep standby (PD5)
    //PORTD |= (1 << PD5);    // enable pull-up resistor

    // https://www.avrfreaks.net/forum/calibration-internal-oscillator
    // https://github.com/pkarsy/rcCalibrator
    //OSCCAL -= 10;  // -18 .. -3
    
    // uart_init start
    UBRRH = UBRRH_VALUE;
    UBRRL = UBRRL_VALUE;

#if USE_2X
    UCSRA |= _BV(U2X);
#else
    UCSRA &= ~(_BV(U2X));
#endif

    UCSRC = _BV(UCSZ1) | _BV(UCSZ0); /* 8-bit data */
    //UCSR0B = _BV(TXEN0);  /* Enable TX */
    UCSRB = _BV(RXEN) | _BV(TXEN);   /* Enable RX and TX */
    // uart_init end
    
    last_key = 0;
    last_key_valid = 0;

    // Initialize IR lib
    ir_init();

    uart_putstring("START\n");
#ifndef IR_POWER_SWITCH
    /*For example, Zgemma H9S need receive at least one key code from remote 
     * to be able to wake up later using this remote, so we send key at start 
     * to satisfied this
     */
    handle_power_key(1);
#endif

    // Internal oscillator calibration
#if 0
    while(1)
    {
        uart_putstring("UUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUU ");
        put_key('O', OSCCAL);
    }
#endif

    while(1)
    {
        // check if we have char
        if (UCSRA & (1<<RXC))
        {
            char val = UDR;
            if (val == REBOOT_CMD[reboot_idx])
            {
                reboot_idx += 1;
                if (reboot_idx == sizeof(REBOOT_CMD))
                {
                    // reset
                    wdt_enable(WDTO_15MS);
                    wdt_reset();
                    while(1);
                }
            }
            else
            {
                reboot_idx = 0;
            }
        }

        // Check if new code is received
        if(ir.status & (1<<IR_RECEIVED))
        {
            // Reset state
            ir.status &= ~(1<<IR_RECEIVED);

#if !defined(IR_RECEIVE_ALL) || IR_RECEIVE_ALL == 0
            if (ir.address_h != IR_ADDRES_H || ir.address_l != IR_ADDRES_L)
            {
                continue;
            }
#endif

            if (last_key != ir.command || !last_key_valid)
            {
                if (last_key_valid)
                {
                    put_key('R', last_key);
                }

                last_key = ir.command;
#if defined(IR_RECEIVE_ALL) && IR_RECEIVE_ALL
                put_key('L', ir.address_l);
                put_key('H', ir.address_h);
#endif
                if (IR_POWER_KEY == last_key)
                {
                    handle_power_key(0);
                }
                last_key_valid = 1;
                put_key('P', last_key);
            }
        }
        else if (rc_timeout)
        {
            if (last_key_valid)
            {
                put_key('R', last_key);
                last_key_valid = 0;
            }
            rc_timeout = 0;
        }

        if (ir_rerror)
        {
            put_key('E', ir_rerror);
            ir_rerror = 0;
        }
    }
}
