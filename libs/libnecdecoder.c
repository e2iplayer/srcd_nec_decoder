// #############################################################################
// #              --- Infrared Remote Decoder (NEC Protocol) ---               #
// #############################################################################
// # libnecdecoder.c - NEC IR Library                                          #
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

#include <avr/io.h>
#include <avr/interrupt.h>
#include "libnecdecoder.h"

volatile uint8_t ir_state;
volatile uint8_t ir_bitctr;
volatile uint8_t ir_tmp_address_l;
volatile uint8_t ir_tmp_address_h;

volatile uint8_t ir_tmp_command;
volatile uint8_t ir_tmp_ovf;

volatile uint8_t ir_rerror;
volatile uint8_t rc_timeout;

///////////////////////////////////////////////////////////////////////////////
// ###### Initializes ir function ######
///////////////////////////////////////////////////////////////////////////////
void ir_init( void )
{
    ir_rerror = 0;
    rc_timeout = 0;

    // tAGC_burst = 9ms, tBIT = 0.56ms
#if defined (__AVR_ATmega48__) || defined(__AVR_ATmega88__) || defined(__AVR_ATmega168__)
    // Timer: 8bit, Clock: 8MHz, Prescaler 1024, Overflow = 32.768ms, Tick = 0.128ms
    TCCR0A &= ~( (1<<COM0A0) | (1<<COM0A1) | (1<<COM0B0) | (1<<COM0B1) | (1<<WGM00) | (1<<WGM01) );
    TCCR0B |= (1<<CS00) | (1<<CS02);
    TIMSK0 |= (1<<TOIE0);

    // http://www.arduinoslovakia.eu/application/timer-calculator # 128 ms interrupt, 1000 / 128 = 7,8125 MHz
    OCR1A = 6249; // 20 Hz (8000000/((6249+1)*64))
    TCCR1A = 0;
    TCCR1B = (1 << CS11) | (1 << CS10) | (1 << WGM12);
    TIMSK1 |= (1 << OCIE1A);  // Output Compare Match A Interrupt Enable
    TCNT1 = 0;

    // Interrupt 0 (PD2): Inverted signal input, triggered by logical change
    DDRD   &= ~(1<<PD2);
    EICRA  |= (1<<ISC00);
    EIMSK  |= (1<<INT0);
#elif defined (__AVR_ATtiny2313__) || (__AVR_ATtiny2313A__) || (__AVR_ATtiny4313__)
    // Timer: 8bit, Clock: 8MHz, Prescaler 1024, Overflow = 32.768ms, Tick = 0.128ms
    TCCR0A &= ~( (1<<COM0A0) | (1<<COM0A1) | (1<<COM0B0) | (1<<COM0B1) | (1<<WGM00) | (1<<WGM01) );
    TCCR0B |= (1<<CS00) | (1<<CS02);
    TIMSK  |= (1<<TOIE0);
    
    // http://www.arduinoslovakia.eu/application/timer-calculator # 128 ms interrupt, 1000 / 128 = 7,8125 MHz
    OCR1A = 6249; // 20 Hz (8000000/((6249+1)*64))
    TCCR1A = 0;
    TCCR1B = (1 << CS11) | (1 << CS10) | (1 << WGM12);
    TIMSK |= (1 << OCIE1A);  // Output Compare Match A Interrupt Enable
    TCNT1 = 0;

    // Interrupt 0 (PD2): Inverted signal input, triggered by logical change
    DDRD   &= ~(1<<PD2);
    MCUCR  |= (1<<ISC00);
    GIMSK  |= (1<<INT0);
    
#else
    #warning "MCU not supported"
#endif   

    // Reset state
    ir_state = IR_BURST;

    // Reset global variables
    ir_tmp_ovf = 0;

    // Global interrupt enable
    sei();
}


///////////////////////////////////////////////////////////////////////////////
// check_short_pulse
///////////////////////////////////////////////////////////////////////////////
static uint8_t check_short_pulse( uint8_t port_state, uint8_t cnt_state)
{
    if (port_state)
    {
        // Must be short pulse
        if ((cnt_state > TIME_PULSE_MIN) && (cnt_state < TIME_PULSE_MAX))
        {
            TCNT0 = 0; // Reset counter
            return 1;
        }
        // Should not happen, must be invalid. Reset.
        rc_timeout = 1;
        ir_state = IR_BURST;
    }
    return 0;
}

///////////////////////////////////////////////////////////////////////////////
// process_bit
///////////////////////////////////////////////////////////////////////////////
static void process_bit(uint8_t port_state, uint8_t cnt_state, uint8_t next_ir_state, volatile uint8_t *val)
{
    if (!check_short_pulse(port_state, cnt_state))
    {
        if ((cnt_state>TIME_ZERO_MIN)&&(cnt_state<TIME_ZERO_MAX))
        {
            // 0
            (*val) &= ~(1<<ir_bitctr++);
            TCNT0 = 0; // Reset counter
            if (ir_bitctr>=8)
            {
                ir_state = next_ir_state; // Next state
                ir_bitctr = 0; // Reset bitcounter
            }
            return;
        } 
        else if ((cnt_state>TIME_ONE_MIN)&&(cnt_state<TIME_ONE_MAX))
        {
            // 1
            (*val) |= (1<<ir_bitctr++);
            TCNT0 = 0; // Reset counter
            if (ir_bitctr>=8)
            {
                ir_state = next_ir_state; // Next state
                ir_bitctr = 0; // Reset bitcounter
            }
            return;
        }
        // Should not happen, must be invalid. Reset.
        rc_timeout = 1;
        ir_rerror = 1;
        ir_state = IR_BURST;
    }
}

///////////////////////////////////////////////////////////////////////////////
// ###### INT0 for decoding ######  
///////////////////////////////////////////////////////////////////////////////
ISR( INT0_vect )
{
    // Get current port state to check if we triggered on rising or falling edge
    uint8_t port_state = ( PIND & (1<<PD2) );
    uint8_t cnt_state = TCNT0;

    if (cnt_state > TIME_ZERO_MIN)
    {
        TCNT1 = 0;
    }

    if (ir_tmp_ovf)
    {
        // Overflow, so reset and ignore.
        ir_tmp_ovf = 0;
        ir_state = IR_BURST;
        TCNT0 = 0;
        return;
    }

    switch(ir_state)
    {
    case IR_BURST:
        if (!port_state)
        {
            TCNT0 = 0; // Reset counter
        } 
        else if ((cnt_state>TIME_BURST_MIN)&&(cnt_state<TIME_BURST_MAX))
        {
            ir_state = IR_GAP; // Next state
            TCNT0 = 0; // Reset counter
        }
    break;
    case IR_GAP: 
        if (!port_state)
        {
            if ((cnt_state>TIME_GAP_MIN)&&(cnt_state<TIME_GAP_MAX))
            {
                TCNT0 = 0; // Reset counter
                ir_state = IR_ADDRESS; // Next state
                ir_bitctr = 0; // Reset bitcounter
                break;
            }
            else if ((cnt_state>TIME_HOLD_MIN)&&(cnt_state<TIME_HOLD_MAX))
            {
                ir_state = IR_BURST;
                break;
            }
        }
        // Should not happen, must be invalid. Reset.
        rc_timeout = 1;
        ir_state = IR_BURST;
    break;
    case IR_ADDRESS:
        process_bit(port_state, cnt_state, IR_ADDRESS_INV, &ir_tmp_address_l);
    break;
    case IR_ADDRESS_INV:
        process_bit(port_state, cnt_state, IR_COMMAND, &ir_tmp_address_h);
    break;
    case IR_COMMAND:
        process_bit(port_state, cnt_state, IR_COMMAND_INV, &ir_tmp_command);
    break;
    case IR_COMMAND_INV:
        if (!check_short_pulse(port_state, cnt_state))
        {
            if ((cnt_state>TIME_ZERO_MIN)&&(cnt_state<TIME_ZERO_MAX))
            {
                // 0 (inverted)
                if (!(ir_tmp_command & (1<<ir_bitctr++))) 
                {
                    // Should not happen, must be invalid. Reset.
                    rc_timeout = 1;
                    ir_rerror = 9;
                    ir_state = IR_BURST;
                    break;
                }
                TCNT0 = 0; // Reset counter
                if (ir_bitctr >= 8)
                {
                    ir_state = IR_BURST; // Decoding finished.
                    rc_timeout = 0;
                    TCNT1 = 1; // Reset counter

                    // Only apply if received flag is not set, must be done
                    // by the main program after reading address and command
                    if(!(ir.status & (1<<IR_RECEIVED)))
                    {
                        ir.address_l = ir_tmp_address_l;
                        ir.address_h = ir_tmp_address_h;
                        ir.command = ir_tmp_command;
                        ir.status |= (1<<IR_RECEIVED) | (1<<IR_SIGVALID);
                    }
                    ir_bitctr = 0; // Reset bitcounter
                }
                break;
            }
            else if ((cnt_state>TIME_ONE_MIN)&&(cnt_state<TIME_ONE_MAX))
            {
                // 1 (inverted)
                if (ir_tmp_command&(1<<ir_bitctr++)) 
                {
                    // Should not happen, must be invalid. Reset.
                    rc_timeout = 1;
                    ir_state = IR_BURST;
                    ir_rerror = 11;
                    break;
                }
                TCNT0 = 0; // Reset counter
                if(ir_bitctr>=8)
                {
                    ir_state = IR_BURST; // Decoding finished.
                    rc_timeout = 0;
                    TCNT1 = 0; // Reset counter

                    // Only apply if received flag is not set, must be done
                    // by the main program after reading address and command
                    if(!(ir.status & (1<<IR_RECEIVED)))
                    {
                        ir.address_l = ir_tmp_address_l;
                        ir.address_h = ir_tmp_address_h;
                        ir.command = ir_tmp_command;
                        ir.status |= (1<<IR_RECEIVED) | (1<<IR_SIGVALID);
                    }
                    ir_bitctr = 0; // Reset bitcounter
                }
                break;
            }
            // Should not happen, must be invalid. Reset.
            rc_timeout = 1;
            ir_rerror = 12;
            ir_state = IR_BURST;
            break; 
        }
    break;
    }
}


// ###### Timer 0 Overflow for hold flag clear ###### 
ISR (TIMER0_OVF_vect)
{
    ir_tmp_ovf = 1;
}

ISR(TIMER1_COMPA_vect) 
{
  rc_timeout = 1;
}
