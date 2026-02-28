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

#define BAUD 9600

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <avr/eeprom.h>

#include <util/setbaud.h>

// disable WDT according to: https://www.nongnu.org/avr-libc/user-manual/group__avr__watchdog.html
uint8_t mcusr_mirror __attribute__ ((section (".noinit")));
uint8_t cold_boot_flag __attribute__ ((section (".noinit")));
uint8_t cold_boot_inv __attribute__ ((section (".noinit")));
void get_mcusr(void) __attribute__((naked)) __attribute__((section(".init3")));
void get_mcusr(void)
{
  mcusr_mirror = MCUSR;
  MCUSR = 0;
  wdt_disable();
}
// end


//////////////////////////////////////////////////////////////
#include "libs/libuart.c"
#include "libs/libnecdecoder.c"
#include "libs/libnecencoder.c"
#include "libs/libcfg.c"
//////////////////////////////////////////////////////////////


int main(void)
{
    const char REBOOT_CMD[] = {'R', 's', 'T'};
    const char CONFIG_CMD[] = {'C', 'f', 'G'};
    uint8_t reboot_idx = 0;
    uint8_t config_idx = 0;

    // output pin
    DDRD |= (1 << PD4);     // send power on (PD4)
    PIN_POWER_DISABLE       // switch off

#ifdef IR_DEEP_STANDBY_DETECTION_BY_PIN
    // input pin
    DDRD &= ~(1 << PD5);    // is deep standby (PD5)
    //PORTD |= (1 << PD5);    // enable pull-up resistor
#endif

    uart_init();   // UART start
    ir_init();     // Twoja inicjalizacja INT0 + Timer0

    ///////////////////////////////////
    // Read config from eeprom
    read_cfg();

    uart_puts("StArU\n");
    uart_putdata(cfg.data, sizeof(cfg));

    if (mcusr_mirror & ((1 << PORF) | (1 << BORF))) 
    {
        cold_boot_flag = 0x42;
        cold_boot_inv = ~0x42;
    }
    ///////////////////////////////////

    // Global interrupt enable
    sei();

    ir_state = IR_BURST;
    ir_error = 0;


    while (1)
    {
        // check if we have char
        if (!has_key && (UCSRA & (1<<RXC)))
        {
            char val = UDR;
            if (val == REBOOT_CMD[reboot_idx])
            {
                reboot_idx += 1;
                if (reboot_idx == sizeof(REBOOT_CMD))
                {

                    if (cold_boot_flag == 0x42 && cold_boot_inv == (uint8_t)~0x42) 
                    {
                        cold_boot_flag = 0;
                        cold_boot_inv = 0;
                        uart_puts("CoLd\n");
                        _delay_ms(5);
                    }

                    // reset
                    wdt_enable(WDTO_15MS);
                    wdt_reset();
                    while(1);
                }
            }
            else
                reboot_idx = 0;

            if (val == CONFIG_CMD[config_idx])
            {
                config_idx += 1;
                if (config_idx == sizeof(CONFIG_CMD))
                {
                    uint8_t hash = 0xAA;
                    uart_puts("CfT\n");
                    for (config_idx=0; config_idx < sizeof(cfg); ++config_idx)
                    {
                        loop_until_bit_is_set(UCSRA, RXC);
                        cfg.data[config_idx] = UDR;
                        hash ^= cfg.data[config_idx];
                    }

                    if (hash == 0)
                    {
                        eeprom_write_block(cfg.data, (void*)10, sizeof(cfg));
                        uart_puts("CfO\n");
                    }
                    else
                    {
                        uart_puts("CfK\n");
                        put_key('E', hash);
                    }
                }
            }
            else
                config_idx = 0;
        }


        if (get_event_delta(&cnt_state))
        {
            // Format: L=0 T=1234
            if (cfg.s.flags & CFG_FLAGS_SHOW_PULSE)
            {
                uart_putc('L');
                uart_putc(port_state ? '1' : '0');

                uart_puts(" T");
                uart_put_u8(cnt_state);
                uart_putc('\n');
                continue;
            }

            uint8_t ready = 0;
            if (cfg.s.flags & CFG_FLAGS_YAMU)
            {
                ready = process_yamu();
            }
            else
            {
                ready = process_nec();
            }
            
            if ( has_key && ready == 2 ) // 2 means repeat
            {
                has_key = 2;
                continue;
            }
            else if ( ready == 1 )
            {
                if (cfg.s.flags & CFG_FLAGS_RECV_ALL)
                {
                    put_key('L', ir.address_l);
                    put_key('H', ir.address_h);
                    if (cfg.s.flags & CFG_FLAGS_YAMU)
                    {
                        put_key('p', ir.product);
                    }
                    continue;
                }

                if ((ir.command_h ^ ir.command_l) == 0xFF && 
                    ir.address_h == cfg.s.address_h &&
                    ir.address_l == cfg.s.address_l &&
                    (0 == (cfg.s.flags & CFG_FLAGS_YAMU) || ir.product == cfg.s.product))
                {
                    if (has_key)
                    {
                        // if remote use repeat then all full command means new key pressed
                        if (0 == (cfg.s.flags & CFG_FLAGS_HAS_REPEAT) && prev_key == ir.command_l)
                            continue;

                        put_key('R', prev_key);
                    }

                    put_key('P', ir.command_l);
                    if (cfg.s.power_key == ir.command_l)
                    {
                        handle_power_key(0);
                    }

                    has_key = 1;
                    prev_key = ir.command_l;
                }
                else
                {
                    ir_error = 255;
                }
            }

            if (ir_error)
            {
                if (has_key)
                {
                    put_key('R', prev_key);
                    has_key = 0;
                }

#if 0
                uart_puts(" L=");
                uart_put_u8(port_state);
                uart_puts(" T=");
                uart_put_u8(cnt_state);
                uart_puts(" E=");
                uart_put_u8(ir_error);
                uart_puts("\r\n");
#endif
                put_key('E', ir_error);
                ir_error = 0;
            }
        }
        else if (has_key == 1) // timeout after press
        {
            
            //if ((timer0_overflow >= 2) || (timer0_overflow == 1 && TCNT0 > 220)) // ~58ms ir pause YAMU
            //if ((timer0_overflow >= 2))// || (timer0_overflow == 1 && TCNT0 > 70)) // ~39ms NEC -> 255 + 70 * 128us = 41 600us = 41,6ms
            //if ((timer0_overflow >= 2) || (timer0_overflow == 1 && TCNT0 > 120)) // 56 ir pause 

            if ((timer0_overflow >= 2) || (timer0_overflow == 1 && TCNT0 > cfg.s.ir_pause))
            {
                put_key('R', prev_key);
                has_key = 0;
            }
        }
        else if (has_key == 2) // timeout after repeat
        {
            // 110ms - (9ms + 2.5ms) = 98,5 ms // to make sure 103ms -> 804 ticks => 3 overflow + 39
            if ((timer0_overflow >= 3) || (timer0_overflow == 3 && TCNT0 > 39))
            {
                put_key('R', prev_key);
                has_key = 0;
            }
        }
    }
}

/*
120

16 + 8 + 16

40

TCNT0
32.768 *3 

128us
*/