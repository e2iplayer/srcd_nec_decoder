#if defined(IR_ORIGINAL_POWER_KEY) && defined(IR_ORIGINAL_ADDRES_L) && defined(IR_ORIGINAL_ADDRES_H)

#ifdef IR_DEEP_WAKEUP_MODE_REMOTE_BASED_ON_TRANSISTOR

#define PIN_POWER_ENABLE PORTD |= (1 << PD4);
#define PIN_POWER_DISABLE PORTD &= ~(1 << PD4);
#else
#define PIN_POWER_ENABLE PORTD &= ~(1 << PD4);
#define PIN_POWER_DISABLE PORTD |= (1 << PD4);
#endif

#define PIN_POWER_PULSE_DURATION_US 563
static uint8_t power_raw_code[4] = {IR_ORIGINAL_ADDRES_L, IR_ORIGINAL_ADDRES_H, IR_ORIGINAL_POWER_KEY, ~IR_ORIGINAL_POWER_KEY};

#else
#define IR_POWER_SWITCH
#define PIN_POWER_ENABLE PORTD |= (1 << PD4);
#define PIN_POWER_DISABLE PORTD &= ~(1 << PD4);
#endif

///////////////////////////////////////////////////////////////////////////////
// handle_power_key
///////////////////////////////////////////////////////////////////////////////
static void handle_power_key(uint8_t deepStandby)
{
    // handle only in deep standby or if forced
    // check if in deep standby
    if (!deepStandby)
    {
#ifdef IR_DEEP_STANDBY_DETECTION_BY_PIN
        if (0 == (PIND & (1 << PD5)))
        {
            deepStandby = 1;
        }
#else
        deepStandby = 1;

        // we already send power key to process, wait a little bit
        // and check if we have confirmation char
       _delay_ms(50);
        if (UCSRA & (1<<RXC))
        {
            char val = UDR;
            if (val == 'C')
            {
                // we receive confirmation char, so power key was aleady handled
                deepStandby = 0;
            }
        }
#endif
    }

    if (deepStandby)
    {
#ifdef IR_POWER_SWITCH
        cli();
        PIN_POWER_ENABLE
        _delay_ms(200);
        PIN_POWER_DISABLE
        sei();
#else
        do
        {
            cli();
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
            sei();

            _delay_ms(200);
            /* We need to resend the power key code because during our transmision the main ir receive diode 
             * can also receive signal directly from remote.
             * This is simple workaround which does not requre addional hardware and for my usage it is enough. 
             * With this workaround STB will be waked up when user release the power button.
             */
            deepStandby += 1;
        }
#ifdef IR_DEEP_STANDBY_DETECTION_BY_PIN
        while (deepStandby < 10 && 0 == (PIND & (1 << PD5)));
#else
        while (deepStandby < 10);
#endif
#endif
    }

    cold_boot_flag = 0;
    cold_boot_inv = 0;
}