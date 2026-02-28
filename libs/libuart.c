static inline __attribute__((always_inline)) 
inline void uart_init(void)
{
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

}

static void uart_putc(char c)
{
    /* Wait until data register empty. */
    loop_until_bit_is_set(UCSRA, UDRE);
    UDR = c;
    //loop_until_bit_is_set(UCSRA, TXC); /* Wait until transmission ready. */
}

static void uart_puts(const char *s)
{
    while (*s) {
        uart_putc(*s++);
    }
}

static void uart_putdata(const uint8_t* p, uint8_t len)
{
    while (len > 0)
    {
        uart_putc(*p);
        --len;
        ++p;
    }
}

static void uart_put_u8(uint8_t val)
{
    uart_putc('0' + (val / 100));
    val %= 100;
    uart_putc('0' + (val / 10));
    uart_putc('0' + (val % 10));
}

///////////////////////////////////////////////////////////////////////////////
// put_key
///////////////////////////////////////////////////////////////////////////////
static void put_key(char type, uint8_t val)
{
    uart_putc(type);
    uart_putc(' ');
    uart_put_u8(val);
    uart_putc('\n');
}