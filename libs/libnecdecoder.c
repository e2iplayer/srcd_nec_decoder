#define BUF_SIZE 32  // musi być potęgą 2

/*
YAMU
LEADER    // 4.5ms (1) + 4.5ms (1)
VENDOR    // 16-bits
SEPARATOR //4.5 ms (0)
PRODUCT   //4-bits
DATA      //8-bits
DATA_BAR  //8-bits
*/

// LEADER_OF, 4.5ms
#define TIME_LEADER_ON_MIN   30
#define TIME_LEADER_ON_MAX   40

// LEADER_ON, 4.5ms
#define TIME_LEADER_OFF_MIN   30
#define TIME_LEADER_OFF_MAX   40

// SEOPARATOR, 4.5ms
#define TIME_SEPARATOR_MIN   30
#define TIME_SEPARATOR_MAX   40


// AGC Burst, 9ms typ
#define TIME_BURST_MIN 65
#define TIME_BURST_MAX 75

// Gap after AGC Burst, 4.5ms typ
#define TIME_GAP_MIN   30
#define TIME_GAP_MAX   40

// Gap (key hold) after AGC Burst, 2.25ms typ
#define TIME_HOLD_MIN   15
#define TIME_HOLD_MAX   20

// Short pulse for each bit, 560us typ
#define TIME_PULSE_MIN  2
#define TIME_PULSE_MAX  7

// Gap for logical 0, 560us typ
#define TIME_ZERO_MIN   2
#define TIME_ZERO_MAX   7

// Gap for logical 1, 1.69ms typ
#define TIME_ONE_MIN    9
#define TIME_ONE_MAX   19

// Definition for state machine 
enum ir_state_t {
    IR_BURST,
    IR_GAP,
    IR_ADDRESS_L_PRE,
    IR_ADDRESS_L_BIT,
    IR_ADDRESS_H_PRE,
    IR_ADDRESS_H_BIT,
    IR_SEPARATOR_PRE,
    IR_SEPARATOR,
    IR_PRODUCT_PRE,
    IR_PRODUCT_BIT,
    IR_COMMAND_L_PRE,
    IR_COMMAND_L_BIT,
    IR_COMMAND_H_PRE,
    IR_COMMAND_H_BIT,
    IR_READY
};
uint8_t ir_state;
uint8_t ir_bitctr;
uint8_t ir_error;

uint8_t has_key = 0;
uint8_t prev_key = 0;

// Struct definition
struct ir_struct
{
    uint8_t address_l; // YAMU: Vendor
    uint8_t address_h; // YAMU: Vendor
    uint8_t product;   // YAMU: only 4-bits product
    uint8_t command_l; // YAMU: DATA
    uint8_t command_h; // YAMU: DATA
};

 // Global status structure
struct ir_struct ir;
uint8_t cnt_state;
uint8_t port_state;


volatile uint8_t nec_buf[BUF_SIZE];
volatile uint8_t buf_head = 0;
volatile uint8_t buf_tail = 0;

#define IR_PACK(flaga, dane)   (uint8_t)(((flaga) << 7) | ((dane) & 0x7F))

// Rozpakowanie flagi (najstarszy bit)
#define IR_FLAG(pakiet)     ((pakiet) >> 7)

// Rozpakowanie danych (7 najmłodszych bitów)
#define IR_DATA(pakiet)     ((pakiet) & 0x7F)

volatile uint8_t timer0_overflow = 0;
volatile uint8_t last_time = 0;  // pełny czas w tickach

static inline __attribute__((always_inline)) 
void ir_init(void)
{
    // tAGC_burst = 9ms, tBIT = 0.56ms
#if defined (__AVR_ATtiny2313__) || defined (__AVR_ATtiny2313A__) || defined (__AVR_ATtiny4313__)

    // --- TIMER0 ---
    // 8-bit, 8MHz / 1024 = 7812.5 Hz
    // Tick = 128us, Overflow ≈ 32.768ms
    TCCR0A = 0x00;  // Normal mode, OC0A/OC0B disconnected

    // wyczyść preskaler i ustaw 1024 (CS02 + CS00)
    TCCR0B = (1<<CS02) | (1<<CS00);

    // wyczyść licznik (opcjonalnie, ale zalecane)
    TCNT0 = 0;

    // włącz przerwanie overflow (do rozszerzenia czasu)
    TIMSK |= (1<<TOIE0);


    // --- INT0 (PD2) ---
    // PD2 jako wejście (IR receiver – sygnał odwrócony)
    DDRD  &= ~(1<<PD2);

    // włącz pull-up (zalecane dla stabilnego wejścia IR)
    PORTD |= (1<<PD2);

    // INT0 na dowolną zmianę stanu (rising + falling)
    // ISC01 = 0, ISC00 = 1
    MCUCR = (MCUCR & ~((1<<ISC01) | (1<<ISC00))) | (1<<ISC00);

    // wyczyść ewentualną starą flagę przerwania
    GIFR  |= (1<<INTF0);

    // włącz INT0 (ATtiny używa GIMSK, nie GICR)
    GIMSK |= (1<<INT0);

#else
    #warning "MCU not supported"
#endif

}

ISR(TIMER0_OVF_vect)
{
    if (__builtin_expect(timer0_overflow < 255, 1))
    {
        timer0_overflow++;
    }
}

ISR(INT0_vect)
{
    uint8_t sreg = SREG;
    cli();

    // odczyt bieżącego timera
    //uint8_t tcnt0 = TCNT0;
    uint8_t tcnt = TCNT0;
    TCNT0 = 0;

    // odczyt stanu pinu
    uint8_t level = (PIND & (1<<PD2)) ? 1 : 0;
    
    //uint8_t tcnt = tcnt0 - last_time; 

    if (tcnt > 128)
    {
        last_time = tcnt;
        tcnt = 127;
    }

    // ring buffer
    uint8_t next = (buf_head + 1) & (BUF_SIZE - 1);
    if (next != buf_tail) {
        nec_buf[buf_head] = IR_PACK(level, tcnt);//tcnt - last_time; //tcnt > last_time ? tcnt - last_time : 256 - last_time + tcnt;
        //nec_buf[buf_head].level = level;
        buf_head = next;
    }
    
    //last_time = tcnt0;
    timer0_overflow = 0;

    SREG = sreg;
}

static inline __attribute__((always_inline)) 
uint8_t get_event_delta(uint8_t *ev) {
    if (buf_head == buf_tail) return 0;

    uint8_t sreg = SREG;
    cli();

    uint8_t tail = buf_tail;
    buf_tail = (buf_tail + 1) & (BUF_SIZE - 1);

    SREG = sreg;

    cnt_state = nec_buf[tail];
    port_state = IR_FLAG(cnt_state);
    cnt_state = IR_DATA(cnt_state);

    return 1;
}


///////////////////////////////////////////////////////////////////////////////
// process_bit
///////////////////////////////////////////////////////////////////////////////
static inline __attribute__((always_inline)) 
void process_error(uint8_t error)
{
    //if ((port_state == 1) && (cnt_state >= TIME_LEADER_ON_MIN) && (cnt_state <= TIME_LEADER_ON_MAX))
    //{
    //    ir_state = IR_GAP; // Next state
    //    return;
    //}

    ir_error = error;
    ir_state = IR_BURST;
}

static void process_bit(uint8_t next_ir_state, uint8_t *ir_bits)
{
    const uint8_t size = ir_state == IR_PRODUCT_BIT ? 4 : 8;
    if ((cnt_state>=TIME_ZERO_MIN)&&(cnt_state<=TIME_ZERO_MAX))
    {
        // 0
        *ir_bits &= ~(1<<ir_bitctr++);
        if (ir_bitctr>=size)
        {
            ir_state = next_ir_state; // Next state
            ir_bitctr = 0; // Reset bitcounter
        }
        else
        {
            ir_state -= 1;
        }
        return;
    } 
    else if ((cnt_state>=TIME_ONE_MIN)&&(cnt_state<=TIME_ONE_MAX))
    {
        // 1
        *ir_bits |= (1<<ir_bitctr++);
        if (ir_bitctr>=size)
        {
            ir_state = next_ir_state; // Next state
            ir_bitctr = 0; // Reset bitcounter
        }
        else
        {
            ir_state -= 1;
        }
        return;
    }
    // Should not happen, must be invalid. Reset.
    process_error(4);
}

static inline __attribute__((always_inline)) 
uint8_t process_yamu(void)
{
    if (port_state == 1)
    {
        if ((cnt_state >= TIME_LEADER_ON_MIN) && (cnt_state <= TIME_LEADER_ON_MAX))
        {
            ir_state = IR_GAP;
            return 0;
        }
    }
#if 0
    else if (has_key && cnt_state > TIME_GAP_MAX)
    {
        uart_puts("-");
        uart_put_u8(cnt_state);
        uart_puts("\r\n");
    }
#endif
    switch(ir_state)
    {
    /*
    case IR_BURST:
        if ((cnt_state >= TIME_LEADER_ON_MIN) && (cnt_state <= TIME_LEADER_ON_MAX))
        {
            //uart_puts(port_state ? "B_1\n": "B_0\n");
            ir_state = IR_GAP; // Next state
        }
        //  something goes wrong - ignore
        //ir_error = 1;
    break;
    */
    case IR_GAP: 
        if ((cnt_state >= TIME_LEADER_OFF_MIN) && (cnt_state <= TIME_LEADER_OFF_MAX))
        {
            ir_state = IR_ADDRESS_L_PRE; // Next state
            ir_bitctr = 0; // Reset bitcounter
            break;
        }
        else if((cnt_state >= TIME_HOLD_MIN) && (cnt_state <= TIME_HOLD_MAX))
        {
            ir_state = IR_BURST;
            return 2;
        }

        // something goes wrong
        process_error(2);
    break;
    case IR_ADDRESS_L_PRE:
    case IR_ADDRESS_H_PRE:
    case IR_PRODUCT_PRE:
    case IR_COMMAND_L_PRE:
    case IR_COMMAND_H_PRE:
    case IR_SEPARATOR_PRE:
        if ((cnt_state >= TIME_PULSE_MIN) && (cnt_state <= TIME_PULSE_MAX))
        {
            ir_state += 1; // Next state, PRE -> BIT
            break;
        }

        // something goes wrong
        process_error(3);
    break;
    case IR_ADDRESS_L_BIT:
        process_bit(IR_ADDRESS_H_PRE, &ir.address_l);
    break;
    case IR_ADDRESS_H_BIT:
        process_bit(IR_SEPARATOR_PRE, &ir.address_h);
    break;
    case IR_SEPARATOR: 
        //uart_puts(port_state ? "S_1\n": "S_0\n");
        if ((cnt_state >= TIME_SEPARATOR_MIN)&&(cnt_state <= TIME_SEPARATOR_MAX))
        {
            ir_state = IR_PRODUCT_PRE; // Next state
            ir_bitctr = 0; // Reset bitcounter
            break;
        }

        // something goes wrong
        process_error(5);
    break;
    case IR_PRODUCT_BIT:
        process_bit(IR_COMMAND_L_PRE, &ir.product);
    break;

    case IR_COMMAND_L_BIT:
        process_bit(IR_COMMAND_H_PRE, &ir.command_l);
    break;    
    case IR_COMMAND_H_BIT:
        process_bit(IR_READY, &ir.command_h); 
        if (ir_state == IR_READY)
        {
            ir_state = IR_BURST;
            return 1;
        }
        
    break;
    }
    return 0;
}

static inline __attribute__((always_inline)) 
uint8_t process_nec(void)
{
    if (port_state == 1)
    {
        //SAMSUNG NEC variant with burst on 4.5ms burst off 4.5ms
        if ((cnt_state >= TIME_LEADER_ON_MIN) && (cnt_state <= TIME_BURST_MAX))
        {
            ir_state = IR_GAP;
            return 0;
        }
    }
#if 0
    else if (has_key && cnt_state > TIME_GAP_MAX)
    {
        uart_puts("-");
        uart_put_u8(cnt_state);
        uart_puts("\r\n");
    }
#endif

    switch(ir_state)
    {
    case IR_GAP: 
        if ((cnt_state >= TIME_GAP_MIN) && (cnt_state <= TIME_GAP_MAX))
        {
            ir_state = IR_ADDRESS_L_PRE; // Next state
            ir_bitctr = 0; // Reset bitcounter
            break;
        }
        else if((cnt_state >= TIME_HOLD_MIN) && (cnt_state <= TIME_HOLD_MAX))
        {
            ir_state = IR_BURST;
            return 2;
        }

        // something goes wrong
        process_error(2);
    break;
    case IR_ADDRESS_L_PRE:
    case IR_ADDRESS_H_PRE:
    case IR_COMMAND_L_PRE:
    case IR_COMMAND_H_PRE:
        if ((cnt_state >= TIME_PULSE_MIN) && (cnt_state <= TIME_PULSE_MAX))
        {
            ir_state += 1; // Next state, PRE -> BIT
            break;
        }

        // something goes wrong
        process_error(3);
    break;
    case IR_ADDRESS_L_BIT:
        process_bit(IR_ADDRESS_H_PRE, &ir.address_l);
    break;
    case IR_ADDRESS_H_BIT:
        process_bit(IR_COMMAND_L_PRE, &ir.address_h);
    break;
    case IR_COMMAND_L_BIT:
        process_bit(IR_COMMAND_H_PRE, &ir.command_l);
    break;    
    case IR_COMMAND_H_BIT:
        process_bit(IR_READY, &ir.command_h); 
        if (ir_state == IR_READY)
        {
            ir_state = IR_BURST;
            return 1;
        }
        
    break;
    }
    return 0;
}