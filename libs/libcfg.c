

#define CFG_FLAGS_HAS_REPEAT 1
#define CFG_FLAGS_RECV_ALL 2
#define CFG_FLAGS_YAMU 4
#define CFG_FLAGS_SHOW_PULSE 8

typedef struct {
    uint8_t address_l;
    uint8_t address_h;
    uint8_t product;
    uint8_t ir_pause;
    uint8_t power_key;
    uint8_t flags;
    uint8_t hash;
} scfg_t;


typedef union {
    scfg_t s;
    uint8_t data[0];
} cfg_t;


cfg_t cfg;

// Read config from eeprom
static inline __attribute__((always_inline)) 
inline void read_cfg(void)
{
    uint8_t config_idx = 0;
    uint8_t hash = 0xAA;
    eeprom_read_block(cfg.data, (void*)10, sizeof(cfg));
    for (config_idx=0; config_idx < sizeof(cfg); ++config_idx)
    {
        hash ^= cfg.data[config_idx];
    }

    if (0 != hash)
    {
        // use default values
        cfg.s.address_l = IR_ADDRES_L;
        cfg.s.address_h = IR_ADDRES_H;
        cfg.s.power_key = IR_POWER_KEY;
        cfg.s.flags = 0;
#if defined(IR_RECEIVE_ALL) && IR_RECEIVE_ALL
        cfg.s.flags |= CFG_FLAGS_RECV_ALL;
#endif
        cfg.s.flags |= CFG_FLAGS_HAS_REPEAT;
        cfg.s.address_l = 39;
        cfg.s.address_h = 205;
        cfg.s.ir_pause = 120;
        cfg.s.hash = 0;
    }
}