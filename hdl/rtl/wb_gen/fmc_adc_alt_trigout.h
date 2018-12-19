#ifndef __CHEBY__ALT_TRIGOUT__H__
#define __CHEBY__ALT_TRIGOUT__H__

/* Status register */
#define ALT_TRIGOUT_STATUS 0x0UL
#define ALT_TRIGOUT_WR_ENABLE 0x1UL
#define ALT_TRIGOUT_WR_LINK 0x2UL
#define ALT_TRIGOUT_WR_VALID 0x4UL
#define ALT_TRIGOUT_TS_PRESENT 0x100UL

/* Enable register */
#define ALT_TRIGOUT_ENABLE 0x4UL
#define ALT_TRIGOUT_CH1_ENABLE 0x1UL
#define ALT_TRIGOUT_CH2_ENABLE 0x2UL
#define ALT_TRIGOUT_CH3_ENABLE 0x4UL
#define ALT_TRIGOUT_CH4_ENABLE 0x8UL
#define ALT_TRIGOUT_EXT_ENABLE 0x100UL

/* Time (seconds) of the last event */
#define ALT_TRIGOUT_TS_MASK_SEC 0x8UL
#define ALT_TRIGOUT_TS_SEC_MASK 0xffffffffffULL
#define ALT_TRIGOUT_TS_SEC_SHIFT 0
#define ALT_TRIGOUT_CH1_MASK 0x1000000000000ULL
#define ALT_TRIGOUT_CH2_MASK 0x2000000000000ULL
#define ALT_TRIGOUT_CH3_MASK 0x4000000000000ULL
#define ALT_TRIGOUT_CH4_MASK 0x8000000000000ULL
#define ALT_TRIGOUT_EXT_MASK 0x100000000000000ULL

/* Cycles part of timestamp fifo. */
#define ALT_TRIGOUT_TS_CYCLES 0x10UL
#define ALT_TRIGOUT_CYCLES_MASK 0xfffffffUL
#define ALT_TRIGOUT_CYCLES_SHIFT 0

struct alt_trigout {
  /* [0x0]: REG (ro) Status register */
  uint32_t status;
  
  /* [0x4]: REG (rw) Enable register */
  uint32_t enable;
  
  /* [0x8]: REG (ro) Time (seconds) of the last event */
  uint64_t ts_mask_sec;
  
  /* [0x10]: REG (ro) Cycles part of timestamp fifo. */
  uint32_t ts_cycles;
};

#endif /* __CHEBY__ALT_TRIGOUT__H__ */
