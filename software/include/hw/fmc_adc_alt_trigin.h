#ifndef __CHEBY__ALT_TRIGIN__H__
#define __CHEBY__ALT_TRIGIN__H__

/* Core version */
#define ALT_TRIGIN_VERSION 0x0UL
#define ALT_TRIGIN_VERSION_PRESET 0xadc10001UL

/* Control register */
#define ALT_TRIGIN_CTRL 0x4UL
#define ALT_TRIGIN_CTRL_ENABLE 0x1UL

/* Time (seconds) to trigger */
#define ALT_TRIGIN_SECONDS 0x8UL

/* Time (cycles) to trigger */
#define ALT_TRIGIN_CYCLES 0x10UL

struct alt_trigin {
  /* [0x0]: REG (ro) Core version */
  uint32_t version;
  
  /* [0x4]: REG (rw) Control register */
  uint32_t ctrl;
  
  /* [0x8]: REG (rw) Time (seconds) to trigger */
  uint64_t seconds;
  
  /* [0x10]: REG (rw) Time (cycles) to trigger */
  uint32_t cycles;
};

#endif /* __CHEBY__ALT_TRIGIN__H__ */
