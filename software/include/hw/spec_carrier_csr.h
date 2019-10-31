#ifndef __CHEBY__SPEC_CARRIER_CSR__H__
#define __CHEBY__SPEC_CARRIER_CSR__H__
#define SPEC_CARRIER_CSR_SIZE 16

/* Carrier type and PCB version */
#define SPEC_CARRIER_CSR_CARRIER 0x0UL
#define SPEC_CARRIER_CSR_CARRIER_PCB_REV_MASK 0xfUL
#define SPEC_CARRIER_CSR_CARRIER_PCB_REV_SHIFT 0
#define SPEC_CARRIER_CSR_CARRIER_RESERVED_MASK 0xfff0UL
#define SPEC_CARRIER_CSR_CARRIER_RESERVED_SHIFT 4
#define SPEC_CARRIER_CSR_CARRIER_TYPE_MASK 0xffff0000UL
#define SPEC_CARRIER_CSR_CARRIER_TYPE_SHIFT 16

/* Status */
#define SPEC_CARRIER_CSR_STAT 0x4UL
#define SPEC_CARRIER_CSR_STAT_FMC_PRES 0x1UL
#define SPEC_CARRIER_CSR_STAT_P2L_PLL_LCK 0x2UL
#define SPEC_CARRIER_CSR_STAT_SYS_PLL_LCK 0x4UL
#define SPEC_CARRIER_CSR_STAT_DDR3_CAL_DONE 0x8UL

/* Control */
#define SPEC_CARRIER_CSR_CTRL 0x8UL
#define SPEC_CARRIER_CSR_CTRL_LED_GREEN 0x1UL
#define SPEC_CARRIER_CSR_CTRL_LED_RED 0x2UL

/* Reset Register */
#define SPEC_CARRIER_CSR_RST 0xcUL
#define SPEC_CARRIER_CSR_RST_FMC0 0x1UL

struct spec_carrier_csr {
  /* [0x0]: REG (ro) Carrier type and PCB version */
  uint32_t carrier;

  /* [0x4]: REG (ro) Status */
  uint32_t stat;

  /* [0x8]: REG (rw) Control */
  uint32_t ctrl;

  /* [0xc]: REG (wo) Reset Register */
  uint32_t rst;
};

#endif /* __CHEBY__SPEC_CARRIER_CSR__H__ */
