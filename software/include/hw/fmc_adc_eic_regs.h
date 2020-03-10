#ifndef __CHEBY__FMC_ADC_EIC_REGS__H__
#define __CHEBY__FMC_ADC_EIC_REGS__H__
#define FMC_ADC_EIC_REGS_SIZE 16

/* Interrupt Disable Register */
#define FMC_ADC_EIC_REGS_IDR 0x0UL

/* Interrupt Enable Register */
#define FMC_ADC_EIC_REGS_IER 0x4UL

/* Interrupt Mask Register */
#define FMC_ADC_EIC_REGS_IMR 0x8UL

/* Interrupt Status Register */
#define FMC_ADC_EIC_REGS_ISR 0xcUL

struct fmc_adc_eic_regs {
  /* [0x0]: REG (wo) Interrupt Disable Register */
  uint32_t idr;

  /* [0x4]: REG (wo) Interrupt Enable Register */
  uint32_t ier;

  /* [0x8]: REG (ro) Interrupt Mask Register */
  uint32_t imr;

  /* [0xc]: REG (rw) Interrupt Status Register */
  uint32_t isr;
};

#endif /* __CHEBY__FMC_ADC_EIC_REGS__H__ */
