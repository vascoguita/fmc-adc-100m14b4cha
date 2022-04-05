#ifndef __CHEBY__SPEC_REF_FMC_ADC_100M_MMAP__H__
#define __CHEBY__SPEC_REF_FMC_ADC_100M_MMAP__H__

#include "fmc_adc_mezzanine_mmap.h"
#define SPEC_REF_FMC_ADC_100M_MMAP_SIZE 24576 /* 0x6000 = 24KB */

/* a ROM containing the application metadata */
#define SPEC_REF_FMC_ADC_100M_MMAP_METADATA 0x2000UL
#define ADDR_MASK_SPEC_REF_FMC_ADC_100M_MMAP_METADATA 0x7fc0UL
#define SPEC_REF_FMC_ADC_100M_MMAP_METADATA_SIZE 64 /* 0x40 */

/* FMC ADC Mezzanine */
#define SPEC_REF_FMC_ADC_100M_MMAP_FMC_ADC_MEZZANINE 0x4000UL
#define ADDR_MASK_SPEC_REF_FMC_ADC_100M_MMAP_FMC_ADC_MEZZANINE 0x6000UL
#define SPEC_REF_FMC_ADC_100M_MMAP_FMC_ADC_MEZZANINE_SIZE 8192 /* 0x2000 = 8KB */

struct spec_ref_fmc_adc_100m_mmap {

  /* padding to: 2048 words */
  uint32_t __padding_0[2048];
  /* [0x2000]: SUBMAP a ROM containing the application metadata */
  uint32_t metadata[16];

  /* padding to: 4096 words */
  uint32_t __padding_1[2032];

  /* [0x4000]: SUBMAP FMC ADC Mezzanine */
  struct fmc_adc_mezzanine_mmap fmc_adc_mezzanine;
};

#endif /* __CHEBY__SPEC_REF_FMC_ADC_100M_MMAP__H__ */
