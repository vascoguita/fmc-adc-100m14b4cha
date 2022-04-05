#ifndef __CHEBY__FMC_ADC_MEZZANINE_MMAP__H__
#define __CHEBY__FMC_ADC_MEZZANINE_MMAP__H__

#include "fmc_adc_100ms_csr.h"
#include "fmc_adc_eic_regs.h"
#include "wb_ds182x_regs.h"
#include "timetag_core_regs.h"
#define FMC_ADC_MEZZANINE_MMAP_SIZE 8192 /* 0x2000 = 8KB */

/* FMC ADC 100M CSR */
#define FMC_ADC_MEZZANINE_MMAP_FMC_ADC_100M_CSR 0x1000UL
#define ADDR_MASK_FMC_ADC_MEZZANINE_MMAP_FMC_ADC_100M_CSR 0x1e00UL
#define FMC_ADC_MEZZANINE_MMAP_FMC_ADC_100M_CSR_SIZE 512 /* 0x200 */

/* FMC ADC Embedded Interrupt Controller */
#define FMC_ADC_MEZZANINE_MMAP_FMC_ADC_EIC 0x1500UL
#define ADDR_MASK_FMC_ADC_MEZZANINE_MMAP_FMC_ADC_EIC 0x1ff0UL
#define FMC_ADC_MEZZANINE_MMAP_FMC_ADC_EIC_SIZE 16 /* 0x10 */

/* Si570 control I2C master */
#define FMC_ADC_MEZZANINE_MMAP_SI570_I2C_MASTER 0x1600UL
#define ADDR_MASK_FMC_ADC_MEZZANINE_MMAP_SI570_I2C_MASTER 0x1f00UL
#define FMC_ADC_MEZZANINE_MMAP_SI570_I2C_MASTER_SIZE 256 /* 0x100 */

/* DS18B20 OneWire master */
#define FMC_ADC_MEZZANINE_MMAP_DS18B20_ONEWIRE_MASTER 0x1700UL
#define ADDR_MASK_FMC_ADC_MEZZANINE_MMAP_DS18B20_ONEWIRE_MASTER 0x1ff0UL
#define FMC_ADC_MEZZANINE_MMAP_DS18B20_ONEWIRE_MASTER_SIZE 16 /* 0x10 */

/* Mezzanine SPI master (ADC control + DAC offsets) */
#define FMC_ADC_MEZZANINE_MMAP_FMC_SPI_MASTER 0x1800UL
#define ADDR_MASK_FMC_ADC_MEZZANINE_MMAP_FMC_SPI_MASTER 0x1fe0UL
#define FMC_ADC_MEZZANINE_MMAP_FMC_SPI_MASTER_SIZE 32 /* 0x20 */

/* Timetag Core */
#define FMC_ADC_MEZZANINE_MMAP_TIMETAG_CORE 0x1900UL
#define ADDR_MASK_FMC_ADC_MEZZANINE_MMAP_TIMETAG_CORE 0x1f80UL
#define FMC_ADC_MEZZANINE_MMAP_TIMETAG_CORE_SIZE 128 /* 0x80 */

struct fmc_adc_mezzanine_mmap {

  /* padding to: 1024 words */
  uint32_t __padding_0[1024];
  /* [0x1000]: SUBMAP FMC ADC 100M CSR */
  struct fmc_adc_100ms_csr fmc_adc_100m_csr;

  /* padding to: 1344 words */
  uint32_t __padding_1[192];

  /* [0x1500]: SUBMAP FMC ADC Embedded Interrupt Controller */
  struct fmc_adc_eic_regs fmc_adc_eic;

  /* padding to: 1408 words */
  uint32_t __padding_2[60];

  /* [0x1600]: SUBMAP Si570 control I2C master */
  uint32_t si570_i2c_master[64];

  /* [0x1700]: SUBMAP DS18B20 OneWire master */
  struct wb_ds182x_regs ds18b20_onewire_master;

  /* padding to: 1536 words */
  uint32_t __padding_3[60];

  /* [0x1800]: SUBMAP Mezzanine SPI master (ADC control + DAC offsets) */
  uint32_t fmc_spi_master[8];

  /* padding to: 1600 words */
  uint32_t __padding_4[56];

  /* [0x1900]: SUBMAP Timetag Core */
  struct timetag_core_regs timetag_core;

  /* padding to: 1600 words */
  uint32_t __padding_5[416];
};

#endif /* __CHEBY__FMC_ADC_MEZZANINE_MMAP__H__ */
