#ifndef __CHEBY__FMC_ADC_MEZZANINE_MMAP__H__
#define __CHEBY__FMC_ADC_MEZZANINE_MMAP__H__

#include "timetag_core_regs.h"
#include "wb_ds182x_regs.h"
#include "fmc_adc_100ms_csr.h"
#define FMC_ADC_MEZZANINE_MMAP_SIZE 8192

/* FMC ADC 100M CSR */
#define FMC_ADC_MEZZANINE_MMAP_FMC_ADC_100M_CSR 0x1000UL
#define FMC_ADC_MEZZANINE_MMAP_FMC_ADC_100M_CSR_SIZE 512

/* FMC ADC Embedded Interrupt Controller */
#define FMC_ADC_MEZZANINE_MMAP_FMC_ADC_EIC 0x1500UL
#define FMC_ADC_MEZZANINE_MMAP_FMC_ADC_EIC_SIZE 16

/* Si570 control I2C master */
#define FMC_ADC_MEZZANINE_MMAP_SI570_I2C_MASTER 0x1600UL
#define FMC_ADC_MEZZANINE_MMAP_SI570_I2C_MASTER_SIZE 256

/* DS18B20 OneWire master */
#define FMC_ADC_MEZZANINE_MMAP_DS18B20_ONEWIRE_MASTER 0x1700UL
#define FMC_ADC_MEZZANINE_MMAP_DS18B20_ONEWIRE_MASTER_SIZE 16

/* Mezzanine SPI master (ADC control + DAC offsets) */
#define FMC_ADC_MEZZANINE_MMAP_FMC_SPI_MASTER 0x1800UL
#define FMC_ADC_MEZZANINE_MMAP_FMC_SPI_MASTER_SIZE 32

/* Timetag Core */
#define FMC_ADC_MEZZANINE_MMAP_TIMETAG_CORE 0x1900UL
#define FMC_ADC_MEZZANINE_MMAP_TIMETAG_CORE_SIZE 128

struct fmc_adc_mezzanine_mmap {

  /* padding to: 1024 words */
  uint32_t __padding_0[1024];
  /* [0x1000]: SUBMAP FMC ADC 100M CSR */
  struct fmc_adc_100ms_csr fmc_adc_100m_csr;

  /* padding to: 1344 words */
  uint32_t __padding_1[192];

  /* [0x1500]: SUBMAP FMC ADC Embedded Interrupt Controller */
  uint32_t fmc_adc_eic[4];

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
