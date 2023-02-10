// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright CERN 2012-2020
 * Author: Federico Vaga <federico.vaga@gmail.com>
 */

#ifndef FMC_ADC_100M14B4C_PRV_H_
#define FMC_ADC_100M14B4C_PRV_H_

#include <linux/types.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/scatterlist.h>
#include <linux/workqueue.h>
#include <linux/debugfs.h>
#include <linux/platform_device.h>
#include <linux/fmc.h>
#include <linux/completion.h>
#include <linux/version.h>

#include <linux/zio.h>
#include <linux/zio-dma.h>
#include <linux/zio-sysfs.h>
#include <linux/zio-buffer.h>
#include <linux/zio-trigger.h>

#include "field-desc.h"
#include "fmc-adc-100m14b4cha.h"
#include <platform_data/fmc-adc-100m14b4cha.h>

enum fa_versions {
	ADC_VER = 0,
};

/**
 * struct device_meta_id Metadata
 */
struct device_meta_id {
	uint32_t vendor;
	uint32_t device;
	uint32_t version;
	uint32_t bom;
	uint32_t src[4];
	uint32_t cap;
	uint32_t uuid[4];
};


enum fa100m14b4c_trg_ext_attr_krn {
       FA100M14B4C_TATTR_TRG_SU = __FA100M14B4C_TATTR_TRG_MAX,
       FA100M14B4C_TATTR_TRG_SL,
       FA100M14B4C_TATTR_TRG_C,
};
#define ADC_CSR_OFF 0x1000
#define ADC_EIC_OFF 0x1500
#define ADC_OW_OFF  0x1700
#define ADC_SPI_OFF 0x1800
#define ADC_UTC_OFF 0x1900

#define DAC_VAL_MASK 0xFFFF

#define ADC_DMA 0

#define ADC_CSR_CTL_REG_OFFSET 0x00000000
#define ADC_CSR_STA_REG_OFFSET 0x00000004


enum fa_irq_resource {
	ADC_IRQ_TRG = 0,
};

enum fa_mem_resource {
	ADC_MEM_BASE = 0,
	ADC_MEM_META,
};

enum fa_bus_resource {
	ADC_CARR_VME_ADDR,
};

struct fa_memory_ops {
#if KERNEL_VERSION(5, 8, 0) <= LINUX_VERSION_CODE
	u32 (*read)(const void *addr);
#else
	u32 (*read)(void *addr);
#endif
	void (*write)(u32 value, void *addr);
};

/*
 * ZFA_CHx_MULT : the trick which requires channel regs id grouped and ordered
 * address offset between two registers of the same type on consecutive channel
 */
#define ZFA_CHx_MULT 9

/* Device registers */
enum zfadc_dregs_enum {
	/* Device */
	/* Control registers */
	ZFA_CTL_FMS_CMD,
	ZFA_CTL_CLK_EN,
	ZFA_CTL_DAC_CLR_N,
	ZFA_CTL_BSLIP,
	ZFA_CTL_TEST_DATA_EN,
	ZFA_CTL_TRIG_LED,
	ZFA_CTL_ACQ_LED,
	ZFA_CTL_RST_TRG_STA,
	ZFA_CTL_CALIB_APPLY,
	/* Status registers */
	ZFA_STA_FSM,
	ZFA_STA_SERDES_PLL,
	ZFA_STA_SERDES_SYNCED,
	ZFA_STA_ACQ_CFG,
	ZFA_STA_FMC_NR,
	ZFA_STA_CALIB_BUSY,
	/* Configuration register */
	ZFAT_CFG_STA,
	ZFAT_CFG_SRC,
	ZFAT_CFG_POL,
	/* Delay*/
	ZFAT_EXT_DLY,
	/* Software */
	ZFAT_SW,
	/* Number of shots */
	ZFAT_SHOTS_NB,
	/* Remaining shots counter */
	ZFAT_SHOTS_REM,
	/* Sample rate */
	ZFAT_SR_UNDER,
	/* Sampling clock frequency */
	ZFAT_SAMPLING_HZ,
	/* Position address */
	ZFAT_POS,
	/* Pre-sample */
	ZFAT_PRE,
	/* Post-sample */
	ZFAT_POST,
	/* Sample counter */
	ZFAT_CNT,
	/* start:declaration block requiring some order */
	/* Channel 1 */
	ZFA_CH1_CTL_RANGE,
	ZFA_CH1_CTL_TERM,
	ZFA_CH1_STA,
	ZFA_CH1_GAIN,
	ZFA_CH1_OFFSET,
	ZFA_CH1_SAT,
	ZFA_CH1_THRES,
	ZFA_CH1_HYST,
	ZFA_CH1_DLY,

	/* Channel 2 */
	ZFA_CH2_CTL_RANGE,
	ZFA_CH2_CTL_TERM,
	ZFA_CH2_STA,
	ZFA_CH2_GAIN,
	ZFA_CH2_OFFSET,
	ZFA_CH2_SAT,
	ZFA_CH2_THRES,
	ZFA_CH2_HYST,
	ZFA_CH2_DLY,

	/* Channel 3 */
	ZFA_CH3_CTL_RANGE,
	ZFA_CH3_CTL_TERM,
	ZFA_CH3_STA,
	ZFA_CH3_GAIN,
	ZFA_CH3_OFFSET,
	ZFA_CH3_SAT,
	ZFA_CH3_THRES,
	ZFA_CH3_HYST,
	ZFA_CH3_DLY,

	/* Channel 4 */
	ZFA_CH4_CTL_RANGE,
	ZFA_CH4_CTL_TERM,
	ZFA_CH4_STA,
	ZFA_CH4_GAIN,
	ZFA_CH4_OFFSET,
	ZFA_CH4_SAT,
	ZFA_CH4_THRES,
	ZFA_CH4_HYST,
	ZFA_CH4_DLY,

	/*
	 * CHx__ are specifc ids used by some internal arithmetic
	 * Be carefull: the arithmetic expects
	 * that ch1 to ch4 are declared in the enum just above
	 * in the right order and grouped.
	 * Don't insert any other id in this area
	 */
	ZFA_CHx_CTL_RANGE,
	ZFA_CHx_CTL_TERM,
	ZFA_CHx_STA,
	ZFA_CHx_GAIN,
	ZFA_CHx_OFFSET,
	ZFA_CHx_SAT,
	ZFA_CHx_THRES,
	ZFA_CHx_HYST,
	ZFA_CHx_DLY,
	/* Other options */
	ZFA_MULT_MAX_SAMP,
	/* end:declaration block requiring some order */
	/* two wishbone core for IRQ: VIC, ADC */
	ZFA_IRQ_ADC_DISABLE_MASK,
	ZFA_IRQ_ADC_ENABLE_MASK,
	ZFA_IRQ_ADC_MASK_STATUS,
	ZFA_IRQ_ADC_SRC,
	ZFA_IRQ_VIC_CTRL,
	ZFA_IRQ_VIC_DISABLE_MASK,
	ZFA_IRQ_VIC_ENABLE_MASK,
	ZFA_IRQ_VIC_MASK_STATUS,
	/* DS18B20 UID/Temperature */
	ZFA_DS18B20_ID_U,
	ZFA_DS18B20_ID_L,
	ZFA_DS18B20_TEMP,
	ZFA_DS18B20_STAT,
	/* UTC core */
	ZFA_UTC_SECONDS_U,
	ZFA_UTC_SECONDS_L,
	ZFA_UTC_COARSE,
	ZFA_UTC_TRIG_TIME_SECONDS_U,
	ZFA_UTC_TRIG_TIME_SECONDS_L,
	ZFA_UTC_TRIG_TIME_COARSE,
	ZFA_UTC_TRIG_SECONDS_U,
	ZFA_UTC_TRIG_SECONDS_L,
	ZFA_UTC_TRIG_COARSE,
	ZFA_UTC_ACQ_START_SECONDS_U,
	ZFA_UTC_ACQ_START_SECONDS_L,
	ZFA_UTC_ACQ_START_COARSE,
	ZFA_UTC_ACQ_STOP_SECONDS_U,
	ZFA_UTC_ACQ_STOP_SECONDS_L,
	ZFA_UTC_ACQ_STOP_COARSE,
	ZFA_UTC_ACQ_END_SECONDS_U,
	ZFA_UTC_ACQ_END_SECONDS_L,
	ZFA_UTC_ACQ_END_COARSE,
	ZFA_HW_PARAM_COMMON_LAST,
};


/*
 * Acquisition metadata. It contains the trigger timestamp and the trigger
 * source. This block is added after the post-trigger-samples in the DDR.
 */
#define FA_TRIG_TIMETAG_BYTES 0x10

/*
 * ADC parameter id not mapped to Hw register
 * Id is used as zio attribute id
 */
enum fa_sw_param_id {
	/* to guarantee unique zio attr id */
	ZFA_SW_R_NOADDRES_NBIT = ZFA_HW_PARAM_COMMON_LAST,

	ZFA_SW_R_NOADDRES_TEMP,
	ZFA_SW_R_NOADDERS_AUTO,
	ZFA_SW_R_NOADDERS_RAND,
	ZFA_SW_CH1_OFFSET_ZERO,
	ZFA_SW_CH2_OFFSET_ZERO,
	ZFA_SW_CH3_OFFSET_ZERO,
	ZFA_SW_CH4_OFFSET_ZERO,
	ZFA_SW_PARAM_COMMON_LAST,
};


/* adc IRQ values */
enum fa_irq_adc {
	FA_IRQ_ADC_NONE =	0x0,
	FA_IRQ_ADC_ACQ_END =	0x2,
};

/* Carrier-specific operations (gateware does not fully decouple
   carrier specific stuff, such as DMA or resets, from
   mezzanine-specific operations). */
struct fa_dev; /* forward declaration */

/*
 * Flag bit set when the ADC uses pattern data
 */
#define FA_DEV_F_PATTERN_DATA BIT(0)

/*
 * fa_dev: is the descriptor of the FMC ADC mezzanine
 *
 * @pdev: the pointer to the fmc_device generic structure
 * @zdev: is the pointer to the real zio_device in use
 * @hwzdev: is the pointer to the fake zio_device, used to initialize and
 *          to remove a zio_device
 *
 * @n_shots: total number of programmed shots for an acquisition
 * @n_fires: number of trigger fire occurred within an acquisition
 *
 * @n_dma_err: number of errors
 * @user_offset: user offset
 * @zero_offset: necessary offset to push the channel to zero
 */
struct fa_dev {
	unsigned long flags;
	struct device *msgdev; /**< device used to print messages */
	/* the pointer to the platform_device generic structure */
	struct platform_device	*pdev;
	/* the pointer to the real zio_device in use */
	struct zio_device	*zdev;
	/* the pointer to the fake zio_device, used for init/remove */
	struct zio_device	*hwzdev;
	struct dma_chan *dchan;

	struct fmc_slot	*slot;
	struct fa_memory_ops	memops;

	struct device_meta_id meta;

	/* carrier common base offset addresses */
	void *fa_adc_csr_base;
	void *fa_spi_base;
	void *fa_ow_base;
	void *fa_top_level;
	void *fa_irq_vic_base;
	void *fa_irq_adc_base;
	void *fa_utc_base;

	/* DMA description */
	struct zio_dma_sgt *zdma;
	struct sg_table sgt;

	struct work_struct irq_work;
	/*
	 * keep last core having fired an IRQ
	 * Used to check irq sequence: ACQ followed by DMA
	 */
	int last_irq_core_src;

	/* Acquisition */
	unsigned int		n_shots;
	unsigned int		n_fires;
	unsigned int		transfers_left;
	unsigned int		mshot_max_samples;

	/* Statistic informations */
	unsigned int		n_dma_err;

	/* Configuration */
	uint16_t		user_offset[FA100M14B4C_NCHAN]; /* one per channel */
	uint16_t		zero_offset[FA100M14B4C_NCHAN];
	/* one-wire */
	uint8_t ds18_id[8];
	unsigned long		next_t;

	/* HWMON */
	char *hwmon_temp_sensor_id;
	struct device *hwmon_dev;

	/* Calibration Data */
	struct fa_calib calib;
	int32_t range[FA100M14B4C_NCHAN];
	struct timer_list calib_timer;

	/* flag  */
	int enable_auto_start;

	struct dentry *dbg_dir;
	struct debugfs_regset32 dbg_reg32;
	struct dentry *dbg_reg_spi;
	struct dentry *dbg_trg_sw;
	struct dentry *dbg_data_pattern;

	/* Operations */
	int (*sg_alloc_table_from_pages)(struct sg_table *sgt,
					 struct page **pages,
					 unsigned int n_pages,
					 unsigned int offset,
					 unsigned long size,
					 unsigned int max_segment,
					 gfp_t gfp_mask);
};

/*
 * zfad_block
 * @block is zio_block which contains data and metadata from a single shot
 * @first_nent is the index of the first nent used for this block
 * @cset: channel set source for the block
 * @tx: DMA transfer descriptor
 * @cookie: transfer token
 */
struct zfad_block {
	struct zio_block *block;
	unsigned int first_nent;
	struct zio_cset *cset;
	struct dma_async_tx_descriptor *tx;
	dma_cookie_t cookie;
	struct sg_table sgt;
	void *dma_ctx;
	unsigned int shot_n;
	struct dma_slave_config sconfig;
	struct completion shot_done;
};

/*
 * Channel signal transmission delay
 * Trigger and channel signals are not going through the
 * same path on the board and trigger is faster.
 * Trying to sample the trigger itself by connecting
 * it to a channel, one can see a delay of 30ns between trigger and
 * its sampling. This constant is added to the trigger delay to
 * conpensate the channel signal transmission delay.
 * Expressed in tick count 3*10ns = 30ns
 */
#define FA_CH_TX_DELAY		3
#define FA_CAL_OFFSET		0x0100 /* Offset in EEPROM */

#define FA_CAL_NO_OFFSET	((int16_t)0x0000)
#define FA_CAL_NO_GAIN		((uint16_t)0x8000)

/* SPI Slave Select lines (as defined in spec_top_fmc_adc_100Ms.vhd) */
#define FA_SPI_SS_ADC		0
#define FA_SPI_SS_DAC(ch)	((ch) + 1)

/* Global variable exported by fa-zio-trg.c */
extern struct zio_trigger_type zfat_type;


static inline struct fa_dev *get_zfadc(struct device *dev)
{
	switch (to_zio_head(dev)->zobj_type) {
		case ZIO_DEV:
			return to_zio_dev(dev)->priv_d;
		case ZIO_CSET:
			return to_zio_cset(dev)->zdev->priv_d;
		case ZIO_CHAN:
			return to_zio_chan(dev)->cset->zdev->priv_d;
		case ZIO_TI:
			return to_zio_ti(dev)->cset->zdev->priv_d;
		default:
			return NULL;
	}
	return NULL;
}

static inline u32 fa_ioread(struct fa_dev *fa, void *addr)
{
	return fa->memops.read(addr);
}

static inline void fa_iowrite(struct fa_dev *fa, u32 value, void *addr)
{
	fa->memops.write(value, addr);
}

static inline uint32_t fa_readl(struct fa_dev *fa,
				void *base_off,
				const struct zfa_field_desc *field)
{
	uint32_t cur;

	cur = fa_ioread(fa, base_off + field->offset);
	if (field->is_bitfield) {
		/* apply mask and shift right accordlying to the mask */
		cur &= field->mask;
		cur /= (field->mask & -(field->mask));
	} else {
		cur &= field->mask; /* bitwise and with the mask */
	}

	return cur;
}

static inline void fa_writel(struct fa_dev *fa,
			     void *base_off,
				const struct zfa_field_desc *field,
				uint32_t usr_val)
{
	uint32_t val = usr_val;

	/* Read current register value first if it's a bitfield */
	if (field->is_bitfield) {
		uint32_t cur = fa_ioread(fa, base_off+field->offset);
		/* */
		cur &= ~field->mask; /* clear bits according to the mask */
		val = usr_val * (field->mask & -(field->mask));
		if (val & ~field->mask)
			dev_warn(fa->msgdev,
				"addr %p: value 0x%x doesn't fit mask 0x%x\n",
				base_off+field->offset, val, field->mask);
		val &= field->mask;
		val |= cur;
	}
	fa_iowrite(fa, val, base_off + field->offset);
}

static inline int fa_is_flag_set(struct fa_dev *fa, unsigned long flag)
{
	const struct fmc_adc_platform_data *fmc_adc_pdata;

        fmc_adc_pdata = fa->pdev->dev.platform_data;

        return !!(fmc_adc_pdata->flags & flag);
}

extern struct bin_attribute dev_attr_calibration;

/* Global variable exported by fa-core.c */
extern struct workqueue_struct *fa_workqueue;

/* Global variable exported by fa-regtable.c */
extern const struct zfa_field_desc zfad_regs[];

/* Functions exported by fa-core.c */
extern int zfad_fsm_command(struct fa_dev *fa, uint32_t command);
extern int zfad_convert_hw_range(uint32_t bitmask);
extern int fa_temperature_read(struct fa_dev *fa, int *temp);
extern int fa_trigger_software(struct fa_dev *fa);
extern int fa_fsm_wait_state(struct fa_dev *fa,
                             enum fa100m14b4c_fsm_state state,
                             unsigned int timeout_us);
extern int fa_adc_data_pattern_set(struct fa_dev *fa, uint16_t pattern,
                                   unsigned int enable);
extern int fa_adc_data_pattern_get(struct fa_dev *fa, uint16_t *pattern,
                                   unsigned int *enable);
extern int fa_adc_output_randomizer_set(struct fa_dev *fa, bool enable);
extern bool fa_adc_is_output_randomizer(struct fa_dev *fa);

/* Temporarily, user values are the same as hardware values */
extern int zfad_convert_user_range(uint32_t user_val);
extern int fa_adc_range_set(struct fa_dev *fa, struct zio_channel *chan,
			    int range);
extern int zfad_get_chx_index(unsigned long addr, unsigned int chan);

/* Function exported by fa-dma.c */
extern void fa_irq_work(struct work_struct *work);
extern int fa_dma_request_channel(struct fa_dev *fa);
extern void fa_dma_release_channel(struct fa_dev *fa);

/* Functions exported by fa-zio-drv.c */
extern int fa_zio_register(void);
extern void fa_zio_unregister(void);
extern int fa_zio_init(struct fa_dev *fa);
extern void fa_zio_exit(struct fa_dev *fa);

/* Functions exported by fa-zio-trg.c */
extern void zfat_trigger_source_reset(struct fa_dev *fa);
extern int fa_trig_init(void);
extern void fa_trig_exit(void);

/* Functions exported by fa-irq.c */
extern void zfat_irq_trg_fire(struct zio_cset *cset);
extern int fa_setup_irqs(struct fa_dev *fa);
extern int fa_free_irqs(struct fa_dev *fa);
extern int fa_enable_irqs(struct fa_dev *fa);
extern int fa_disable_irqs(struct fa_dev *fa);

/* functions exported by spi.c */
extern int fa_spi_xfer(struct fa_dev *fa, int cs, int num_bits,
		       uint32_t tx, uint32_t *rx);
extern int fa_spi_init(struct fa_dev *fd);
extern void fa_spi_exit(struct fa_dev *fd);

/* function exporetd by fa-calibration.c */
extern int fa_calib_init(struct fa_dev *fa);
extern void fa_calib_exit(struct fa_dev *fa);
extern void fa_calib_config(struct fa_dev *fa);
#define FA_CALIB_FLAG_READ_TEMP BIT(0)
extern void fa_calib_adc_config_chan(struct fa_dev *fa, unsigned int chan,
				     int32_t temperature, unsigned int flags);
extern int fa_calib_dac_config_chan(struct fa_dev *fa, unsigned int chan,
				    int32_t temperature, unsigned int flags);
extern void fa_calib_config_chan(struct fa_dev *fa, unsigned int chan,
			     int32_t temperature, unsigned int flags);

/* functions exported by fa-debug.c */
extern int fa_debug_init(struct fa_dev *fa);
extern void fa_debug_exit(struct fa_dev *fa);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,10,0)
extern int fa_hwmon_init(struct fa_dev *fa);
#endif

#endif /*  FMC_ADC_H_ */
