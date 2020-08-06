// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright CERN 2012-2019
 * Author: Federico Vaga <federico.vaga@gmail.com>
 */

#ifndef FMC_ADC_100M14B4C_H_
#define FMC_ADC_100M14B4C_H_

#ifdef __KERNEL__
#include <linux/types.h>
#else
#include <stdint.h>
#ifndef BIT
#define BIT(nr) (1UL << (nr))
#endif
#endif

/* Trigger sources */
#define FA100M14B4C_TRG_SRC_EXT BIT(0)
#define FA100M14B4C_TRG_SRC_SW BIT(1)
#define FA100M14B4C_TRG_SRC_TIM BIT(4)
#define FA100M14B4C_TRG_SRC_ALT BIT(5)
#define FA100M14B4C_TRG_SRC_CH1 BIT(8)
#define FA100M14B4C_TRG_SRC_CH2 BIT(9)
#define FA100M14B4C_TRG_SRC_CH3 BIT(10)
#define FA100M14B4C_TRG_SRC_CH4 BIT(11)
#define FA100M14B4C_TRG_SRC_CHx(_x) (FA100M14B4C_TRG_SRC_CH1 << ((_x) - 1))

/* Trigger Polarity */
#define FA100M14B4C_TRG_POL_EXT FA100M14B4C_TRG_SRC_EXT
#define FA100M14B4C_TRG_POL_CH1 FA100M14B4C_TRG_SRC_CH1
#define FA100M14B4C_TRG_POL_CH2 FA100M14B4C_TRG_SRC_CH2
#define FA100M14B4C_TRG_POL_CH3 FA100M14B4C_TRG_SRC_CH3
#define FA100M14B4C_TRG_POL_CH4 FA100M14B4C_TRG_SRC_CH4
#define FA100M14B4C_TRG_POL_CHx(_x) (FA100M14B4C_TRG_POL_CH1 << ((_x) - 1))

enum fa_versions {
	ADC_VER = 0,
};

/*
 * Trigger Extended Attribute Enumeration
 */
enum fa100m14b4c_trg_ext_attr {
	/*
	 * The trigger extended attribute order is the same in the declaration
	 * and in the zio_control, so we can always use enumeration. But, the
	 * enumeration must start with 0 followed by only consecutive value.
	 *
	 * The parameters are not exposed to user space by zio_controle, so it
	 * is not necessary to export to user space the correspondent enum
	 */
	FA100M14B4C_TATTR_STA = 0,
	FA100M14B4C_TATTR_SRC,
	FA100M14B4C_TATTR_POL,
	FA100M14B4C_TATTR_EXT_DLY,
	FA100M14B4C_TATTR_CH1_THRES,
	FA100M14B4C_TATTR_CH2_THRES,
	FA100M14B4C_TATTR_CH3_THRES,
	FA100M14B4C_TATTR_CH4_THRES,
	FA100M14B4C_TATTR_CH1_HYST,
	FA100M14B4C_TATTR_CH2_HYST,
	FA100M14B4C_TATTR_CH3_HYST,
	FA100M14B4C_TATTR_CH4_HYST,
	FA100M14B4C_TATTR_CH1_DLY,
	FA100M14B4C_TATTR_CH2_DLY,
	FA100M14B4C_TATTR_CH3_DLY,
	FA100M14B4C_TATTR_CH4_DLY,
	FA100M14B4C_TATTR_TRG_TIM_SU,
	FA100M14B4C_TATTR_TRG_TIM_SL,
	FA100M14B4C_TATTR_TRG_TIM_C,

#ifdef __KERNEL__
	FA100M14B4C_TATTR_SW_FIRE,
	FA100M14B4C_TATTR_TRG_SU,
	FA100M14B4C_TATTR_TRG_SL,
	FA100M14B4C_TATTR_TRG_C,
#endif
};

/*
 * Device Extended Attribute Enumeration
 */
enum fa100m14b4c_dev_ext_attr {
	/*
	 * NOTE: At the moment the only extended attributes we have in
	 * the device hierarchy are in the cset level, so we can safely
	 * start from index 0
	 */
	FA100M14B4C_DATTR_DECI = 0,
	FA100M14B4C_DATTR_CH0_OFFSET,
	FA100M14B4C_DATTR_CH1_OFFSET,
	FA100M14B4C_DATTR_CH2_OFFSET,
	FA100M14B4C_DATTR_CH3_OFFSET,
	FA100M14B4C_DATTR_CH0_VREF,
	FA100M14B4C_DATTR_CH1_VREF,
	FA100M14B4C_DATTR_CH2_VREF,
	FA100M14B4C_DATTR_CH3_VREF,
	FA100M14B4C_DATTR_CH0_50TERM,
	FA100M14B4C_DATTR_CH1_50TERM,
	FA100M14B4C_DATTR_CH2_50TERM,
	FA100M14B4C_DATTR_CH3_50TERM,
	FA100M14B4C_DATTR_ACQ_START_S,
	FA100M14B4C_DATTR_ACQ_START_C,
	FA100M14B4C_DATTR_ACQ_START_F,
};

#define FA100M14B4C_UTC_CLOCK_FREQ 125000000
#define FA100M14B4C_UTC_CLOCK_NS  8
#define FA100M14B4C_NCHAN 4 /* We have 4 of them,no way out of it */
#define FA100M14B4C_NBIT 14

/* ADC DDR memory */
#define FA100M14B4C_MAX_ACQ_BYTE 0x10000000 /* 256MB */

enum fa100m14b4c_input_range {
	FA100M14B4C_RANGE_10V = 0x0,
	FA100M14B4C_RANGE_1V,
	FA100M14B4C_RANGE_100mV,
	FA100M14B4C_RANGE_OPEN,	/* Channel disconnected from ADC */
	FA100M14B4C_RANGE_10V_CAL,	/* Channel disconnected from ADC */
	FA100M14B4C_RANGE_1V_CAL,	/* Channel disconnected from ADC */
	FA100M14B4C_RANGE_100mV_CAL,	/* Channel disconnected from ADC */
};

enum fa100m14b4c_fsm_cmd {
	FA100M14B4C_CMD_NONE =	0x0,
	FA100M14B4C_CMD_START =	0x1,
	FA100M14B4C_CMD_STOP =	0x2,
};
/* All possible state of the state machine, other values are invalid*/
enum fa100m14b4c_fsm_state {
	FA100M14B4C_STATE_IDLE = 0x1,
	FA100M14B4C_STATE_PRE,
	FA100M14B4C_STATE_POST,
	FA100M14B4C_STATE_WAIT,
	FA100M14B4C_STATE_DECR,
};

/* ADC and DAC Calibration, from  EEPROM */
struct fa_calib_stanza {
	int16_t offset[4]; /* One per channel */
	uint16_t gain[4];  /* One per channel */
	uint16_t temperature;
};

#define FA_CALIB_STANZA_N 3
struct fa_calib {
	struct fa_calib_stanza adc[FA_CALIB_STANZA_N];  /* For input, one per range */
	struct fa_calib_stanza dac[FA_CALIB_STANZA_N];  /* For user offset, one per range */
};

#ifdef __KERNEL__ /* All the rest is only of kernel users */
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/scatterlist.h>
#include <linux/workqueue.h>
#include <linux/debugfs.h>
#include <linux/platform_device.h>
#include <linux/fmc.h>

#include <linux/zio.h>
#include <linux/zio-dma.h>
#include <linux/zio-sysfs.h>
#include <linux/zio-buffer.h>
#include <linux/zio-trigger.h>

#include "field-desc.h"
#include <platform_data/fmc-adc-100m14b4cha.h>

#define ADC_CSR_OFF 0x1000
#define ADC_EIC_OFF 0x1500
#define ADC_OW_OFF  0x1700
#define ADC_SPI_OFF 0x1800
#define ADC_UTC_OFF 0x1900

extern int fa_enable_test_data_adc;

#define ADC_DMA 0

enum fa_irq_resource {
	ADC_IRQ_TRG = 0,
};

enum fa_mem_resource {
	ADC_MEM_BASE = 0,
};

enum fa_bus_resource {
	ADC_CARR_VME_ADDR,
};

struct fa_memory_ops {
	u32 (*read)(void *addr);
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
	/* Pattern data for the ADC chip */
	ZFAT_ADC_TST_PATTERN,
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
 * @user_offset: user offset (micro-Volts)
 * @zero_offset: necessary offset to push the channel to zero (micro-Volts)
 */
struct fa_dev {
	struct device *msgdev; /**< device used to print messages */
	/* the pointer to the platform_device generic structure */
	struct platform_device	*pdev;
	/* the pointer to the real zio_device in use */
	struct zio_device	*zdev;
	/* the pointer to the fake zio_device, used for init/remove */
	struct zio_device	*hwzdev;

	struct fmc_slot	*slot;
	struct fa_memory_ops	memops;

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
	int32_t		user_offset[4]; /* one per channel */
	int32_t		zero_offset[FA100M14B4C_NCHAN];
	/* one-wire */
	uint8_t ds18_id[8];
	unsigned long		next_t;
	int			temp;	/* temperature: scaled by 4 bits */

	/* Calibration Data */
	struct fa_calib calib;
	int32_t range[FA100M14B4C_NCHAN];
	struct timer_list calib_timer;

	/* flag  */
	int enable_auto_start;

	struct dentry *dbg_dir;
	struct debugfs_regset32 dbg_reg32;
	struct dentry *dbg_reg;
	struct dentry *dbg_reg_spi;

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
	uint32_t cur, val;

	val = usr_val;
	/* Read current register value first if it's a bitfield */
	if (field->is_bitfield) {
		cur = fa_ioread(fa, base_off+field->offset);
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
extern int zfad_apply_offset(struct zio_channel *chan);
extern void zfad_reset_offset(struct fa_dev *fa);
extern int zfad_convert_hw_range(uint32_t bitmask);
extern uint32_t fa_temperature_read(struct fa_dev *fa);

/* Temporarily, user values are the same as hardware values */
extern int zfad_convert_user_range(uint32_t user_val);
extern int zfad_set_range(struct fa_dev *fa, struct zio_channel *chan,
			  int range);
extern int zfad_get_chx_index(unsigned long addr, unsigned int chan);
extern int zfad_pattern_data_enable(struct fa_dev *fa, uint16_t pattern,
				    unsigned int enable);

/* Function exported by fa-dma.c */
extern void fa_irq_work(struct work_struct *work);

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
extern int fa_calib_adc_config(struct fa_dev *fa);

/* functions exported by fa-debug.c */
extern int fa_debug_init(struct fa_dev *fa);
extern void fa_debug_exit(struct fa_dev *fa);

#endif /* __KERNEL__ */
#endif /*  FMC_ADC_H_ */
