/*
 * Copyright CERN 2012
 * Author: Federico Vaga <federico.vaga@gmail.com>
 *
 * Header for the mezzanine ADC for the SPEC
 */

#ifndef _fa_dev_H_
#define _fa_dev_H_
#include <linux/scatterlist.h>

#include <linux/fmc.h>

#include <linux/zio.h>
#include <linux/zio-utils.h>

#define FA_GATEWARE_DEFAULT_NAME "fmc/spec-fmc-adc-v1.0.bin"
extern int enable_auto_start;

/* ADC register offset */
#define FA_DMA_MEM_OFF	0x01000
#define FA_CAR_MEM_OFF	0x01300
#define FA_UTC_MEM_OFF	0x01400
#define FA_IRQ_MEM_OFF	0x01500
#define FA_SPI_MEM_OFF	0x01700
#define FA_ADC_MEM_OFF	0x01900
#define FA_OWI_MEM_OFF	0X01A00 /* one-wire */

/* ADC DDR memory */
#define FA_MAX_ACQ_BYTE 0x10000000 /* 256MB */


/* ADC Calibration */
#define FA_CAL_PTR 0x0400 /* Pointer to calibration data in EEPROM (1 KByte) */
#define FA_CAL_LEN 108 /* Length of the calibration data */

enum fa_input_range {
	ZFA_10V = 0x0,
	ZFA_1V,
	ZFA_100mV,
	ZFA_OPEN, /* Open Range */
};
/*
 * fa_calibration_data: Calibration item
 * @offset calibration data for 4 channels
 * @gain calibration data for 4 channels
 * @temp calibration data temperature
 */
struct fa_calibration_data {
	uint16_t offset[4];
	uint16_t gain[4];
	uint16_t temp;
};

/*
 * dma_item: The information about a DMA transfer
 * @start_addr: pointer where start to retrieve data from device memory
 * @dma_addr_l: low 32bit of the dma address on host memory
 * @dma_addr_h: high 32bit of the dma address on host memory
 * @dma_len: number of bytes to transfer from device to host
 * @next_addr_l: low 32bit of the address of the next memory area to use
 * @next_addr_h: high 32bit of the address of the next memory area to use
 * @attribute: dma information about data transferm. At the moment it is used
 *             only to provide the "last item" bit, direction is fixed to
 *             device->host
 */
struct dma_item {
	uint32_t start_addr;	/* 0x00 */
	uint32_t dma_addr_l;	/* 0x04 */
	uint32_t dma_addr_h;	/* 0x08 */
	uint32_t dma_len;	/* 0x0C */
	uint32_t next_addr_l;	/* 0x10 */
	uint32_t next_addr_h;	/* 0x14 */
	uint32_t attribute;	/* 0x18 */
};

/*
 * fa_dev: is the descriptor of the FMC ADC mezzanine
 *
 * @fmc: the pointer to the fmc_device generic structure
 * @zdev: is the pointer to the real zio_device in use
 * @hwzdev: is the pointer to the fake zio_device, used to initialize and
 *          to remove a zio_device
 *
 * @n_shots: total number of programmed shots for an acquisition
 * @n_fires: number of trigger fire occurred within an acquisition
 * @n_trans: number of DMA transfer done for an acquisition
 *
 * @n_dma_err: number of errors
 */
struct fa_dev {
	struct fmc_device	*fmc;
	struct zio_device	*zdev;
	struct zio_device	*hwzdev;

	/* Acquisition */
	unsigned int		n_shots;
	unsigned int		n_fires;
	unsigned int		n_trans;

	/* Statistic informations */
	unsigned int		n_dma_err;

	/* one-wire */
	uint8_t ds18_id[8];
	unsigned long		next_t;
	int			temp;	/* temperature: scaled by 4 bits */

	/* Calibration Data */
	struct fa_calibration_data adc_cal_data[3];
	struct fa_calibration_data dac_cal_data[3];
};

/*
 *
 */
struct zfad_block {
	struct zio_block *block;
	unsigned int dev_addr;

	struct sg_table	sgt;
	struct dma_item	*items;
	dma_addr_t	dma_list_item;
	uint32_t	dev_mem_ptr;
};

extern int zfad_map_dma(struct zio_cset *cset,
			struct zfad_block *zfad_block);
extern void zfad_unmap_dma(struct zio_cset *cset,
			   struct zfad_block *zfad_block);

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
	/* Status registers */
	ZFA_STA_FSM,
	ZFA_STA_SERDES_PLL,
	ZFA_STA_SERDES_SYNCED,
	/* Configuration register */
	ZFAT_CFG_HW_SEL,
	ZFAT_CFG_HW_POL,
	ZFAT_CFG_HW_EN,
	ZFAT_CFG_SW_EN,
	ZFAT_CFG_INT_SEL,
	ZFAT_CFG_THRES,
	/* Delay*/
	ZFAT_DLY,
	/* Software */
	ZFAT_SW,
	/* Number of shots */
	ZFAT_SHOTS_NB,
	/* Sample rate */
	ZFAT_SR_DECI,
	/* Position address */
	ZFAT_POS,
	/* Pre-sample */
	ZFAT_PRE,
	/* Post-sample */
	ZFAT_POST,
	/* Sample counter */
	ZFAT_CNT,
	/* Channel 1 */
	ZFA_CH1_CTL_RANGE,
	ZFA_CH1_STA,
	ZFA_CH1_GAIN,
	ZFA_CH1_OFFSET,
	ZFA_CH1_CTL_TERM,
	/* Channel 2 */
	ZFA_CH2_CTL_RANGE,
	ZFA_CH2_STA,
	ZFA_CH2_GAIN,
	ZFA_CH2_OFFSET,
	ZFA_CH2_CTL_TERM,
	/* Channel 3 */
	ZFA_CH3_CTL_RANGE,
	ZFA_CH3_STA,
	ZFA_CH3_GAIN,
	ZFA_CH3_OFFSET,
	ZFA_CH3_CTL_TERM,
	/* Channel 4 */
	ZFA_CH4_CTL_RANGE,
	ZFA_CH4_STA,
	ZFA_CH4_GAIN,
	ZFA_CH4_OFFSET,
	ZFA_CH4_CTL_TERM,
	/* Other*/
	ZFA_CHx_CTL_RANGE,
	ZFA_CHx_STA,
	ZFA_CHx_GAIN,
	ZFA_CHx_OFFSET,
	ZFA_CHx_CTL_TERM,
	/* DMA */
	ZFA_DMA_CTL_SWP,
	ZFA_DMA_CTL_ABORT,
	ZFA_DMA_CTL_START,
	ZFA_DMA_STA,
	ZFA_DMA_ADDR,
	ZFA_DMA_ADDR_L,
	ZFA_DMA_ADDR_H,
	ZFA_DMA_LEN,
	ZFA_DMA_NEXT_L,
	ZFA_DMA_NEXT_H,
	ZFA_DMA_BR_DIR,
	ZFA_DMA_BR_LAST,
	/* IRQ */
	ZFA_IRQ_MULTI,
	ZFA_IRQ_SRC,
	ZFA_IRQ_MASK,
	/* UTC core */
	ZFA_UTC_SECONDS,
	ZFA_UTC_COARSE,
	ZFA_UTC_TRIG_META,
	ZFA_UTC_TRIG_SECONDS,
	ZFA_UTC_TRIG_COARSE,
	ZFA_UTC_TRIG_FINE,
	ZFA_UTC_ACQ_START_META,
	ZFA_UTC_ACQ_START_SECONDS,
	ZFA_UTC_ACQ_START_COARSE,
	ZFA_UTC_ACQ_START_FINE,
	ZFA_UTC_ACQ_STOP_META,
	ZFA_UTC_ACQ_STOP_SECONDS,
	ZFA_UTC_ACQ_STOP_COARSE,
	ZFA_UTC_ACQ_STOP_FINE,
	ZFA_UTC_ACQ_END_META,
	ZFA_UTC_ACQ_END_SECONDS,
	ZFA_UTC_ACQ_END_COARSE,
	ZFA_UTC_ACQ_END_FINE,
	/* Carrier CSR */
	ZFA_CAR_FMC_PRES,
	ZFA_CAR_P2L_PLL,
	ZFA_CAR_SYS_PLL,
	ZFA_CAR_DDR_CAL,
	/* Other "address" */
	ZFA_SW_R_NOADDRES_NBIT,
	ZFA_SW_R_NOADDRES_TEMP,
	ZFA_SW_R_NOADDERS_AUTO,
};
/* Registers lists used in fd-zio-drv.c */
extern const struct zio_field_desc zfad_regs[];

/*
 * ZFA_CHx_MULT
 * address offset between two registers of the same type on consecutive channel
 */
#define ZFA_CHx_MULT 4
/*
 * ZFA_CHx_CH1_DIFF
 * index difference between fake register index CHx and the first channel
 * index CH1
 */
#define ZFA_CHx_CH1_DIFF ZFA_CHx_CTL_RANGE - ZFA_CH1_CTL_RANGE

enum zfa_fsm_cmd {
	ZFA_NONE =	0x0,
	ZFA_START =	0x1,
	ZFA_STOP =	0x2,
};
/* All possible state of the state machine, other values are invalid*/
enum zfa_fsm_state {
	ZFA_STATE_IDLE = 0x1,
	ZFA_STATE_PRE,
	ZFA_STATE_POST,
	ZFA_STATE_WAIT,
	ZFA_STATE_DECR,
};
/* All possible interrupt available */
enum zfat_irq {
	ZFAT_NONE =	0x0,
	ZFAT_DMA_DONE =	0x1,
	ZFAT_DMA_ERR =	0x2,
	ZFAT_TRG_FIRE =	0x4,
	ZFAT_ACQ_END =	0x8,
	ZFAT_ALL =	0xF,
};

#ifdef __KERNEL__ /* All the rest is only of kernel users */
#include <linux/zio.h>
#include <linux/zio-trigger.h>
#include <linux/zio-utils.h>

#include <linux/dma-mapping.h>
#include <linux/scatterlist.h>

extern struct zio_trigger_type zfat_type;

static inline int zfat_overflow_detection(struct zio_ti *ti, unsigned int addr,
					  uint32_t val)
{
	struct zio_attribute *ti_zattr = ti->zattr_set.std_zattr;
	uint32_t pre_t, post_t, nshot_t;
	size_t size;

	if (!addr)
		return 0;

	pre_t = addr == ZFAT_PRE ? val :
			ti_zattr[ZIO_ATTR_TRIG_PRE_SAMP].value;
	post_t = addr == ZFAT_POST ? val :
			ti_zattr[ZIO_ATTR_TRIG_POST_SAMP].value;
	if (ti->cset->trig != &zfat_type)
		nshot_t = 1; /* with any other trigger work in one-shot mode */
	else
		nshot_t = addr == ZFAT_SHOTS_NB ? val :
			  ti_zattr[ZIO_ATTR_TRIG_N_SHOTS].value;

	size = ((pre_t + post_t) * ti->cset->ssize * nshot_t) *
		(ti->cset->n_chan - 1);
	if (size >= FA_MAX_ACQ_BYTE) {
		dev_err(&ti->head.dev, "Cannot acquire, dev memory overflow\n");
		return -ENOMEM;
	}
	return 0;
}

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

static inline int zfa_common_conf_set(struct fa_dev *fa,
				      enum zfadc_dregs_enum index,
				      uint32_t usr_val)
{
	uint32_t cur, val;

	if ((usr_val & (~zfad_regs[index].mask))) {
		dev_err(fa->fmc->hwdev, "value 0x%x must fit mask 0x%x\n",
			usr_val, zfad_regs[index].mask);
		return -EINVAL;
	}
	/* Read current register*/
	cur = fmc_readl(fa->fmc, zfad_regs[index].addr);
	val = zio_set_field(&zfad_regs[index], cur, usr_val);
	/* FIXME re-write usr_val when possible (zio need a patch) */
	/* If the attribute has a valid address */
	fmc_writel(fa->fmc, val, zfad_regs[index].addr);
	return 0;
}
static inline void zfa_common_info_get(struct fa_dev *fa,
				       enum zfadc_dregs_enum index,
				       uint32_t *usr_val)
{
	uint32_t cur;

	/* Read current register*/
	cur = fmc_readl(fa->fmc, zfad_regs[index].addr);
	/* Return the value */
	*usr_val = zio_get_field(&zfad_regs[index], cur);
}


/* Functions exported by fa-zio.c */
extern int fa_zio_register(void);
extern void fa_zio_unregister(void);
extern int fa_zio_init(struct fa_dev *fa);
extern void fa_zio_exit(struct fa_dev *fa);

/* Functions exported by fa-spec.c */
extern int fa_spec_init(void);
extern void fa_spec_exit(void);
/* Functions exported by onewire.c */
extern int fa_onewire_init(struct fa_dev *fa);
extern void fa_onewire_exit(struct fa_dev *fa);
extern int fa_read_temp(struct fa_dev *fa, int verbose);
/* functions exported by spi.c */
extern int fa_spi_xfer(struct fa_dev *fa, int cs, int num_bits,
		       uint32_t tx, uint32_t *rx);
extern int fa_spi_init(struct fa_dev *fd);
extern void fa_spi_exit(struct fa_dev *fd);
/* function in fa-zio-drv.c */
extern int zfad_fsm_command(struct fa_dev *fa, uint32_t command);

#endif /* __KERNEL__ */
#endif /* _fa_dev_H_ */
