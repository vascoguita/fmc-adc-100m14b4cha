/*
 * Copyright CERN 2012
 * Author: Federico Vaga <federico.vaga@gmail.com>
 *
 * Header for the mezzanine ADC for the SPEC
 */

#ifndef FMC_ADC_H_ /* too generic, maybe */
#define FMC_ADC_H_
#include <linux/scatterlist.h>
#include <linux/fmc.h>
#include <linux/zio.h>
#include "field-desc.h"

#define FA_GATEWARE_DEFAULT_NAME "fmc/adc-100m14b.bin"
extern int enable_auto_start;

#define FA_NCHAN 4 /* We have 4 of them,no way out of it */

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


enum fa_input_range {
	ZFA_RANGE_10V = 0x0,
	ZFA_RANGE_1V,
	ZFA_RANGE_100mV,
	ZFA_RANGE_OPEN,		/* Channel disconnected from ADC */
};
#define ZFA_RANGE_MIN  0 /* 10V above */
#define ZFA_RANGE_MAX  2 /* 100mV above */


/* ADC and DAC Calibration, from  EEPROM */
struct fa_calib_stanza {
	int16_t offset[4]; /* One per channel */
	uint16_t gain[4];  /* One per channel */
	uint16_t temperature;
};

struct fa_calib {
	struct fa_calib_stanza adc[3];  /* For input, one per range */
	struct fa_calib_stanza dac[3];  /* For user offset, one per range */
};

#define FA_CAL_OFFSET		 0x0100 /* Offset in EEPROM */

#define FA_CAL_NO_OFFSET	((int16_t)0x0000)
#define FA_CAL_NO_GAIN		((uint16_t)0x8000)

/*
 * fa_dma_item: The information about a DMA transfer
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
struct fa_dma_item {
	uint32_t start_addr;	/* 0x00 */
	uint32_t dma_addr_l;	/* 0x04 */
	uint32_t dma_addr_h;	/* 0x08 */
	uint32_t dma_len;	/* 0x0C */
	uint32_t next_addr_l;	/* 0x10 */
	uint32_t next_addr_h;	/* 0x14 */
	uint32_t attribute;	/* 0x18 */
	uint32_t reserved;	/* ouch */
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
 *
 * @n_dma_err: number of errors
 *
 * @sgt is the scatter/gather table that describe the DMA acquisition
 * @item a list on fa_dma_item to describe
 * @dma_list_item is a DMA address pointer to the fa_dma_item list
 */
struct fa_dev {
	struct fmc_device	*fmc;
	struct zio_device	*zdev;
	struct zio_device	*hwzdev;

	/* Acquisition */
	unsigned int		n_shots;
	unsigned int		n_fires;

	/* Statistic informations */
	unsigned int		n_dma_err;

	/* Configuration */
	int			user_offset[4]; /* one per channel */

	/* one-wire */
	uint8_t ds18_id[8];
	unsigned long		next_t;
	int			temp;	/* temperature: scaled by 4 bits */

	/* Calibration Data */
	struct fa_calib calib;

	/* DMA attributes */
	struct sg_table	sgt;
	struct fa_dma_item	*items;
	dma_addr_t		dma_list_item;
};

/*
 * zfad_block
 * @block is zio_block which contains data and metadata from a single shot
 * @dev_mem_off is the offset in ADC internal memory. It points to the first
 *              sample of the stored shot
 * @first_nent is the index of the first nent used for this block
 */
struct zfad_block {
	struct zio_block *block;
	uint32_t	dev_mem_off;
	unsigned int first_nent;
};

extern int zfad_map_dma(struct zio_cset *cset,
			struct zfad_block *zfad_block,
			unsigned int n_blocks);
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
extern const struct zfa_field_desc zfad_regs[];

/*
 * ZFA_CHx_MULT
 * address offset between two registers of the same type on consecutive channel
 */
#define ZFA_CHx_MULT 5

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
#include <linux/zio-trigger.h>

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

	size = ((pre_t + post_t) * ti->cset->ssize * nshot_t) * FA_NCHAN;
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

/* SPI Slave Select lines (as defined in spec_top_fmc_adc_100Ms.vhd) */
#define FA_SPI_SS_ADC		0
#define FA_SPI_SS_DAC(ch)	((ch) + 1)

/* Hardware filed-based access */
static inline int zfa_hardware_write(struct fa_dev *fa,
				      enum zfadc_dregs_enum index,
				      uint32_t usr_val)
{
	uint32_t cur, val;

	if ((usr_val & (~zfad_regs[index].mask))) {
		dev_info(&fa->fmc->dev, "value 0x%x must fit mask 0x%x\n",
			usr_val, zfad_regs[index].mask);
		return -EINVAL;
	}
	/* Read current register*/
	cur = fmc_readl(fa->fmc, zfad_regs[index].addr);
	val = zfa_set_field(&zfad_regs[index], cur, usr_val);
	/* FIXME re-write usr_val when possible (zio need a patch) */
	/* If the attribute has a valid address */
	fmc_writel(fa->fmc, val, zfad_regs[index].addr);
	return 0;
}
static inline void zfa_hardware_read(struct fa_dev *fa,
				       enum zfadc_dregs_enum index,
				       uint32_t *usr_val)
{
	uint32_t cur;

	/* Read current register*/
	cur = fmc_readl(fa->fmc, zfad_regs[index].addr);
	/* Return the value */
	*usr_val = zfa_get_field(&zfad_regs[index], cur);
}

/* Functions exported by fa-core.c */
extern int zfad_fsm_command(struct fa_dev *fa, uint32_t command);
extern int zfad_apply_user_offset(struct fa_dev *fa, struct zio_channel *chan,
				  uint32_t usr_val);
extern void zfad_reset_offset(struct fa_dev *fa);
extern int zfad_convert_hw_range(uint32_t bitmask);
extern int zfad_set_range(struct fa_dev *fa, struct zio_channel *chan,
			  int range);
extern int zfad_get_chx_index(unsigned long addr, struct zio_channel *chan);

/* Functions exported by fa-zio-drv.c */
extern int fa_zio_register(void);
extern void fa_zio_unregister(void);
extern int fa_zio_init(struct fa_dev *fa);
extern void fa_zio_exit(struct fa_dev *fa);

/* Functions exported by fa-zio-trg.c */
extern int fa_trig_init(void);
extern void fa_trig_exit(void);

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

/* function exporetd by fa-calibration.c */
extern void fa_read_eeprom_calib(struct fa_dev *fa);

#endif /* __KERNEL__ */
#endif /*  FMC_ADC_H_ */
