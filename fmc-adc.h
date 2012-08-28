/*
 * Copyright CERN 2012
 * Author: Federico Vaga <federico.vaga@gmail.com>
 *
 * Header for the mezzanine ADC for the SPEC
 */

#ifndef _fa_dev_H_
#define _fa_dev_H_

#include <linux/zio.h>
#include "spec.h"

#define FA_GATEWARE_DEFAULT_NAME "fmc/fmc-adc.bin"

/* ADC register offset */
#define FA_DMA_MEM_OFF	0x00000
#define FA_CAR_MEM_OFF	0x30000
#define FA_UTC_MEM_OFF	0x40000
#define FA_IRQ_MEM_OFF	0x50000
#define FA_SPI_MEM_OFF	0x70000
#define FA_ADC_MEM_OFF	0x90000
#define FA_OWI_MEM_OFF	0XA0000 /* one-wire */

/* ADC DDR memory */
#define FA_MAX_ACQ_BYTE 0x10000000 /* 256MB */

/* The information about a DMA transfer */
struct dma_item {
	uint32_t start_addr;	/* 0x00 */
	uint32_t dma_addr_l;	/* 0x04 */
	uint32_t dma_addr_h;	/* 0x08 */
	uint32_t dma_len;	/* 0x0C */
	uint32_t next_addr_l;	/* 0x10 */
	uint32_t next_addr_h;	/* 0x14 */
	uint32_t attribute;	/* 0x18 */
	/*
	 * attribute is used only to provide the "last item" bit, direction is
	 * fixed to device->host
	 */
};

/*
 * fa_dev: is the descriptor of the FMC ADC mezzanine
 *
 * @fmc: the pointer to the fmc_device generic structure
 * @spec: this driver depends on spec (FIXME no svec at the moment)
 * @zdev: is the pointer to the real zio_device in use
 * @hwzdev: is the pointer to the fake zio_device, used to initialize and
 *          to remove a zio_device
 *
 * DMA variable: these variables are used by the fa-dma.c engine
 * @sgt: scatter/gather table
 * @items: vector of dma_item for a single DMA transfer
 * @dma_list_item: contains the address of items mapped for DMA access. The
 *                 device will access items to retrive information about the
 *                 DMA items to transfer.
 * @lst_dev_mem: contains the device address where last block is stored.
 *               FIXME when it reset?
 * @cur_dev_mem: contains the device address to the current block of data in
 *               transmission from device. It is updated for each dma_items
 *               transfer
 * @base: the base address for to access device registers (BAR0)
 */
struct fa_dev {
	struct fmc_device	*fmc;
	struct spec_dev		*spec;
	struct zio_device	*zdev;
	struct zio_device	*hwzdev;

	/* DMA variable */
	struct sg_table		sgt;
	struct dma_item		*items;
	dma_addr_t		dma_list_item;
	uint32_t		lst_dev_mem;
	uint32_t		cur_dev_mem;

	unsigned char __iomem	*base;

	/* one-wire */
	uint8_t ds18_id[8];
	unsigned long		next_t;
	int			temp;	/* temperature: scaled by 4 bits */
};

extern int zfad_map_dma(struct zio_cset *cset);
extern void zfad_unmap_dma(struct zio_cset *cset);

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
	ZFDAC_CFG_HW_SEL,
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
	/* Channel 2 */
	ZFA_CH2_CTL_RANGE,
	ZFA_CH2_STA,
	ZFA_CH2_GAIN,
	ZFA_CH2_OFFSET,
	/* Channel 3 */
	ZFA_CH3_CTL_RANGE,
	ZFA_CH3_STA,
	ZFA_CH3_GAIN,
	ZFA_CH3_OFFSET,
	/* Channel 4 */
	ZFA_CH4_CTL_RANGE,
	ZFA_CH4_STA,
	ZFA_CH4_GAIN,
	ZFA_CH4_OFFSET,
	/* Other*/
	ZFA_CHx_CTL_RANGE,
	ZFA_CHx_STA,
	ZFA_CHx_GAIN,
	ZFA_CHx_OFFSET,
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
};
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

/* Device registers */
extern const struct zio_reg_desc zfad_regs[];

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

#include <linux/dma-mapping.h>
#include <linux/scatterlist.h>

static inline struct fa_dev *get_zfadc(struct device *dev)
{
	switch (to_zio_head(dev)->zobj_type) {
		case ZDEV:
			return to_zio_dev(dev)->priv_d;
		case ZCSET:
			return to_zio_cset(dev)->zdev->priv_d;
		case ZCHAN:
			return to_zio_chan(dev)->cset->zdev->priv_d;
		case ZTI:
			return to_zio_ti(dev)->cset->zdev->priv_d;
		default:
			return NULL;
	}
	return NULL;
}

static inline struct spec_dev *get_spec(struct device *dev)
{
	return get_zfadc(dev)->spec;
}

/* FIXME convert fa_{read|write}_reg to fmc_{writel|readl} when fmc is fixed */
static inline int zfa_common_conf_set(struct fa_dev *fa,
				      const struct zio_reg_desc *reg,
				      uint32_t usr_val)
{
	uint32_t cur, val;

	if ((usr_val & (~reg->mask))) {
		dev_err(fa->fmc->hwdev, "value 0x%x must fit mask 0x%x\n",
			usr_val, reg->mask);
		return -EINVAL;
	}
	/* Read current register*/
	cur = readl(fa->base + reg->addr);
	val = zio_reg_set(reg, cur, usr_val);
	/* FIXME re-write usr_val when possible (zio need a patch) */
	/* If the attribute has a valid address */
	writel(val, fa->base + reg->addr);
	return 0;
}
static inline void zfa_common_info_get(struct fa_dev *fa,
				       const struct zio_reg_desc *reg,
				       uint32_t *usr_val)
{
	uint32_t cur;

	/* Read current register*/
	cur = readl(fa->base + reg->addr);
	/* Return the value */
	*usr_val = zio_reg_get(reg, cur);
}

extern struct zio_trigger_type zfat_type;
/* Registers lists used in fd-zio-drv.c and fd-zio-trg.c */
extern const struct zio_reg_desc zfad_regs[];


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

#endif /* __KERNEL__ */
#endif /* _fa_dev_H_ */
