/*
 * Copyright CERN 2012 (author Federico Vaga)
 *
 * Driver for the mezzanine ADC for the SPEC
 */

#ifndef _FMC_ADC_H_
#define _FMC_ADC_H_

#include <linux/zio.h>
#include "spec.h"

#define FA_GATEWARE_DEFAULT_NAME "fmc/fmc-adc.bin"

/* ADC register offset */
#define FA_DMA_MEM_OFF	0x00000
#define FA_UTC_MEM_OFF	0x40000
#define FA_IRQ_MEM_OFF	0x50000
#define FA_ADC_MEM_OFF	0x90000
#define FA_OWI_MEM_OFF	0XA0000 /* one-wire */

/* ADC DDR memory */
#define FA_MAX_ACQ_BYTE 0x10000000 /* 256MB */

struct spec_fa {
	struct fmc_device	*fmc;
	struct spec_dev		*spec;
	struct zio_device	*hwzdev;

	struct sg_table		sgt;	/* scatter/gather table */
	unsigned char __iomem	*base;	/* regs files are byte-oriented */

	/* one-wire */
	uint8_t ds18_id[8];
	unsigned long		next_t;
	int			temp;	/* temperature: scaled by 4 bits */
};

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

extern int zfad_map_dma(struct zio_cset *cset);
extern void zfad_unmap_dma(struct zio_cset *cset);

/*
 * ZFA_CHx_MULT
 * address offset between two registers of the same type on consecutive channel
 */
#define ZFA_CHx_MULT 5

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
	ZFA_CH1_CTL_TERM,
	ZFA_CH1_STA,
	ZFA_CH1_GAIN,
	ZFA_CH1_OFFSET,
	/* Channel 2 */
	ZFA_CH2_CTL_RANGE,
	ZFA_CH2_CTL_TERM,
	ZFA_CH2_STA,
	ZFA_CH2_GAIN,
	ZFA_CH2_OFFSET,
	/* Channel 3 */
	ZFA_CH3_CTL_RANGE,
	ZFA_CH3_CTL_TERM,
	ZFA_CH3_STA,
	ZFA_CH3_GAIN,
	ZFA_CH3_OFFSET,
	/* Channel 4 */
	ZFA_CH4_CTL_RANGE,
	ZFA_CH4_CTL_TERM,
	ZFA_CH4_STA,
	ZFA_CH4_GAIN,
	ZFA_CH4_OFFSET,
	/* Other*/
	ZFA_CHx_CTL_RANGE,
	ZFA_CHx_CTL_TERM,
	ZFA_CHx_STA,
	ZFA_CHx_GAIN,
	ZFA_CHx_OFFSET,
	ZFA_SW_R_NOADDRES,
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
};
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

static inline struct spec_fa *get_zfadc(struct device *dev)
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

static inline uint32_t fa_read_reg(struct spec_fa *fa,
				    const struct zio_reg_desc *reg)
{
	return readl(fa->base + reg->addr);
}
static inline void fa_write_reg(uint32_t val, struct spec_fa *fa,
				 const struct zio_reg_desc *reg)
{
	writel(val, fa->base + reg->addr);
}


static inline int zfa_common_conf_set(struct device *dev,
				      const struct zio_reg_desc *reg,
				      uint32_t usr_val)
{
	uint32_t cur, val;

	if ((usr_val & (~reg->mask))) {
		dev_err(dev, "the value 0x%x must fit the mask 0x%x\n",
			usr_val, reg->mask);
		return -EINVAL;
	}
	/* Read current register*/
	cur = fa_read_reg(get_zfadc(dev), reg);
	/* Mask the value */
	cur &= (~(reg->mask << reg->shift));
	/* Write the new value */
	val = cur | (usr_val << reg->shift);
	/* FIXME re-write usr_val when possible (zio need a patch) */
	/* If the attribute has a valid address */
	fa_write_reg(val, get_zfadc(dev), reg);
	return 0;
}
static inline void zfa_common_info_get(struct device *dev,
				       const struct zio_reg_desc *reg,
				       uint32_t *usr_val)
{
	uint32_t cur;

	/* Read current register*/
	cur = fa_read_reg(get_zfadc(dev), reg);
	/* Mask the value */
	cur &= (reg->mask << reg->shift);
	/* Return the value */
	*usr_val = cur >> reg->shift;
}

extern struct zio_trigger_type zfat_type;
/* Registers lists used in fd-zio-drv.c and fd-zio-trg.c */
extern const struct zio_reg_desc zfad_regs[];


/* Functions exported by fa-zio.c */
extern int fa_zio_register(void);
extern void fa_zio_unregister(void);
extern int fa_zio_init(struct spec_fa *fa);
extern void fa_zio_exit(struct spec_fa *fa);

/* Functions exported by fa-spec.c */
extern int fa_spec_init(void);
extern void fa_spec_exit(void);
/* Functions exported by onewire.c */
extern int fa_onewire_init(struct spec_fa *fa);
extern void fa_onewire_exit(struct spec_fa *fa);

#endif /* __KERNEL__ */
#endif /* _FMC_ADC_H_ */
