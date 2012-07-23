/*
 * Copyright CERN 2012 (author Federico Vaga)
 *
 * Driver for the mezzanine ADC for the SPEC
 */

#ifndef _FMC_ADC_H_
#define _FMC_ADC_H_

#include "spec.h"

/* ADC register offset */
#define FA_DMA_MEM_OFF	0x00000
#define FA_UTC_MEM_OFF	0x40000
#define FA_IRQ_MEM_OFF	0x50000
#define FA_ADC_MEM_OFF	0x90000

struct spec_fa {
	struct spec_dev		*spec;
	struct zio_device	*hwzdev;

	struct sg_table		sgt;	/* scatter/gather table */
	unsigned char __iomem	*base;	/* regs files are byte-oriented */
};

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

};
/* All possible state of the state machine, other values are invalid*/
enum zfa_fsm_state {
	ZFA_STATE_IDLE =	0x1,
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


static inline uint32_t sg_dma_address_l(struct scatterlist *sg)
{
	return ((uint32_t) (sg_dma_address(sg) & 0xFFFFFFFF));
}
static inline uint32_t sg_dma_address_h(struct scatterlist *sg)
{
	return ((uint32_t) (32 >> (sg_dma_address(sg) & (~0xFFFFFFFF))));
}


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
		dev_err(dev, "the value must fit the mask 0x%x\n", reg->mask);
		return -EINVAL;
	}
	/* Read current register*/
	cur = fa_read_reg(get_zfadc(dev), reg);
	/* Mask the value */
	cur &= (~(reg->mask << reg->off));
	/* Write the new value */
	val = cur | (usr_val << reg->off);
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
	cur &= (reg->mask << reg->off);
	/* Return the value */
	*usr_val = cur >> reg->off;
}

extern struct zio_trigger_type zfat_type;
/* Registers lists used in fd-zio-drv.c and fd-zio-trg.c */
extern const struct zio_reg_desc zfad_regs[];

/* Functions exported by fd-core.c */
extern int fa_probe(struct spec_dev *dev);
extern void fa_remove(struct spec_dev *dev);
/* Functions exported by fa-zio.c */
extern int fa_zio_register(void);
extern void fa_zio_unregister(void);
extern int fa_zio_init(struct spec_fa *fa);
extern void fa_zio_exit(struct spec_fa *fa);

/* Functions exported by fa-spec.c */
extern int fa_spec_init(void);
extern void fa_spec_exit(void);

#endif /* __KERNEL__ */
#endif /* _FMC_ADC_H_ */
