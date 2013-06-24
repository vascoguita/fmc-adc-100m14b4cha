/*
 * Copyright 2012 Federico Vaga
 *
 * GNU GPLv2 or later
 */


#ifndef _FIELD_DESC_H_
#define _FIELD_DESC_H_

/*
 * zfa_field_desc is a field register descriptor. By using address, mask
 * and shift the driver can describe every fields in registers.
 */
struct zfa_field_desc {
	unsigned long addr; /* address of the register, or base offset */
	uint32_t mask; /* bit mask a register field */
	uint32_t shift; /* shift of the mask into the register */
};
/*
 * Get a field from a register value. You read a register from your device,
 * then you use this function to get a filed defined with zfa_field_desc
 * from the read value
 */
static inline uint32_t zfa_get_field(const struct zfa_field_desc *fld,
				     uint32_t fld_val)
{
	return (fld_val & (fld->mask << fld->shift)) >> fld->shift;
}
/*
 * Set a field to a register value. You read a register from your device,
 * then you use this function to set a field defined with zfa_field_desc
 * into the read value. Then you can write the register
 */
static inline uint32_t zfa_set_field(const struct zfa_field_desc *fld,
				     uint32_t fld_val, uint32_t usr_val)
{
	return (fld_val & (~(fld->mask << fld->shift))) |
	       (usr_val << fld->shift);
}

#endif /* _FIELD_DESC_H_ */
