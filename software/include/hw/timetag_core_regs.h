// SPDX-FileCopyrightText: 2022 CERN (home.cern)
// SPDX-License-Identifier: GPL-2.0-or-later

#ifndef __CHEBY__TIMETAG_CORE_REGS__H__
#define __CHEBY__TIMETAG_CORE_REGS__H__
#define TIMETAG_CORE_REGS_SIZE 128 /* 0x80 */

/* Timetag seconds register (upper) */
#define TIMETAG_CORE_REGS_SECONDS_UPPER 0x0UL
#define TIMETAG_CORE_REGS_SECONDS_UPPER_MASK 0xffUL
#define TIMETAG_CORE_REGS_SECONDS_UPPER_SHIFT 0

/* Timetag seconds register (lower) */
#define TIMETAG_CORE_REGS_SECONDS_LOWER 0x4UL

/* Timetag coarse time register, system clock ticks (125MHz) */
#define TIMETAG_CORE_REGS_COARSE 0x8UL
#define TIMETAG_CORE_REGS_COARSE_MASK 0xfffffffUL
#define TIMETAG_CORE_REGS_COARSE_SHIFT 0

/* Time trigger seconds register (upper) */
#define TIMETAG_CORE_REGS_TIME_TRIG_SECONDS_UPPER 0xcUL
#define TIMETAG_CORE_REGS_TIME_TRIG_SECONDS_UPPER_MASK 0xffUL
#define TIMETAG_CORE_REGS_TIME_TRIG_SECONDS_UPPER_SHIFT 0

/* Time trigger seconds register (lower) */
#define TIMETAG_CORE_REGS_TIME_TRIG_SECONDS_LOWER 0x10UL

/* Time trigger coarse time register, system clock ticks (125MHz) */
#define TIMETAG_CORE_REGS_TIME_TRIG_COARSE 0x14UL
#define TIMETAG_CORE_REGS_TIME_TRIG_COARSE_MASK 0xfffffffUL
#define TIMETAG_CORE_REGS_TIME_TRIG_COARSE_SHIFT 0

/* Trigger time-tag seconds register (upper) */
#define TIMETAG_CORE_REGS_TRIG_TAG_SECONDS_UPPER 0x18UL
#define TIMETAG_CORE_REGS_TRIG_TAG_SECONDS_UPPER_MASK 0xffUL
#define TIMETAG_CORE_REGS_TRIG_TAG_SECONDS_UPPER_SHIFT 0

/* Trigger time-tag seconds register (lower) */
#define TIMETAG_CORE_REGS_TRIG_TAG_SECONDS_LOWER 0x1cUL

/* Trigger time-tag coarse time (system clock ticks 125MHz) register */
#define TIMETAG_CORE_REGS_TRIG_TAG_COARSE 0x20UL
#define TIMETAG_CORE_REGS_TRIG_TAG_COARSE_MASK 0xfffffffUL
#define TIMETAG_CORE_REGS_TRIG_TAG_COARSE_SHIFT 0

/* Acquisition start time-tag seconds register (upper) */
#define TIMETAG_CORE_REGS_ACQ_START_TAG_SECONDS_UPPER 0x24UL
#define TIMETAG_CORE_REGS_ACQ_START_TAG_SECONDS_UPPER_MASK 0xffUL
#define TIMETAG_CORE_REGS_ACQ_START_TAG_SECONDS_UPPER_SHIFT 0

/* Acquisition start time-tag seconds register (lower) */
#define TIMETAG_CORE_REGS_ACQ_START_TAG_SECONDS_LOWER 0x28UL

/* Acquisition start time-tag coarse time (system clock ticks 125MHz) register */
#define TIMETAG_CORE_REGS_ACQ_START_TAG_COARSE 0x2cUL
#define TIMETAG_CORE_REGS_ACQ_START_TAG_COARSE_MASK 0xfffffffUL
#define TIMETAG_CORE_REGS_ACQ_START_TAG_COARSE_SHIFT 0

/* Acquisition stop time-tag seconds register (upper) */
#define TIMETAG_CORE_REGS_ACQ_STOP_TAG_SECONDS_UPPER 0x30UL
#define TIMETAG_CORE_REGS_ACQ_STOP_TAG_SECONDS_UPPER_MASK 0xffUL
#define TIMETAG_CORE_REGS_ACQ_STOP_TAG_SECONDS_UPPER_SHIFT 0

/* Acquisition stop time-tag seconds register (lower) */
#define TIMETAG_CORE_REGS_ACQ_STOP_TAG_SECONDS_LOWER 0x34UL

/* Acquisition stop time-tag coarse time (system clock ticks 125MHz) register */
#define TIMETAG_CORE_REGS_ACQ_STOP_TAG_COARSE 0x38UL
#define TIMETAG_CORE_REGS_ACQ_STOP_TAG_COARSE_MASK 0xfffffffUL
#define TIMETAG_CORE_REGS_ACQ_STOP_TAG_COARSE_SHIFT 0

/* Acquisition end time-tag seconds register (upper) */
#define TIMETAG_CORE_REGS_ACQ_END_TAG_SECONDS_UPPER 0x3cUL
#define TIMETAG_CORE_REGS_ACQ_END_TAG_SECONDS_UPPER_MASK 0xffUL
#define TIMETAG_CORE_REGS_ACQ_END_TAG_SECONDS_UPPER_SHIFT 0

/* Acquisition end time-tag seconds register (lower) */
#define TIMETAG_CORE_REGS_ACQ_END_TAG_SECONDS_LOWER 0x40UL

/* Acquisition end time-tag coarse time (system clock ticks 125MHz) register */
#define TIMETAG_CORE_REGS_ACQ_END_TAG_COARSE 0x44UL
#define TIMETAG_CORE_REGS_ACQ_END_TAG_COARSE_MASK 0xfffffffUL
#define TIMETAG_CORE_REGS_ACQ_END_TAG_COARSE_SHIFT 0

struct timetag_core_regs {
  /* [0x0]: REG (rw) Timetag seconds register (upper) */
  uint32_t seconds_upper;

  /* [0x4]: REG (rw) Timetag seconds register (lower) */
  uint32_t seconds_lower;

  /* [0x8]: REG (rw) Timetag coarse time register, system clock ticks (125MHz) */
  uint32_t coarse;

  /* [0xc]: REG (rw) Time trigger seconds register (upper) */
  uint32_t time_trig_seconds_upper;

  /* [0x10]: REG (rw) Time trigger seconds register (lower) */
  uint32_t time_trig_seconds_lower;

  /* [0x14]: REG (rw) Time trigger coarse time register, system clock ticks (125MHz) */
  uint32_t time_trig_coarse;

  /* [0x18]: REG (ro) Trigger time-tag seconds register (upper) */
  uint32_t trig_tag_seconds_upper;

  /* [0x1c]: REG (ro) Trigger time-tag seconds register (lower) */
  uint32_t trig_tag_seconds_lower;

  /* [0x20]: REG (ro) Trigger time-tag coarse time (system clock ticks 125MHz) register */
  uint32_t trig_tag_coarse;

  /* [0x24]: REG (ro) Acquisition start time-tag seconds register (upper) */
  uint32_t acq_start_tag_seconds_upper;

  /* [0x28]: REG (ro) Acquisition start time-tag seconds register (lower) */
  uint32_t acq_start_tag_seconds_lower;

  /* [0x2c]: REG (ro) Acquisition start time-tag coarse time (system clock ticks 125MHz) register */
  uint32_t acq_start_tag_coarse;

  /* [0x30]: REG (ro) Acquisition stop time-tag seconds register (upper) */
  uint32_t acq_stop_tag_seconds_upper;

  /* [0x34]: REG (ro) Acquisition stop time-tag seconds register (lower) */
  uint32_t acq_stop_tag_seconds_lower;

  /* [0x38]: REG (ro) Acquisition stop time-tag coarse time (system clock ticks 125MHz) register */
  uint32_t acq_stop_tag_coarse;

  /* [0x3c]: REG (ro) Acquisition end time-tag seconds register (upper) */
  uint32_t acq_end_tag_seconds_upper;

  /* [0x40]: REG (ro) Acquisition end time-tag seconds register (lower) */
  uint32_t acq_end_tag_seconds_lower;

  /* [0x44]: REG (ro) Acquisition end time-tag coarse time (system clock ticks 125MHz) register */
  uint32_t acq_end_tag_coarse;

  /* padding to: 17 words */
  uint32_t __padding_0[14];
};

#endif /* __CHEBY__TIMETAG_CORE_REGS__H__ */
