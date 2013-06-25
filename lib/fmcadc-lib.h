/*
 * Copyright CERN 2013
 * Author: Federico Vaga <federico.vaga@gmail.com>
 */

#ifndef FMCADC_LIB_H_
#define FMCADC_LIB_H_

/* Error codes start from 1024 to void conflicting with libc codes */
#define __FMCADC_ERRNO_START 1024
#define FMCADC_ENOP		1024
#define FMCADC_ENOCAP		1025
#define FMCADC_ENOCFG		1026
#define FMCADC_ENOGET		1027
#define FMCADC_ENOSET		1028
#define FMCADC_ENOCHAN		1029
#define FMCADC_ENOMASK		1030

struct fmcadc_dev;

enum fmcadc_supported_board {
	FMCADC_100MS_4CH_14BIT,
	__FMCADC_SUPPORTED_BOARDS_LAST_INDEX,
};

extern const struct fmcadc_board_type
		*fmcadc_board_types[__FMCADC_SUPPORTED_BOARDS_LAST_INDEX];
/*
 * @data buffer with samples
 * @metadata it describe the buffer; it depends on the board_type in use.
 *           If the device uses a ZIO driver, this fields is a zio_control
 *           structure. Other kind of driver can use this structure to
 *           store their own metadata information.
 */
struct fmcadc_buffer {
	void *data;
	void *metadata;
	int samplesize;
	int nsamples;
	char *drivername;
};

/*
 * It is exactly the zio_timestamp, but this definition avoid a ZIO dependency
 * for eventually non ZIO drivers in the future (or present)
 */
struct fmcadc_timestamp {
	uint64_t secs;
	uint64_t ticks;
	uint64_t bins;
};

/*
 * The following enum can be use to se the mask of valid configurations
 */
enum fmcadc_configuration_trigger {
	FMCADC_CONF_TRG_SOURCE = 0,
	FMCADC_CONF_TRG_SOURCE_CHAN,
	FMCADC_CONF_TRG_THRESHOLD,
	FMCADC_CONF_TRG_POLARITY,
	FMCADC_CONF_TRG_DELAY,
	__FMCADC_CONF_TRG_ATTRIBUTE_LAST_INDEX,
};
enum fmcadc_configuration_acquisition {
	FMCADC_CONF_ACQ_N_SHOTS = 0,
	FMCADC_CONF_ACQ_POST_SAMP,
	FMCADC_CONF_ACQ_PRE_SAMP,
	FMCADC_CONF_ACQ_DECIMATION,
	FMCADC_CONF_ACQ_FREQ_HZ,
	FMCADC_CONF_ACQ_N_BITS,
	__FMCADC_CONF_ACQ_ATTRIBUTE_LAST_INDEX,
};
enum fmcadc_configuration_channel {
	FMCADC_CONF_CHN_RANGE = 0,
	FMCADC_CONF_CHN_TERMINATION,
	FMCADC_CONF_CHN_OFFSET,
	__FMCADC_CONF_CHN_ATTRIBUTE_LAST_INDEX,
};
enum fmcadc_board_status {
	FMCADC_CONF_BRD_STATUS = 0,
	FMCADC_CONF_BRD_MAX_FREQ_HZ,
	FMCADC_CONF_BRD_MIN_FREQ_HZ,
	FMCADC_CONF_BRD_STATE_MACHINE_STATUS,
	FMCADC_CONF_BRD_N_CHAN,
	__FMCADC_CONF_BRD_ATTRIBUTE_LAST_INDEX,
};
enum fmcadc_configuration_type {
	FMCADC_CONF_TYPE_TRG = 0,	/* Trigger */
	FMCADC_CONF_TYPE_ACQ,		/* Acquisition */
	FMCADC_CONF_TYPE_CHN,		/* Channel */
	FMCADC_CONT_TYPE_BRD,		/* Board */
	__FMCADC_CONF_TYPE_LAST_INDEX,
};


/*
 * @type: type of configuration
 * @dev_type: device identificator
 * @route_to: this field help the routing process to send the configuration
 *            to the correct entity. This field is processed by the device
 *            operations, so, its value depend on the library implementation.
 *            For example. In ZIO, this field is evaluated only when you are
 *            configuring a channel; its meaning is the channel index
 * @mask: mask of valid value. The user must set this field to choose which
 *        value configure. On fmcadc_get_config(), the user can set this field
 *        to ask specific values. The library can change this value if the
 *        device does not support the value required by the user
 * @value: list of configuration value. Each type has its own enumeration for
 *         configuration values
 */
#define __FMCADC_CONF_LEN 64 /* number of allocated items in each structure */
struct fmcadc_conf {
	enum fmcadc_configuration_type type;
	uint32_t dev_type;
	uint32_t route_to;
	uint32_t flags; /* how to identify invalid? */
	uint64_t mask;
	uint32_t value[__FMCADC_CONF_LEN];
};


static inline void fmcadc_set_conf_mask(struct fmcadc_conf *conf,
				        unsigned int conf_index)
{
	conf->mask |= (1LL << conf_index);
}
/*
 * fmcadc_set_conf
 * @conf: where set the configuration
 * @conf_index: the configuration to set
 * @val: the value to apply
 *
 * it is a little helper to set correctly a configuration into the
 * configuration structure
 */
static inline void fmcadc_set_conf(struct fmcadc_conf *conf,
				   unsigned int conf_index,
				   uint32_t val)
{
	conf->value[conf_index] = val;
	fmcadc_set_conf_mask(conf, conf_index);
}

/*
 * fmcadc_get_conf
 * @conf: where get the configuration
 * @conf_index: the configuration to get
 * @val: the value of the configuration
 *
 * it is a little helper to get correctly a configuration from the
 * configuration structure
 */
static inline int fmcadc_get_conf(struct fmcadc_conf *conf,
				  unsigned int conf_index,
				  uint32_t *val)
{
	if (conf->mask & (1LL << conf_index)) {
		*val = conf->value[conf_index];
		return 0;
	} else {
		return -1;
	}
}


extern int fmcadc_init(void);
extern void fmcadc_exit(void);

/* fmcadc_open
 * @name: name of the device type to open
 * @dev_id: device identification of a specific card
 * @buffersize: hint about the buffersize used in this device
 * @nbuffer: hint about how many buffers are needed (e.g. multishot)
 * @flags: driver-specific detail information
 */
extern struct fmcadc_dev *fmcadc_open(char *name, unsigned int dev_id,
				      unsigned long buffersize,
				      unsigned int nbuffer,
				      unsigned long flags);

/*
 * fmcadc_open_by_lun
 * @name: name of the device type to open
 * @lun: Logical Unit Number of the device
 * @buffersize: hint about the buffersize used in this device
 * @nbuffer: hint about how many buffers are needed (e.g. multishot)
 * @flags: driver-specific detail information
 *
 * TODO
 */
extern struct fmcadc_dev *fmcadc_open_by_lun(char *name, int lun,
					     unsigned long buffersize,
					     unsigned int nbuffer,
					     unsigned long flags);

/*
 * fmcadc_close
 * @dev: the device to close
 */
extern int fmcadc_close(struct fmcadc_dev *dev);

/*
 * fmcadc_acq_start
 * @dev: device where to start acquiring
 * @flags:
 * @timeout: it can be used to specify how much time wait that acquisition is
 *           over. This value follow the select() policy: NULL to wait until
 *           acquisition is over; {0, 0} to return immediately without wait;
 *           {x, y} to wait acquisition end for a specified time
 */
extern int fmcadc_acq_start(struct fmcadc_dev *dev, unsigned int flags,
			    struct timeval *timeout);

/*
 * fmcadc_acq_stop
 * @dev: device where to stop acquisition
 * @flags:
 */
extern int fmcadc_acq_stop(struct fmcadc_dev *dev, unsigned int flags);

/*
 * fmcadc_apply_config
 * @dev: device to configure
 * @flags:
 * @conf: configuration to apply on device.
 */
extern int fmcadc_apply_config(struct fmcadc_dev *dev, unsigned int flags,
			       struct fmcadc_conf *conf);

/*
 * fmcadc_retrieve_config
 * @dev: device where retireve configuration
 * @flags:
 * @conf: configuration to retrieve. The mask tell which value acquire, then
 *        the library will acquire and set the value in the "value" array
 */
extern int fmcadc_retrieve_config(struct fmcadc_dev *dev,
				 struct fmcadc_conf *conf);

/*
 * fmcadc_request_buffer
 * @dev: device where look for a buffer
 * @buf: where store the buffer. The user must allocate this structure.
 * @flags:
 * @timeout: it can be used to specify how much time wait that a buffer is
 *           ready. This value follow the select() policy: NULL to wait until
 *           acquisition is over; {0, 0} to return immediately without wait;
 *           {x, y} to wait acquisition end for a specified time
 */
extern int fmcadc_request_buffer(struct fmcadc_dev *dev,
				 struct fmcadc_buffer *buf,
				 unsigned int flags,
				 struct timeval *timeout);

/*
 * fmcadc_release_buffer
 * @dev: device that generate the buffer
 * @buf: buffer to release
 */
extern int fmcadc_release_buffer(struct fmcadc_dev *dev,
				 struct fmcadc_buffer *buf);

/*
 * fmcadc_strerror
 * @errnum: error number
 */
extern char *fmcadc_strerror(int errnum);

/*
 * fmcadc_get_driver_type
 * @dev: device which want to know the driver type
 */
extern char *fmcadc_get_driver_type(struct fmcadc_dev *dev);

#ifdef FMCADCLIB_INTERNAL
/*
 * fmcadc_op: it describes the set of operation that a device library should
 * 	      support
 *
 * @start_acquisition start the acquisition
 *	@dev: device where to start acquiring
 *	@flags:
 *	@timeout: it can be used to specify how much time wait that acquisition
 *		  is over. This value follow the select() policy: NULL to wait
 *		  until acquisition is over; {0, 0} to return immediately
 *		  without wait; {x, y} to wait acquisition end for a specified
 *		  time
 *
 * @stop_acquisition stop the acquisition
 *	@dev: device where to stop acquisition
 *	@flags:
 *
 * @apply_config specific operation to apply a configuration to the device
 *	@dev: device to configure
 *	@flags:
 *	@conf: configuration to apply on device.
 *
 * @retrieve_config specific board operation to get the current configuration
 *		    of the device
 *	@dev: device where retireve configuration
 *	@flags:
 *	@conf: configuration to retrieve. The mask tell which value acquire,
 *	       then the library will acquire and set the value in the "value"
 *	       array
 *
 * @request_buffer get from the device a buffer
 * 	@dev: device where look for a buffer
 * 	@buf: where store the buffer. The user must allocate this structure.
 *	@flags:
 *	@timeout: it can be used to specify how much time wait that a buffer is
 *		  ready. This value follow the select() policy: NULL to wait
 *		  until acquisition is over; {0, 0} to return immediately
 *		  without wait; {x, y} to wait acquisition end for a specified
 *		  time
 *
 * @release_buffer release the resources of a given buffer
 *	@dev: device that generate the buffer
 *	@buf: buffer to release
 */
struct fmcadc_op {
	/* Handle board */
	struct fmcadc_dev *(*open)(const struct fmcadc_board_type *dev,
				   unsigned int dev_id,
				   unsigned int details);
	struct fmcadc_dev *(*open_by_lun)(char *devname, int lun);
	int (*close)(struct fmcadc_dev *dev);
	/* Handle acquisition */
	int (*start_acquisition)(struct fmcadc_dev *dev,
				 unsigned int flags,
				 struct timeval *timeout);
	int (*stop_acquisition)(struct fmcadc_dev *dev,
				unsigned int flags);
	/* Handle configuration */
	int (*apply_config)(struct fmcadc_dev *dev,
			    unsigned int flags,
			    struct fmcadc_conf *conf);
	int (*retrieve_config)(struct fmcadc_dev *dev,
			       struct fmcadc_conf *conf);
	/* Handle buffers */
	int (*request_buffer)(struct fmcadc_dev *dev,
			      struct fmcadc_buffer *buf,
			      unsigned int flags,
			      struct timeval *timeout);
	int (*release_buffer)(struct fmcadc_dev *dev,
			      struct fmcadc_buffer *buf);
	char *(*strerror)(int errnum);
};
/*
 * This structure describes the board supported by the library
 * @name name of the board type, for example "fmc-adc-100MS"
 * @devname name of the device in Linux
 * @driver_type: the kind of driver that hanlde this kind of board (e.g. ZIO)
 * @capabilities bitmask of device capabilities for trigger, channel
 *               acquisition
 * @fa_op pointer to a set of operations
 */
struct fmcadc_board_type {
	char *name;
	char *devname;
	char *driver_type;
	uint32_t capabilities[__FMCADC_CONF_TYPE_LAST_INDEX];
	struct fmcadc_op *fa_op;
};

/*
 * Generic Instance Descriptor
 */
struct fmcadc_gid {
	const struct fmcadc_board_type *board;
};

/* Definition of board types */
extern struct fmcadc_board_type fmcadc_100ms_4ch_14bit;

#endif

#endif /* FMCADC_LIB_H_ */
