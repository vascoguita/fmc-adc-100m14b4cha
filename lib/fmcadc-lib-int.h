/*
 * Copyright CERN 2013
 * Author: Federico Vaga <federico.vaga@gmail.com>
 */
#ifndef FMCADC_LIB_INT_H_
#define FMCADC_LIB_INT_H_

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
	struct fmcadc_buffer *(*request_buffer)(struct fmcadc_dev *dev,
					       int nsamples,
					       void *(*alloc_fn)(size_t),
					       unsigned int flags,
					       struct timeval *timeout);
	int (*release_buffer)(struct fmcadc_dev *dev,
			      struct fmcadc_buffer *buf,
			      void (*free_fn)(void *));
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


/* The following functions are defined in fmc-adc-zio, at the time being */

struct fmcadc_dev *fmcadc_zio_open(const struct fmcadc_board_type *dev,
				   unsigned int dev_id,
				   unsigned int details);
struct fmcadc_dev *fmcadc_zio_open_by_lun(char *name, int lun);
int fmcadc_zio_close(struct fmcadc_dev *dev);
int fmcadc_zio_start_acquisition(struct fmcadc_dev *dev,
				 unsigned int flags, struct timeval *timeout);
int fmcadc_zio_stop_acquisition(struct fmcadc_dev *dev,
				unsigned int flags);
int fmcadc_zio_apply_config(struct fmcadc_dev *dev, unsigned int flags,
			    struct fmcadc_conf *conf);
int fmcadc_zio_retrieve_config(struct fmcadc_dev *dev,
			       struct fmcadc_conf *conf);
struct fmcadc_buffer *fmcadc_zio_request_buffer(struct fmcadc_dev *dev,
						int nsamples,
						void *(*alloc)(size_t),
						unsigned int flags,
						struct timeval *timeout);
int fmcadc_zio_release_buffer(struct fmcadc_dev *dev,
			      struct fmcadc_buffer *buf,
			      void (*free_fn)(void *));



#endif /* FMCADC_LIB_INT_H_ */
