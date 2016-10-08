/* FPC1020 Touch sensor driver
 *
 * Copyright (c) 2013,2014 Fingerprint Cards AB <tech@fingerprints.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License Version 2
 * as published by the Free Software Foundation.
 */

#define DEBUG

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/spi/spi.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/clk.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/regulator/consumer.h>
#include <linux/wait.h>
#include <linux/kthread.h>
#include <linux/poll.h>
#include <linux/types.h>

#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <linux/wakelock.h>
#include <linux/of.h>

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Fingerprint Cards AB <tech@fingerprints.com>");
MODULE_DESCRIPTION("FPC1020 touch sensor driver.");

typedef struct {
	struct spi_device      *spi;
	struct class           *class;
	struct device          *device;
	struct cdev            cdev;
	dev_t                  devno;
	u32                    reset_gpio;
	u32                    irq_gpio;
	int                    irq;
	bool                   interrupt_done;
	struct semaphore       mutex;
	struct regulator       *vdd_io;
	bool                   power_enabled;

	wait_queue_head_t      irq_wq;
	bool                   tee_interrupt_done;
	bool                   wait_abort;
	bool                   wake_irq_state;
	bool                   clk_enable;
	struct clk             *iface_clk;
	struct clk             *core_clk;
	struct wake_lock       irq_wakelock;
} fpc1020_data_t;

typedef enum {
	/* --- Common registers --- */
	FPC102X_REG_FPC_STATUS			= 20,	/* RO, 1 bytes	*/
	FPC102X_REG_READ_INTERRUPT		= 24,	/* RO, 1 byte	*/
	FPC102X_REG_READ_INTERRUPT_WITH_CLEAR	= 28,	/* RO, 1 byte	*/
	FPC102X_REG_READ_ERROR_WITH_CLEAR	= 56,	/* RO, 1 byte	*/
	FPC102X_REG_MISO_EDGE_RIS_EN		= 64,	/* WO, 1 byte	*/
	FPC102X_REG_FPC_CONFIG			= 68,	/* RW, 1 byte	*/
	FPC102X_REG_IMG_SMPL_SETUP		= 76,	/* RW, 3 bytes	*/
	FPC102X_REG_CLOCK_CONFIG		= 80,	/* RW, 1 byte	*/
	FPC102X_REG_IMG_CAPT_SIZE		= 84,	/* RW, 4 bytes	*/
	FPC102X_REG_IMAGE_SETUP			= 92,	/* RW, 1 byte	*/
	FPC102X_REG_ADC_TEST_CTRL		= 96,	/* RW, 1 byte	*/
	FPC102X_REG_IMG_RD			= 100,	/* RW, 1 byte	*/
	FPC102X_REG_SAMPLE_PX_DLY		= 104,	/* RW, 8 bytes	*/
	FPC102X_REG_PXL_RST_DLY			= 108,	/* RW, 1 byte	*/
	FPC102X_REG_TST_COL_PATTERN_EN		= 120,	/* RW, 2 bytes	*/
	FPC102X_REG_CLK_BIST_RESULT		= 124,	/* RW, 4 bytes	*/
	FPC102X_REG_ADC_WEIGHT_SETUP		= 132,	/* RW, 1 byte	*/
	FPC102X_REG_ANA_TEST_MUX		= 136,	/* RW, 4 bytes	*/
	FPC102X_REG_FINGER_DRIVE_CONF		= 140,	/* RW, 1 byte	*/
	FPC102X_REG_FINGER_DRIVE_DLY		= 144,	/* RW, 1 byte	*/
	FPC102X_REG_OSC_TRIM			= 148,	/* RW, 2 bytes	*/
	FPC102X_REG_ADC_WEIGHT_TABLE		= 152,	/* RW, 10 bytes	*/
	FPC102X_REG_ADC_SETUP			= 156,	/* RW, 5 bytes	*/
	FPC102X_REG_ADC_SHIFT_GAIN		= 160,	/* RW, 2 bytes	*/
	FPC102X_REG_BIAS_TRIM			= 164,	/* RW, 1 byte	*/
	FPC102X_REG_PXL_CTRL			= 168,	/* RW, 2 bytes	*/
	FPC102X_REG_FPC_DEBUG			= 208,	/* RO, 1 bytes	*/
	FPC102X_REG_FINGER_PRESENT_STATUS	= 212,	/* RO, 2 bytes	*/
	FPC102X_REG_HWID			= 252,	/* RO, 2 bytes	*/
	/* --- fpc1020/21 specific --- */
	FPC1020_REG_FNGR_DET_THRES		= 216,	/* RW, 1 byte	*/
	FPC1020_REG_FNGR_DET_CNTR		= 220,	/* RW, 2 bytes	*/
	/* --- fpc1150 specific --- */
	FPC1150_REG_OFFSET			= 1000, /* Not a register ! */
	FPC1150_REG_FNGR_DET_THRES		= 1216,	/* RW, 4 byte	*/
	FPC1150_REG_FNGR_DET_CNTR		= 1220,	/* RW, 4 bytes	*/
} fpc1020_reg_t;

typedef struct {
	fpc1020_reg_t reg;
	bool          write;
	u16           reg_size;
	u8            *dataptr;
} fpc1020_reg_access_t;

#define FPC_HW_RESET		_IOW('K', 0, int)
#define FPC_GET_INTERRUPT	_IOR('K', 1, int)
#define FPC_GET_IRQ_GPIO	_IOR('K', 2, int)
#define FPC_ABORT		_IOR('K', 6, int)
#define FPC_ENABLE_SPI_CLK	_IOR('K', 7, int)
#define FPC_DISABLE_SPI_CLK	_IOR('K', 8, int)

static int fpc1020_device_count;

#define FPC_1020_IRQ_REG_BITS_REBOOT 0xff

#define FPC1020_SPI_CLOCK_SPEED	(8 * 1000000U)
#define FPC1020_RESET_RETRIES	2
#define FPC1020_RESET_LOW_US	1000
#define FPC1020_RESET_HIGH1_US	100
#define FPC1020_RESET_HIGH2_US	1250

#define SUPPLY_IO_MIN		1800000UL
#define SUPPLY_IO_MAX		1800000UL
#define SUPPLY_IO_REQ_CURRENT	6000U

#define FPC1020_CLASS_NAME	"fpsensor"
#define FPC1020_DEV_NAME	"fpc1020"
#define FPC1020_MAJOR		230

static int fpc1020_gpio_reset(fpc1020_data_t *fpc1020)
{
	int error = 0;
	int counter = FPC1020_RESET_RETRIES;

	while (counter) {
		counter--;

		gpio_set_value(fpc1020->reset_gpio, 1);
		udelay(FPC1020_RESET_HIGH1_US);

		gpio_set_value(fpc1020->reset_gpio, 0);
		udelay(FPC1020_RESET_LOW_US);

		gpio_set_value(fpc1020->reset_gpio, 1);
		udelay(FPC1020_RESET_HIGH2_US);

		error = gpio_get_value(fpc1020->irq_gpio) ? 0 : -EIO;

		if (!error) {
			printk(KERN_INFO "%s OK !\n", __func__);
			counter = 0;
		} else {
			printk(KERN_INFO "%s timed out,retrying ...\n",
				__func__);

			udelay(1250);
		}
	}
	return error;
}

static void do_copy(u8 *srcptr, u8 *dstptr, size_t len)
{
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
	int src = 0;
	int dst = len - 1;

	while (src < len) {
		dstptr[dst--] = srcptr[src++];
	}
#else
	memcpy(dstptr, srcptr, len);
#endif
}

static int fpc1020_reg_access(fpc1020_data_t *fpc1020,
			      fpc1020_reg_access_t *reg_data)
{
	int error = 0;
	u8 temp_buffer[10];
	struct spi_message msg;

	struct spi_transfer cmd = {
		.cs_change = 0,
		.delay_usecs = 0,
		.speed_hz = FPC1020_SPI_CLOCK_SPEED,
		.tx_buf = &(reg_data->reg),
		.rx_buf = NULL,
		.len    = 1,
		.tx_dma = 0,
		.rx_dma = 0,
		.bits_per_word = 0,
	};

	struct spi_transfer data = {
		.cs_change = 0, //modified by oppo
		.delay_usecs = 0,
		.speed_hz = FPC1020_SPI_CLOCK_SPEED,
		.tx_buf = (reg_data->write)  ? temp_buffer : NULL,
		.rx_buf = (!reg_data->write) ? temp_buffer : NULL,
		.len    = reg_data->reg_size,
		.tx_dma = 0,
		.rx_dma = 0,
		.bits_per_word = 0,
	};

	if (reg_data->reg_size > sizeof(temp_buffer)) {
		dev_err(&fpc1020->spi->dev,
			"%s : illegal register size\n",
			__func__);
		return -ENOMEM;
	}

	if (reg_data->write) {
		do_copy(reg_data->dataptr, temp_buffer, reg_data->reg_size);
	}

	spi_message_init(&msg);
	spi_message_add_tail(&cmd,  &msg);
	spi_message_add_tail(&data, &msg);

	error = spi_sync(fpc1020->spi, &msg);
	if (error) {
		dev_err(&fpc1020->spi->dev, "%s : spi_sync failed.\n", __func__);
		return error;
	}

	if (!reg_data->write) {
		do_copy(temp_buffer, reg_data->dataptr, reg_data->reg_size);
	}
	return 0;
}

static int fpc1020_read_and_clear_irq(fpc1020_data_t *fpc1020)
{
	int error = 0;
	u8 irq_status;
	fpc1020_reg_access_t reg_clear = {
		.reg = FPC102X_REG_READ_INTERRUPT_WITH_CLEAR,
		.write = false,
		.reg_size = 1,
		.dataptr = &irq_status
	};

	error = fpc1020_reg_access(fpc1020, &reg_clear);
	if (error < 0)
		return error;

	if (irq_status == FPC_1020_IRQ_REG_BITS_REBOOT) {
		dev_err(&fpc1020->spi->dev,
			"%s: unexpected irq_status = 0x%x\n"
			, __func__, irq_status);
		error = -EIO;
	}

	return (error < 0) ? error : irq_status;
}

static int fpc1020_check_irq_after_reset(fpc1020_data_t *fpc1020)
{
	int error = 0;
	u8 irq_status;

	fpc1020_reg_access_t reg_clear = {
		.reg = FPC102X_REG_READ_INTERRUPT_WITH_CLEAR,
		.write = false,
		.reg_size = 1,
		.dataptr = &irq_status
	};

	error = fpc1020_reg_access(fpc1020, &reg_clear);
	if (error < 0)
		return error;

	if (irq_status != FPC_1020_IRQ_REG_BITS_REBOOT) {
		dev_err(&fpc1020->spi->dev,
			"IRQ register, expected 0x%x, got 0x%x.\n",
			FPC_1020_IRQ_REG_BITS_REBOOT,
			irq_status);
		error = -EIO;
	}

	return (error < 0) ? error : irq_status;
}

static int fpc1020_reset(fpc1020_data_t *fpc1020)
{
	int error = 0;

	dev_dbg(&fpc1020->spi->dev, "%s\n", __func__);

	fpc1020_gpio_reset(fpc1020);

	disable_irq(fpc1020->irq);
	fpc1020->interrupt_done = false;
	enable_irq(fpc1020->irq);

	error = fpc1020_check_irq_after_reset(fpc1020);
	if (error < 0)
		return error;

	error = (gpio_get_value(fpc1020->irq_gpio) != 0) ? -EIO : 0;

	if (error)
		dev_err(&fpc1020->spi->dev, "IRQ pin, not low after clear.\n");

	error = fpc1020_read_and_clear_irq(fpc1020);

	if (error != 0) {
		dev_err(&fpc1020->spi->dev,
			"IRQ register, expected 0x%x, got 0x%x.\n",
			0,
			(u8)error);

		error = -EIO;
	}

	return error;
}

static int fpc1020_spi_setup(fpc1020_data_t *fpc1020)
{
	int error = 0;

	printk(KERN_INFO "%s\n", __func__);

	fpc1020->spi->mode = SPI_MODE_0;
	fpc1020->spi->bits_per_word = 8;

	error = spi_setup(fpc1020->spi);
	if (error) {
		dev_err(&fpc1020->spi->dev, "spi_setup failed\n");
	}

	return error;
}

static int fpc1020_open(struct inode *inode, struct file *file)
{
	fpc1020_data_t *fpc1020;
	int ret = 0;

	printk(KERN_INFO "%s\n", __func__);

	fpc1020 = container_of(inode->i_cdev, fpc1020_data_t, cdev);

	if (down_interruptible(&fpc1020->mutex))
		return -ERESTARTSYS;

	file->private_data = fpc1020;

	if (!fpc1020->clk_enable) {
		ret = clk_prepare_enable(fpc1020->iface_clk);
		if (ret) {
			dev_err(&fpc1020->spi->dev,
				"error on clk_prepare_enable(iface_clk):%d\n", ret);
		}

		ret = clk_prepare_enable(fpc1020->core_clk);
		if (ret) {
			clk_disable_unprepare(fpc1020->iface_clk);
			dev_err(&fpc1020->spi->dev,
				"error clk_prepare_enable(core_clk):%d\n", ret);
		}

		fpc1020->clk_enable = 1;
	}
	fpc1020->wait_abort = false;

	up(&fpc1020->mutex);

	return 0;
}

static int fpc1020_release(struct inode *inode, struct file *file)
{
	fpc1020_data_t *fpc1020 = file->private_data;
	int status = 0;

	printk(KERN_INFO "%s\n", __func__);

	if (fpc1020->clk_enable) {
		clk_disable_unprepare(fpc1020->core_clk);
		clk_disable_unprepare(fpc1020->iface_clk);
		fpc1020->clk_enable = 0;
	}

	return status;
}

static long fpc1020_ioctl(struct file *file,
			 unsigned int cmd, unsigned long arg)
{
	fpc1020_data_t *fpc1020 = file->private_data;
	void __user *up = (void __user *)arg;
	int value;
	int ret;

	switch (cmd) {
		case FPC_HW_RESET:
			dev_dbg(&fpc1020->spi->dev, "FPC_HW_RESET");
			fpc1020_gpio_reset(fpc1020);
			break;

		case FPC_GET_INTERRUPT:
			value = gpio_get_value(fpc1020->irq_gpio);
			if (value) {
				dev_dbg(&fpc1020->spi->dev, "gpio (%d) no need wait", value);
				if (put_user(value,(int *)up))
					return -EFAULT;

				return 0;
			}

			dev_dbg(&fpc1020->spi->dev, "FPC_GET_INTERRUPT:  waiting irq....\n");
			if (fpc1020->tee_interrupt_done) {
				fpc1020->tee_interrupt_done = false;
				enable_irq(fpc1020->irq);
				dev_dbg(&fpc1020->spi->dev, "FPC_GET_INTERRUPT: enable_irq\n");
			}

			ret = wait_event_interruptible(fpc1020->irq_wq, fpc1020->tee_interrupt_done || fpc1020->wait_abort);
			dev_dbg(&fpc1020->spi->dev, "tee_interrupt_done(%d) wait_abort(%d) ret = %d\n" , fpc1020->tee_interrupt_done, fpc1020->wait_abort, ret);
			if (ret == -ERESTARTSYS)
				return ret;

			if (fpc1020->wait_abort) {
				fpc1020->wait_abort = false;
			} else {
				value = gpio_get_value(fpc1020->irq_gpio);
			}

			dev_dbg(&fpc1020->spi->dev, "FPC_GET_INTERRUPT irq gpio (%d) comming\n" , value);

			if (put_user(value,(int *)up))
				return -EFAULT;
			break;

		case FPC_GET_IRQ_GPIO:
			value = gpio_get_value(fpc1020->irq_gpio);
			dev_dbg(&fpc1020->spi->dev, "FPC_GET_IRQ_GPIO: (%d)", value);

			if (put_user(value,(int *)up))
				return -EFAULT;
			break;

		case FPC_ABORT:
			dev_dbg(&fpc1020->spi->dev, "FPC_ABORT \n");
			fpc1020->tee_interrupt_done = true;
			fpc1020->wait_abort = true;
			wake_up_interruptible(&fpc1020->irq_wq);
			break;

		case FPC_ENABLE_SPI_CLK:
			if (!fpc1020->clk_enable) {
				dev_dbg(&fpc1020->spi->dev, "spi clk enable\n");
				ret = clk_prepare_enable(fpc1020->iface_clk);
				if (ret) {
					dev_err(&fpc1020->spi->dev,
							"error on clk_prepare_enable(iface_clk):%d\n", ret);
				}

				ret = clk_prepare_enable(fpc1020->core_clk);
				if (ret) {
					clk_disable_unprepare(fpc1020->iface_clk);
					dev_err(&fpc1020->spi->dev,
							"error clk_prepare_enable(core_clk):%d\n", ret);
				}

				fpc1020->clk_enable = 1;
			}
			break;

		case FPC_DISABLE_SPI_CLK:
			if (fpc1020->clk_enable) {
				dev_dbg(&fpc1020->spi->dev, "spi clk disable\n");
				clk_disable_unprepare(fpc1020->core_clk);
				clk_disable_unprepare(fpc1020->iface_clk);
				fpc1020->clk_enable = 0;
			}
			break;

		default:
			dev_dbg(&fpc1020->spi->dev, "ENOIOCTLCMD: cmd=%d ", cmd);
			return -ENOIOCTLCMD;
	}

	return 0;
}

static const struct file_operations fpc1020_fops = {
	.owner          = THIS_MODULE,
	.open           = fpc1020_open,
	.release        = fpc1020_release,
	.unlocked_ioctl = fpc1020_ioctl,
	.compat_ioctl   = fpc1020_ioctl,
};

static int fpc1020_regulator_release(fpc1020_data_t *fpc1020)
{
	if (fpc1020->vdd_io != NULL) {
		regulator_put(fpc1020->vdd_io);
		fpc1020->vdd_io = NULL;
	}

	fpc1020->power_enabled = false;

	return 0;
}

static int fpc1020_regulator_configure(fpc1020_data_t *fpc1020)
{
	int error = 0;

	dev_dbg(&fpc1020->spi->dev, "%s\n", __func__);

	fpc1020->vdd_io = regulator_get(&fpc1020->spi->dev, "vdd_io");
	if (IS_ERR(fpc1020->vdd_io)) {
		error = PTR_ERR(fpc1020->vdd_io);
		dev_err(&fpc1020->spi->dev,
			"Regulator get failed, vdd_io, error=%d\n", error);
		goto supply_err;
	}

	if (regulator_count_voltages(fpc1020->vdd_io) > 0) {
		error = regulator_set_voltage(fpc1020->vdd_io,
						SUPPLY_IO_MIN, SUPPLY_IO_MAX);
		if (error) {
			dev_err(&fpc1020->spi->dev,
				"regulator set(io) failed, error=%d\n", error);
			goto supply_err;
		}
	}

	return 0;

supply_err:
	fpc1020_regulator_release(fpc1020);
	return error;
}

static int fpc1020_regulator_set(fpc1020_data_t *fpc1020, bool enable)
{
	int error = 0;

	if (fpc1020->vdd_io == NULL) {
		dev_err(&fpc1020->spi->dev,
			"Regulators not set\n");
			return -EINVAL;
	}

	if (enable) {
		dev_dbg(&fpc1020->spi->dev, "%s on\n", __func__);

		regulator_set_optimum_mode(fpc1020->vdd_io,
					SUPPLY_IO_REQ_CURRENT);

		error = (regulator_is_enabled(fpc1020->vdd_io) == 0) ?
					regulator_enable(fpc1020->vdd_io) : 0;

		if (error) {
			dev_err(&fpc1020->spi->dev,
				"Regulator vdd_io enable failed, error=%d\n",
				error);
			goto out_err;
		}

	} else {
		dev_dbg(&fpc1020->spi->dev, "%s off\n", __func__);

		error = (fpc1020->power_enabled &&
			regulator_is_enabled(fpc1020->vdd_io) > 0) ?
				 regulator_disable(fpc1020->vdd_io) : 0;

		if (error) {
			dev_err(&fpc1020->spi->dev,
				"Regulator vdd_io disable failed, error=%d\n",
				 error);
			goto out_err;
		}
	}

	fpc1020->power_enabled = enable;

	return 0;

out_err:
	fpc1020_regulator_release(fpc1020);
	return error;
}

static irqreturn_t fpc1020_interrupt(int irq, void *_fpc1020)
{
	fpc1020_data_t *fpc1020 = _fpc1020;
	int value = gpio_get_value(fpc1020->irq_gpio);

	dev_dbg(&fpc1020->spi->dev, "%s gpio(%d)\n", __func__, value);

	if (value) {
		fpc1020->interrupt_done = true;
		disable_irq_nosync(irq);

		if (!fpc1020->tee_interrupt_done) {
			fpc1020->tee_interrupt_done = true;
			wake_lock_timeout(&fpc1020->irq_wakelock, HZ/2);
		}
		wake_up_interruptible(&fpc1020->irq_wq);
		return IRQ_HANDLED;
	}
	return IRQ_NONE;
}

static int fpc1020_irq_init(fpc1020_data_t *fpc1020)
{
	int error = 0;

	fpc1020->irq = gpio_to_irq(fpc1020->irq_gpio);
	if (fpc1020->irq < 0) {
		dev_err(&fpc1020->spi->dev, "gpio_to_irq failed.\n");
		error = fpc1020->irq;
		return error;
	}

	error = request_irq(fpc1020->irq, fpc1020_interrupt,
			IRQF_TRIGGER_RISING, "fpc1020", fpc1020);

	if (error) {
		dev_err(&fpc1020->spi->dev,
			"request_irq %i failed.\n",
			fpc1020->irq);

		fpc1020->irq = -EINVAL;

		return error;
	}
	disable_irq(fpc1020->irq);

	return error;
}

static int fpc1020_cleanup(fpc1020_data_t *fpc1020)
{
	dev_dbg(&fpc1020->spi->dev, "%s\n", __func__);

	if (!IS_ERR_OR_NULL(fpc1020->device))
		device_destroy(fpc1020->class, fpc1020->devno);

	class_destroy(fpc1020->class);

	if (fpc1020->irq >= 0)
		free_irq(fpc1020->irq, fpc1020);

	if (gpio_is_valid(fpc1020->irq_gpio))
		gpio_free(fpc1020->irq_gpio);

	if (gpio_is_valid(fpc1020->reset_gpio))
		gpio_free(fpc1020->reset_gpio);

	fpc1020_regulator_release(fpc1020);

	return 0;
}

static int fpc1020_create_class(fpc1020_data_t *fpc1020)
{
	int error = 0;

	dev_dbg(&fpc1020->spi->dev, "%s\n", __func__);

	fpc1020->class = class_create(THIS_MODULE, FPC1020_CLASS_NAME);

	if (IS_ERR(fpc1020->class)) {
		dev_err(&fpc1020->spi->dev, "failed to create class.\n");
		error = PTR_ERR(fpc1020->class);
	}

	return error;
}

static int fpc1020_create_device(fpc1020_data_t *fpc1020)
{
	int error = 0;

	dev_dbg(&fpc1020->spi->dev, "%s\n", __func__);

	if (FPC1020_MAJOR > 0) {
		fpc1020->devno = MKDEV(FPC1020_MAJOR, fpc1020_device_count++);

		error = register_chrdev_region(fpc1020->devno,
						1,
						FPC1020_DEV_NAME);
	} else {
		error = alloc_chrdev_region(&fpc1020->devno,
					fpc1020_device_count++,
					1,
					FPC1020_DEV_NAME);
	}

	if (error < 0) {
		dev_err(&fpc1020->spi->dev,
				"%s: FAILED %d.\n", __func__, error);
		goto out;

	} else {
		dev_info(&fpc1020->spi->dev, "%s: major=%d, minor=%d\n",
						__func__,
						MAJOR(fpc1020->devno),
						MINOR(fpc1020->devno));
	}

	fpc1020->device = device_create(fpc1020->class, NULL, fpc1020->devno,
						NULL, "%s", FPC1020_DEV_NAME);

	if (IS_ERR(fpc1020->device)) {
		dev_err(&fpc1020->spi->dev, "device_create failed.\n");
		error = PTR_ERR(fpc1020->device);
	}

out:
	return error;
}

static int fpc1020_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	int error = 0;
	fpc1020_data_t *fpc1020 = NULL;

	fpc1020 = devm_kzalloc(&spi->dev, sizeof(*fpc1020), GFP_KERNEL);
	if (!fpc1020) {
		dev_err(&spi->dev,
		"failed to allocate memory for struct fpc1020_data\n");

		return -ENOMEM;
	}

	printk(KERN_INFO "%s\n", __func__);

	dev_set_drvdata(&spi->dev, fpc1020);
	fpc1020->spi = spi;

	fpc1020->reset_gpio = -EINVAL;
	fpc1020->irq_gpio   = -EINVAL;

	fpc1020->irq        = -EINVAL;
	fpc1020->clk_enable = 0;

	wake_lock_init(&fpc1020->irq_wakelock, WAKE_LOCK_SUSPEND, "fingerprint_irq");

	init_waitqueue_head(&fpc1020->irq_wq);
	fpc1020->tee_interrupt_done = true;
	fpc1020->wait_abort = false;
	fpc1020->wake_irq_state = false;

	fpc1020->irq_gpio= of_get_named_gpio(dev->of_node, "fpc,gpio_irq", 0);
	if (!gpio_is_valid(fpc1020->irq_gpio)) {
		error = -EINVAL;
		goto err;
	}
	error = gpio_request_one(fpc1020->irq_gpio, GPIOF_DIR_IN, "fpc_irq_gpio");
	if (error)
		goto err;

	fpc1020->reset_gpio = of_get_named_gpio(dev->of_node, "fpc,gpio_reset", 0);
	if (!gpio_is_valid(fpc1020->reset_gpio)) {
		error = -EINVAL;
		goto err;
	}
	error = gpio_request_one(fpc1020->reset_gpio, GPIOF_OUT_INIT_HIGH, "fpc_reset_gpio");
	if (error)
		goto err;

	error = fpc1020_regulator_configure(fpc1020);
	if (error) {
		dev_err(&fpc1020->spi->dev,
			"fpc1020_probe - regulator configuration failed.\n");
		goto err;
	}

	error = fpc1020_regulator_set(fpc1020, true);
	if (error) {
		dev_err(&fpc1020->spi->dev,
			"fpc1020_probe - regulator enable failed.\n");
		goto err;
	}

	error = fpc1020_irq_init(fpc1020);
	if (error)
		goto err;

	error = fpc1020_spi_setup(fpc1020);
	if (error)
		goto err;

	error = fpc1020_reset(fpc1020);
	if (error)
		goto err;

	error = fpc1020_create_class(fpc1020);
	if (error)
		goto err;

	error = fpc1020_create_device(fpc1020);
	if (error)
		goto err;

	sema_init(&fpc1020->mutex, 0);

	cdev_init(&fpc1020->cdev, &fpc1020_fops);
	fpc1020->cdev.owner = THIS_MODULE;

	error = cdev_add(&fpc1020->cdev, fpc1020->devno, 1);
	if (error) {
		dev_err(&fpc1020->spi->dev, "cdev_add failed.\n");
		goto err_chrdev;
	}

	fpc1020->core_clk = clk_get(&spi->dev, "core_clk");
	if (IS_ERR(fpc1020->core_clk)) {
		error = PTR_ERR(fpc1020->core_clk);
		dev_err(&fpc1020->spi->dev, "error on clk_get(core_clk):%d\n", error);
		goto err_cdev;
	}

	fpc1020->iface_clk = clk_get(&spi->dev, "iface_clk");
	if (IS_ERR(fpc1020->iface_clk)) {
		error = PTR_ERR(fpc1020->iface_clk);
		dev_err(&fpc1020->spi->dev, "error on clk_get(iface_clk):%d\n", error);
		goto err_core_clk;
	}

	up(&fpc1020->mutex);

	return 0;

err_core_clk:
	clk_put(fpc1020->core_clk);

err_cdev:
	cdev_del(&fpc1020->cdev);

err_chrdev:
	unregister_chrdev_region(fpc1020->devno, 1);

err:
	wake_lock_destroy(&fpc1020->irq_wakelock);
	fpc1020_cleanup(fpc1020);
	return error;
}

static int __exit fpc1020_remove(struct spi_device *spi)
{
	fpc1020_data_t *fpc1020 = dev_get_drvdata(&spi->dev);

	cdev_del(&fpc1020->cdev);
	unregister_chrdev_region(fpc1020->devno, 1);
	fpc1020_cleanup(fpc1020);

	return 0;
}

static int fpc1020_suspend(struct device *dev)
{
	fpc1020_data_t *fpc1020 = dev_get_drvdata(dev);
	dev_dbg(&fpc1020->spi->dev, "%s\n", __func__);

	if (!fpc1020->tee_interrupt_done && !fpc1020->wake_irq_state) {
		dev_dbg(&fpc1020->spi->dev, "%s enable irq wake\n", __func__);
		enable_irq_wake(fpc1020->irq);
		fpc1020->wake_irq_state = true;
	}
	return 0;
}

static int fpc1020_resume(struct device *dev)
{
	fpc1020_data_t *fpc1020 = dev_get_drvdata(dev);
	dev_dbg(&fpc1020->spi->dev, "%s\n", __func__);

	if (fpc1020->wake_irq_state) {
		dev_dbg(&fpc1020->spi->dev, "%s disable irq wake\n", __func__);
		disable_irq_wake(fpc1020->irq);
		fpc1020->wake_irq_state = false;
	}

	return 0;
}

static const struct dev_pm_ops fpc1020_pm = {
	.suspend = fpc1020_suspend,
	.resume = fpc1020_resume
};

static struct of_device_id fpc1020_of_match[] = {
	{ .compatible = "fpc,fpc1020", },
	{}
};

static struct spi_driver fpc1020_driver = {
	.driver = {
		.name	= FPC1020_DEV_NAME,
		.bus	= &spi_bus_type,
		.owner	= THIS_MODULE,
		.pm	= &fpc1020_pm,
		.of_match_table = fpc1020_of_match,
	},
	.probe	= fpc1020_probe,
	.remove	= fpc1020_remove,
};

static int fpc1020_init(void)
{
	return spi_register_driver(&fpc1020_driver);
}


static void __exit fpc1020_exit(void)
{
	spi_unregister_driver(&fpc1020_driver);
}

module_init(fpc1020_init);
module_exit(fpc1020_exit);
