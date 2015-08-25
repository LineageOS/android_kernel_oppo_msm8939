#include <linux/module.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/spi/spi.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/regulator/consumer.h>
#include <linux/string.h>
#include <linux/of_gpio.h>
#include <linux/sensors_ftm.h>
#ifdef CONFIG_SPI_IR_LEARN
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/vmalloc.h>
#endif /* CONFIG_SPI_IR_LEARN */

#ifdef CONFIG_OF
static struct of_device_id sdc_spi_ir_table[] = {
	{ .compatible = "sdc,spi-ir",},
	{ },
};
#else
#define sdc_spi_ir_table NULL
#endif /* CONFIG_OF */

#define BUFFER_SIZE (64*1024)
#define WORD_SIZE 2

#ifdef CONFIG_SPI_IR_LEARN
#define RX_BUFF_SIZE            250000 //approximately 2.0 sec at spi clock 1Mhz.

#define SPI_IR_IRQ_PORT         (80)
#define SPI_IR_ENABLE           (14)
#define SPI_IR_DEVICE_NAME      "spi_ir_receiver"
#define SPI_IR_DEVICE_ID        "spi_ir_device"
#define SPI_IR_DEVICE_NAME2     "spi_ir_receiver2"
#define SPI_IR_DEVICE_ID2       "spi_ir_device2"
#define SPI_IR_UPLOAD_BUF_SIZE  2048
#define SPI_IR_MSP_SIZE         (SPI_IR_UPLOAD_BUF_SIZE*4)
#define SPI_IR_IRQ_SIZE         1024
#define SPI_IR_IRQ_DELAY        50
#define SPI_IR_RAWDATA_SIZE     50000
#define SPI_IR_DATA_DUMP

static struct proc_dir_entry *spi_ir_dir;

// 0: none, 1: start, 2: start ack, 3: stop, 4: stop ack  5: acquisition indication, 6: conversiion complete
static volatile char current_signals = '0';

static int SPI_IR_IRQ_ID = 0;
static int irDataToUpload[SPI_IR_UPLOAD_BUF_SIZE];
static bool learning_should_stop = false;
static int interruptCount = 0;
static u8 rawBuffer[RX_BUFF_SIZE*2];

struct spi_ir_msp_table {
    int preamble_msp;
    int preamble_mark;
    int carrier;
};

struct candidate_table {
    int preamble_msp;
    int preamble_mark;
};
#endif /* CONFIG_SPI_IR_LEARN */

struct spi_ir {
	struct spi_device *spi;
	struct bin_attribute bin;

    struct spi_message spi_msg;
    struct spi_transfer spi_xfer;

#ifdef CONFIG_SPI_IR_LEARN
    struct work_struct ir_work;
    struct workqueue_struct *ir_wq;

    u8 *rx_buf;
#endif

    u8 *tx_buf;
    u32 hz;
    int duty;;
};

static u16 g_duty[] =
{
	(0x0000) << 15,
	(0x0001) << 14,
	(0x0003) << 13,
	(0x0007) << 12,
	(0x000f) << 11,
	(0x001f) << 10,
	(0x003f) << 9,
	(0x007f) << 8,
	(0x00ff) << 7,
	(0x01ff) << 6,
	(0x03ff) << 5,
	(0x07ff) << 4,
	(0x0fff) << 3,
	(0x1fff) << 2,
	(0x3fff) << 1,
	(0x7fff) << 0,
};

static unsigned int g_support_hz[] = {
#if 0    
    12570, // 12569
    26000, // 25961
    26500, // 26455
    27000, // 26969
    27500, // 27503
    28000, // 28058
    28500, // 28637
    29000, // 28935
    29500, // 29551
    30000, // 29869
#endif

    30300, // 30303
    30500, // 30525
    30750, // 30864
    31000, // 30864
    31250, // 31211
    31500, // 31566
    31750, // 31928
    32000, // 31928
    32250, // 32300
    32500, // 32680
    32750, // 32680
    33000, // 32938
    33250, // 33069
    33500, // 33467
    33750, // 33875
    34000, // 33875
    34250, // 34294
    34500, // 34294
    34750, // 34722
    35000, // 35162
    35250, // 35162
    35500, // 35613
    35750, // 35613
    36000, // 36075
    36250, // 36075
    36500, // 36550
    36750, // 36550
    37000, // 37037
    37250, // 37037
    37500, // 37538
    37750, // 37538
    38000, // 38052
    38250, // 38052
    38500, // 38580
    38750, // 38580
    39000, // 39124
    39250, // 39124
    39500, // 39683
    39750, // 39683
    40000, // 40016
    40500, // 40355
    41000, // 41051
    41500, // 41408
    42000, // 42141
    42500, // 42517
    43000, // 42900
    43500, // 43687
    44000, // 44092
    44500, // 44504
    45000, // 44924
    45500, // 45351
    46000, // 45788
    46500, // 46685
    47000, // 47148
    47500, // 47619
    48000, // 48100
    48500, // 48591
    49000, // 49092
    49500, // 49603
    50000, // 50125
    50500, // 50659
    51000, // 51203
    51500, // 51760
    52000, // 51760
    52500, // 52329
    53000, // 52910
    53500, // 53505
    54000, // 54113
    54500, // 54735
    55000, // 54735
    55500, // 55371
    56000, // 56022
};

#ifdef CONFIG_SPI_IR_LEARN
static struct spi_ir *g_spi_ir = NULL;
static ssize_t spi_ir_bin_read(struct file *filp, struct kobject *kobj, struct bin_attribute *bin_attr, char *buf, loff_t off, size_t count);
#endif


static int spi_ir_write(struct spi_ir *spi_ir, u32 hz, char *buf, size_t count)
{
    int ret = -1;
    struct spi_message *spi_msg = &spi_ir->spi_msg;
    struct spi_device *spi = spi_ir->spi;
    struct spi_transfer *spi_xfer = &spi_ir->spi_xfer;
    u8 *tx_buf = spi_ir->tx_buf;
    u32 size = (count<BUFFER_SIZE*WORD_SIZE)?count:BUFFER_SIZE*WORD_SIZE;

    spi_message_init(spi_msg);
    spi_xfer->tx_buf = tx_buf;
    spi_xfer->len = size;
    spi_xfer->bits_per_word = WORD_SIZE*8;
    spi_xfer->speed_hz = hz;
    spi_message_add_tail(spi_xfer, spi_msg);

    ret = spi_sync(spi, spi_msg);

    return ret;
}

#ifdef CONFIG_SPI_IR_LEARN
static ssize_t
spi_ir_bin_read(struct file *filp, struct kobject *kobj, struct bin_attribute *bin_attr,
    char *buf, loff_t off, size_t count)
{
    if(count > SPI_IR_MSP_SIZE)  count = SPI_IR_MSP_SIZE;
    memcpy(buf, (char*)irDataToUpload, count);
    return count;
}
#endif

static ssize_t
spi_ir_bin_write(struct file *fp, struct kobject *kobj, struct bin_attribute *bin_attr,
    char *buf, loff_t off, size_t count)
{
    struct device   *dev;
    struct spi_ir   *spi_ir;
    int size;
    int i;

    unsigned int min_hz = 0xffffffff, min_val = 0xffffffff;
    unsigned int delta_hz, cur_hz, hz;

    dev = container_of(kobj, struct device, kobj);
    spi_ir = dev_get_drvdata(dev);

    if(off==0 && count==sizeof(int))
    {
        hz = *(u32*)(buf+off);

        for(i=0 ; i<ARRAY_SIZE(g_support_hz) ; i++)
        {
            cur_hz = g_support_hz[i];

            if(hz<cur_hz)
                delta_hz = cur_hz - hz;
            else
                delta_hz = hz - cur_hz;

            if(min_val>delta_hz)
            {
                min_val = delta_hz;
                min_hz = cur_hz;
            }
        }
        spi_ir->hz = min_hz;

        printk(KERN_ERR"Set Hz = %d\n", min_hz);
    }
    else if(count>0)
    {
        memcpy(spi_ir->tx_buf+off, buf, count);
        size = (off+count)/WORD_SIZE;

        if(size>BUFFER_SIZE)
            size=BUFFER_SIZE;

        if(count!=PAGE_SIZE)
        {
            spi_ir_write(spi_ir, spi_ir->hz, (char*)spi_ir->tx_buf, size*WORD_SIZE);
        }
    }

    return count;
}

#ifdef CONFIG_SPI_IR_LEARN
static int spi_ir_read(struct spi_device *spi)
{
    int ret = 0;
    ret = spi_read(spi, &g_spi_ir->rx_buf[0], RX_BUFF_SIZE);
    //printk(KERN_ERR"%s spi_ir_read is %s, ret: %d\n", __FUNCTION__, !ret ? "success" : "failed", ret);
    return ret;
}
#endif

#ifdef CONFIG_SPI_IR_LEARN
static void spi_ir_indication_learning(void)
{
    current_signals = '5';  // ACQUISITION INDICATION
}

static void spi_ir_complete_learning(void)
{
    current_signals = '6';
}

static int spi_ir_modifyIrDataFromSpiRaw(u8* irRaw, u32* rawIrData, int size)
{
    int bit=0, rawPos=0;
    int highPulses=0, lowPulses=0;
    int totalPulses = 0;
    char low=0, high=0;
    unsigned short val = 0;
    int interWordDelay = 0;
    int noiseMarigin = 5;

    if(!irRaw || !rawIrData) {
        printk(KERN_ERR"%s, IR Raw Data or modified IR Data is NULL\n", __func__);
        return 0;
    }

    while(rawPos < size) {
        low = *irRaw++;
        high = *irRaw++;
         val = high << 8 | low;
        for(bit=15; bit>=0; bit--) {
            if((val >> bit) & 0x1) {
                if(lowPulses > noiseMarigin) {
                    *rawIrData++ = lowPulses + interWordDelay;
                    totalPulses++;
                    interWordDelay=0;
                }
                highPulses++;
                lowPulses = 0;
            } else {
                if(highPulses > noiseMarigin) {
                    *rawIrData++ = highPulses + interWordDelay;
                    totalPulses++;
                    interWordDelay=0;
                }
                lowPulses++;
                highPulses = 0;
            }
        }
        rawPos+=2;
        interWordDelay+=2;
    }

    // For the last alternate.
    if(lowPulses > 0) {
        *rawIrData++ = lowPulses + interWordDelay;
        totalPulses++;
    } else if(highPulses > 0) {
        *rawIrData++ = highPulses + interWordDelay;
        totalPulses++;
    }

    //printk(KERN_ERR"%s, size(%d), TotalPulses(%d)\n", __func__, size, totalPulses);
    return (totalPulses);
}

static void spi_ir_setCarrierFrequency(int carrier)
{
    irDataToUpload[0] = 0;   // Data Format
    irDataToUpload[1] = carrier; // Carrier
}

static void spi_ir_makeUploadData(u32* modifiedIrData, int mspSize)
{
    int pos=2, mark=0, irDataPos=1;
    u32 totalTime = 0;

    if(mspSize > SPI_IR_UPLOAD_BUF_SIZE-2) mspSize = SPI_IR_UPLOAD_BUF_SIZE-2;
    while(pos < mspSize-2) {
        irDataToUpload[pos++] = mark = *(modifiedIrData+irDataPos); // Mark
        irDataPos++;

        irDataToUpload[pos++] = mark + *(modifiedIrData+irDataPos); // MSP = Mark + Space
        irDataPos++;

        totalTime += irDataToUpload[pos-1];
        if(totalTime > 1000000) {
            irDataToUpload[pos] = 0;
            break;
        }
    }

    printk(KERN_ERR"%s, total learning count(%d), time(%d)\n", __func__, pos, totalTime);
}

static int spi_ir_makeMarkSpaceData(u32* rawIrData, u32* modifiedIrData, int size)
{
    bool isMark = false;
    int rawPos=0, totalPulses=0;
    int periodSum=0, periodCount=0, carrierFreuency=0;

    if(!rawIrData || !modifiedIrData) {
        printk(KERN_ERR"%s, IR Raw Data or modified IR Data is NULL\n", __func__);
        return 0;
    }

    while(rawPos < size) {
        if(rawIrData[rawPos] > 100)  { // Space
            if(isMark) { // store Mark
                *modifiedIrData += (periodSum/(periodCount*2));
                modifiedIrData++;
                totalPulses++;
            }
            *modifiedIrData++ = rawIrData[rawPos] - periodSum/(periodCount*2);  // store Space
            totalPulses++;
            isMark = false;
        } else {  // Mark
            *modifiedIrData += rawIrData[rawPos];
            if(isMark==false) {
                periodSum += rawIrData[rawPos+2]+rawIrData[rawPos+3];
                periodCount++;
                carrierFreuency = (periodCount*1000*1000)/periodSum;
            }
            isMark = true;
        }
        rawPos++;
    }

    spi_ir_setCarrierFrequency(carrierFreuency);

    return totalPulses;
}

static void spi_ir_uploadSpiIrData(void)
{
    int alternates = 0;
    u32* modifiedIrData = kmalloc(SPI_IR_MSP_SIZE*sizeof(u32), GFP_KERNEL);
    u32* rawIrData = kmalloc(SPI_IR_RAWDATA_SIZE*sizeof(u32), GFP_KERNEL);

    int rawLen=RX_BUFF_SIZE*2;
    u8* raw = &rawBuffer[0];

    if(!modifiedIrData || !raw || !rawIrData) {
        printk(KERN_ERR"%s, Memory allocation failure.\n", __func__);
        return;
    }

    memset(rawIrData, 0x00, SPI_IR_RAWDATA_SIZE*sizeof(u32));
    memset(modifiedIrData, 0x00, SPI_IR_MSP_SIZE*sizeof(u32));

    alternates=spi_ir_modifyIrDataFromSpiRaw(raw, rawIrData, rawLen);
    printk(KERN_ERR"%s, alternates=%d\n", __func__, alternates);

    if(alternates <= 2) return;

    alternates=spi_ir_makeMarkSpaceData(rawIrData, modifiedIrData, alternates);
    printk(KERN_ERR"%s, # of mark and space =%d\n", __func__, alternates);

    if(alternates <= 2) return;

    spi_ir_makeUploadData(modifiedIrData, alternates);

    if(modifiedIrData) {
        kfree(modifiedIrData);
        modifiedIrData=NULL;
    }

    if(rawIrData) {
        kfree(rawIrData);
        rawIrData=NULL;
    }
}

#ifdef SPI_IR_DATA_DUMP
static void spi_ir_dumpIrData(void)
{
    int i =0;

    for(i=0; i<SPI_IR_UPLOAD_BUF_SIZE-4; i+=4) {
        printk(KERN_ERR"IrData: %d, %d, %d, %d\n", irDataToUpload[i],irDataToUpload[i+1],irDataToUpload[i+2],irDataToUpload[i+3]);
        if(irDataToUpload[i+3]==0) break;
    }
}
#endif

static void spi_ir_interrupt_release(void)
{
    if(SPI_IR_IRQ_ID != 0) {
        disable_irq_nosync(SPI_IR_IRQ_ID);
        free_irq(SPI_IR_IRQ_ID, SPI_IR_DEVICE_ID);
        gpio_free(SPI_IR_IRQ_PORT);
        SPI_IR_IRQ_ID=0;
    }

    gpio_set_value(SPI_IR_ENABLE, 0);
    gpio_free(SPI_IR_ENABLE);

    SPI_IR_IRQ_ID=0;
}

static void spi_ir_destroyWQ (void)
{
    if(g_spi_ir->ir_wq) {
        destroy_workqueue(g_spi_ir->ir_wq);
        g_spi_ir->ir_wq = NULL;
    }
}

static void spi_ir_WorkQueueHandler(struct work_struct *work)
{
    int ret;
    int rawBufferPos = 0;
    int rawBufferCycle = 0;

    memset(rawBuffer, 0x00, RX_BUFF_SIZE*2);

    // 1. Wait the stop signal from HAL
        do {
        ret = spi_ir_read(g_spi_ir->spi);
        memcpy(&rawBuffer[rawBufferPos], &g_spi_ir->rx_buf[0], RX_BUFF_SIZE);
        rawBufferPos = (++rawBufferCycle%2)*RX_BUFF_SIZE;
        } while (!learning_should_stop);

    if(rawBufferCycle > 2 && rawBufferCycle%2 == 1) {
        memcpy(&rawBuffer[0], &rawBuffer[RX_BUFF_SIZE], RX_BUFF_SIZE);
        memcpy(&rawBuffer[RX_BUFF_SIZE], &g_spi_ir->rx_buf[0], RX_BUFF_SIZE);
    }

    // 2. Data conversion: SPI frequency based data --> Carrier Frequency based data
    memset(irDataToUpload, 0x00, SPI_IR_UPLOAD_BUF_SIZE);
    spi_ir_uploadSpiIrData();
    spi_ir_complete_learning();

    // for debugging
#ifdef SPI_IR_DATA_DUMP
    spi_ir_dumpIrData();
#endif

    return;
}

static irqreturn_t spi_ir_IrqHandler(int irq, void* dev_id, struct pt_regs* regs)
{
    if(current_signals != '2' && current_signals != '5' ) {
        //printk(KERN_ERR"%s, Interrupt (%d) for device %s was triggered. But it is not handled.\n", __func__, irq, (char*)dev_id);
        return IRQ_HANDLED;
    }

    if(interruptCount++ > 0) {
        disable_irq_nosync(SPI_IR_IRQ_ID);
    }

    spi_ir_indication_learning();

    return IRQ_HANDLED;
}

static void spi_ir_interrupt_register(void)
{
    if(gpio_request(SPI_IR_IRQ_PORT, SPI_IR_DEVICE_NAME)) {
        printk(KERN_ERR"%s, GPIO request failure: %s\n", __func__, SPI_IR_DEVICE_NAME);
        return;
    }

    if((SPI_IR_IRQ_ID=gpio_to_irq(SPI_IR_IRQ_PORT)) < 0) {
        printk(KERN_ERR"%s, GPIO to IRQ mapping failure: %s\n", __func__, SPI_IR_DEVICE_NAME);
        return;
    }

    if(request_irq(SPI_IR_IRQ_ID, (irq_handler_t) spi_ir_IrqHandler, IRQF_TRIGGER_RISING, SPI_IR_DEVICE_NAME, SPI_IR_DEVICE_ID)) {
        printk(KERN_ERR"%s, IRQ request failure\n", __func__);
        return;
    }

    return;

}

static void spi_ir_start_learning(void)
{
    interruptCount = 0;
    learning_should_stop = false;

    if(g_spi_ir == NULL) return;

    if(!g_spi_ir->ir_wq) {
        g_spi_ir->ir_wq = create_singlethread_workqueue("ir_wq");
        if(!g_spi_ir->ir_wq) {
            printk(KERN_ERR"%s: create_singlethread_workqueue failed\n", __func__);
            return;
        }
        INIT_WORK(&g_spi_ir->ir_work, spi_ir_WorkQueueHandler);
    }

    if(gpio_request(SPI_IR_ENABLE, NULL)){
        printk(KERN_ERR"%s, GPIO-%d request failure.\n", __func__, SPI_IR_ENABLE);
        return;
    }
    gpio_set_value(SPI_IR_ENABLE, 1);

    spi_ir_interrupt_register();
    current_signals = '2';  // START ACK

     queue_work(g_spi_ir->ir_wq, &g_spi_ir->ir_work);
}

static void spi_ir_stop_learning(void)
{
    current_signals = '4';  // STOP ACK
    learning_should_stop = true;

    //gpio_set_value(SPI_IR_ENABLE, 0);
    //gpio_free(SPI_IR_ENABLE);

    spi_ir_interrupt_release();

    printk(KERN_ERR"%s, IR Capturing is stopped.\n", __func__);
}

static int spi_ir_read_proc_signals(char *page, char **start, off_t offset,
                    int count, int *eof, void *data)
{
    return sprintf(page, "%c", current_signals);
}

static int spi_ir_write_proc_signals(struct file *file, const char *buffer,
                    unsigned long count, void *data)
{
    char *buf;

    if (count < 1)
        return -EINVAL;

    buf = kmalloc(count, GFP_KERNEL);
    if (!buf)
        return -ENOMEM;

    if (copy_from_user(buf, buffer, count)) {
        kfree(buf);
        return -EFAULT;
    }

    current_signals = buf[0];

    //printk(KERN_ERR"%s, signal: %c\n", __func__, current_signals);

    if (current_signals  == '1') {
        spi_ir_start_learning();
    } else if (current_signals  == '3') {
        spi_ir_stop_learning();
    } else {
        current_signals  = '0';
        kfree(buf);
        return -EINVAL;
    }

    kfree(buf);
    return count;
}
#endif /* CONFIG_SPI_IR_LEARN */
/*---------------------------------------------------------------------------------------*/
static unsigned int soft_ir_version = 0xFFFF;
#define HW_VER_PINS "hw_ver_pins"

static unsigned int ver_h_gpio = 0xFFFF;
static unsigned int ver_m_gpio = 0xFFFF;
static unsigned int ver_l_gpio = 0xFFFF;

static ssize_t soft_ir_version_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	unsigned int tmp[3] = {0};
	

	//if (soft_ir_version == 0xFFFF)
	{
		tmp[0] = __gpio_get_value(ver_h_gpio);
		tmp[1] = __gpio_get_value(ver_m_gpio);
		tmp[3] = __gpio_get_value(ver_l_gpio);

		soft_ir_version = (tmp[0]<<2) | (tmp[1]<<1) | (tmp[3]);
	}
		
	return snprintf(buf, PAGE_SIZE, "0x%x\n", soft_ir_version);
}
static ssize_t soft_ir_version_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	sscanf(buf, "%x", &soft_ir_version);
	return count;
}
static struct kobj_attribute ver_check = {
	.attr = {"version",0664},
	.show = soft_ir_version_show,
	.store = soft_ir_version_store,
};
static const struct attribute *soft_ir_attrs[] = {
	&ver_check.attr,
	NULL
};
static struct dev_ftm ir_ftm;
/*---------------------------------------------------------------------------------------*/
static int spi_ir_probe(struct spi_device *spi)
{
    u32 default_hz = 38000;
    struct spi_ir *spi_ir;
    u8 *tx_buf;
#ifdef CONFIG_SPI_IR_LEARN
    u8 *rx_buf;
#endif
    int rc;
    int duty = 6; // 33%

	dev_err(&spi->dev, "%s\n", __func__);

	spi_ir = kzalloc(sizeof(struct spi_ir),GFP_KERNEL);

	//allocate memory for transfer
	tx_buf = kmalloc(BUFFER_SIZE*WORD_SIZE+128, GFP_ATOMIC);
	if(tx_buf == NULL){
		dev_err(&spi->dev, "%s: mem alloc failed\n", __func__);
		return -ENOMEM;
	}

#ifdef CONFIG_SPI_IR_LEARN
    //allocate memory for receiver
    rx_buf = kmalloc(RX_BUFF_SIZE, GFP_ATOMIC);
    if(rx_buf == NULL){
        dev_err(&spi->dev, "%s: mem alloc failed\n", __func__);
        return -ENOMEM;
    }
#endif

    //Parse data using dt.
    if(spi->dev.of_node){
        int len;
        const __be32 *prop;

		/* Device default speed */
		prop = of_get_property(spi->dev.of_node, "sdc_spi_ir,default_hz", &len);
		if (!prop || len < sizeof(*prop)) {
			dev_err(&spi->dev, "%s has no 'default_hz' property\n",
				spi->dev.of_node->full_name);
		}
		else
		{
			default_hz = be32_to_cpup(prop);
			printk(KERN_ERR"sdc_spi_ir: default Hz = %d\n", default_hz);
		}

		/* Duty */
		prop = of_get_property(spi->dev.of_node, "sdc_spi_ir,high_bit_size", &len);
		if (!prop || len < sizeof(*prop)) {
			dev_err(&spi->dev, "%s has no 'default_hz' property\n",
				spi->dev.of_node->full_name);
		}
		else
		{
			duty = be32_to_cpup(prop);

			if (duty >= ARRAY_SIZE(g_duty))
			{
				duty = ARRAY_SIZE(g_duty) - 1;
			}
			else if (duty < 0)
			{
				duty = 0;
			}

			printk(KERN_ERR"sdc_spi_ir: Duty = %d%%\n", duty*100/18);
		}
	}

	spi->bits_per_word = WORD_SIZE*8;

    rc = spi_setup(spi);
    if (rc < 0) {
        printk(KERN_ERR"SPI Setup error (%d)\n", rc);
        return rc;
    }

    spi_ir->spi = spi;
    spi_ir->tx_buf = tx_buf;
#ifdef CONFIG_SPI_IR_LEARN
    spi_ir->rx_buf = rx_buf;
#endif
    spi_ir->hz = default_hz;
    spi_ir->bin.attr.name = "ir_data";
    spi_ir->bin.attr.mode = S_IRUSR | S_IRGRP | S_IROTH;
#ifdef CONFIG_SPI_IR_LEARN
    spi_ir->bin.read = spi_ir_bin_read;
#else
    spi_ir->bin.read = NULL;
#endif
    spi_ir->bin.size = BUFFER_SIZE*sizeof(unsigned short); // IR pattern size
    spi_ir->bin.write = spi_ir_bin_write;
    spi_ir->bin.attr.mode |= S_IWUSR | S_IWOTH | S_IWGRP;
    spi_ir->duty = duty;
    spi_set_drvdata(spi, spi_ir);

#ifdef CONFIG_SPI_IR_LEARN
    g_spi_ir = spi_ir;
#endif
	rc = sysfs_create_bin_file(&spi->dev.kobj, &spi_ir->bin);

	memset(tx_buf, 0, BUFFER_SIZE*WORD_SIZE);
/*-------------------------------------------------------------------------------------*/
	ver_h_gpio = of_get_named_gpio(spi->dev.of_node, "hw-ver-gpio-h", 0);
	
	ver_m_gpio = of_get_named_gpio(spi->dev.of_node, "hw-ver-gpio-m", 0);
	
	ver_l_gpio = of_get_named_gpio(spi->dev.of_node, "hw-ver-gpio-l", 0);
	printk(KERN_EMERG"%s:hardware version gpio %d %d %d\n",__func__, ver_h_gpio, ver_m_gpio, ver_l_gpio);
	
	if ((ver_h_gpio != 0xFFFF) && (ver_m_gpio != 0xFFFF) && (ver_l_gpio != 0xFFFF))
	{
		gpio_request(ver_h_gpio, HW_VER_PINS);
		gpio_request(ver_m_gpio, HW_VER_PINS);
		gpio_request(ver_l_gpio, HW_VER_PINS);
	}
	
	ir_ftm.attrs = soft_ir_attrs;
	ir_ftm.i2c_client = NULL;
	ir_ftm.priv_data = NULL;
	ir_ftm.name = "ir";

	register_single_dev_ftm(&ir_ftm);
/*-------------------------------------------------------------------------------------*/
	return rc;
}

static int spi_ir_remove(struct spi_device *spi)
{
	struct spi_ir	*spi_ir;
	spi_ir = spi_get_drvdata(spi);

	if(spi_ir->tx_buf) {
		kfree(spi_ir->tx_buf);
		spi_ir->tx_buf = 0;
	}

#ifdef CONFIG_SPI_IR_LEARN
    if(spi_ir->rx_buf) {
        kfree(spi_ir->rx_buf);
        spi_ir->rx_buf = 0;
    }
#endif

    sysfs_remove_bin_file(&spi->dev.kobj, &spi_ir->bin);

    spi_ir->spi = NULL;
    spi_set_drvdata(spi, NULL);

	return 0;
}

static struct spi_driver spi_ir_driver = {
	.driver = {
		.name = "sdc_spi_ir",
		.owner = THIS_MODULE,
		.of_match_table = sdc_spi_ir_table,
	},
	.probe = spi_ir_probe,
	.remove = spi_ir_remove,
};

static int __init spi_ir_init(void)
{
#ifdef CONFIG_SPI_IR_LEARN
    struct proc_dir_entry *ent;

    spi_ir_dir = proc_mkdir("spi_ir", NULL);
    if (spi_ir_dir == NULL) {
        printk(KERN_ERR"Unable to create /proc/spi_ir directory\n");
        return -ENOMEM;
    }

    /* Creating read/write "signals" entry */
    ent = create_proc_entry("signals", S_IRWXU | S_IRWXG | S_IRWXO, spi_ir_dir);
    if (ent == NULL) {
        printk(KERN_ERR"Unable to create /proc/spi_ir/signals entry\n");
        remove_proc_entry("signals", spi_ir_dir);
        remove_proc_entry("spi_ir", 0);
        return -ENOMEM;
    }
    ent->read_proc = spi_ir_read_proc_signals;
    ent->write_proc = spi_ir_write_proc_signals;
#endif

    return spi_register_driver(&spi_ir_driver);
}

static void __exit spi_ir_exit(void)
{
    spi_unregister_driver(&spi_ir_driver);

#ifdef CONFIG_SPI_IR_LEARN
    spi_ir_destroyWQ ();
    remove_proc_entry("signals", spi_ir_dir);
    remove_proc_entry("spi_ir", 0);
#endif
}

module_init(spi_ir_init);
module_exit(spi_ir_exit);
MODULE_DESCRIPTION("SDC SPI IR");
MODULE_LICENSE("GPL v2");

