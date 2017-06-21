#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/export.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/miscdevice.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <sound/soc.h>
#include <soc/oppo/oppo_project.h>

#define NAME "ts4621"
#define I2C_RETRY_DELAY		5 /* ms */
#define I2C_RETRIES		5

static struct i2c_client *ts4621_client;
static int DEFAULT_GAIN = 0x38;

#define PROC_DEBUG
#ifdef  PROC_DEBUG
#include <linux/proc_fs.h>
static struct proc_dir_entry* Debug_entry;
struct proc_file{
    struct file_operations opera;
    char name[20];
};
#endif

/* I2C Read/Write Functions */
static int ts4621_i2c_read(u8 reg, u8 *value)
{
	int err;
	int tries = 0;

	struct i2c_msg msgs[] = {
		{
		 .flags = 0,
		 .len = 1,
		 .buf = &reg,
		 },
		{
		 .flags = I2C_M_RD,
		 .len = 1,
		 .buf = value,
		 },
	};

    if(!ts4621_client) {
        pr_err("%s: ts4621 not work!!!!", __func__);
        return -EINVAL;
    }
    msgs[0].addr = ts4621_client->addr;
    msgs[1].addr = ts4621_client->addr;
	do {
		err = i2c_transfer(ts4621_client->adapter, msgs,
					ARRAY_SIZE(msgs));
		if (err != ARRAY_SIZE(msgs))
			msleep_interruptible(I2C_RETRY_DELAY);
	} while ((err != ARRAY_SIZE(msgs)) && (++tries < I2C_RETRIES));

	if (err != ARRAY_SIZE(msgs)) {
		dev_err(&ts4621_client->dev, "read transfer error %d\n", err);
		err = -EIO;
	} else {
		err = 0;
	}

	return err;
}

static int ts4621_i2c_write(u8 reg, u8 value)
{
	int err;
	int tries = 0;
	u8 buf[2] = {0, 0};

	struct i2c_msg msgs[] = {
		{
//		 .addr = ts4621_client->addr,
		 .flags = 0,
		 .len = 2,
		 .buf = buf,
		 },
	};


    if(!ts4621_client) {
        pr_err("%s: ts4621 not work!!!!", __func__);
        return -EINVAL;
    }
    msgs[0].addr = ts4621_client->addr;

	buf[0] = reg;
	buf[1] = value;
	do {
		err = i2c_transfer(ts4621_client->adapter, msgs,
				ARRAY_SIZE(msgs));
		if (err != ARRAY_SIZE(msgs))
			msleep_interruptible(I2C_RETRY_DELAY);
	} while ((err != ARRAY_SIZE(msgs)) && (++tries < I2C_RETRIES));

	if (err != ARRAY_SIZE(msgs)) {
		dev_err(&ts4621_client->dev, "write transfer error\n");
		err = -EIO;
	} else {
		err = 0;
	}

	return err;
}


int ts4621_reg_read(u8 reg, u8 *value)
{
	int ret = -EINVAL;
	ret = ts4621_i2c_read(reg, value);

	return ret;
}

int ts4621_reg_write(u8 reg, u8 value)
{
	int retval;
	u8 old_value;

	retval = ts4621_reg_read(reg, &old_value);
	pr_err("Old value = 0x%08X reg no. = 0x%02X\n", old_value, reg);

	if (retval != 0)
		goto error;

	retval = ts4621_i2c_write(reg, value);
	if (retval != 0)
	    goto error;

    retval = ts4621_reg_read(reg, &old_value);
	pr_err("after write value = 0x%08X  reg no. = 0x%02X\n", old_value, reg);

error:
	return retval;
}

#ifdef PROC_DEBUG
static ssize_t ts4621_proc_read_status(struct file *file, char __user *buf, size_t count, loff_t *pos)
{
    u8 ucAddr = 0;
    u8 val = 0;
    int len = 0;
    char pri_buf[512];
    int err = 0;
    memset(pri_buf,0,sizeof(pri_buf));

    for(ucAddr = 1; ucAddr < 0x4; ucAddr++)
    {
        ts4621_reg_read(ucAddr, &val);
        len += sprintf(pri_buf+len, "R:%02x V:%02x\n", ucAddr, val);
        val = 0;
    }
    err= copy_to_user(buf, pri_buf, len);

    if (len > *pos)
    {
        len -= *pos;
    }
    else
    {
        len = 0;
    }
    *pos =*pos + len;
    count -= len;

    return len;
}
static ssize_t ts4621_proc_set_reg(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
    int reg = 0;
	int val = 0;
	int read = 0;
	char tmp[10] = {0};


	read = copy_from_user(tmp, buf, count);
    read = sscanf(tmp, "0x%x,0x%x", &reg, &val);
	if (read)
	{
	    ts4621_reg_write(reg, val);
		pr_info("%s reg = 0x%x, val = 0x%x, read = %d\n", __func__, reg, val, read);
	}
	else
	{
		pr_info("%s fail!!\n", __func__);
	}

	return count;
}

/*static ssize_t ts4621_proc_get_reg(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
    kal_uint8 reg = 0;
	int val = 0;
	int read = 0;
	char tmp[10] = {0};

	copy_from_user(tmp, buf, count);
	if (read = sscanf(tmp, "%x", &reg))
	{
	    ts4621_ReadReg(reg, &val);
		AUDIO_DBG("%s reg = 0x%x, val = 0x%x, read = %d\n", __func__, reg, val, read);
	}
	else
	{
		AUDIO_DBG("%s fail!!\n", __func__);
	}

	return count;
}*/

static struct proc_file proc_file_st[] = {
    {.name = "status",  .opera = {.read = ts4621_proc_read_status}},
    {.name = "set_reg", .opera = {.write = ts4621_proc_set_reg}},
//    {.name = "get_reg", .opera = {.write = ts4621_proc_get_reg}},
};
static void ts4621_create_proc_files(struct proc_dir_entry *entry)
{
    int count = sizeof(proc_file_st)/sizeof(proc_file_st[0]);
    int i = 0;
    for (i = 0; i < count; i++)
    {
        proc_create(proc_file_st[i].name, 0660, entry, &proc_file_st[i].opera);
    }
}
static void ts4621_create_proc(void)
{
    Debug_entry = proc_mkdir_mode("ts4621", 0660, NULL);
    ts4621_create_proc_files(Debug_entry);
}
#endif
static int ts4621_init_reg(void){
    int iResult = 0;

    iResult = ts4621_reg_write(0x01, 0x01);//path disable and I2C disable
    iResult |= ts4621_reg_write(0x02, 0xC0);//mute and set volume -64dB
    iResult |= ts4621_reg_write(0x03, 0x00);//clear flag

    if(iResult)
    {
        pr_err("[ts4621] %s failed \n", __func__);
        return iResult;
    }

    pr_err("[ts4621] %s Success \n", __func__);
    return iResult;
}
//static int __devinit ts4621_probe(struct i2c_client *client,
//			   const struct i2c_device_id *id)
static int  ts4621_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	int err = 0;

    pr_err("zppp Enter %s", __func__);
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "check_functionality failed\n");
		return -EIO;
	}
	/* set global variables */
	ts4621_client = client;
	err = ts4621_init_reg();
    ts4621_create_proc();// addf for debug
    pr_err("exit %s err = %d", __func__, err);
	return err;
}

void  ts4621_amp_on(int on, u8 gain)
{
	if (on) {
        int iResult = 0;
        char cValue = 0;
        char cTemp = 0;

        iResult = ts4621_reg_write(0x01, 0x00);//path disable and I2C enable
        iResult |= ts4621_reg_read(0x03, &cValue);
        cTemp = cValue & ~0x03;
        iResult |= ts4621_reg_write(0x03, cTemp);//clear flag
        iResult |= ts4621_reg_write(0x02, 0xc0);//mute and set volume
        iResult |= ts4621_reg_read(0x01, &cValue);
        cTemp = cValue | 0xc0;
        iResult |= ts4621_reg_write(0x01, cTemp);//path enable
        iResult |= ts4621_reg_write(0x02, gain);
        if(iResult)
        {
            pr_err("ts4621_amp_on 1 Failed");
            return;
        }
        pr_err("ts4621_amp_on Success 1");

    } else {
        int iResult = 0;
        char cValue = 0;
        char cTemp = 0;

        iResult = ts4621_reg_read(0x02, &cValue);
        cTemp = cValue | 0xc0;//mute
        iResult |= ts4621_reg_write(0x02, cTemp);

        iResult |= ts4621_reg_read(0x01, &cValue);
        cTemp = (cValue & ~0xc0) | 0x1;
        iResult |= ts4621_reg_write(0x01, cTemp);//path disable and I2C disable
        if(iResult)
        {
            pr_err("ts4621_amp_on 2 Failed");
            return;
        }
        pr_err("ts4621_amp_on Success 2");
    }
    return;
}
/*OPPO 2015-06-12 zhangping Add for short sound clear*/
void  ts4621_amp_on_hph_speaker(int on, u8 gain)
{
	if (on) {
        int iResult = 0;
        char cValue = 0;
        char cTemp = 0;

        iResult = ts4621_i2c_write(0x01, 0x00);//path disable and I2C enable
        iResult |= ts4621_reg_read(0x03, &cValue);
        cTemp = cValue & ~0x03;
        iResult |= ts4621_i2c_write(0x03, cTemp);//clear flag
        iResult |= ts4621_reg_read(0x01, &cValue);
        cTemp = cValue | 0xc0;
        iResult |= ts4621_i2c_write(0x01, cTemp);//path enable
        iResult |= ts4621_i2c_write(0x02, gain);
        if(iResult)
        {
            pr_err("ts4621_amp_on 1 Failed");
            return;
        }
        pr_err("ts4621_amp_on Success 1");

    } else {
        int iResult = 0;
        char cValue = 0;
        char cTemp = 0;

        iResult = ts4621_reg_read(0x02, &cValue);
        cTemp = cValue | 0xc0;//mute
        iResult |= ts4621_i2c_write(0x02, cTemp);

        iResult |= ts4621_reg_read(0x01, &cValue);
        cTemp = (cValue & ~0xc0) | 0x1;
        iResult |= ts4621_i2c_write(0x01, cTemp);//path disable and I2C disable
        if(iResult)
        {
            pr_err("ts4621_amp_on 2 Failed");
            return;
        }
        pr_err("ts4621_amp_on Success 2");
    }
    return;
}
/*OPPO 2015-06-12 zhangping Add for short sound clear end*/

//John.Xu@PhoneSw.AudioDriver, 2015/04/21, Add for MMI test
void  ts4621_amp_on_l(int on, u8 gain)
{
	if (on) {
        int iResult = 0;
        char cValue = 0;
        char cTemp = 0;

        iResult = ts4621_reg_write(0x01, 0x00);//path disable and I2C enable
        iResult |= ts4621_reg_read(0x03, &cValue);
        cTemp = cValue & ~0x03;
        iResult |= ts4621_reg_write(0x03, cTemp);//clear flag
        iResult |= ts4621_reg_write(0x02, 0xc0);//mute and set volume
        iResult |= ts4621_reg_read(0x01, &cValue);
        cTemp = cValue | 0xc0;
        iResult |= ts4621_reg_write(0x01, cTemp);//path enable
        iResult |= ts4621_reg_read(0x02, &cValue);
        pr_err("ts4621_amp_on_l cValue: 0x%x", cValue);
        cValue = cValue & 0x40;
        gain = cValue | gain;
        iResult |= ts4621_reg_write(0x02, gain);
        pr_err("ts4621_amp_on_l gain: 0x%x", gain);
        if(iResult)
        {
            pr_err("ts4621_amp_on_l 1 Failed");
            return;
        }
        pr_debug("ts4621_amp_on_l Success 1");

    } else {
        int iResult = 0;
        char cValue = 0;
        char cTemp = 0;

        iResult = ts4621_reg_read(0x02, &cValue);
        cTemp = cValue | 0x80;//mute
        iResult |= ts4621_reg_write(0x02, cTemp);

        iResult |= ts4621_reg_read(0x01, &cValue);
        cTemp = (cValue & ~0x80);
        iResult |= ts4621_reg_write(0x01, cTemp);
        pr_err("ts4621_amp_on_l cTemp: 0x%x", cTemp);
        if(iResult)
        {
            pr_err("ts4621_amp_on_l 2 Failed");
            return;
        }
        pr_debug("ts4621_amp_on_l Success 2");
    }
    return;
}

void  ts4621_amp_on_r(int on, u8 gain)
{
	if (on) {
        int iResult = 0;
        char cValue = 0;
        char cTemp = 0;

        iResult = ts4621_reg_write(0x01, 0x00);//path disable and I2C enable
        iResult |= ts4621_reg_read(0x03, &cValue);
        cTemp = cValue & ~0x03;
        iResult |= ts4621_reg_write(0x03, cTemp);//clear flag
        iResult |= ts4621_reg_write(0x02, 0xc0);//mute and set volume
        iResult |= ts4621_reg_read(0x01, &cValue);
        cTemp = cValue | 0x40;
        iResult |= ts4621_reg_write(0x01, cTemp);//path enable
        iResult |= ts4621_reg_read(0x02, &cValue);
         pr_err("ts4621_amp_on_r cValue: 0x%x", cValue);
        cValue = cValue & 0x80;
        gain = cValue | gain;
        iResult |= ts4621_reg_write(0x02, gain);
         pr_err("ts4621_amp_on_r gain: 0x%x", gain);
        if(iResult)
        {
            pr_err("ts4621_amp_on_r 1 Failed");
            return;
        }
        pr_debug("ts4621_amp_on_r Success 1");

    } else {
        int iResult = 0;
        char cValue = 0;
        char cTemp = 0;

        iResult = ts4621_reg_read(0x02, &cValue);
        cTemp = cValue | 0x40;//mute
        iResult |= ts4621_reg_write(0x02, cTemp);

        iResult |= ts4621_reg_read(0x01, &cValue);
        cTemp = (cValue & ~0x40);
        iResult |= ts4621_reg_write(0x01, cTemp);//hpr path disable
         pr_err("ts4621_amp_on_r cTemp: 0x%x", cTemp);
        if(iResult)
        {
            pr_err("ts4621_amp_on_r 2 Failed");
            return;
        }
        pr_debug("ts4621_amp_on_r Success 2");
    }
    return;
}

void ts4621_get_gain(u8 *value)
{
    int iResult = 0;
    iResult = ts4621_reg_read(0x02, value) | 0x3C;
}

int hp_pa_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	return 0;
}

int hp_pa_put(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	switch (ucontrol->value.integer.value[0]) {
	case 1:
		pr_err("hp_pa_put:enable ts4621\n");
        ts4621_amp_on(1, DEFAULT_GAIN);
		break;
	case 0:
	default:
		pr_err("hp_pa_put:disable ts4621\n");
        ts4621_amp_on(0, DEFAULT_GAIN);
		break;
	}
	return 0;
}

EXPORT_SYMBOL_GPL(ts4621_amp_on);
EXPORT_SYMBOL_GPL(ts4621_amp_on_l);
EXPORT_SYMBOL_GPL(ts4621_amp_on_r);
EXPORT_SYMBOL_GPL(hp_pa_get);
EXPORT_SYMBOL_GPL(hp_pa_put);
EXPORT_SYMBOL_GPL(ts4621_get_gain);
EXPORT_SYMBOL_GPL(ts4621_reg_write);
EXPORT_SYMBOL_GPL(ts4621_reg_read);

static int __exit ts4621_remove(struct i2c_client *client)
{
	return 0;
}

static const struct i2c_device_id ts4621_id[] = {
	{NAME, 0},
	{},
};

static const struct of_device_id ts4621_match[] = {
    { .compatible = "st,ts4621" },
    { },
};

MODULE_DEVICE_TABLE(i2c, ts4621_id);

static struct i2c_driver ts4621_driver = {
	.driver = {
			.name = NAME,
			.owner = THIS_MODULE,
			.of_match_table = of_match_ptr(ts4621_match),
	},
	.probe = ts4621_probe,
	.remove = ts4621_remove,
	.id_table = ts4621_id,
};

static int __init ts4621_init(void)
{
    pr_err("zp Enter %s", __func__);
	if(is_project(OPPO_15109))
	{
		// For 15035 ,2dB
		DEFAULT_GAIN = 0x3A;

		return i2c_add_driver(&ts4621_driver);
	}
	else
	{
		pr_err("%s: This project doesn't have ts4621, won't register it\n", __func__);
		return 0;
	}
}

static void __exit ts4621_exit(void)
{
    pr_err("Enter %s", __func__);
	if(is_project(OPPO_15109))
	{
		i2c_del_driver(&ts4621_driver);
	}
	else
	{
		pr_err("%s: This project doesn't have ts4621, ignore it\n", __func__);
	}

	return;
}


module_init(ts4621_init);
module_exit(ts4621_exit);

MODULE_DESCRIPTION("ts4621 driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("OPPO");
