#include <linux/init.h>
#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/wait.h>
#include <linux/fs.h>
#include <linux/uaccess.h>

wait_queue_head_t mdmrst_wq;
unsigned int mdmrest_flg;

ssize_t mdmrst_read(struct file *file, char __user *buf,
		size_t count, loff_t *ppos)
{
    char *tmp = "1";

	wait_event_interruptible(mdmrst_wq, mdmrest_flg);
    if (copy_to_user(buf, tmp, strlen(tmp)))
        return -EFAULT;
	mdmrest_flg = 0;

	return strlen(tmp);
}

static const struct file_operations mdmrst_device_fops = {
	.owner  = THIS_MODULE,
	.read   = mdmrst_read,
};

static struct miscdevice mdmrst_device = {
	MISC_DYNAMIC_MINOR, "mdmrst", &mdmrst_device_fops
};

static int __init mdmrst_init(void)
{
	init_waitqueue_head(&mdmrst_wq);
	return misc_register(&mdmrst_device);
}

static void __exit mdmrst_exit(void)
{
	misc_deregister(&mdmrst_device);
}

module_init(mdmrst_init);
module_exit(mdmrst_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Geliang Tang <geliang.tang@oppo.com>");
