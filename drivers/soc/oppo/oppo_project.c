#include <asm/uaccess.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include <linux/fs.h>
#include <soc/qcom/smem.h>
#include <soc/oppo/oppo_project.h>
#include <linux/io.h>

static struct proc_dir_entry *oppoVersion = NULL;
static ProjectInfoCDTType *format = NULL;

unsigned int init_project_version(void)
{
	unsigned int len = (sizeof(ProjectInfoCDTType) + 3) & (~0x3);

	format = (ProjectInfoCDTType *)smem_alloc(SMEM_ID_VENDOR1, len, 0, 0);
	if (format)
		return format->nProject;

	return 0;
}

unsigned int get_project(void)
{
	if (format)
		return format->nProject;
	else
		return init_project_version();

	return 0;
}

unsigned int is_project(OPPO_PROJECT project)
{
	return (get_project() == project ? 1 : 0);
}

unsigned char get_PCB_Version(void)
{
	if (format)
		return format->nPCBVersion;

	return 0;
}

unsigned char get_Modem_Version(void)
{
	if (format)
		return format->nModem;

	return 0;
}

unsigned char get_Operator_Version(void)
{
	if (format)
		return format->nOperator;

	return 0;
}

static ssize_t prjVersion_read_proc(struct file *file, char __user *buf,
		size_t count,loff_t *off)
{
	char page[256] = {0};
	int len = 0;

	len = sprintf(page, "%d", get_project());
	if (len > *off)
		len -= *off;
	else
		len = 0;

	if (copy_to_user(buf, page, (len < count ? len : count)))
		return -EFAULT;

	*off += len < count ? len : count;
	return (len < count ? len : count);
}

struct file_operations prjVersion_proc_fops = {
	.read = prjVersion_read_proc,
};

static ssize_t pcbVersion_read_proc(struct file *file, char __user *buf,
		size_t count,loff_t *off)
{
	char page[256] = {0};
	int len = 0;

	len = sprintf(page, "%d", get_PCB_Version());
	if (len > *off)
		len -= *off;
	else
		len = 0;

	if (copy_to_user(buf, page, (len < count ? len : count)))
		return -EFAULT;

	*off += len < count ? len : count;
	return (len < count ? len : count);
}

struct file_operations pcbVersion_proc_fops = {
	.read = pcbVersion_read_proc,
};

static ssize_t operatorName_read_proc(struct file *file, char __user *buf,
		size_t count,loff_t *off)
{
	char page[256] = {0};
	int len = 0;

	len = sprintf(page, "%d", get_Operator_Version());
	if (len > *off)
		len -= *off;
	else
		len = 0;

	if (copy_to_user(buf, page, (len < count ? len : count)))
		return -EFAULT;

	*off += len < count ? len : count;
	return (len < count ? len : count);
}

struct file_operations operatorName_proc_fops = {
	.read = operatorName_read_proc,
};

static ssize_t modemType_read_proc(struct file *file, char __user *buf,
		size_t count,loff_t *off)
{
	char page[256] = {0};
	int len = 0;

	len = sprintf(page, "%d", get_Modem_Version());
	if (len > *off)
		len -= *off;
	else
		len = 0;

	if (copy_to_user(buf, page, (len < count ? len : count)))
		return -EFAULT;

	*off += len < count ? len : count;
	return (len < count ? len : count);
}

struct file_operations modemType_proc_fops = {
	.read = modemType_read_proc,
};

static ssize_t secureType_read_proc(struct file *file, char __user *buf,
		size_t count,loff_t *off)
{
	char page[256] = {0};
	int len = 0;
	void __iomem *oem_config_base;
	uint32_t secure_oem_config = 0;

	oem_config_base = ioremap(0x58034, 10);
	secure_oem_config = __raw_readl(oem_config_base);
	iounmap(oem_config_base);
	pr_info("secure_oem_config 0x%x\n", secure_oem_config);

	len = sprintf(page,"%d", secure_oem_config);
	if (len > *off)
		len -= *off;
	else
		len = 0;

	if (copy_to_user(buf, page, (len < count ? len : count)))
		return -EFAULT;

	*off += len < count ? len : count;
	return (len < count ? len : count);
}

struct file_operations secureType_proc_fops = {
	.read = secureType_read_proc,
};

static int __init oppo_project_init(void)
{
	int ret = 0;
	struct proc_dir_entry *pentry;

	oppoVersion = proc_mkdir("oppoVersion", NULL);
	if (!oppoVersion) {
		pr_err("can't create oppoVersion proc\n");
		goto ERROR_INIT_VERSION;
	}
	pentry = proc_create("prjVersion", S_IRUGO, oppoVersion,
			&prjVersion_proc_fops);
	if (!pentry) {
		pr_err("create prjVersion proc failed.\n");
		goto ERROR_INIT_VERSION;
	}
	pentry = proc_create("pcbVersion", S_IRUGO, oppoVersion,
			&pcbVersion_proc_fops);
	if (!pentry) {
		pr_err("create pcbVersion proc failed.\n");
		goto ERROR_INIT_VERSION;
	}
	pentry = proc_create("operatorName", S_IRUGO, oppoVersion,
			&operatorName_proc_fops);
	if (!pentry) {
		pr_err("create operatorName proc failed.\n");
		goto ERROR_INIT_VERSION;
	}
	pentry = proc_create("modemType", S_IRUGO, oppoVersion,
			&modemType_proc_fops);
	if (!pentry) {
		pr_err("create modemType proc failed.\n");
		goto ERROR_INIT_VERSION;
	}
	pentry = proc_create("secureType", S_IRUGO, oppoVersion,
			&secureType_proc_fops);
	if (!pentry) {
		pr_err("create secureType proc failed.\n");
		goto ERROR_INIT_VERSION;
	}
	return ret;
ERROR_INIT_VERSION:
	remove_proc_entry("oppoVersion", NULL);
	return -ENOENT;
}
arch_initcall(oppo_project_init);

MODULE_DESCRIPTION("OPPO project version");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Joshua <gyx@oppo.com>");
