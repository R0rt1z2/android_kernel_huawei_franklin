#include <linux/bootdevice.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/printk.h>

#define MAX_NAME_LEN 32
#define MAX_REV_LEN 8

struct __bootdevice {
	enum bootdevice_type type;
	const struct device *dev;
	sector_t size;
	u32 cid[4];
	char product_name[MAX_NAME_LEN + 1];
	u8 pre_eol_info;
	u8 life_time_est_typ_a;
	u8 life_time_est_typ_b;
	unsigned int manfid;
	u8 ptn_index;
	int (*get_rev)(const struct device *, char *);
};


static struct semaphore flash_find_index_sem;

static struct __bootdevice bootdevice;

void set_bootdevice_type(enum bootdevice_type type)
{
	bootdevice.type = type;
}

enum bootdevice_type get_bootdevice_type()
{
	return bootdevice.type;
}

void set_bootdevice_name(struct device *dev)
{
	bootdevice.dev = dev;
}

void set_bootdevice_rev_handler(int (*get_rev_func)(const struct device *, char *))
{
	bootdevice.get_rev = get_rev_func;
}

static int rev_proc_show(struct seq_file *m, void *v)
{
	char rev[MAX_REV_LEN + 1] = {0};
	int ret = -EINVAL;

	if (bootdevice.get_rev) {
		ret = bootdevice.get_rev(bootdevice.dev, rev);
		seq_printf(m, "%s\n", rev);
	}
	return ret;
}

static int rev_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, rev_proc_show, NULL);
}

static const struct file_operations rev_proc_fops = {
	.open = rev_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int type_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%d", bootdevice.type);
	return 0;
}

static int type_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, type_proc_show, NULL);
}

static const struct file_operations type_proc_fops = {
	.open = type_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int name_proc_show(struct seq_file *m, void *v)
{
#ifdef CONFIG_MTK_UFS_SUPPORT
	seq_printf(m, "%s\n", "mtk-platform");
#else
	seq_printf(m, "%s", dev_name(bootdevice.dev));
#endif
	return 0;
}

static int name_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, name_proc_show, NULL);
}

static const struct file_operations name_proc_fops = {
	.open = name_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

void set_bootdevice_size(sector_t size)
{
	bootdevice.size = size;
}

static int size_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%llu\n", (unsigned long long)bootdevice.size);
	return 0;
}

static int size_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, size_proc_show, NULL);
}

static const struct file_operations size_proc_fops = {
	.open = size_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

void set_bootdevice_cid(u32 *cid)
{
	memcpy(bootdevice.cid, cid, sizeof(bootdevice.cid));
}

static int cid_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%08x%08x%08x%08x\n", bootdevice.cid[0],
		   bootdevice.cid[1], bootdevice.cid[2], bootdevice.cid[3]);

	return 0;
}

static int cid_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, cid_proc_show, NULL);
}

static const struct file_operations cid_proc_fops = {
	.open = cid_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

void set_bootdevice_product_name(char *product_name)
{
	strlcpy(bootdevice.product_name,
		product_name,
		sizeof(bootdevice.product_name));
}

/*
 * len is expected to be sizeof(product_name),
 * include last byte space for '\0'
 */
void get_bootdevice_product_name(char* product_name, u32 len)
{
	strlcpy(product_name, bootdevice.product_name, len);
}

static int product_name_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%s\n", bootdevice.product_name);

	return 0;
}

static int product_name_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, product_name_proc_show, NULL);
}

static const struct file_operations product_name_proc_fops = {
	.open		= product_name_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

void set_bootdevice_pre_eol_info(u8 pre_eol_info)
{
	bootdevice.pre_eol_info = pre_eol_info;
}

static int pre_eol_info_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "0x%02X\n", bootdevice.pre_eol_info);

	return 0;
}

static int pre_eol_info_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, pre_eol_info_proc_show, NULL);
}

static const struct file_operations pre_eol_info_proc_fops = {
	.open		= pre_eol_info_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

void set_bootdevice_life_time_est_typ_a(u8 life_time_est_typ_a)
{
	bootdevice.life_time_est_typ_a = life_time_est_typ_a;
}

static int life_time_est_typ_a_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "0x%02X\n", bootdevice.life_time_est_typ_a);

	return 0;
}

static int life_time_est_typ_a_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, life_time_est_typ_a_proc_show, NULL);
}

static const struct file_operations life_time_est_typ_a_proc_fops = {
	.open		= life_time_est_typ_a_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

void set_bootdevice_life_time_est_typ_b(u8 life_time_est_typ_b)
{
	bootdevice.life_time_est_typ_b = life_time_est_typ_b;
}

static int life_time_est_typ_b_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "0x%02X\n", bootdevice.life_time_est_typ_b);

	return 0;
}

static int life_time_est_typ_b_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, life_time_est_typ_b_proc_show, NULL);
}

static const struct file_operations life_time_est_typ_b_proc_fops = {
	.open		= life_time_est_typ_b_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

void set_bootdevice_manfid(unsigned int manfid)
{
	bootdevice.manfid = manfid;
}

unsigned int get_bootdevice_manfid(void)
{
	return bootdevice.manfid;
}

static int manfid_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "0x%06x\n", bootdevice.manfid);

	return 0;
}

static int manfid_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, manfid_proc_show, NULL);
}

static const struct file_operations manfid_proc_fops = {
	.open		= manfid_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};
int __init proc_bootdevice_init(void)
{
	if (!proc_mkdir("bootdevice", NULL)) {
		pr_err("make proc dir bootdevice failed\n");
		return -EFAULT;
	}

	proc_create("bootdevice/rev", 0, NULL, &rev_proc_fops);
	proc_create("bootdevice/type", 0, NULL, &type_proc_fops);
	proc_create("bootdevice/name", 0, NULL, &name_proc_fops);
	proc_create("bootdevice/size", 0, NULL, &size_proc_fops);
	proc_create("bootdevice/cid", 0, NULL, &cid_proc_fops);
	proc_create("bootdevice/product_name", 0, NULL, &product_name_proc_fops);
	proc_create("bootdevice/pre_eol_info", 0, NULL, &pre_eol_info_proc_fops);
	proc_create("bootdevice/life_time_est_typ_a", 0, NULL,
		&life_time_est_typ_a_proc_fops);
	proc_create("bootdevice/life_time_est_typ_b", 0, NULL,
		&life_time_est_typ_b_proc_fops);
	proc_create("bootdevice/manfid", 0, NULL, &manfid_proc_fops);

	return 0;
}
//module_init(proc_bootdevice_init);
