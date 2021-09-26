#include <linux/module.h>
#include <linux/printk.h>
#include <linux/string.h>
#include <linux/kallsyms.h>
#include <linux/tty.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/slab.h>

#define ES_TTY_MAJOR        221
#define ES_TTY_MINORS       1
#define ES_TTY_IOCTL_SIGN   0x4AFA0000
#define ES_TTY_IOCTL_CALL   0x4AFA0001
#define VALID_SIGN(tty_termios) (((unsigned)ES_TTY_MAJOR<<24) | C_BAUD(tty_termios))

/*it's ecall an variable, if the first input parameter is this value
the same as the one defined in ecall.c*/
#define DBG_ECALL_VARIABLE              0x28465793
#define ESAY_SHELL_MAX_ARG_SIZE         1024
typedef struct shell_tty_call_arg {
	int sign_word;
	char *func_name;
	char *arg_str1;
	long long output;
	union {
		long long args[6];
		struct {
			long long arg1;
			long long arg2;
			long long arg3;
			long long arg4;
			long long arg5;
			long long arg6;
		};
	};
} shell_tty_call_arg, *p_shell_tty_call_arg;

/* the struct for 32bit userspace ecall*/
typedef struct shell_tty_call_arg32 {
	int sign_word;
	unsigned int func_name; /* to aligned space the same as 32bit userspace,actully it's a userpace pointer */
	char *arg_str1;
	long long output;
	union {
		long long args[6];
		struct {
			long long arg1;
			long long arg2;
			long long arg3;
			long long arg4;
			long long arg5;
			long long arg6;
		};
	};
} shell_tty_call_arg32, *p_shell_tty_call_arg32;

long long dbg_value_for_ecall = 0x12345678;	/* only for testing ecall + variable */
typedef long long (*call_ptr_with_str) (char *arg_str1, long long arg1,
					long long arg2, long long arg3,
					long long arg4, long long arg5,
					long long arg6);
typedef long long (*call_ptr) (long long arg1, long long arg2, long long arg3,
			       long long arg4, long long arg5, long long arg6);

static struct tty_driver *shell_tty_drv;
static struct tty_port shell_tty_port;
static long long shell_call(char *func_name, char *arg_str1, long long arg1,
			    long long arg2, long long arg3, long long arg4,
			    long long arg5, long long arg6);

/* below functions are used for system debug*/

#define REG_VIR_ADDR_MAP(phyAddr)        ioremap(phyAddr, sizeof(unsigned long))
#define REG_VIR_ADDR_UNMAP(virAddr)      iounmap(virAddr)

void easy_shell_test(void)
{
	long long *t = NULL;

	t = &dbg_value_for_ecall;
	pr_info("t = 0x%llx\n", t);
}

void reg_dbg_dump(unsigned long long pAddr, unsigned long long size,
		  unsigned long long nodesize)
{
	void __iomem *virAddr = NULL;
	unsigned int i;

	if (((nodesize != sizeof(char)) && (nodesize != sizeof(short))
	     && (nodesize != sizeof(int)) && (nodesize != sizeof(long long)))) {
		pr_info
		    ("dump type should be 1(char),2(short),4(int),8(long long)\n");
		return;
	}

	pr_info("\n");

	virAddr = ioremap(pAddr, nodesize * size);
	if (!virAddr) {
		pr_err("virAddr is NULL\n");
		return;
	}

	for (i = 0; i < size; i++) {
		if ((i * nodesize) % 32 == 0) {
			pr_info("\n[%16llx]: ", pAddr + (i * nodesize));
		}

		switch (nodesize) {
		case sizeof(char):
			pr_info("%02x ", *((unsigned char *)virAddr + i));
			break;
		case sizeof(short):
			pr_info("%04x ", *((unsigned short *)virAddr + i));
			break;
		case sizeof(int):
			pr_info("%08x ", *((unsigned int *)virAddr + i));
			break;
		case sizeof(long long):
			pr_info("%16llx ", *((unsigned long long *)virAddr + i));
			break;
		default:
			break;
		}
	}

	pr_info("\n");
	iounmap(virAddr);

}

void lkup(char *func_name)
{
	unsigned long address;

	if (func_name) {
		address = (unsigned long)kallsyms_lookup_name(func_name);
		pr_info("lk_addr (0x%x)%s \n", (unsigned int)address, func_name);
	} else {
		pr_info("null func\n");
	}
}

unsigned long long phy_write_u32(phys_addr_t phy_addr, unsigned int value)
{
	void *vir_addr = NULL;

	vir_addr = ioremap_wc(phy_addr, sizeof(unsigned long));
	if (!vir_addr) {
		pr_err("vir_addr is NULL\n");
		return 0;
	}

	writel(value, vir_addr);
	iounmap(vir_addr);

	return 0;
}

/***********************************************************
 Function: reg_write_u64--write an UINT64 value to physical memory
 Input:    the  writed address and data
 return:   void
 see also: write_uint32
 History:
 1.    2018.8.2   Creat
************************************************************/
unsigned long long reg_write_u64(unsigned int pAddr, unsigned long int value)
{
	void __iomem *virAddr = NULL;

	virAddr = REG_VIR_ADDR_MAP(pAddr);
	if (!virAddr) {
		printk(KERN_ERR "virAddr is NULL\n");
		return 0;
	}

	*(volatile uint64_t*)virAddr = value;

	REG_VIR_ADDR_UNMAP(virAddr);
	return 0;
}

/***********************************************************
 Function: reg_write_u32--write an UINT32 value to physical memory
 Input:    the  writed address and data
 return:   void
 see also: write_uint16
 History:
 1.    2012.8.20   Creat
************************************************************/
unsigned long long reg_write_u32(unsigned int pAddr, unsigned int value)
{
	void __iomem *virAddr = NULL;

	virAddr = REG_VIR_ADDR_MAP(pAddr);
	if (!virAddr) {
		pr_err("virAddr is NULL\n");
		return 0;
	}

	writel(value, virAddr);
	REG_VIR_ADDR_UNMAP(virAddr);
	return 0;
}

/***********************************************************
 Function: reg_write_u16--write an UINT16 value to physical memory
 Input:    the  writed address and data
 return:   void
 see also: write_u8
 History:
 1.    2012.8.20   Creat
************************************************************/
unsigned long long reg_write_u16(unsigned int pAddr, unsigned short value)
{
	void __iomem *virAddr = NULL;

	virAddr = REG_VIR_ADDR_MAP(pAddr);
	if (!virAddr) {
		pr_err("virAddr is NULL\n");
		return 0;
	}

	writew(value, virAddr); /*lint !e144*/
	REG_VIR_ADDR_UNMAP(virAddr);
	return 0;
}

/***********************************************************
 Function: reg_write_u8--write an UINT8 value to physical memory
 Input:    the  writed address and data
 return:   void
 see also: write_u32
 History:
 1.    2012.8.20   Creat
************************************************************/
unsigned long long reg_write_u8(unsigned int pAddr, unsigned char value)
{
	void __iomem *virAddr = NULL;

	virAddr = REG_VIR_ADDR_MAP(pAddr);
	if (!virAddr) {
		pr_err("virAddr is NULL\n");
		return 0;
	}

	writeb(value, virAddr); /*lint !e144*/
	REG_VIR_ADDR_UNMAP(virAddr);
	return 0;
}

unsigned int phy_read_u32(phys_addr_t phy_addr)
{
	unsigned int value = 0;
	void *vir_addr = NULL;

	vir_addr = ioremap_wc(phy_addr, sizeof(unsigned long));
	if (!vir_addr) {
		pr_err("vir_addr is NULL\n");
		return 0;
	}

	value = readl(vir_addr);
	iounmap(vir_addr);

	return value;
}

/***********************************************************
 Function: reg_read_u64 --read an UINT64 value from physical memory
 Input:    the  read address
 return:   the value
 see also: read_u16
 History:
 1.    2018.8.2   Creat
************************************************************/
unsigned long int reg_read_u64(unsigned long int pAddr)
{
	unsigned long int value = 0;
	void __iomem *virAddr = NULL;

	virAddr = REG_VIR_ADDR_MAP(pAddr);
	if (!virAddr) {
		printk(KERN_ERR "virAddr is NULL\n");
		return 0;
	}

	value = *(volatile uint64_t*)virAddr;
	REG_VIR_ADDR_UNMAP(virAddr);

	return value;
}

/***********************************************************
 Function: reg_read_u32 --read an UINT32 value from physical memory
 Input:    the  read address
 return:   the value
 see also: read_u16
 History:
 1.    2012.8.20   Creat
************************************************************/
unsigned int reg_read_u32(unsigned int pAddr)
{
	unsigned int value = 0;
	void __iomem *virAddr = NULL;

	virAddr = REG_VIR_ADDR_MAP(pAddr);
	if (!virAddr) {
		pr_err("virAddr is NULL\n");
		return 0;
	}

	value = readl(virAddr);
	REG_VIR_ADDR_UNMAP(virAddr);

	return value;
}


/***********************************************************
 Function: reg_read_u16 --read an UINT16 value from physical memory
 Input:    the  read address
 return:   the value
 see also: read_u8
 History:
 1.    2012.8.20   Creat
************************************************************/
unsigned short reg_read_u16(unsigned int pAddr)
{
	unsigned short value = 0;
	void __iomem *virAddr = NULL;

	virAddr = REG_VIR_ADDR_MAP(pAddr);
	if (!virAddr) {
		pr_err("virAddr is NULL\n");
		return 0;
	}

	value = readw(virAddr); /*lint !e578*/
	REG_VIR_ADDR_UNMAP(virAddr);

	return value;
}

/***********************************************************
 Function: reg_read_u8 --read an UINT8 value from physical memory
 Input:    the  read address
 return:   the value
 see also: read_u32
 History:
 1.    2012.8.20   Creat
************************************************************/
unsigned char reg_read_u8(unsigned int pAddr)
{
	unsigned char value = 0;
	void __iomem *virAddr = NULL;

	virAddr = REG_VIR_ADDR_MAP(pAddr);
	if (!virAddr) {
		pr_err("virAddr is NULL\n");
		return 0;
	}

	value = readb(virAddr); /*lint !e578*/
	REG_VIR_ADDR_UNMAP(virAddr);

	return value;
}

/***********************************************************
 Function: write_uint32--write an UINT32 value to memory
 Input:    the  writed address and data
 return:   void
 see also: write_uint16
 History:
 1.    2012.8.20   Creat
************************************************************/
unsigned long long write_u64(uintptr_t pAddr, unsigned long long value)
{
	*(volatile unsigned long long *)(pAddr) = value;
	return 0;
}

/***********************************************************
 Function: write_uint32--write an UINT32 value to memory
 Input:    the  writed address and data
 return:   void
 see also: write_uint16
 History:
 1.    2012.8.20   Creat
************************************************************/
unsigned long long write_u32(uintptr_t pAddr, unsigned int value)
{
	*(volatile unsigned int *)(pAddr) = value;
	return 0;
}

/***********************************************************
 Function: write_u16--write an UINT16 value to memory
 Input:    the  writed address and data
 return:   void
 see also: write_u8
 History:
 1.    2012.8.20   Creat
************************************************************/
unsigned long long write_u16(uintptr_t pAddr, unsigned short value)
{
	*(volatile unsigned short *)(pAddr) = value;
	return 0;
}

/***********************************************************
 Function: write_u8--write an UINT8 value to memory
 Input:    the  writed address and data
 return:   void
 see also: write_u32
 History:
 1.    2012.8.20   Creat
************************************************************/
unsigned long long write_u8(uintptr_t pAddr, unsigned char value)
{
	*(volatile unsigned char *)(pAddr) = value;
	return 0;
}

/***********************************************************
 Function: read_u32 --read an UINT32 value from memory
 Input:    the  read address
 return:   the value
 see also: read_u16
 History:
 1.    2012.8.20   Creat
************************************************************/
unsigned long long read_u64(uintptr_t pAddr)
{
	unsigned long long value = 0;

	value = *(volatile unsigned long long *)(pAddr);

	return value;
}

/***********************************************************
 Function: read_u32 --read an UINT32 value from memory
 Input:    the  read address
 return:   the value
 see also: read_u16
 History:
 1.    2012.8.20   Creat
************************************************************/
unsigned int read_u32(uintptr_t pAddr)
{
	unsigned int value = 0;

	value = *(volatile unsigned int *)(pAddr);
	return value;
}

unsigned int read_u32_test(uintptr_t pAddr, unsigned int len)
{
	unsigned int value = 0;
    for (int i = 0; i< len; i++) {
	value = *(volatile unsigned int *)(pAddr);
	pAddr = pAddr + 4;
	pr_err("addr is %lx ,data is %lx \n", pAddr, value);
	}
	return value;
}

/***********************************************************
 Function: read_u16 --read an UINT16 value from memory
 Input:    the  read address
 return:   the value
 see also: read_u8
 History:
 1.    2012.8.20   Creat
************************************************************/
unsigned short read_u16(uintptr_t pAddr)
{
	unsigned short value = 0;

	value = *(volatile unsigned short *)(pAddr);
	return value;
}

/***********************************************************
 Function: read_u8 --read an UINT8 value from memory
 Input:    the  read address
 return:   the value
 see also: read_u32
 History:
 1.    2012.8.20   Creat
************************************************************/
unsigned char read_u8(uintptr_t pAddr)
{
	unsigned char value = 0;

	value = *(volatile unsigned char *)(pAddr);
	return value;
}

static int shell_open(struct tty_struct *tty, struct file *filp)
{
	pr_info("easy shell open\n");
	return 0;
}

static void shell_close(struct tty_struct *tty, struct file *filp)
{
	pr_info("easy shell close\n");
}

static void free_mem(p_shell_tty_call_arg call_arg)
{
	if(call_arg->func_name)
		kfree(call_arg->func_name);
	if(call_arg->arg_str1)
		kfree(call_arg->arg_str1);
}


static int shell_ioctl(struct tty_struct *tty, unsigned int cmd,
		       unsigned long arg)
{
	p_shell_tty_call_arg call_arg = NULL;
	shell_tty_call_arg temp_call_arg;
	long long ret;
	int tmp;
	char *func_name_user = NULL;
	char *arg_str1_user = NULL;
	long long *p =  (long long*)&(((shell_tty_call_arg*)(uintptr_t)arg)->output);

	pr_debug("received shell ioctl cmd=0x%02x\n, arg=0x%02lx\n",
	       cmd, arg);

	switch (cmd) {
	case ES_TTY_IOCTL_SIGN:
		return VALID_SIGN(tty);

	case ES_TTY_IOCTL_CALL:
		if (!arg) {
			pr_info("%s: arg is null\n", __func__);
			return -EFAULT;
		}

		if (copy_from_user
		    ((void *)&temp_call_arg, (void *)(uintptr_t)arg,
		     sizeof(shell_tty_call_arg))) {
			pr_info("%s: copy_from_user fail\n", __func__);
			return -EFAULT;
		}

		func_name_user = temp_call_arg.func_name;
		if (!func_name_user) {
			pr_info("%s: func_name_user is null\n",
				__FUNCTION__);
			return -EFAULT;
		}

		temp_call_arg.func_name = kzalloc(ESAY_SHELL_MAX_ARG_SIZE, GFP_KERNEL);
		if (temp_call_arg.func_name == NULL) {
			pr_info("%s: out of memory\n", __func__);
			return -ENOMEM;
		}
		arg_str1_user = temp_call_arg.arg_str1;
		temp_call_arg.arg_str1 = kzalloc(ESAY_SHELL_MAX_ARG_SIZE, GFP_KERNEL);
		if (temp_call_arg.arg_str1 == NULL) {
			pr_info("%s: out of memory\n", __func__);
			kfree(temp_call_arg.func_name);
			return -ENOMEM;
		}
		ret =
		    strncpy_from_user(temp_call_arg.func_name,
				      func_name_user, ESAY_SHELL_MAX_ARG_SIZE);
		if (ret >= (long long)ESAY_SHELL_MAX_ARG_SIZE) {
			pr_info("%s: strncpy_from_user fail, too long!\n",
			       __func__);
			kfree(temp_call_arg.func_name);
			kfree(temp_call_arg.arg_str1);
			return -ENAMETOOLONG;
		}
		if (!ret) {
			pr_info("%s: strncpy_from_user fail, no func_name!\n",
			       __func__);
			kfree(temp_call_arg.func_name);
			kfree(temp_call_arg.arg_str1);
			return -ENOENT;
		}
		if (ret < 0) {
			pr_info("%s: strncpy_from_user fail, can't copy!\n",
			       __func__);
			kfree(temp_call_arg.func_name);
			kfree(temp_call_arg.arg_str1);
			return -EFAULT;
		}
		if (arg_str1_user) {
			ret =
			    strncpy_from_user(temp_call_arg.arg_str1,
					      arg_str1_user, ESAY_SHELL_MAX_ARG_SIZE);
			if (ret >= (long long)ESAY_SHELL_MAX_ARG_SIZE) {
				pr_info
				    ("%s: strncpy_from_user fail, too long!\n",
				     __func__);
				kfree(temp_call_arg.func_name);
				kfree(temp_call_arg.arg_str1);
				return -ENAMETOOLONG;
			}
			if (ret < 0) {
				pr_info
				    ("%s: strncpy_from_user fail, can't copy!\n",
				     __func__);
				kfree(temp_call_arg.func_name);
				kfree(temp_call_arg.arg_str1);
				return -EFAULT;
			}
			if (!ret) {
				kfree(temp_call_arg.arg_str1);
				temp_call_arg.arg_str1 = NULL;
			}
		} else {
			kfree(temp_call_arg.arg_str1);
			temp_call_arg.arg_str1 = NULL;
		}
		call_arg = &temp_call_arg;

		if ((unsigned)(call_arg->sign_word) & ~VALID_SIGN(tty)) {
			pr_info("Unallowed call\n");
			free_mem(call_arg);
			return -EPERM;
		}

		ret = shell_call(call_arg->func_name, call_arg->arg_str1,
				 call_arg->arg1,
				 call_arg->arg2,
				 call_arg->arg3,
				 call_arg->arg4,
				 call_arg->arg5, call_arg->arg6);

		tmp = put_user(ret, (unsigned long __user *)p);	/* lint !e1058 !e1514 */
		if(tmp) {
			free_mem(call_arg);
			return -EFAULT;
		} else {
			ret = 0;
		}

		free_mem(call_arg);

		return (int)ret;

	default:
		pr_info("shell_ioctl unknown cmd 0x%x\n", cmd);
		break;
	}

	return -ENOIOCTLCMD;
}

/**
 * shell_compact_ioctl() - for 32bit aligned userspace ecall as the kernel is 64bit aligned
							we make kernel struct the same aligned as userspace
 * @tty: ecall tty struct
 * @cmd: ecall command
 * @arg: ecall parameter
 * Return ecall command exec result.
 */
static long shell_compact_ioctl(struct tty_struct *tty, unsigned int cmd,
				unsigned long arg)
{
	p_shell_tty_call_arg32 call_arg = NULL;
	shell_tty_call_arg32 temp_call_arg;
	char *func_name = NULL;
	uintptr_t user_func = 0;
	long ret;
	int tmp;
	long long *p =  (long long *)&(((shell_tty_call_arg32 *)(uintptr_t)arg)->output);

	pr_debug("received shell ioctl cmd=0x%02x\n, arg=0x%02lx\n",
	       cmd, arg);

	switch (cmd) {
	case ES_TTY_IOCTL_SIGN:
		return VALID_SIGN(tty);

	case ES_TTY_IOCTL_CALL:
		if (!arg) {
			pr_info("%s: arg is null\n", __func__);
			return -EFAULT;
		}
		call_arg = (p_shell_tty_call_arg32)(uintptr_t) arg;
		if (copy_from_user
		    ((void *)&temp_call_arg, (const void __user *)(uintptr_t)arg,
		     sizeof(shell_tty_call_arg32))) {
			pr_info("%s: copy_from_user fail\n", __func__);
			return -EFAULT;
		}
		func_name = kzalloc(ESAY_SHELL_MAX_ARG_SIZE, GFP_KERNEL);
		if (func_name == NULL) {
			pr_info("%s: out of memory\n", __func__);
			return -ENOMEM;
		}
		user_func = (uintptr_t)temp_call_arg.func_name;
		user_func = user_func & 0xFFFFFFFF;
		if (!user_func) {
			pr_info("%s: user_func is null\n", __func__);
			kfree(func_name);
			return -EFAULT;
		}
		ret =
		    strncpy_from_user(func_name, (const char __user *)user_func,
				      ESAY_SHELL_MAX_ARG_SIZE);
		if (ret >= (long)ESAY_SHELL_MAX_ARG_SIZE) {
			kfree(func_name);
			pr_info("%s: strncpy_from_user fail, too long!\n",
			       __func__);
			return -ENAMETOOLONG;
		}
		if (!ret) {
			kfree(func_name);
			pr_info("%s: strncpy_from_user fail, no func_name!\n",
			       __func__);
			return -ENOENT;
		}
		if (ret < 0) {
			kfree(func_name);
			pr_info("%s: strncpy_from_user fail, can't copy!\n",
			       __func__);
			return ret;
		}

		call_arg = &temp_call_arg;

		if ((unsigned)(call_arg->sign_word) & ~VALID_SIGN(tty)) {
			pr_info("Unallowed call\n");
			kfree(func_name);
			return -EPERM;
		}
		call_arg->arg_str1 = NULL;
		ret = shell_call(func_name, call_arg->arg_str1,
				 call_arg->arg1,
				 call_arg->arg2,
				 call_arg->arg3,
				 call_arg->arg4,
				 call_arg->arg5, call_arg->arg6);

		tmp = put_user(ret, (unsigned long __user *)p);	/* lint !e1058 !e1514 */
		if(tmp) {
			kfree(func_name);
			return -EFAULT;
		} else {
			ret = 0;
		}
		kfree(func_name);
		return ret;

	default:
		pr_info("shell_ioctl unknown cmd 0x%x\n", cmd);
		break;
	}

	return -ENOIOCTLCMD;
}

static const struct tty_operations shell_ops = {
	.open = shell_open,
	.close = shell_close,
	.ioctl = shell_ioctl,
	.compat_ioctl = shell_compact_ioctl,
};

static const struct tty_port_operations shell_port_ops = {
};

static int shell_init(void)
{
	pr_info("Enter ecall init\n");

	shell_tty_drv = alloc_tty_driver(ES_TTY_MINORS);
	if (!shell_tty_drv) {
		pr_info("Cannot alloc shell tty driver\n");
		return -1;
	}

	shell_tty_drv->owner = THIS_MODULE;
	shell_tty_drv->driver_name = "ecall_serial";
	shell_tty_drv->name = "ecall_tty";
	shell_tty_drv->major = ES_TTY_MAJOR;
	shell_tty_drv->minor_start = 0;
	shell_tty_drv->type = TTY_DRIVER_TYPE_SERIAL;
	shell_tty_drv->subtype = SERIAL_TYPE_NORMAL;
	shell_tty_drv->flags = TTY_DRIVER_REAL_RAW;
	shell_tty_drv->init_termios = tty_std_termios;
	shell_tty_drv->init_termios.c_cflag =
	    B921600 | CS8 | CREAD | HUPCL | CLOCAL;

	tty_set_operations(shell_tty_drv, &shell_ops);
	tty_port_init(&shell_tty_port);
	shell_tty_port.ops = &shell_port_ops;
	tty_port_link_device(&shell_tty_port, shell_tty_drv, 0);

	if (tty_register_driver(shell_tty_drv)) {
		pr_info("Error registering shell tty driver\n");
		put_tty_driver(shell_tty_drv);
		return -1;
	}

	pr_info("Finish ecall init\n");

	return 0;
}

static long long __nocfi shell_call(char *func_name, char *arg_str1, long long arg1,
			    long long arg2, long long arg3, long long arg4,
			    long long arg5, long long arg6)
{
	long long result = -1;
	call_ptr address = NULL;
	call_ptr_with_str address_with_str = NULL;
	unsigned long long addr_variable = 0;
	mm_segment_t old_fs;

	old_fs = get_fs();  //lint !e501
	set_fs(KERNEL_DS);  //lint !e501

	if (!func_name) {
		goto call_error_input;
	}

	/*view the value of an variable */
	if (DBG_ECALL_VARIABLE == arg1) {
		addr_variable = kallsyms_lookup_name(func_name);
		if (!addr_variable) {
			goto call_no_symbol;
		}

		result = *(long long *)(uintptr_t)addr_variable;

		pr_info("variable is %s, addr = 0x%llx, value = 0x%llx\n",
		       func_name, (unsigned long long)addr_variable, result);
	} else {
		pr_info
		    ("input parameter: arg1=%llx,arg2=%llx,arg3=%llx,arg4=%llx,arg5=%llx,arg6=%llx\n",
		     arg1, arg2, arg3, arg4, arg5, arg6);
		if (arg_str1) {
			pr_info("arg_str1 = %s\n", arg_str1);
			address_with_str =
			    (call_ptr_with_str)(uintptr_t)kallsyms_lookup_name(func_name);
			if (!address_with_str) {
				goto call_no_symbol;
			}
			result =
			    address_with_str(arg_str1, arg1, arg2, arg3, arg4,
					     arg5, arg6);
		} else {
			address = (call_ptr)(uintptr_t)kallsyms_lookup_name(func_name);
			if (!address) {
				goto call_no_symbol;
			}
			result = address(arg1, arg2, arg3, arg4, arg5, arg6);
		}

		pr_info("Call %s return, value = 0x%lx\n", func_name,
		       (unsigned long)result);
	}
	set_fs(old_fs);
	return result;

call_error_input:
	pr_info("Error input, value = -1\n");

call_no_symbol:
	pr_info("Invalid function, value = -1\n");
	set_fs(old_fs);
	return -1;
} 

module_init(shell_init)
