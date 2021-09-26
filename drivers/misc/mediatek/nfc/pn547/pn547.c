/*
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/jiffies.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/interrupt.h>

#include <linux/gpio.h>
#include <linux/spinlock.h>
#include <linux/of_gpio.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/of_platform.h>
#include <linux/of_device.h>

#include <linux/regulator/consumer.h>
#include <linux/miscdevice.h>

#include <linux/io.h>
#include <linux/err.h>
#include <linux/time.h>
#include <linux/spinlock_types.h>
#include <linux/kthread.h>
#include <linux/reboot.h>
#include <linux/pwm.h>
#include <linux/arm-smccc.h>
#include "securec.h"
#include "pn547.h"
#include "mt6357/mtk_pmic_api.h"

#include <mtk_clkbuf_ctl.h>
#include <mt-plat/upmu_common.h>
#ifdef CONFIG_HUAWEI_HW_DEV_DCT
#include <hwmanufac/dev_detect/dev_detect.h>
#endif
#include <chipset_common/hwpower/hardware_ic/boost_5v.h>

#ifdef CONFIG_HUAWEI_DSM
#include <dsm/dsm_pub.h>
#include <linux/errno.h>
#endif
#define _read_sample_test_size    40
#define NFC_TRY_NUM 3
#define UICC_SUPPORT_CARD_EMULATION (1<<0)
#define eSE_SUPPORT_CARD_EMULATION (1<<1)
#define UICC_2_SUPPORT_CARD_EMULATION (1<<2)
#define CARD_UNKNOWN    0
#define CARD1_SELECT    1
#define CARD2_SELECT    2
#define UICC_CARD2_SELECT    3
#define MAX_ATTRIBUTE_BUFFER_SIZE 128
#define ENABLE_START 0
#define ENABLE_END 1
#define MAX_CONFIG_NAME_SIZE     64
#define MAX_NFC_CHIP_TYPE_SIZE   32
#define MAX_BUFFER_SIZE    512
#define SIGNAL_INTERRUPT_WAIT    (-512)
#define NFC_MAX_IRQ_COUNT    10
#define NFC_PINCTRL_STATE_ACTIVE    "nfc_active"
#define NFC_PINCTRL_STATE_SUSPEND    "nfc_suspend"

static char g_nfc_nxp_config_name[MAX_CONFIG_NAME_SIZE];
static char g_nfc_brcm_config_name[MAX_CONFIG_NAME_SIZE];
static char nfc_chip_type[MAX_NFC_CHIP_TYPE_SIZE];
static bool g_is_sn110 = false;
static bool g_is_uid = false;
static int g_ese_spi_bus = 0;
static int g_ese_svdd_pwr_req = 0;

/* 0 -- cpu(default); 1 -- pmu; 4 -- XTAL */
static int g_nfc_clk_src = NFC_CLK_SRC_CPU;
static int firmware_update = 0;
static int g_nfc_single_channel;
static int wait_event_interruptible_ret = 0;
static int nfc_at_result;
static int nfc_switch_state;
static int g_nfc_activated_se_info; /* record activated se info when nfc enable process */
static int g_nfc_hal_dmd_no = 0;    /* record last hal dmd no */
static int g_nfc_ese_type = NFC_WITHOUT_ESE; /* record ese type wired to nfcc by dts */
static struct dsm_client *nfc_dclient = NULL;
static bool g_is_gtboost = false;

struct pn547_dev {
	wait_queue_head_t read_wq;
	struct mutex read_mutex;
	struct mutex irq_wake_mutex;
	struct device *dev;
	struct i2c_client *client;
	struct miscdevice pn547_device;
	struct clk *nfc_clk;
	struct pinctrl *pn547_pinctrl;
	struct pinctrl_state *pinctrl_state_srclkenai;
	struct pinctrl_state *pinctrl_state_esespics;
	struct pinctrl_state *pinctrl_state_esespiclk;
	struct pinctrl_state *pinctrl_state_esespimo;
	struct pinctrl_state *pinctrl_state_esespimi;
	struct pinctrl_state *pinctrl_state_esepwrreq;
	unsigned int sim_status;
	unsigned int sim_switch;
	unsigned int enable_status;
	unsigned int ven_gpio;
	unsigned int firm_gpio;
	unsigned int irq_gpio;
	unsigned int clk_req_gpio;
	bool irq_enabled;
	bool irq_wake_enabled;
	spinlock_t irq_enabled_lock;
	struct wakeup_source wl;
	bool cancel_read;
};
static struct pn547_dev *nfcdev = NULL;

#ifdef CONFIG_HUAWEI_DSM
static struct dsm_dev dsm_nfc = {
	.name = "dsm_nfc",
	.device_name = NULL,
	.ic_name = NULL,
	.module_name = NULL,
	.fops = NULL,
	.buff_size = 1024, // 1k buffer
};
#endif

/*
 * FUNCTION: pn547_disable_irq_wake
 * DESCRIPTION: disable irq wakeup function
 * Parameters
 * struct  pn547_dev *: device structure
 * RETURN VALUE
 * none
 */
static void pn547_disable_irq_wake(struct pn547_dev *pn547_dev)
{
	int ret;

	mutex_lock(&pn547_dev->irq_wake_mutex);
	if (pn547_dev->irq_wake_enabled) {
		pn547_dev->irq_wake_enabled = false;
		ret = irq_set_irq_wake(pn547_dev->client->irq, 0);
		if (ret) {
			pr_err("%s failed: ret=%d\n", __func__, ret);
		}
	}
	mutex_unlock(&pn547_dev->irq_wake_mutex);
}

static int nfc_get_dts_config_string(const char *node_name,
	const char *prop_name,
	char *out_string,
	int out_string_len)
{
	struct device_node *np = NULL;
	const char *out_value;
	int ret = -1;

	for_each_node_with_property(np, node_name) {
		ret = of_property_read_string(np, prop_name, (const char **)&out_value);
		if (ret != 0) {
			pr_err("%s: can not get prop values with prop_name: %s\n",
							__func__, prop_name);
		} else {
			if (NULL == out_value) {
				pr_info("%s: error out_value = NULL\n", __func__);
				ret = -1;
			} else if (strlen(out_value) >= out_string_len) {
				pr_info("%s: error out_value len :%d >= out_string_len:%d\n",
					__func__, (int)strlen(out_value), (int)out_string_len);
				ret = -1;
			} else {
				strncpy(out_string, out_value, strlen(out_value));
				pr_info("%s: =%s\n", __func__, out_string);
			}
		}
	}
	return ret;
}

static void get_nfc_chip_type(void)
{
	char nfc_on_str[MAX_DETECT_SE_SIZE] = {0};
	int ret;

	memset(nfc_on_str, 0, MAX_DETECT_SE_SIZE);
	ret = nfc_get_dts_config_string("nfc_chip_type", "nfc_chip_type",
		nfc_on_str, sizeof(nfc_on_str));

	if (ret != 0) {
		memset(nfc_on_str, 0, MAX_DETECT_SE_SIZE);
		pr_err("%s: can't find nfc_chip_type node\n", __func__);
		return;
	} else {
	if (!strncasecmp(nfc_on_str, "nfc_sz", strlen("nfc_sz"))) {
			g_is_sn110 = true;
		} else {
			g_is_sn110 = false;
		}
	}
	pr_info("%s: nfc_chip_type:%d\n", __func__, g_is_sn110);
	return;
}

static void get_nfc_wired_ese_type(void)
{
	char nfc_on_str[MAX_DETECT_SE_SIZE] = {0};
	int ret;

	memset(nfc_on_str, 0, MAX_DETECT_SE_SIZE);
	ret = nfc_get_dts_config_string("nfc_ese_type", "nfc_ese_type",
		nfc_on_str, sizeof(nfc_on_str));

	if (ret != 0) {
		memset(nfc_on_str, 0, MAX_DETECT_SE_SIZE);
		g_nfc_ese_type = NFC_WITHOUT_ESE;
		pr_err("%s: can't find nfc_ese_type node\n", __func__);
		return;
	} else {
		if (!strncasecmp(nfc_on_str, "hisee", strlen("hisee"))) {
			g_nfc_ese_type = NFC_ESE_HISEE;
		} else if (!strncasecmp(nfc_on_str, "p61", strlen("p61"))) {
			g_nfc_ese_type = NFC_ESE_P61;
		} else {
			g_nfc_ese_type = NFC_WITHOUT_ESE;
		}
	}
	pr_info("%s: g_nfc_ese_type:%d\n", __func__, g_nfc_ese_type);
	return;
}

static void get_nfc_gt_boost(void)
{
	char nfc_on_str[MAX_DETECT_GT_SIZE] = {0};
	int boost_ret = -1;
	int ret;

	memset(nfc_on_str, 0, MAX_DETECT_GT_SIZE);
	ret = nfc_get_dts_config_string("nfc_gt_boost", "nfc_gt_boost",
		nfc_on_str, sizeof(nfc_on_str));

	if (ret != 0) {
		memset(nfc_on_str, 0, MAX_DETECT_GT_SIZE);
		pr_info("%s: gt boost not supported on this board, ret = %d\n",
			__func__, ret);
		return;
	} else {
		pr_info("%s: enabling 5v, and setting g_is_gtboost = true\n",
			__func__);
		g_is_gtboost = true;
		boost_ret  = boost_5v_enable(BOOST_5V_ENABLE, BOOST_CTRL_NFC);
		if (boost_ret)
			pr_err("%s: boost_5v enable failed\n", __func__);
		else
			pr_info("%s: boost_5v enable succeeded\n", __func__);

		/* delay 10ms for volt stability */
		usleep_range(9500, 10500);
	}
	return;
}

/*
 * FUNCTION: pn547_enable_irq_wake
 * DESCRIPTION: enable irq wakeup function
 * Parameters
 * struct  pn547_dev *: device structure
 * RETURN VALUE
 * none
 */
static void pn547_enable_irq_wake(struct pn547_dev *pn547_dev)
{
	int ret;

	mutex_lock(&pn547_dev->irq_wake_mutex);
	if (!pn547_dev->irq_wake_enabled) {
		pn547_dev->irq_wake_enabled = true;
		ret = irq_set_irq_wake(pn547_dev->client->irq, 1);
		if (ret) {
			pr_err("%s failed: ret=%d\n", __func__, ret);
		}
	}
	mutex_unlock(&pn547_dev->irq_wake_mutex);
}

/* FUNCTION: get_nfc_config_name
 * DESCRIPTION: get nfc configure files' name from device tree system, save result in global variable
 * Parameters
 * none
 * RETURN VALUE
 * none
 */
static void get_nfc_config_name(struct device *dev)
{
	int ret;
	struct device_node *np = dev->of_node;
	const char *out_value = NULL;

	memset(g_nfc_nxp_config_name, 0, sizeof(g_nfc_nxp_config_name));
	memset(g_nfc_brcm_config_name, 0, sizeof(g_nfc_brcm_config_name));

	/* get nfc_nxp_conf_name */
	ret = of_property_read_string(np, "nfc_nxp_conf_name", &out_value);
	strncpy(g_nfc_nxp_config_name, out_value, MAX_CONFIG_NAME_SIZE - 1);
	if (ret != 0) {
		memset(g_nfc_nxp_config_name, 0, MAX_CONFIG_NAME_SIZE);
		pr_err("%s: can't get nfc nxp config name\n", __func__);
	}
	pr_info("%s: nfc nxp config name:%s\n", __func__, g_nfc_nxp_config_name);

	/* get nfc_brcm_conf_name */
	ret = of_property_read_string(np, "nfc_brcm_conf_name", &out_value);
	strncpy(g_nfc_brcm_config_name, out_value, MAX_CONFIG_NAME_SIZE - 1);
	if (ret != 0) {
		memset(g_nfc_brcm_config_name, 0, sizeof(g_nfc_brcm_config_name));
		pr_err("%s: can't get nfc brcm config name\n", __func__);
	}
	pr_info("%s: nfc brcm config name:%s\n", __func__, g_nfc_brcm_config_name);

	return ;
}

/* FUNCTION: pn547_enable_nfc
 * DESCRIPTION: reset cmd sequence to enable pn547
 * Parameters
 * struct  pn547_dev *pdev: device structure
 * RETURN VALUE
 * none
 */
static void pn547_enable_nfc(struct  pn547_dev *pdev)
{
	/* hardware reset, power on */
	gpio_set_value(pdev->ven_gpio, 1);
	msleep(20);

	/* power off */
	gpio_set_value(pdev->ven_gpio, 0);
	msleep(60);

	/* power on */
	gpio_set_value(pdev->ven_gpio, 1);
	msleep(20);

	return ;
}

/* FUNCTION: pn547_disable_irq
 * DESCRIPTION: disable irq function
 * Parameters
 * struct  pn547_dev *: device structure
 * RETURN VALUE
 * none
 */
static void pn547_disable_irq(struct pn547_dev *pn547_dev)
{
	unsigned long flags;

	spin_lock_irqsave(&pn547_dev->irq_enabled_lock, flags);
	if (pn547_dev->irq_enabled) {
		disable_irq_nosync(pn547_dev->client->irq);
		pn547_dev->irq_enabled = false;
	}
	spin_unlock_irqrestore(&pn547_dev->irq_enabled_lock, flags);
}

/* FUNCTION: pn547_enable_irq
 * DESCRIPTION: enable irq function
 * Parameters
 * struct  pn547_dev *: device structure
 * RETURN VALUE
 * none
 */
static void pn547_enable_irq(struct pn547_dev *pn547_dev)
{
	unsigned long flags;

	spin_lock_irqsave(&pn547_dev->irq_enabled_lock, flags);
	if (!pn547_dev->irq_enabled) {
		pn547_dev->irq_enabled = true;
		enable_irq(pn547_dev->client->irq);
	}
	spin_unlock_irqrestore(&pn547_dev->irq_enabled_lock, flags);
}

/* FUNCTION: pn547_dev_irq_handler
 * DESCRIPTION: irq handler, jump here when receive an irq request from NFC chip
 * Parameters
 * int irq: irq number
 * void *dev_id:device structure
 * RETURN VALUE
 * irqreturn_t: irq handle result
 */
static irqreturn_t pn547_dev_irq_handler(int irq, void *dev_id)
{
	struct pn547_dev *pn547_dev = dev_id;
	if(gpio_get_value(pn547_dev->irq_gpio) != 1) {
		return IRQ_HANDLED;
	}

	pr_err("%s: enter\n", __func__);
	pn547_disable_irq(pn547_dev);
	if (SIGNAL_INTERRUPT_WAIT == wait_event_interruptible_ret) {
		__pm_wakeup_event(&pn547_dev->wl, jiffies_to_msecs(3 * HZ));
	} else {
		__pm_wakeup_event(&pn547_dev->wl, jiffies_to_msecs(1 * HZ));
	}
	/* Wake up waiting readers */
	wake_up(&pn547_dev->read_wq);

	return IRQ_HANDLED;
}

// hide bank card number
uint16_t nfc_check_number(const char *string, uint16_t len)
{
	uint16_t i;
	if (string == NULL)
		return 0;
	for (i = 0; i < len; i++) {
		if (string[i] > '9' || string[i] < '0') // '9' & '0' means char must between 1-9
			return 0;
	}
	return 1; // 1 means success
}
// end

/* FUNCTION: pn547_dev_read
 * DESCRIPTION: read i2c data
 * Parameters
 * struct file *filp:device structure
 * char __user *buf:return to user buffer
 * size_t count:read data count
 * loff_t *offset:offset
 * RETURN VALUE
 * ssize_t: result
 */
static ssize_t pn547_dev_read(struct file *filp, char __user *buf,
		size_t count, loff_t *offset)
{
	struct pn547_dev *pn547_dev = filp->private_data;
	char tmp[MAX_BUFFER_SIZE] = {0};
	char *tmpStr = NULL;
	int ret = -1;
	int i;
	int retry = 0;
	bool isSuccess = false;
	uint16_t buffer_count;
	uint16_t tmp_length;
	// 2 means ascii length of char, 1 means '\0'
	uint16_t tmp_size = sizeof(tmp) * 2 + 1;
	errno_t err_ret;

	/* max size is 512 */
	if (count > MAX_BUFFER_SIZE) {
		count = MAX_BUFFER_SIZE;
	}

	mutex_lock(&pn547_dev->read_mutex);

	/* read data when interrupt occur */
	if (!gpio_get_value(pn547_dev->irq_gpio)) {
		if (filp->f_flags & O_NONBLOCK) {
			ret = -EAGAIN;
			pr_err("%s : EAGAIN,%d\n", __func__, ret);
			goto fail;
		}

		for (i = 0; i < NFC_MAX_IRQ_COUNT; i++) {
			pn547_enable_irq(pn547_dev);
			pr_info("%s: wait ireq\n", __func__);
			ret = wait_event_interruptible(
				pn547_dev->read_wq,
				!pn547_dev->irq_enabled);
			pr_info("%s: ireq received\n", __func__);
			pn547_disable_irq(pn547_dev);

			if (ret)
				goto fail;

			if (gpio_get_value(pn547_dev->irq_gpio))
				break;
		}
	}

	tmpStr = (char *)kzalloc(sizeof(tmp) * 2 + 1, GFP_KERNEL);
	if (!tmpStr) {
		pr_err("%s:Cannot allocate memory for read tmpStr.\n", __func__);
		ret = -ENOMEM;
		goto fail;
	}
	/* Read data, at most tree times */
	for (retry = 0; retry < NFC_TRY_NUM; retry++) {
		ret = i2c_master_recv(pn547_dev->client, tmp, count);
		for (i = 0; i < count; i++) {
			snprintf(&tmpStr[i * 2], 3, "%02X", tmp[i]);
		}
		// 00 means Type A 0A means max UID len
		if ((count > MIN_NCI_CMD_LEN_WITH_DOOR_NUM) &&
			(tmp[NFC_A_PASSIVE] == 0x00) &&
			(tmp[NFCID_LEN] >= DOORCARD_MIN_LEN) &&
			(tmp[NFCID_LEN] <= 0x0A) && g_is_uid) {
				err_ret = memcpy_s(tmpStr + DOORCARD_OVERRIDE_LEN,
					tmp_size - DOORCARD_OVERRIDE_LEN,
					"XXXXXXXXXXXXXXXXXXXX",
					tmp[NFCID_LEN] * BYTE_CHAR_LEN);
				if (err_ret != EOK)
					pr_err("%s: memcpy_s fail,%d\n",
						__func__, err_ret);
		}
		if ((count > MIN_NCI_CMD_LEN_WITH_DOOR_NUM) && (tmp[NCI_CMD_HEAD_OFFSET] == 0x10) && g_is_uid) {
			err_ret = memcpy_s(tmpStr + MIFARE_DATA_UID_OFFSET,
				tmp_size - MIFARE_DATA_UID_OFFSET,
				"XXXXXXXX",
				BANKCARD_NUM_OVERRIDE_LEN);
			if (err_ret != EOK)
				pr_err("%s: memcpy_s fail,%d\n",
				__func__, err_ret);
		}
		// hide bank card number,'6'&'2' means card number head
		buffer_count = count * 2 + 1; // 2 means ascii length of char, 1 means '\0'
		tmp_length = sizeof(tmp) * 2 + 1; // 2 means ascii length of char, 1 means '\0'
		if (buffer_count > MIN_NCI_CMD_LEN_WITH_BANKCARD_NUM) {
			for (i = NCI_CMD_HEAD_OFFSET; i < buffer_count - BANKCARD_NUM_LEN; i += BANKCARD_NUM_HEAD_LEN) {
				if ((*(tmpStr + i) == '6') && (*(tmpStr + i + 1) == '2') &&
					((i + BANKCARD_NUM_OVERRIDE_OFFSET) < tmp_length) &&
					nfc_check_number(tmpStr + i + BANKCARD_NUM_HEAD_LEN, BANKCARD_NUM_VALUE_LEN)) {
					err_ret = memcpy_s(tmpStr + i + BANKCARD_NUM_OVERRIDE_OFFSET,
						tmp_size - BANKCARD_NUM_OVERRIDE_OFFSET - i,
						"XXXXXXXX", BANKCARD_NUM_OVERRIDE_LEN);
					if (err_ret != EOK)
						pr_err("%s: bankcard memcpy_s fail,%d\n",
						__func__, err_ret);
				}
			}
		}
		// end
		/* hide cplc
		 * 4 is start pos of 9F7F and length of 9F7F
		 * 18 is length of cplc and 2 is trans byte "9F" into two char
		 * 13 is length of print info and \0
		 */
		if ((2 * count > 18) && (strncmp(tmpStr + 4, "9F7F", 4) == 0))
			snprintf(tmpStr, 13, "%s", "xxxx9F7Fxxxx");
		pr_info("%s : retry = %d, ret = %d, count = %3d > %s\n", __func__, retry, ret, (int)count, tmpStr);
		if (ret == (int)count) {
			isSuccess = true;
			break;
		} else {
			pr_info("%s : read data try =%d returned %d\n", __func__, retry, ret);
			msleep(1);
			continue;
		}
	}
	// 0&1 means '6105' or '000012' mifare, 2 means nci length
	if (((count >= FIRST_LINE_NCI_LEN) && (tmp[0] == 0x61) &&
		(tmp[1] == 0x05) && (tmp[2] >= 0x0A)) ||
		((tmp[0] == 0x00) && (tmp[1] == 0x00) && (tmp[2] == 0x12))) {
		g_is_uid = true;
	} else {
		g_is_uid = false;
	}
	kfree(tmpStr);
	mutex_unlock(&pn547_dev->read_mutex);
	if (isSuccess == false) {
		pr_err("%s : i2c_master_recv returned %d\n", __func__, ret);
		ret = -EIO;
	}
	if (ret < 0) {
		pr_err("%s: PN547 i2c_master_recv returned %d\n", __func__, ret);
		return ret;
	}
	if (ret > count) {
		pr_err("%s: received too many bytes from i2c (%d)\n", __func__, ret);
		return -EIO;
	}
	/* copy data to user */
	if (copy_to_user(buf, tmp, ret)) {
		pr_warning("%s : failed to copy to user space\n", __func__);
		return -EFAULT;
	}
	return ret;

fail:
	mutex_unlock(&pn547_dev->read_mutex);

	return ret;
}

/* FUNCTION: pn547_dev_write
 * DESCRIPTION: write i2c data
 * Parameters
 * struct file *filp:device structure
 * char __user *buf:user buffer to write
 * size_t count:write data count
 * loff_t *offset:offset
 * RETURN VALUE
 * ssize_t: result
 */
static ssize_t pn547_dev_write(struct file *filp, const char __user *buf,
		size_t count, loff_t *offset)
{
	struct pn547_dev  *pn547_dev = filp->private_data;
	char tmp[MAX_BUFFER_SIZE];
	char *tmpStr = NULL;
	int ret = -1;
	int retry = 0;
	int i;

	/* max size is 512 */
	if (count > MAX_BUFFER_SIZE){
		count = MAX_BUFFER_SIZE;
	}

	/* copy data from user */
	if (copy_from_user(tmp, buf, count)) {
		pr_err("%s : failed to copy from user space\n", __func__);
		return -EFAULT;
	}

	/* Write data */
	tmpStr = (char *)kzalloc(sizeof(tmp) * 2 + 1, GFP_KERNEL);
	if (!tmpStr) {
		pr_info("%s:Cannot allocate memory for write tmpStr.\n", __func__);
		return -ENOMEM;
	}
	/* Write data, at most tree times */
	for (retry = 0; retry < NFC_TRY_NUM; retry++) {
		ret = i2c_master_send(pn547_dev->client, tmp, count);
		for (i = 0; i < count; i++) {
			snprintf(&tmpStr[i * 2], 3, "%02X", tmp[i]);
		}
		pr_info("%s : retry = %d, ret = %d, count = %3d > %s\n", __func__, retry, ret, (int)count, tmpStr);
		if (ret == (int)count) {
			break;
		} else {
			if (retry > 0) {
				pr_info("%s : send retry times =%d returned %d\n", __func__, retry, ret);
			}
			msleep(1);
			continue;
		}
	}
	kfree(tmpStr);

	if (ret != (int)count) {
		pr_err("%s : i2c_master_send returned %d\n", __func__, ret);
		ret = -EIO;
	}

	return ret;
}

/* FUNCTION: pn547_i2c_write
 * DESCRIPTION: write i2c data, only use in check_sim_status
 * Parameters
 * struct  pn547_dev *pdev:device structure
 * const char *buf:user buffer to write
 * int count:write data count
 * RETURN VALUE
 * ssize_t: result
 */
static ssize_t pn547_i2c_write(struct  pn547_dev *pdev,const char *buf,int count)
{
	int ret = -1;
	int retry = 0;
	char *tmpStr = NULL;
	int i;

	tmpStr = (char *)kzalloc(255 * 2, GFP_KERNEL);
	if (!tmpStr) {
		pr_info("%s:Cannot allocate memory for write tmpStr.\n", __func__);
		return -ENOMEM;
	}
	/* Write data, we have 3 chances */
	for (retry = 0; retry < NFC_TRY_NUM; retry++) {
		ret = i2c_master_send(pdev->client, buf, (int)count);
		for (i = 0; i < count; i++) {
			snprintf(&tmpStr[i * 2], 3, "%02X", buf[i]);
		}
		pr_info("%s : retry = %d, ret = %d, count = %3d > %s\n", __func__, retry, ret, (int)count, tmpStr);
		if (ret == (int)count) {
			break;
		} else {
			if (retry > 0) {
				pr_info("%s : send data try =%d returned %d\n", __func__, retry, ret);
			}
			msleep(10);
			continue;
		}
	}
	kfree(tmpStr);
	if (ret != (int)count) {
		pr_err("%s : i2c_master_send returned %d\n", __func__, ret);
		ret = -EIO;
	}

	return (size_t)ret;
}

/* FUNCTION: pn547_i2c_read
 * DESCRIPTION: read i2c data, only use in check_sim_status
 * Parameters
 * struct  pn547_dev *pdev:device structure
 * const char *buf:read to user buffer
 * int count:read data count
 * RETURN VALUE
 * ssize_t: result
 */
static ssize_t pn547_i2c_read(struct  pn547_dev *pdev,char *buf,int count)
{
	int ret = -1;
	int retry = 0;
	char *tmpStr = NULL;
	int i;

	mutex_lock(&pdev->read_mutex);
	if (!gpio_get_value(pdev->irq_gpio)) {
		/* for -ERESTARTSYS, we have 3 chances */
		for (retry = 0; retry < NFC_TRY_NUM; retry++) {
			pn547_disable_irq(pdev);
			pn547_enable_irq(pdev);

			/* wait_event_interruptible_timeout Returns:
			 * 0: if the @timeout elapsed
			 * -ERESTARTSYS: if it was interrupted by a signal
			 * >0:the remaining jiffies (at least 1) if the @condition = true before the @timeout elapsed.*/
			ret = wait_event_interruptible_timeout(pdev->read_wq,
				!pdev->irq_enabled,msecs_to_jiffies(1000));
			pn547_disable_irq(pdev);
			wait_event_interruptible_ret = ret;
			if (ret <= 0) {
				pr_err("%s : wait retry count %d!,ret = %d\n", __func__,retry,ret);
				continue;
			}
			break;
		}

		if (ret <= 0) {
			pr_err("%s : wait_event_interruptible_timeout error!,ret = %d\n", __func__, ret);
			ret = -1;
			goto fail;
		}
	}

	tmpStr = (char *)kzalloc(255 * 2, GFP_KERNEL);
	if (!tmpStr) {
		pr_info("%s:Cannot allocate memory for write tmpStr.\n", __func__);
		ret = -ENOMEM;
		goto fail;
	}
	/* Read data, we have 3 chances */
	for (retry = 0; retry < NFC_TRY_NUM; retry++) {
		ret = i2c_master_recv(pdev->client, buf, (int)count);
		for (i = 0; i < count; i++) {
			snprintf(&tmpStr[i * 2], 3, "%02X", buf[i]);
		}
		pr_info("%s : retry = %d, ret = %d, count = %3d > %s\n", __func__, retry, ret, (int)count, tmpStr);
		if (ret == (int)count) {
			break;
		} else {
			pr_err("%s : read data try =%d returned %d\n", __func__, retry, ret);
			msleep(10);
			continue;
		}
	}
	kfree(tmpStr);
	mutex_unlock(&pdev->read_mutex);

	if (ret != (int)count) {
		pr_err("%s : i2c_master_recv returned %d\n", __func__, ret);
		ret = -EIO;
	}
	return (size_t)ret;

fail:
	mutex_unlock(&pdev->read_mutex);
	return (size_t)ret;
}

/* FUNCTION: check_sim_status
 * DESCRIPTION: To test if the SWP interfaces are operational between the NFCC and the connected NFCEEs.
 *  Test sequence:
 *  1) hardware reset chip
 *  2) send CORE_RESET_CMD
 *  3) send CORE_INIT_CMD
 *  4) send NCI_PROPRIETARY_ACT_CMD
 *  5) send SYSTEM_TEST_SWP_INTERFACE_CMD(SWP1/2)
 * Parameters
 * struct i2c_client *client:i2c device structure
 * struct  pn547_dev *pdev:device structure
 * RETURN VALUE
 * int: check result
 */
static int check_sim_status(struct i2c_client *client, struct  pn547_dev *pdev)
{
	int ret;
	unsigned char recvBuf[40] = {0}; // read buffer max len 40
	const  char send_reset[] = {0x20,0x00,0x01,0x01}; // CORE_RESET_CMD
	const  char init_cmd[] = {0x20,0x01,0x00}; // CORE_INIT_CMD
	const  char read_config[] = {0x2F,0x02,0x00}; // SYSTEM_PROPRIETARY_ACT_CMD
	const  char read_config_UICC[] = {0x2F,0x3E,0x01,0x00}; // SYSTEM_TEST_SWP_INTERFACE_CMD(swp1)
	pr_info("pn547 - %s : enter\n", __func__);

	pdev->sim_status = 0;
	/* hardware reset */
	gpio_set_value(pdev->firm_gpio, 0);
	pn547_enable_nfc(pdev);

	/* write CORE_RESET_CMD */
	ret = pn547_i2c_write(pdev, send_reset, sizeof(send_reset));
	if (ret < 0) {
		pr_err("%s: CORE_RESET_CMD pn547_i2c_write failed, ret:%d\n", __func__, ret);
		goto failed;
	}
	/* read response */
	ret = pn547_i2c_read(pdev, recvBuf, _read_sample_test_size);
	if (ret < 0) {
		pr_err("%s: CORE_RESET_RSP pn547_i2c_read failed, ret:%d\n", __func__, ret);
		goto failed;
	}

	udelay(500);
	/* write CORE_INIT_CMD */
	ret = pn547_i2c_write(pdev, init_cmd, sizeof(init_cmd));
	if (ret < 0) {
		pr_err("%s: CORE_INIT_CMD pn547_i2c_write failed, ret:%d\n", __func__, ret);
		goto failed;
	}
	/* read response */
	ret = pn547_i2c_read(pdev, recvBuf, _read_sample_test_size);
	if (ret < 0) {
		pr_err("%s: CORE_INIT_RSP pn547_i2c_read failed, ret:%d\n", __func__, ret);
		goto failed;
	}

	udelay(500);
	/* write NCI_PROPRIETARY_ACT_CMD */
	ret = pn547_i2c_write(pdev, read_config, sizeof(read_config));
	if (ret < 0) {
		pr_err("%s: PRO_ACT_CMD pn547_i2c_write failed, ret:%d\n", __func__, ret);
		goto failed;
	}
	/* read response */
	ret = pn547_i2c_read(pdev, recvBuf, _read_sample_test_size);
	if (ret < 0) {
		pr_err("%s: PRO_ACT_RSP pn547_i2c_read failed, ret:%d\n", __func__, ret);
		goto failed;
	}

	udelay(500);
	/* write TEST_SWP_CMD UICC */
	ret = pn547_i2c_write(pdev, read_config_UICC, sizeof(read_config_UICC));
	if (ret < 0) {
		pr_err("%s: TEST_UICC_CMD pn547_i2c_write failed, ret:%d\n", __func__, ret);
		goto failed;
	}
	/* read response */
	ret = pn547_i2c_read(pdev, recvBuf, _read_sample_test_size);
	if (ret < 0) {
		pr_err("%s: TEST_UICC_RSP pn547_i2c_read failed, ret:%d\n", __func__, ret);
		goto failed;
	}

	mdelay(10);
	/* read notification */
	ret = pn547_i2c_read(pdev, recvBuf, _read_sample_test_size);
	if (ret < 0) {
		pr_err("%s: TEST_UICC_NTF pn547_i2c_read failed, ret:%d\n", __func__, ret);
		goto failed;
	}

	/* NTF's format: 6F 3E 02 XX1 XX2 -> "XX1 == 0" means SWP link OK */
	if (recvBuf[0] == 0x6F && recvBuf[1] == 0x3E && recvBuf[3] == 0x00) {
		pdev->sim_status |= UICC_SUPPORT_CARD_EMULATION;
	}

	return pdev->sim_status;
failed:
	pdev->sim_status = ret;
	return ret;
}

static void gt_set_idle(struct pn547_dev *pdev)
{
	int ret;

	/* 10 bytes read response buf for idle configure */
	unsigned char recv_buf[10] = {0};
	/* Set NFCC to Idle */
	const char idle_config[] = { 0x21, 0x06, 0x01, 0x00 };

	/* write idle_config */
	ret = pn547_i2c_write(pdev, idle_config, sizeof(idle_config));
	if (ret < 0)
		pr_err("%s: pn547_i2c_write failed, ret:%d\n", __func__, ret);

	ret = pn547_i2c_read(pdev, recv_buf, sizeof(recv_buf));
	if (ret < 0)
		pr_err("%s: pn547_i2c_read failed, ret:%d\n", __func__, ret);

	/* delay 500us for idle config */
	udelay(500);
}

static int gt_dcdc_config(struct pn547_dev *pdev, const char *dcdc_config)
{
	int ret;
	int retry;
	/* 10 bytes read response buf for idle configure */
	unsigned char recv_buf[10] = {0};
	/* read register cmd length = 18 */
	const int nfc_cmd_length = 18;

	/* dcdc_config */
	for (retry = 0; retry < NFC_TRY_NUM; retry++) {
		ret = pn547_i2c_write(pdev, dcdc_config, nfc_cmd_length);
		if (ret < 0) {
			pr_err("%s: pn547_i2c_write failed, ret:%d\n", __func__, ret);
			return -1;
		}

		ret = pn547_i2c_read(pdev, recv_buf, sizeof(recv_buf));
		if (ret < 0) {
			pr_err("%s: pn547_i2c_read failed, ret:%d\n", __func__, ret);
			msleep(1);
			continue;
		}
		/* recv_buf[3] means status, 00 means status OK */
		if (recv_buf[3] == 0x00) {
			pr_info("%s : dcdc config successful", __func__);
			break;
		} else {
			pr_info("%s : dcdc config try =%d returned status =%02X\n",
				__func__, retry, recv_buf[3]);
			/* delay 1ms for retry */
			msleep(1);
			continue;
		}
	}
	/* response recv_buf[3] != 0x00 means config status error */
	if (recv_buf[3] != 0x00)
		return -1;

	return 1;
}

static int gt_boost_config(struct pn547_dev *pdev)
{
	int ret;
	int retry;
	/* read register response length = 19 */
	const int nfc_response_length = 19;
	/* recv_config_buf: read register response buf */
	unsigned char recv_config_buf[19] = {0};
	/* NXP_EXT_TVDD_CFG_2 configure cmd, cmd length = 18 */
	char dcdc_config[18] = {
		0x20, 0x02, 0x0F, 0x01, 0xA0, 0x0E, 0x0B, 0x11, 0x01,
		0xC2, 0xC2, 0x00, 0xBA, 0x26, 0x13, 0x00, 0xD0, 0xFF
	};
	/* read register cmd/recv head_length = 7/8 */
	const int cmd_head_length = 7;
	const int recv_head_length = 8;
	/* Read register A00E cmd */
	const char get_config[] = { 0x20, 0x03, 0x03, 0x01, 0xA0, 0x0E };

	gt_set_idle(pdev);

	/* get configure from register A00E */
	for (retry = 0; retry < NFC_TRY_NUM; retry++) {
		ret = pn547_i2c_write(pdev, get_config, sizeof(get_config));
		if (ret < 0) {
			pr_err("%s: write get_dcdc_config failed, ret:%d\n",
				__func__, ret);
			continue;
		}

		ret = pn547_i2c_read(pdev, recv_config_buf, sizeof(recv_config_buf));
		if (ret < 0) {
			pr_err("%s: pn547_i2c_read failed, ret:%d\n", __func__, ret);
			continue;
		}

		/* 4003 means register read response, 00 means status OK */
		if ((recv_config_buf[0] == 0x40) && (recv_config_buf[1] == 0x03)
			&& (recv_config_buf[3] == 0x00)) {
			pr_info("%s: get_dcdc_config successful", __func__);
			memcpy(dcdc_config + cmd_head_length,
				recv_config_buf + recv_head_length,
				nfc_response_length - recv_head_length);
			/* 12 is tx_p_req position, BA means enabled */
			/* 13 is DCDC parameter position, 26 is recommended parameter */
			dcdc_config[12] = 0xBA;
			dcdc_config[13] = 0x26;
			break;
		} else {
			pr_info("%s: read register try =%d, returned status =%02X\n",
				__func__, retry, recv_config_buf[3]);
			continue;
		}
	}

	/* 4003 means register read response, 00 means status OK */
	if (!((recv_config_buf[0] == 0x40) && (recv_config_buf[1] == 0x03)
		&& (recv_config_buf[3] == 0x00))) {
		pr_err("%s : read register failed, aborting...", __func__);
		return -1;
	}

	ret = gt_dcdc_config(pdev, (const char *)dcdc_config);
	return ret;
}

/* FUNCTION: nfc_fwupdate_store
 * DESCRIPTION: store nfc firmware update result.
 * Parameters
 * struct device *dev:device structure
 * struct device_attribute *attr:device attribute
 * const char *buf:user buffer
 * size_t count:data count
 * RETURN VALUE
 * ssize_t: result
 */
static ssize_t nfc_fwupdate_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	/* if success, store firmware update result into variate firmware_update */
	if (buf[0] == '1') {
		firmware_update = 1;
		pr_err("%s:firmware update success\n", __func__);
	}

	return (ssize_t)count;
}

/* FUNCTION: nfc_fwupdate_show
 * DESCRIPTION: show nfc firmware update result.
 * Parameters
 * struct device *dev:device structure
 * struct device_attribute *attr:device attribute
 * const char *buf:user buffer
 * RETURN VALUE
 * ssize_t:  result
 */
static ssize_t nfc_fwupdate_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	/* get firmware update result from variate firmware_update */
	return (ssize_t)(snprintf(buf, sizeof(firmware_update)+1, "%d", firmware_update));
}

/* FUNCTION: nxp_config_name_store
 * DESCRIPTION: store nxp_config_name, infact do nothing now.
 * Parameters
 *  struct device *dev:device structure
 *  struct device_attribute *attr:device attribute
 *  const char *buf:user buffer
 *  size_t count:data count
 * RETURN VALUE
 *  ssize_t:  result
 */
static ssize_t nxp_config_name_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	/* no need store config name, do nothing now */
	return (ssize_t)count;
}

/* FUNCTION: nxp_config_name_show
 * DESCRIPTION: get nxp_config_name from variate g_nfc_nxp_config_name.
 * Parameters
 *  struct device *dev:device structure
 *  struct device_attribute *attr:device attribute
 *  const char *buf:user buffer
 * RETURN VALUE
 *  ssize_t:  result
 */
static ssize_t nxp_config_name_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return (ssize_t)(snprintf(buf, strlen(g_nfc_nxp_config_name)+1, "%s", g_nfc_nxp_config_name));
}
/*FUNCTION: nfc_brcm_conf_name_store
  *DESCRIPTION: store nfc_brcm_conf_name, infact do nothing now.
  *Parameters
  * struct device *dev:device structure
  * struct device_attribute *attr:device attribute
  * const char *buf:user buffer
  * size_t count:data count
  *RETURN VALUE
  * ssize_t:  result */
static ssize_t nfc_brcm_conf_name_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	/* no need store config name, do nothing now */
	return (ssize_t)count;
}
/* FUNCTION: nfc_brcm_conf_name_show
 * DESCRIPTION: get nfc_brcm_conf_name from variate g_nfc_brcm_config_name.
 * Parameters
 *  struct device *dev:device structure
 *  struct device_attribute *attr:device attribute
 *  const char *buf:user buffer
 * RETURN VALUE
 *  ssize_t:  result
 */
static ssize_t nfc_brcm_conf_name_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return (ssize_t)(snprintf(buf, strlen(g_nfc_brcm_config_name)+1, "%s", g_nfc_brcm_config_name));
}
/* FUNCTION: nfc_sim_status_show
 * DESCRIPTION: get nfc-sim card support result.
 * Parameters
 *  struct device *dev:device structure
 *  struct device_attribute *attr:device attribute
 *  const char *buf:user buffer
 * RETURN VALUE
 *		eSE		UICC	value
 *		0		0		0	(not support)
 *		0		1		1	(swp1 support)
 *		1		0		2	(swp2 support)
 *		1		1		3	(all support)
 *		<0	:error
 */
static ssize_t nfc_sim_status_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int status=-1;
	int retry = 0;

	struct i2c_client *i2c_client_dev = container_of(dev, struct i2c_client, dev);
	struct pn547_dev *pn547_dev;
	pn547_dev = i2c_get_clientdata(i2c_client_dev);

	if (pn547_dev == NULL) {
		pr_err("%s:  pn547_dev == NULL!\n", __func__);
		return status;
	}

	pr_info("%s: enter!\n", __func__);
	/* if failed, we have 3 chances */
	for (retry = 0; retry < NFC_TRY_NUM; retry++){
		status = check_sim_status(i2c_client_dev,pn547_dev);
		if(status < 0){
			pr_err("%s: check_sim_status error!retry count=%d\n", __func__, retry);
			msleep(10);
			continue;
		}
		break;
	}

	pr_info("%s: status=%d\n", __func__, status);
	return (ssize_t)(snprintf(buf, MAX_ATTRIBUTE_BUFFER_SIZE-1, "%d\n", status));
}

/* FUNCTION: nfc_sim_switch_store
 * DESCRIPTION: save user sim card select result.
 * Parameters
 *  struct device *dev:device structure
 *  struct device_attribute *attr:device attribute
 *  const char *buf:user buffer
 *  size_t count:data count
 * RETURN VALUE
 *  ssize_t:result
 */
static ssize_t nfc_sim_switch_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	struct i2c_client *i2c_client_dev = container_of(dev, struct i2c_client, dev);
	struct pn547_dev *pn547_dev;
	int val = 0;

	pn547_dev = i2c_get_clientdata(i2c_client_dev);
	if(pn547_dev == NULL)	{
		pr_err("%s:  pn547_dev == NULL!\n", __func__);
		return 0;
	}

	/* save card select result */
	if (sscanf(buf, "%1d", &val) == 1) {
		pr_err("%s: input val = %d!\n", __func__,val);
		if (val == CARD1_SELECT) {
			pn547_dev->sim_switch = CARD1_SELECT;
		} else if (val == CARD2_SELECT) {
			pn547_dev->sim_switch = CARD2_SELECT;
		} else if (val == UICC_CARD2_SELECT) {
			pn547_dev->sim_switch = UICC_CARD2_SELECT;
		} else {
			return -EINVAL;
		}
	} else {
		return -EINVAL;
	}

	return (ssize_t)count;
}

/* FUNCTION: nfc_sim_switch_show
 * DESCRIPTION: get user sim card select result.
 * Parameters
 *  struct device *dev:device structure
 *  struct device_attribute *attr:device attribute
 *  const char *buf:user buffer
 * RETURN VALUE
 *  ssize_t:  result
 */
static ssize_t nfc_sim_switch_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *i2c_client_dev = container_of(dev, struct i2c_client, dev);
	struct pn547_dev *pn547_dev;
	pn547_dev = i2c_get_clientdata(i2c_client_dev);

	if(pn547_dev == NULL)	{
		pr_err("%s:  pn547_dev == NULL!\n", __func__);
		return 0;
	}
	return (ssize_t)(snprintf(buf,  MAX_ATTRIBUTE_BUFFER_SIZE-1, "%d\n", pn547_dev->sim_switch));
}

static ssize_t nfc_gt_boost_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int status = -1;  // -1: error; 0: not supported on board; 1: successful
	struct i2c_client *i2c_client_dev = container_of(dev, struct i2c_client, dev);
	struct pn547_dev *pn547_dev;

	pn547_dev = i2c_get_clientdata(i2c_client_dev);
	if (pn547_dev == NULL) {
		pr_err("%s: pn547_dev == NULL!\n", __func__);
		return status;
	}
	pr_info("%s: enter!\n", __func__);
	if (g_is_gtboost) {
		status = gt_boost_config(pn547_dev);
		pr_info("%s: status=%d\n", __func__, status);
	} else {
		status = 0;
		pr_info("%s: gt_boost not supported on board\n", __func__);
	}
	return (ssize_t)(snprintf(buf, MAX_ATTRIBUTE_BUFFER_SIZE - 1, "%d\n", status));
}

/* FUNCTION: rd_nfc_sim_status_show
 * DESCRIPTION: get nfc-sim card support result from variate, no need to send check commands again.
 * Parameters
 *  struct device *dev:device structure
 *  struct device_attribute *attr:device attribute
 *  const char *buf:user buffer
 * RETURN VALUE
 *		eSE		UICC	value
 *		0		0		0	(not support)
 *		0		1		1	(swp1 support)
 *		1		0		2	(swp2 support)
 *		1		1		3	(all support)
 *		<0	:error
 */
static ssize_t rd_nfc_sim_status_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int status=-1;
	struct i2c_client *i2c_client_dev = container_of(dev, struct i2c_client, dev);
	struct pn547_dev *pn547_dev;
	pn547_dev = i2c_get_clientdata(i2c_client_dev);
	if (pn547_dev == NULL) {
		pr_err("%s:  pn547_dev == NULL!\n", __func__);
		return status;
	}
	return (ssize_t)(snprintf(buf, MAX_ATTRIBUTE_BUFFER_SIZE-1,"%d\n", pn547_dev->sim_status));
}

/* FUNCTION: nfc_enable_status_store
 * DESCRIPTION: store nfc_enable_status for RD test.
 * Parameters
 *  struct device *dev:device structure
 *  struct device_attribute *attr:device attribute
 *  const char *buf:user buffer
 *  size_t count:data count
 * RETURN VALUE
 *  ssize_t:  result
 */
static ssize_t nfc_enable_status_store(struct device *dev, struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct i2c_client *i2c_client_dev = container_of(dev, struct i2c_client, dev);
	struct pn547_dev *pn547_dev;
	int val = 0;

	pn547_dev = i2c_get_clientdata(i2c_client_dev);
	if(pn547_dev == NULL)	{
		pr_err("%s:  pn547_dev == NULL!\n", __func__);
		return 0;
	}

	/* save nfc enable status */
	if (sscanf(buf, "%1d", &val) == 1) {
		if(val == ENABLE_START){
			pn547_dev->enable_status = ENABLE_START;
		}else if(val == ENABLE_END){
			pn547_dev->enable_status = ENABLE_END;
		}else{
			return -EINVAL;
		}
	}else{
		return -EINVAL;
	}

	return (ssize_t)count;
}

/* FUNCTION: nfc_enable_status_show
 * DESCRIPTION: show nfc_enable_status for RD test.
 * Parameters
 *  struct device *dev:device structure
 *  struct device_attribute *attr:device attribute
 *  const char *buf:user buffer
 * RETURN VALUE
 *  ssize_t:  result
 */
static ssize_t nfc_enable_status_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *i2c_client_dev = container_of(dev, struct i2c_client, dev);
	struct pn547_dev *pn547_dev;
	pn547_dev = i2c_get_clientdata(i2c_client_dev);
	if(pn547_dev == NULL){
		pr_err("%s:  pn547_dev == NULL!\n", __func__);
		return 0;
	}
	return (ssize_t)(snprintf(buf,  MAX_ATTRIBUTE_BUFFER_SIZE-1, "%d\n", pn547_dev->enable_status));
}

/* FUNCTION: nfc_card_num_show
 * DESCRIPTION: show supported nfc_card_num, which config in device tree system.
 * Parameters
 *  struct device *dev:device structure
 *  struct device_attribute *attr:device attribute
 *  const char *buf:user buffer
 * RETURN VALUE
 *  ssize_t:  result
 */
static ssize_t nfc_card_num_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int temp = 0;
	int ret = -1;
	struct device_node *np = dev->of_node;

	/* read supported nfc_card_num from device tree system.*/
	ret=of_property_read_u32(np, "nfc_card_num", &temp);
	if(ret){
		temp = UICC_SUPPORT_CARD_EMULATION;
		pr_err("%s: can't get nfc card num config!\n", __func__);
	}
	return (ssize_t)(snprintf(buf,  MAX_ATTRIBUTE_BUFFER_SIZE-1, "%d\n", (unsigned char)temp));
}

/* FUNCTION: nfc_chip_type_show
 * DESCRIPTION: show nfc_chip_type, which config in device tree system.
 * Parameters
 *  struct device *dev:device structure
 *  struct device_attribute *attr:device attribute
 *  const char *buf:user buffer
 * RETURN VALUE
 *  ssize_t:  result
 */
static ssize_t nfc_chip_type_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret = -1;
	struct device_node *np = dev->of_node;
	const char *out_value = NULL;

	memset(nfc_chip_type, 0, MAX_NFC_CHIP_TYPE_SIZE);
	/* read nfc_chip_type from device tree system.*/
	ret=of_property_read_string(np, "nfc_chip_type", &out_value);
	strncpy(nfc_chip_type, out_value, MAX_NFC_CHIP_TYPE_SIZE-1);
	if(ret != 0){
		pr_err("%s: can't get nfc nfc_chip_type, default pn547\n", __func__);
		strncpy(nfc_chip_type, "pn547", MAX_NFC_CHIP_TYPE_SIZE-1);
	}

	if (!strncasecmp(nfc_chip_type, "nfc_sz", strlen("nfc_sz"))) {
		memset(nfc_chip_type, 0, MAX_NFC_CHIP_TYPE_SIZE);
		strncpy(nfc_chip_type, "sn110u", strlen("sn110u"));
        }

	if (!strncasecmp(nfc_chip_type, "nfc_dg", strlen("nfc_dg"))) {
		memset(nfc_chip_type, 0, MAX_NFC_CHIP_TYPE_SIZE);
		strncpy(nfc_chip_type, "pn80t", strlen("pn80t"));
	}
	return (ssize_t)(snprintf(buf, MAX_ATTRIBUTE_BUFFER_SIZE - 1, "%s", nfc_chip_type));
}


void set_nfc_single_channel(struct device *dev)
{
	int ret = -1;
	struct device_node *np = dev->of_node;
	const char *out_value = NULL;

	pr_err("%s: nfc single channel begin:%d\n", __func__, g_nfc_single_channel);
	ret=of_property_read_string(np, "nfc_single_channel", &out_value);
	if (ret != 0) {
		pr_err("%s: can't get nfc single channel dts config\n", __func__);
		g_nfc_single_channel = 0;
		return;
	}
	pr_err("%s: get nfc single channel dts config %s\n", __func__,out_value);
	if (!strcasecmp(out_value, "true")) {
		g_nfc_single_channel = 1;
	}
	pr_err("%s: nfc single channel:%d\n", __func__, g_nfc_single_channel);
	return;

}
static ssize_t nfc_single_channel_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return (ssize_t)(snprintf(buf, MAX_ATTRIBUTE_BUFFER_SIZE-1, "%d", g_nfc_single_channel));
}

int nfc_record_dmd_info(long dmd_no, const char *dmd_info)
{
#ifdef CONFIG_HUAWEI_DSM
    if (dmd_no < NFC_DMD_NUMBER_MIN || dmd_no > NFC_DMD_NUMBER_MAX
        || dmd_info == NULL || NULL == nfc_dclient) {
        pr_info("%s: para error: %ld\n", __func__, dmd_no);
        return -1;
    }

    pr_info("%s: dmd no: %ld - %s", __func__, dmd_no, dmd_info);
    if (!dsm_client_ocuppy(nfc_dclient)) {
        dsm_client_record(nfc_dclient, "DMD info:%s", dmd_info);
        dsm_client_notify(nfc_dclient, dmd_no);
    }
#endif
    return 0;
}

static ssize_t nfc_hal_dmd_info_store(struct device *dev, struct device_attribute *attr,
             const char *buf, size_t count)
{
    int val = 0;
    char dmd_info_from_hal[64] = {'\0'};
    /* The length of DMD error number is 9 */
    if (sscanf(buf, "%9d", &val) == 1) {
        if (val < NFC_DMD_NUMBER_MIN || val > NFC_DMD_NUMBER_MAX) {
            return (ssize_t)count;
        }
        g_nfc_hal_dmd_no = val;
        /* The max length of content for current dmd description set as 63.
           Example for DMD Buf: '923002014 CoreReset:600006A000D1A72000'.
           A space as a separator is between dmd error no and description.*/
        if (sscanf(buf, "%*s%63s", dmd_info_from_hal) == 1) {
            nfc_record_dmd_info(val, dmd_info_from_hal);
        }

    } else {
        pr_err("%s: get dmd number error\n", __func__);
    }
    return (ssize_t)count;
}

static ssize_t nfc_hal_dmd_info_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    return (ssize_t)(snprintf(buf, MAX_ATTRIBUTE_BUFFER_SIZE-1, "%d", g_nfc_hal_dmd_no));
}
static ssize_t nfc_at_result_store(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	if(buf!=NULL)
	{
		nfc_at_result=buf[0]-CHAR_0; /* file storage str */
	}
	return (ssize_t)count;
}

static ssize_t nfc_at_result_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return (ssize_t)(snprintf(buf, sizeof(nfc_at_result)+1, "%d", nfc_at_result));
}

static ssize_t nfc_clk_src_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return (ssize_t)(snprintf(buf, MAX_ATTRIBUTE_BUFFER_SIZE - 1, "%d", g_nfc_clk_src));
}

static ssize_t nfc_switch_state_store(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	if(buf!=NULL)
	{
		nfc_switch_state=buf[0]-CHAR_0; /* file storage str */
	}
	return (ssize_t)count;
}
static int nfc_get_dts_config_u32(const char *node_name, const char *prop_name, u32 *pvalue)
{
	struct device_node *np = NULL;
	int ret = -1;

	for_each_node_with_property(np, node_name) {
		ret = of_property_read_u32(np, prop_name, pvalue);
		if (ret != 0) {
			pr_debug("%s: can not get prop values with prop_name: %s\n", __func__, prop_name);
		} else {
			pr_info("%s: %s=%d\n", __func__, prop_name, *pvalue);
		}
	}
	return ret;
}

static int nfc_get_ese_spi_bus(void)
{
	int ret = -1;
	ret = nfc_get_dts_config_u32("ese_config_spi_bus", "ese_config_spi_bus", &g_ese_spi_bus);
	if (ret != 0) {
		pr_err("%s: can't get nfc spi_bus config node!\n", __func__);
		return -1;
	}
	pr_err("%s: g_ese_spi_bus = %d\n", __func__, g_ese_spi_bus);
	return 0;
}

static int nfc_get_ese_pwr_req_gpio(void)
{
	int ret = -1;
	ret = nfc_get_dts_config_u32("gpio_svdd_pwr_req", "gpio_svdd_pwr_req", &g_ese_svdd_pwr_req);
	if (ret != 0) {
		pr_err("%s: can't get nfc gpio_svdd_pwr_req config node!\n", __func__);
		return -1;
	}
	pr_err("%s: g_ese_svdd_pwr_req = %d\n", __func__, g_ese_svdd_pwr_req);
	return 0;
}

static int get_dev_name_type()
{
	int type = 0;
	int ret;

	/* read supported nfc_card_num from device tree system.*/
	ret = nfc_get_dts_config_u32("nfc_dev_name_type", "nfc_dev_name_type", &type);
	if (ret != 0){
		pr_err("%s: can't get nfc_dev_name_type!\n", __func__);
	}
	pr_err("%s: get_dev_name_type = %d\n", __func__, type);
	return type;
}

static ssize_t nfc_dev_name_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int temp = get_dev_name_type();
	return (ssize_t)(snprintf(buf,  MAX_ATTRIBUTE_BUFFER_SIZE-1, "%d\n", (unsigned char)temp));
}

void nfc_write_conf_to_tee(void)
{
	struct arm_smccc_res res;
	uint32_t x1;
	uint32_t x2;
	uint16_t gpio_svdd_pwr_req = g_ese_svdd_pwr_req;
	uint8_t spi_bus = g_ese_spi_bus;
	uint8_t ese_type = 3; // ese type 3:sn110u

	if (g_is_sn110) {
		ese_type = 3; // ese type 3:sn110u
	} else {
		ese_type = 2; // ese type 2:pn80T
	}
	x1 = ese_type;
	x1 = x1 | (spi_bus << OFFSET_8);
	x1 = x1 | (ESE_MAGIC_NUM << OFFSET_16);

	x2 = gpio_svdd_pwr_req;
	x2 = x2 | (ESE_MAGIC_NUM << OFFSET_16);
	arm_smccc_1_1_smc(MTK_SIP_KERNEL_ESE_CONF_TO_TEE_ADDR_AARCH64, x1, x2, 0, &res);
	pr_err("%s: nfc_write_conf_to_tee, x1:0x%X x2:0x%X\n", __func__, x1, x2);
	return;
}

static ssize_t nfc_switch_state_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return (ssize_t)(snprintf(buf, sizeof(nfc_switch_state)+1, "%d", nfc_switch_state));
}

static ssize_t nfc_activated_se_info_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return (ssize_t)(snprintf(buf, MAX_ATTRIBUTE_BUFFER_SIZE-1, "%d", g_nfc_activated_se_info));
}

static ssize_t nfc_activated_se_info_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	int val = 0;
	if (sscanf(buf, "%1d", &val) == 1) {
		g_nfc_activated_se_info = val;
	} else {
		g_nfc_activated_se_info = 0;
	}
	return (ssize_t)count;
}

static ssize_t nfc_wired_ese_info_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return (ssize_t)(snprintf(buf, MAX_ATTRIBUTE_BUFFER_SIZE-1, "%d", g_nfc_ese_type));
}

static ssize_t nfc_wired_ese_info_store(struct device *dev, struct device_attribute *attr,
	const char *buf, size_t count)
{
	int val = 0;
	if (sscanf(buf, "%1d", &val) == 1) {
		g_nfc_ese_type = val;
	} else {
		pr_err("%s: set g_nfc_ese_type  error\n", __func__);
	}
	pr_info("%s: g_nfc_ese_type:%d\n", __func__,g_nfc_ese_type);
	return (ssize_t)count;
}

/* register device node to communication with user space */
static struct device_attribute pn547_attr[] = {
	__ATTR(nfc_fwupdate, 0664, nfc_fwupdate_show, nfc_fwupdate_store),
	__ATTR(nxp_config_name, 0664, nxp_config_name_show, nxp_config_name_store),
	__ATTR(nfc_brcm_conf_name, 0664, nfc_brcm_conf_name_show, nfc_brcm_conf_name_store),
	__ATTR(nfc_sim_switch, 0664, nfc_sim_switch_show, nfc_sim_switch_store),
	__ATTR(nfc_sim_status, 0444, nfc_sim_status_show, NULL),
	__ATTR(rd_nfc_sim_status, 0444, rd_nfc_sim_status_show, NULL),
	__ATTR(nfc_enable_status, 0664, nfc_enable_status_show, nfc_enable_status_store),
	__ATTR(nfc_card_num, 0444, nfc_card_num_show, NULL),
	__ATTR(nfc_chip_type, 0444, nfc_chip_type_show, NULL),
	__ATTR(nfc_single_channel, 0444, nfc_single_channel_show, NULL),
	__ATTR(nfc_wired_ese_type, 0664, nfc_wired_ese_info_show, nfc_wired_ese_info_store),
	__ATTR(nfc_activated_se_info, 0664, nfc_activated_se_info_show, nfc_activated_se_info_store),
	__ATTR(nfc_hal_dmd, 0664, nfc_hal_dmd_info_show, nfc_hal_dmd_info_store),
	__ATTR(nfc_clk_src, 0444, nfc_clk_src_show, NULL),
	__ATTR(nfc_switch_state, 0664, nfc_switch_state_show, nfc_switch_state_store),
	__ATTR(nfc_at_result, 0664, nfc_at_result_show, nfc_at_result_store),
	__ATTR(nfc_dev_name, 0664, nfc_dev_name_show, NULL),
	__ATTR(nfc_gt_boost, 0444, nfc_gt_boost_show, NULL),
};

/* FUNCTION: create_sysfs_interfaces
 * DESCRIPTION: create_sysfs_interfaces.
 * Parameters
 * struct device *dev:device structure
 * RETURN VALUE
 * int:  result
 */
static int create_sysfs_interfaces(struct device *dev)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(pn547_attr); i++) {
		if (device_create_file(dev, pn547_attr + i)) {
			goto error;
		}
	}

	return 0;
error:
	for ( ; i >= 0; i--) {
		device_remove_file(dev, pn547_attr + i);
	}

	pr_err("%s:pn547 unable to create sysfs interface.\n", __func__ );
	return -1;
}
/* FUNCTION: remove_sysfs_interfaces
 * DESCRIPTION: remove_sysfs_interfaces.
 * Parameters
 * struct device *dev:device structure
 * RETURN VALUE
 * int:  result
 */
static int remove_sysfs_interfaces(struct device *dev)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(pn547_attr); i++) {
		device_remove_file(dev, pn547_attr + i);
	}

	return 0;
}

static int nfc_ven_low_beforepwd(struct notifier_block *this, unsigned long code,
                void *unused)
{
    pr_info("[%s]: enter!\n", __func__);
    if(nfcdev == NULL)
    {
        pr_err("[%s]: nfcdev NULL.don't need close nfc!\n", __func__);
        return 0;
    }
    gpio_set_value(nfcdev->ven_gpio,0 );
    msleep(10);
    return 0;
}

static struct notifier_block nfc_ven_low_notifier = {
    .notifier_call = nfc_ven_low_beforepwd,
};

/* FUNCTION: check_pn547
 * DESCRIPTION: To test if nfc chip is ok
 * Parameters
 * struct i2c_client *client:i2c device structure
 * struct  pn547_dev *pdev:device structure
 * RETURN VALUE
 * int: check result
 */
static int check_pn547(struct i2c_client *client, struct  pn547_dev *pdev)
{
	int ret = -1;
	int count = 0;
	const char host_to_pn547[1] = {0x20};
	const char firm_dload_cmd[8]={0x00, 0x04, 0xD0, 0x09, 0x00, 0x00, 0xB1, 0x84};

	/* power on */
	gpio_set_value(pdev->firm_gpio, 0);
	pn547_enable_nfc(pdev);

	do {
		ret = i2c_master_send(client, host_to_pn547, sizeof(host_to_pn547));
		if (ret < 0) {
			pr_err("%s:pn547_i2c_write failed and ret = %d,at %d times\n", __func__, ret, count);
		} else {
			pr_info("%s:pn547_i2c_write success and ret = %d,at %d times\n", __func__, ret, count);
			msleep(10);
			pn547_enable_nfc(pdev);

			return ret;
		}
		count++;
		msleep(10);
	} while (count < NFC_TRY_NUM);

	/* In case firmware dload failed, will cause host_to_pn547 cmd send failed */
	for (count = 0; count < NFC_TRY_NUM; count++) {
		gpio_set_value(pdev->firm_gpio, 1);
		pn547_enable_nfc(pdev);

		ret = i2c_master_send(client, firm_dload_cmd, sizeof(firm_dload_cmd));
		if (ret < 0) {
			pr_err("%s:pn547_i2c_write download cmd failed:%d, ret = %d\n", __func__, count, ret);
			continue;
		}
		gpio_set_value(pdev->firm_gpio, 0);
		pn547_enable_nfc(pdev);

		break;
	}

	gpio_set_value(pdev->firm_gpio, 0);

	return ret;
}

/* FUNCTION: pn547_dev_open
 * DESCRIPTION: pn547_dev_open, used by user space to enable pn547
 * Parameters
 * struct inode *inode:device inode
 * struct file *filp:device file
 * RETURN VALUE
 * int: result
 */
static int pn547_dev_open(struct inode *inode, struct file *filp)
{
	struct pn547_dev *pn547_dev = container_of(filp->private_data,
						struct pn547_dev,
						pn547_device);
	filp->private_data = pn547_dev;
	pr_err("%s : %d,%d\n", __func__, imajor(inode), iminor(inode));

	return 0;
}

/* FUNCTION: pn547_dev_ioctl
 * DESCRIPTION: pn547_dev_ioctl, used by user space
 * Parameters
 * struct file *filp:device file
 * unsigned int cmd:command
 * unsigned long arg:parameters
 * RETURN VALUE
 * long: result
 */
static long pn547_dev_ioctl(struct file *filp,
			    unsigned int cmd, unsigned long arg)
{
	struct pn547_dev *pn547_dev = filp->private_data;

	pr_info("%s ++    cmd = 0x%x \n", __func__,cmd);

	switch (cmd) {
	case PN544_SET_PWR:
		if (arg == 6) { // 6 means sn110 download complete
			pr_err("%s FW GPIO set to 0x00 >>>>>>>\n", __func__);
			gpio_set_value(pn547_dev->firm_gpio, 0);

			msleep(5);
		} else if (arg == 4) { // 4 means sn110 download start
			pr_err("%s FW dwldioctl called from NFC \n", __func__);
			gpio_set_value(pn547_dev->firm_gpio, 1);

			msleep(10);
		} else if (arg == 2) {
			/* power on with firmware download (requires hw reset)
			 */
			gpio_set_value(pn547_dev->ven_gpio,0);
			gpio_set_value(pn547_dev->firm_gpio, 1);
			msleep(60);
			gpio_set_value(pn547_dev->ven_gpio, 0);
			msleep(60);
			gpio_set_value(pn547_dev->ven_gpio, 1);
			msleep(60);
		} else if (arg == 1) {
			/* power on */
			pr_err("%s power on\n", __func__);
			gpio_set_value(pn547_dev->firm_gpio, 0);
			gpio_set_value(pn547_dev->ven_gpio, 1);// 1
			pn547_enable_irq_wake(pn547_dev);
			msleep(20);
		} else  if (arg == 0) {
			/* power off */
			pr_err("%s power off\n", __func__);
			gpio_set_value(pn547_dev->firm_gpio, 0);
			if (g_is_sn110) {
				pr_info("%s sn110 power on\n", __func__);
			} else {
				gpio_set_value(pn547_dev->ven_gpio, 0); //0
			}
			pn547_disable_irq_wake(pn547_dev);
			msleep(60);
		} else if (arg == 3) {
			pr_info("%s Read Cancel\n", __func__);
			pn547_dev->cancel_read = true;
			wake_up(&pn547_dev->read_wq);
		} else {
			pr_err("%s bad arg %lu\n", __func__, arg);
			return -EINVAL;
		}
		break;
	default:
		pr_err("%s bad ioctl 0x%x\n", __func__, cmd);
		return -EINVAL;
	}

	return 0;
}

static const struct file_operations pn547_dev_fops = {
	.owner	= THIS_MODULE,
	.llseek	= no_llseek,
	.read	= pn547_dev_read,
	.write	= pn547_dev_write,
	.open	= pn547_dev_open,
	.unlocked_ioctl	= pn547_dev_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl   = pn547_dev_ioctl,
#endif
};

/* FUNCTION: pn547_parse_dt
 * DESCRIPTION: pn547_parse_dt, get gpio configuration from device tree system
 * Parameters
 * struct device *dev:device data
 * struct pn547_i2c_platform_data *pdata:i2c data
 * RETURN VALUE
 * int: result
 */
static int pn547_parse_dt(struct device *dev,
			 struct pn547_i2c_platform_data *pdata)
{
	struct device_node *np = dev->of_node;
 	int ret = 0;

	/* int gpio */
	pdata->irq_gpio =  of_get_named_gpio_flags(np, "nxp,nfc_int", 0,NULL);
	pr_err( "pdata->irq_gpio = %d \n", pdata->irq_gpio);
	if (pdata->irq_gpio < 0) {
		pr_err( "failed to get \"huawei,nfc_int\"\n");
		goto err;
	}

	/* nfc_fm_dload gpio */
	pdata->fwdl_en_gpio = of_get_named_gpio_flags(np, "nxp,nfc_firm", 0,NULL);
	pr_err( "pdata->fwdl_en_gpio = %d \n", pdata->fwdl_en_gpio);
	if (pdata->fwdl_en_gpio< 0) {
		pr_err( "failed to get \"huawei,nfc_firm\"\n");
		goto err;
	}

	/* nfc_ven gpio */
	pdata->ven_gpio = of_get_named_gpio_flags(np, "nxp,nfc_ven", 0,NULL);
	pr_err( "pdata->ven_gpio = %d \n", pdata->ven_gpio);
	if (pdata->ven_gpio < 0) {
		pr_err( "failed to get \"huawei,nfc_ven\"\n");
		goto err;
	}

	pr_info("%s :clk-req-gpio=%d, irq_gpio=%d, fwdl_en_gpio=%d, ven_gpio=%d \n", __func__, pdata->clk_req_gpio,
		pdata->irq_gpio, pdata->fwdl_en_gpio,pdata->ven_gpio);

err:
	return ret;
}

/* FUNCTION: pn547_gpio_request
 * DESCRIPTION: pn547_gpio_request, nfc gpio configuration
 * Parameters
 * struct device *dev:device data
 * struct pn547_i2c_platform_data *pdata:i2c data
 * RETURN VALUE
 * int: result
 */
static int pn547_gpio_request(struct device *dev,
				struct pn547_i2c_platform_data *pdata)
{
	int ret = -1;

	pr_info("%s : pn547_gpio_request enter\n", __func__);

	// NFC_INT
	ret = gpio_request(pdata->irq_gpio, "nfc_int");
	if(ret){
		goto err_irq;
	}
	ret = gpio_direction_input(pdata->irq_gpio);
	if(ret){
		goto err_fwdl_en;
	}

	//NFC_FWDL
	ret = gpio_request(pdata->fwdl_en_gpio, "nfc_wake");
	if(ret){
		goto err_fwdl_en;
	}
	ret = gpio_direction_output(pdata->fwdl_en_gpio,0);
	if(ret){
		goto err_ven;
	}

	//NFC_VEN
	ret=gpio_request(pdata->ven_gpio,"nfc_ven");
	if(ret){
		goto err_ven;
	}
	ret = gpio_direction_output(pdata->ven_gpio, 0);
	if(ret){
		goto err_clk_req;
	}

	return 0;

err_clk_req:
	gpio_free(pdata->ven_gpio);
err_ven:
	gpio_free(pdata->fwdl_en_gpio);
err_fwdl_en:
	gpio_free(pdata->irq_gpio);
err_irq:

	pr_err( "%s: gpio request err %d\n", __func__, ret);
	return ret;
}
/* FUNCTION: pn547_gpio_release
 * DESCRIPTION: pn547_gpio_release, release nfc gpio
 * Parameters
 * struct pn547_i2c_platform_data *pdata:i2c data
 * RETURN VALUE
 * none
 */
static void pn547_gpio_release(struct pn547_i2c_platform_data *pdata)
{
	gpio_free(pdata->ven_gpio);
	gpio_free(pdata->irq_gpio);
	gpio_free(pdata->fwdl_en_gpio);
}

static int pn547_pinctrl_lookup_select(struct pinctrl *pn547_pinctrl,
	struct pinctrl_state **pin_state, const char *pinctrl_name)
{
	int ret = 0;
	struct pinctrl_state *state;
	state = pinctrl_lookup_state(
			pn547_pinctrl, pinctrl_name);
	if (IS_ERR_OR_NULL(state)) {
		ret = PTR_ERR(state);
		pr_err("%s: get %s error: %d\n", __func__, pinctrl_name, ret);
		return -1;
	}

	ret = pinctrl_select_state(pn547_pinctrl, state);
	if (ret < 0) {
		pr_err("%s: select %s error: %d\n", __func__, pinctrl_name, ret);
	} else {
		*pin_state = state;
		pr_info("%s: pn547 select %s success\n", __func__, pinctrl_name);
	}
	return 0;
}

static int pn547_pinctrl_init(struct pn547_dev * nfc_dev)
{
	int ret = 0;

	nfc_dev->pn547_pinctrl = devm_pinctrl_get(nfc_dev->dev);
	if (IS_ERR_OR_NULL(nfc_dev->pn547_pinctrl)) {
		ret = PTR_ERR(nfc_dev->pn547_pinctrl);
		pr_err("%s: get pn547_pinctrl error: %d\n", __func__, ret);
		goto err_pinctrl_get;
	}

	pn547_pinctrl_lookup_select(nfc_dev->pn547_pinctrl,
		&nfc_dev->pinctrl_state_srclkenai, "srclkenai");
	pn547_pinctrl_lookup_select(nfc_dev->pn547_pinctrl,
		&nfc_dev->pinctrl_state_esespics, "esespics");
	pn547_pinctrl_lookup_select(nfc_dev->pn547_pinctrl,
		&nfc_dev->pinctrl_state_esespiclk, "esespiclk");
	pn547_pinctrl_lookup_select(nfc_dev->pn547_pinctrl,
		&nfc_dev->pinctrl_state_esespimo, "esespimo");
	pn547_pinctrl_lookup_select(nfc_dev->pn547_pinctrl,
		&nfc_dev->pinctrl_state_esespimi, "esespimi");
	pn547_pinctrl_lookup_select(nfc_dev->pn547_pinctrl,
		&nfc_dev->pinctrl_state_esepwrreq, "esepwrreq");

	devm_pinctrl_put(nfc_dev->pn547_pinctrl);
err_pinctrl_get:
	nfc_dev->pn547_pinctrl = NULL;
	return ret;
}

static int pn547_get_resource(struct pn547_dev *pdev, struct i2c_client *client)
{
	int ret = 0;
	char *nfc_clk_status = NULL;

	ret = of_property_read_string(client->dev.of_node, "clk_status", (const char **)&nfc_clk_status);
	if (ret) {
		pr_err("[%s,%d]:read clk status fail\n", __func__, __LINE__);
		return -ENODEV;
	} else if (!strcmp(nfc_clk_status, "pmu")) {
		pr_err("[%s,%d]:clock source is pmu\n", __func__, __LINE__);
		g_nfc_clk_src = NFC_CLK_SRC_PMU;
	} else if (!strcmp(nfc_clk_status, "xtal")) {
		pr_err("[%s,%d]:clock source is XTAL\n", __func__, __LINE__);
		g_nfc_clk_src = NFC_CLK_SRC_XTAL;
	} else {
		pr_err("[%s,%d]:clock source is cpu by default\n", __func__, __LINE__);
		g_nfc_clk_src = NFC_CLK_SRC_CPU;
	}

	return 0;
}

/* FUNCTION: pn547_probe
 * DESCRIPTION: pn547_probe
 * Parameters
 * struct i2c_client *client:i2c client data
 * const struct i2c_device_id *id:i2c device id
 * RETURN VALUE
 * int: result
 */
static int pn547_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int ret = 0;
	int nfc_dev_type;
	struct clk *nfc_clk = NULL;
	struct pn547_i2c_platform_data *platform_data = NULL;
	struct pn547_dev *pn547_dev = NULL;

	dev_dbg(&client->dev, "%s begin:2\n", __func__);
#ifdef CONFIG_HUAWEI_DSM
	if(!nfc_dclient){
	   nfc_dclient = dsm_register_client (&dsm_nfc);
	}
#endif

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("%s : need I2C_FUNC_I2C\n", __func__);
		return  -ENODEV;
	}

	get_nfc_wired_ese_type();

	/* create interface node */
	ret = create_sysfs_interfaces(&client->dev);
	if (ret < 0) {
		pr_err("Failed to create_sysfs_interfaces\n");
		return -ENODEV;
	}
	ret = sysfs_create_link(NULL,&client->dev.kobj,"nfc");
	if(ret < 0) {
		pr_err("Failed to sysfs_create_link\n");
		return -ENODEV;
	}
	platform_data = kzalloc(sizeof(struct pn547_i2c_platform_data),
				GFP_KERNEL);
	if (platform_data == NULL) {
		dev_err(&client->dev, "failed to allocate memory\n");
		ret = -ENOMEM;
		goto err_platform_data;
	}

	/* get gpio config */
	ret = pn547_parse_dt(&client->dev, platform_data);
	if (ret < 0) {
		dev_err(&client->dev, "failed to parse device tree: %d\n", ret);
		goto err_parse_dt;
	}
	/* config nfc gpio */
	ret = pn547_gpio_request(&client->dev, platform_data);
	if (ret) {
		dev_err(&client->dev, "failed to request gpio\n");
		goto err_gpio_request;
	}

	pn547_dev = kzalloc(sizeof(*pn547_dev), GFP_KERNEL);
	if (pn547_dev == NULL) {
		dev_err(&client->dev,
				"failed to allocate memory for module data\n");
		ret = -ENOMEM;
		goto err_exit;
	}

	ret = pn547_get_resource(pn547_dev, client);
	if (ret < 0)
		pr_err("[%s,%d]:pn547 probe get resource failed\n", __func__, __LINE__);

	client->irq = gpio_to_irq(platform_data->irq_gpio);

	pn547_dev->irq_gpio = platform_data->irq_gpio;
	pn547_dev->ven_gpio = platform_data->ven_gpio;
	pn547_dev->firm_gpio = platform_data->fwdl_en_gpio;
	pn547_dev->client = client;
	pn547_dev->dev = &client->dev;
	pn547_dev->sim_switch = CARD1_SELECT;/* sim_select = 1,UICC select */
	pn547_dev->sim_status = CARD_UNKNOWN;
	pn547_dev->enable_status = ENABLE_START;
	pn547_dev->nfc_clk = nfc_clk;
	pn547_dev->irq_wake_enabled = false;

	nfcdev = pn547_dev;
	/* notifier for supply shutdown */
	pr_err("%s:register_reboot_notifier\n", __func__);
	register_reboot_notifier(&nfc_ven_low_notifier);
	get_nfc_chip_type();
	pr_err("%s:check pn547\n", __func__);
	get_nfc_gt_boost();
	/* check if nfc chip is ok */
	ret = check_pn547(client, pn547_dev);

	if(ret < 0){
		pr_err("%s: pn547 check failed \n", __func__);
		goto err_i2c;
	}
	pn547_pinctrl_init(pn547_dev);
	gpio_set_value(pn547_dev->firm_gpio, 0);

	/* Initialise mutex and work queue */
	init_waitqueue_head(&pn547_dev->read_wq);
	mutex_init(&pn547_dev->read_mutex);
	mutex_init(&pn547_dev->irq_wake_mutex);
	spin_lock_init(&pn547_dev->irq_enabled_lock);
    wakeup_source_init(&pn547_dev->wl,"nfc_locker");
	pn547_dev->pn547_device.minor = MISC_DYNAMIC_MINOR;
	nfc_dev_type = get_dev_name_type();
	pr_info("%s get_dev_name_type = %d.\n", __func__, nfc_dev_type);
	if (nfc_dev_type == 1)
		pn547_dev->pn547_device.name = "nfc_dg";
	else
		pn547_dev->pn547_device.name = "pn544";
	pn547_dev->pn547_device.fops = &pn547_dev_fops;
	ret = misc_register(&pn547_dev->pn547_device);
	if (ret) {
		dev_err(&client->dev, "%s: misc_register err %d\n",
			__func__, ret);
		goto err_misc_register;
	}

	/* request irq.  the irq is set whenever the chip has data available
	 * for reading.  it is cleared when all data has been read.
	 */
	dev_info(&client->dev, "%s : requesting IRQ %d\n",
		__func__, client->irq);
	pn547_dev->irq_enabled = true;
	ret = request_irq(client->irq, pn547_dev_irq_handler,
			IRQF_TRIGGER_HIGH | IRQF_NO_SUSPEND | IRQF_ONESHOT,
			client->name, pn547_dev);
	if (ret) {
		dev_err(&client->dev, "request_irq failed\n");
		goto err_request_irq_failed;
	}

	pn547_disable_irq(pn547_dev);
	i2c_set_clientdata(client, pn547_dev);

#ifdef CONFIG_HUAWEI_HW_DEV_DCT
	/* detect current device successful, set the flag as present */
	set_hw_dev_flag(DEV_I2C_NFC);
	pr_info("%s set_hw_dev_flag success.\n", __func__);
#endif

	/* get and save configure name */
	get_nfc_config_name(&client->dev);
	set_nfc_single_channel(&client->dev);
	if ((nfc_get_ese_spi_bus() == 0) && (nfc_get_ese_pwr_req_gpio() == 0)) {
		nfc_write_conf_to_tee();
	}
	pr_info("%s success.\n", __func__);
	return 0;

err_request_irq_failed:
	misc_deregister(&pn547_dev->pn547_device);

err_misc_register:
	mutex_destroy(&pn547_dev->read_mutex);
	mutex_destroy(&pn547_dev->irq_wake_mutex);
	kfree(pn547_dev);
err_exit:
err_i2c:
	pn547_gpio_release(platform_data);
err_gpio_request:
	if (nfc_clk) {
		clk_put(nfc_clk);
		nfc_clk = NULL;
	}
err_parse_dt:
	kfree(platform_data);
err_platform_data:
	remove_sysfs_interfaces(&client->dev);

	dev_err(&client->dev, "%s: err %d\n", __func__, ret);
	return ret;
}

/* FUNCTION: pn547_remove
 * DESCRIPTION: pn547_remove
 * Parameters
 * struct i2c_client *client:i2c client data
 * RETURN VALUE
 * int: result
 */
static int pn547_remove(struct i2c_client *client)
{
	struct pn547_dev *pn547_dev = NULL;
	int ret = 0;
	dev_info(&client->dev, "%s ++\n", __func__);
	unregister_reboot_notifier(&nfc_ven_low_notifier);
	pn547_dev = i2c_get_clientdata(client);
	gpio_set_value(pn547_dev->ven_gpio, 0);
	free_irq(client->irq, pn547_dev);
	misc_deregister(&pn547_dev->pn547_device);
	mutex_destroy(&pn547_dev->read_mutex);
	mutex_destroy(&pn547_dev->irq_wake_mutex);
	wakeup_source_trash(&pn547_dev->wl);
	remove_sysfs_interfaces(&client->dev);
	if (pn547_dev->nfc_clk) {
		clk_put(pn547_dev->nfc_clk);
		pn547_dev->nfc_clk = NULL;
	}
	gpio_free(pn547_dev->irq_gpio);
	gpio_free(pn547_dev->ven_gpio);
	gpio_free(pn547_dev->firm_gpio);
	kfree(pn547_dev);
	if (g_is_gtboost) {
		ret = boost_5v_enable(BOOST_5V_DISABLE, BOOST_CTRL_NFC);
		if (ret)
			pr_err("%s: boost_5v disable failed\n", __func__);
		else
			pr_info("%s: boost_5v disable succeeded\n", __func__);
	}

	return 0;
}

static const struct i2c_device_id pn547_id[] = {
	{ "pn547", 0 },
	{ }
};

static struct of_device_id pn547_match_table[] = {
	{ .compatible = "mediatek,nq-pn544", },
	{ },
};

static struct i2c_driver pn547_driver = {
	.id_table	= pn547_id,
	.probe		= pn547_probe,
	.remove		= pn547_remove,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "pn547",
		.of_match_table	= pn547_match_table,
	},
};

static int __init pn547_dev_init(void)
{
	pr_info("### %s begin! \n",__func__);
	return i2c_add_driver(&pn547_driver);
}
module_init(pn547_dev_init);

static void __exit pn547_dev_exit(void)
{
	i2c_del_driver(&pn547_driver);
}
module_exit(pn547_dev_exit);

MODULE_AUTHOR("SERI");
MODULE_DESCRIPTION("NFC pn547 driver");
MODULE_LICENSE("GPL");
