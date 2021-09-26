#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_gpio.h>
#include <linux/of_platform.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/platform_device.h>
#include <linux/pm_wakeup.h>
#include "fingerprint.h"
#include <linux/arm-smccc.h>
#include <securec.h>

#include <linux/platform_data/spi-mt65xx.h>
#include <huawei_platform/touchscreen_interface/touchscreen_interface.h>

#if defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
#endif

#define OPTICAL   1
#define COMMON    0
#define NORMAL_TEMPERATURE 0

#define INIT_STATE        0
#define FINGER_DOWN_STATE 1
#define FINGER_UP_STATE   2
#define HOVER_DOWN_STATE  3
#define HOVER_UP_STATE    4

#define FP_STATUS_OPEN  1
#define FP_STATUS_CLOSE 0

#define HBM_WAIT_TIMEOUT (50 * HZ / 1000)
#define FINGER_UP_WAIT_TIMEOUT (500 * HZ / 1000)
#define BTB_DET_IRQ_DEBOUNCE 100
#define BTB_DET_IRQ_MAX_CNT  5

extern void mt_spi_disable_master_clk(struct spi_device *spidev);
extern void mt_spi_enable_master_clk(struct spi_device *spidev);

int fingerprint_gpio_reset(struct fp_data* fingerprint);

unsigned int snr_flag = 0;

extern unsigned int get_pd_charge_flag(void);
static struct fp_data *g_fp_data = NULL;
static bool g_close_tp_irq;
static bool g_wait_finger_up_flag;
static int g_tp_state;
static bool g_btb_irq_flag = false;
static uint32_t g_btb_irq_count;

static int ud_fingerprint_irq_notify(struct tp_to_udfp_data *tp_data);

struct ud_fp_ops g_ud_fp_ops = {
	.fp_irq_notify = ud_fingerprint_irq_notify,
};

#if defined (CONFIG_HUAWEI_DSM)
#include <dsm/dsm_pub.h>
static struct dsm_dev dsm_fingerprint =
{
    .name = "dsm_fingerprint",
    .device_name = "fpc",
    .ic_name = "NNN",
    .module_name = "NNN",
    .fops = NULL,
    .buff_size = 1024,
};
static struct dsm_client* fingerprint_dclient = NULL;
#endif

typedef enum
{
    SPI_CLK_DISABLE = 0,
    SPI_CLK_ENABLE,
} spi_clk_state;
void fingerprint_gpio_output_dts(struct fp_data* fingerprint, int pin, int level)
{
    if (pin == FINGERPRINT_RST_PIN)
    {
        if (level)
        { pinctrl_select_state(fingerprint->pinctrl, fingerprint->fp_rst_high); }
        else
        { pinctrl_select_state(fingerprint->pinctrl, fingerprint->fp_rst_low); }
    }
    else if (pin == FINGERPRINT_SPI_CS_PIN)
    {
        if (level)
        { pinctrl_select_state(fingerprint->pinctrl, fingerprint->fp_cs_high); }
        else
        { pinctrl_select_state(fingerprint->pinctrl, fingerprint->fp_cs_low); }
    }
    else if (pin == FINGERPRINT_SPI_MO_PIN)
    {
        if (level)
        { pinctrl_select_state(fingerprint->pinctrl, fingerprint->fp_mo_high); }
        else
        { pinctrl_select_state(fingerprint->pinctrl, fingerprint->fp_mo_low); }
    }
    else if (pin == FINGERPRINT_SPI_CK_PIN)
    {
        if (level)
        { pinctrl_select_state(fingerprint->pinctrl, fingerprint->fp_ck_high); }
        else
        { pinctrl_select_state(fingerprint->pinctrl, fingerprint->fp_ck_low); }
    }
    else if (pin == FINGERPRINT_SPI_MI_PIN)
    {
        if (level)
        { pinctrl_select_state(fingerprint->pinctrl, fingerprint->fp_mi_high); }
        else
        { pinctrl_select_state(fingerprint->pinctrl, fingerprint->fp_mi_low); }
    }
}

static ssize_t result_show(struct device* device,
                           struct device_attribute* attribute,
                           char* buffer)
{
    struct fp_data* fingerprint = dev_get_drvdata(device);
    return scnprintf(buffer, PAGE_SIZE, "%i\n", fingerprint->autotest_input);
}

static ssize_t result_store(struct device* device,
                            struct device_attribute* attribute,
                            const char* buffer, size_t count)
{
    struct fp_data* fingerprint = dev_get_drvdata(device);

    fingerprint->autotest_input = simple_strtoul(buffer, NULL, 10);
    sysfs_notify(&fingerprint->pf_dev->dev.kobj, NULL, "result");
    return count;
}

static DEVICE_ATTR(result, S_IRUSR | S_IWUSR, result_show, result_store);

/**
 * sysf node to check the interrupt status of the sensor, the interrupt
 * handler should perform sysf_notify to allow userland to poll the node.
 */
static ssize_t irq_get(struct device* device,
                       struct device_attribute* attribute,
                       char* buffer)
{
	int irq;
	struct fp_data *fingerprint = dev_get_drvdata(device);

	if (buffer == NULL || fingerprint == NULL) {
		fpc_log_err("%s, failed, the pointer is null\n", __func__);
		return -EINVAL;
	}

	if (fingerprint->use_tp_irq == USE_TP_IRQ) {
		irq = g_fp_data->tp_event;
		fpc_log_info("%s: USE_TP_IRQ, tp_event = %d\n",
			__func__, irq);
		return (ssize_t)scnprintf(buffer, PAGE_SIZE, "%i\n", irq);
	}
	irq = __gpio_get_value(fingerprint->irq_gpio);
	fpc_log_info("[fpc] irq_get : %d\n", irq);

	return (ssize_t)scnprintf(buffer, PAGE_SIZE, "%i\n", irq);
}

/**
 * writing to the irq node will just drop a printk message
 * and return success, used for latency measurement.
 */
static ssize_t irq_ack(struct device* device,
                       struct device_attribute* attribute,
                       const char* buffer, size_t count)
{
	struct tp_to_udfp_data data = {0};
	int status;
	long value = 0;

	/* 10 means decimal */
	status = strict_strtol(buffer, 10, &value);
	if (status < 0) {
		fpc_log_err("%s: strict_strtol failed!\n", __func__);
		return 0;
	}

	data.udfp_event = (int)value;
	fpc_log_info("%s: udfp_event: %d\n", __func__, data.udfp_event);

	(void)ud_fingerprint_irq_notify(&data);
	return (ssize_t)count;
}

static DEVICE_ATTR(irq, S_IRUSR | S_IWUSR, irq_get, irq_ack);

static ssize_t read_image_flag_show(struct device* device,
                                    struct device_attribute* attribute,
                                    char* buffer)
{
    struct fp_data* fingerprint = dev_get_drvdata(device);
    return scnprintf(buffer, PAGE_SIZE, "%u", (unsigned int)fingerprint->read_image_flag);
}
static ssize_t read_image_flag_store(struct device* device,
                                     struct device_attribute* attribute,
                                     const char* buffer, size_t count)
{
    struct fp_data* fingerprint = dev_get_drvdata(device);
    fingerprint->read_image_flag = simple_strtoul(buffer, NULL, 10);
    return (ssize_t)count;
}

static DEVICE_ATTR(read_image_flag, S_IRUSR | S_IWUSR, read_image_flag_show, read_image_flag_store);

static ssize_t snr_show(struct device* device,
                        struct device_attribute* attribute,
                        char* buffer)
{
    struct fp_data* fingerprint = dev_get_drvdata(device);
    return scnprintf(buffer, PAGE_SIZE, "%d", fingerprint->snr_stat);
}

static ssize_t snr_store(struct device* device,
                         struct device_attribute* attribute,
                         const char* buffer, size_t count)
{
    struct fp_data* fingerprint = dev_get_drvdata(device);
    fingerprint->snr_stat = simple_strtoul(buffer, NULL, 10);

    if (fingerprint->snr_stat)
    {
        snr_flag = 1;
    }
    else
    { snr_flag = 0; }

    fpc_log_err("snr_store snr_flag = %u\n", snr_flag);
    return count;
}

static DEVICE_ATTR(snr, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP, snr_show, snr_store);

static ssize_t nav_show(struct device* device,
                        struct device_attribute* attribute,
                        char* buffer)
{
    struct fp_data* fingerprint = dev_get_drvdata(device);

    if (NULL == fingerprint)
    {return -EINVAL;}

    return scnprintf(buffer, PAGE_SIZE, "%d", fingerprint->nav_stat);
}

static ssize_t nav_store(struct device* device,
                         struct device_attribute* attribute,
                         const char* buffer, size_t count)
{
    struct fp_data* fingerprint = dev_get_drvdata(device);

    if (NULL == fingerprint)
    {return -EINVAL;}

    fingerprint->nav_stat = simple_strtoul(buffer, NULL, 10);
    return count;
}

/*lint -save -e* */
static DEVICE_ATTR(nav, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP, nav_show, nav_store);
/*lint -restore*/

static ssize_t fingerprint_chip_info_show(struct device* device, struct device_attribute* attribute, char* buf)
{
	char sensor_id[FP_MAX_SENSOR_ID_LEN]={0};
	struct fp_data* fingerprint = NULL;

	if(NULL == device || NULL == buf)
	{
		fpc_log_err("%s failed,the pointer is null\n", __func__);
		return -EINVAL;
	}
	fingerprint = dev_get_drvdata(device);
	if (NULL == fingerprint)
	{
		fpc_log_err("%s failed,the parameters is null\n", __func__);
		return -EINVAL;
	}

	snprintf(sensor_id, FP_MAX_SENSOR_ID_LEN, "%x", fingerprint->sensor_id);
	return scnprintf(buf, FP_MAX_CHIP_INFO_LEN, "%s\n", sensor_id);

}

static DEVICE_ATTR(fingerprint_chip_info, S_IRUSR  | S_IRGRP | S_IROTH, fingerprint_chip_info_show, NULL);

static ssize_t module_id_show(struct device *device, struct device_attribute *attribute, char *buf)
{
    struct fp_data* fingerprint= dev_get_drvdata(device);
    if(NULL == buf || NULL == fingerprint)
    {
        return -EINVAL;
    }
    return scnprintf(buf, sizeof(fingerprint->module_id), "%s", fingerprint->module_id);
}
static ssize_t module_id_store(struct device* device,
                               struct device_attribute* attribute,
                               const char* buffer, size_t count)
{
    struct fp_data* fingerprint = dev_get_drvdata(device);
    if(NULL == fingerprint)
    {
        return -EINVAL;
    }
    strncpy( fingerprint->module_id,buffer ,sizeof(fingerprint->module_id)-1);
    fingerprint->module_id[sizeof(fingerprint->module_id)-1] = '\0';
    return count;
}

static DEVICE_ATTR(module_id, S_IRUSR  | S_IRGRP | S_IROTH, module_id_show, module_id_store);

static ssize_t module_id_ud_show(struct device *device,
	struct device_attribute *attribute, char *buffer)
{
	struct fp_data *fp = dev_get_drvdata(device);

	if (buffer == NULL || fp == NULL) {
		fpc_log_err("%s, failed, the pointer is null!\n", __func__);
		return -EINVAL;
	}
	return (ssize_t)scnprintf(buffer, MAX_MODULE_ID_LEN, "%s",
		fp->module_id_ud);
}

static ssize_t module_id_ud_store(struct device *device,
	struct device_attribute *attribute, const char *buffer, size_t count)
{
	struct fp_data *fp = dev_get_drvdata(device);
	int module_id_len = sizeof(fp->module_id_ud);
	errno_t ret;

	if (fp == NULL) {
		fpc_log_err("%s, fp is null!\n", __func__);
		return -EINVAL;
	}
	ret = strncpy_s(fp->module_id_ud, module_id_len, buffer,
		module_id_len - 1);
	if (ret != EOK) {
		fpc_log_err("%s, strncpy_s failed! ret:%d\n", __func__, ret);
		return -EFAULT;
	}
	fp->module_id_ud[module_id_len - 1] = '\0';
	return (ssize_t)count;
}

static DEVICE_ATTR(module_id_ud, 0660, module_id_ud_show, module_id_ud_store);

static ssize_t chip_info_show(struct device *device,
	unsigned int sensor_type, char *chip_info)
{
	unsigned int sensor_id;
	int ret;
	struct fp_data *fp = NULL;

	if (device == NULL || chip_info == NULL) {
		fpc_log_err("%s failed,the pointer is null\n", __func__);
		return -EINVAL;
	}
	fp = dev_get_drvdata(device);
	if (fp == NULL) {
		fpc_log_err("%s failed,the parameters is null\n", __func__);
		return -EINVAL;
	}

	if (sensor_type == COMMON) {
		sensor_id = fp->sensor_id;
	} else if (sensor_type == OPTICAL) {
		sensor_id = fp->sensor_id_ud;
	} else {
		fpc_log_err("%s: sensor_type = %u, not found\n", __func__,
			sensor_type);
		return -EFAULT;
	}

	ret = sprintf_s(chip_info, FP_MAX_SENSOR_ID_LEN, "%x\n", sensor_id);
	if (ret < 0) {
		fpc_log_err("%s: chip_info sprintf_s failed,ret=%d\n", __func__, ret);
		return -EFAULT;
	}

	return (ssize_t)ret;
}

static ssize_t ud_fingerprint_chip_info_show(struct device *device,
	struct device_attribute *attribute, char *buf)
{
	fpc_log_info("ud_fingerprint_chip_info_show\n");
	return chip_info_show(device, OPTICAL, buf);
}

static DEVICE_ATTR(ud_fingerprint_chip_info, 0440,
	ud_fingerprint_chip_info_show, NULL);

static ssize_t low_temperature_show(struct device *device,
	struct device_attribute *attribute, char *buffer)
{
	if (buffer == NULL)
		return -EINVAL;
	return (ssize_t)scnprintf(buffer, PAGE_SIZE, "%d", NORMAL_TEMPERATURE);
}

static ssize_t low_temperature_store(struct device *device,
	struct device_attribute *attribute, const char *buffer, size_t count)
{
	return (ssize_t)count;
}

static DEVICE_ATTR(low_temperature, 0660, low_temperature_show,
	low_temperature_store);

static struct attribute* attributes[] =
{
    &dev_attr_irq.attr,
    &dev_attr_fingerprint_chip_info.attr,
    &dev_attr_result.attr,
    &dev_attr_read_image_flag.attr,
    &dev_attr_snr.attr,
    &dev_attr_nav.attr,
    &dev_attr_module_id.attr,
	&dev_attr_module_id_ud.attr,
	&dev_attr_ud_fingerprint_chip_info.attr,
	&dev_attr_low_temperature.attr,
    NULL
};

static const struct attribute_group attribute_group =
{
    .attrs = attributes,
};

/* tp event state machine */
static void fingerdown_event_function(int *cur_state, struct fp_data *fp)
{
	if (*cur_state != INIT_STATE && *cur_state != HOVER_DOWN_STATE &&
		*cur_state != HOVER_UP_STATE)
		return;
	*cur_state = FINGER_DOWN_STATE;
	sysfs_notify(&fp->pf_dev->dev.kobj, NULL, dev_attr_irq.attr.name);
}

static void fingerup_event_function(int *cur_state, struct fp_data *fp)
{
	if (*cur_state != FINGER_DOWN_STATE)
		return;

	*cur_state = FINGER_UP_STATE;
	mutex_lock(&fp->mutex_lock_irq_switch);
	if (g_wait_finger_up_flag) {
		wake_up(&fp->wait_finger_up_queue);
		g_wait_finger_up_flag = false;
	}
	g_close_tp_irq = true;
	mutex_unlock(&fp->mutex_lock_irq_switch);
}

static int ud_fingerprint_handle_tp_msg(int udfp_event)
{
	struct fp_data *fp = g_fp_data;

	if (fp == NULL) {
		fpc_log_err("%s fp is NULL!\n", __func__);
		return -EFAULT;
	}

	fp->tp_event = udfp_event;
	switch (udfp_event) {
	case TP_EVENT_FINGER_DOWN:
		fingerdown_event_function(&g_tp_state, fp);
		break;
	case TP_EVENT_FINGER_UP:
		fingerup_event_function(&g_tp_state, fp);
		break;
	case TP_EVENT_HOVER_DOWN:
		if (g_tp_state == INIT_STATE || g_tp_state == HOVER_UP_STATE)
			g_tp_state = HOVER_DOWN_STATE;
		break;
	case TP_EVENT_HOVER_UP:
		if (g_tp_state == HOVER_DOWN_STATE)
			g_tp_state = HOVER_UP_STATE;
		break;
	default:
		break;
	}
	return FP_RETURN_SUCCESS;
}

static int ud_fingerprint_irq_notify(struct tp_to_udfp_data *tp_data)
{
	if (tp_data == NULL) {
		fpc_log_err("%s tp_data is NULL!\n", __func__);
		return -EINVAL;
	}
	fpc_log_info("tp_event = %d, g_tp_state = %d\n", tp_data->udfp_event, g_tp_state);
	if (g_close_tp_irq) {
		fpc_log_info("tp irq status is close, not handle tp event\n");
		return FP_RETURN_SUCCESS;
	}
	return ud_fingerprint_handle_tp_msg(tp_data->udfp_event);
}

static irqreturn_t fingerprint_irq_handler(int irq, void* handle)
{
    struct fp_data* fingerprint = handle;

    smp_rmb();

    if (fingerprint->wakeup_enabled )
    {
        __pm_wakeup_event(&fingerprint->ttw_wl, FPC_TTW_HOLD_TIME);
    }
    sysfs_notify(&fingerprint->pf_dev->dev.kobj, NULL, dev_attr_irq.attr.name);
    return IRQ_HANDLED;
}

void fingerprint_get_navigation_adjustvalue(const struct device* dev, struct fp_data* fp_data)
{
    struct device_node* np;
    unsigned int adjust1 = NAVIGATION_ADJUST_NOREVERSE;
    unsigned int adjust2 = NAVIGATION_ADJUST_NOTURN;

    np = of_find_compatible_node(NULL, NULL, "mediatek,mtk_finger");

    // TODO: Add property in dts
    (void)of_property_read_u32(np, "fingerprint,navigation_adjust1", &adjust1);

    if (adjust1 != NAVIGATION_ADJUST_NOREVERSE && adjust1 != NAVIGATION_ADJUST_REVERSE)
    {
        adjust1 = NAVIGATION_ADJUST_NOREVERSE;
        fpc_log_err("%s navigation_adjust1 set err only support 0 and 1.\n", __func__);
    }

    (void)of_property_read_u32(np, "fingerprint,navigation_adjust2", &adjust2);

    if (adjust2 != NAVIGATION_ADJUST_NOTURN && adjust2 != NAVIGATION_ADJUST_TURN90 &&
        adjust2 != NAVIGATION_ADJUST_TURN180 && adjust2 != NAVIGATION_ADJUST_TURN270)
    {
        adjust2 = NAVIGATION_ADJUST_NOTURN;
        fpc_log_err("%s navigation_adjust2 set err only support 0 90 180 and 270.\n", __func__);
    }

    fp_data->navigation_adjust1 = (int)adjust1;
    fp_data->navigation_adjust2 = (int)adjust2;

    fpc_log_info("%s get navigation_adjust1 = %d, navigation_adjust2 = %d.\n", __func__,
                 fp_data->navigation_adjust1, fp_data->navigation_adjust2);
    return;
}

static void fp_btb_disconnect_dmd_report(int gpio, const char *err_msg)
{
#ifdef CONFIG_HUAWEI_DSM
	if (!dsm_client_ocuppy(fingerprint_dclient)) {
		dsm_client_record(fingerprint_dclient, "%s %d", err_msg, gpio);
		dsm_client_notify(fingerprint_dclient,
			DSM_FINGERPRINT_BTB_DISCONNECT_ERROR_NO);
	}
#endif
}

static void fingerprint_get_btb_dts_data(struct device_node *np,
	struct fp_data *fingerprint)
{
	int ret;

	fingerprint->btb_det_gpio = of_get_named_gpio(np, "fingerprint,btb_det_gpio", 0);
	if (fingerprint->btb_det_gpio < 0) {
		fpc_log_err("%s read fingerprint,btb_det_gpio failed: %d\n",
			__func__, fingerprint->btb_det_gpio);
		return;
	}

	fingerprint->fp_btb_det = pinctrl_lookup_state(fingerprint->pinctrl,
		"fingerprint_fp_btb_det");
	if (IS_ERR(fingerprint->fp_btb_det)) {
		ret = PTR_ERR(fingerprint->fp_btb_det);
		fpc_log_err("fingerprint Cannot find pinctrl fp_btb_det %d\n", ret);
		return;
	}
	fpc_log_info("fingerprint find fp pinctrl fp_btb_det!\n");

	ret = pinctrl_select_state(fingerprint->pinctrl, fingerprint->fp_btb_det);
	if (ret)
		fpc_log_err("fingerprint pinctrl_select_state fp_btb_det failed %d\n", ret);

	if (gpio_get_value(fingerprint->btb_det_gpio) == FP_GPIO_HIGH_LEVEL)
		fp_btb_disconnect_dmd_report(fingerprint->btb_det_gpio,
			"irq report fingerprint btb disconnect, btb_det_gpio");
}

int fingerprint_get_dts_data(struct device* dev, struct fp_data* fingerprint)
{
    struct device_node* np = NULL;
    struct platform_device* pdev = NULL;
    int ret = 0;

	if(dev == NULL)
	{
		fpc_log_err("fingerprint_get_dts_data: dev is NULL\n");
	}
	fpc_log_err("fingerprint_get_dts_data: of_find_compatible_node1\n");
	np = of_find_compatible_node(NULL, NULL, "mediatek,mtk_finger");
	fpc_log_err("fingerprint_get_dts_data: of_find_compatible_node2\n");

	if (np == NULL) {
		fpc_log_err("fingerprint Cannot find node!\n");
		return -EFAULT;
	}

	ret = of_property_read_u32(np, "fingerprint,use_tp_irq",
		&fingerprint->use_tp_irq);
	if (ret) {
		fingerprint->use_tp_irq = USE_SELF_IRQ;
		fpc_log_err("%s: failed to get use_tp_irq\n", __func__);
	}

	ret = of_property_read_u32(np, "fingerprint,product_id",
		&fingerprint->product_id);
	if (ret)
		fpc_log_err("%s: failed to get product_id\n", __func__);
	fpc_log_info("%s get product_id = %u\n", __func__, fingerprint->product_id);

	ret = of_property_read_u32(np, "fingerprint,custom_timing_scheme",
		&fingerprint->custom_timing_scheme);
	if (ret) {
		fingerprint->custom_timing_scheme = 0;
		fpc_log_err("%s:failed get custom_timing_scheme\n", __func__);
	}
	ret = of_property_read_u32(np, "fingerprint,poweroff_scheme",
		&fingerprint->poweroff_scheme);
	if (ret)
		fpc_log_err("the property name poweroff_scheme is null\n");

	pdev = of_find_device_by_node(np);
	fpc_log_err("%s: of_find_device_by_node\n", __func__);
	if (IS_ERR(pdev)) {
		fpc_log_err("platform device is null\n");
		return PTR_ERR(pdev);
	}
	fingerprint->pinctrl = devm_pinctrl_get(&fingerprint->spi->dev);
	if (IS_ERR(fingerprint->pinctrl)) {
		ret = PTR_ERR(fingerprint->pinctrl);
		fpc_log_err("fpsensor Cannot find fp pinctrl1.\n");
		return ret;
	}

	fingerprint->fp_rst_low = pinctrl_lookup_state(fingerprint->pinctrl,
		"fingerprint_rst_low");
	if (IS_ERR(fingerprint->fp_rst_low)) {
		ret = PTR_ERR(fingerprint->fp_rst_low);
		fpc_log_err("fingerprint Cannot find fp pinctrl fp_rst_low!\n");
		return ret;
	}

	fingerprint->fp_rst_high = pinctrl_lookup_state(fingerprint->pinctrl,
		"fingerprint_rst_high");
	if (IS_ERR(fingerprint->fp_rst_high)) {
		ret = PTR_ERR(fingerprint->fp_rst_high);
		fpc_log_err("fingerprint Cannot find pinctrl fp_rst_high!\n");
		return ret;
	}
	fpc_log_info("fingerprint find fp pinctrl fp_rst_high!\n");

	if (fingerprint->use_tp_irq != USE_TP_IRQ) {
		fingerprint->eint_as_int =
			pinctrl_lookup_state(fingerprint->pinctrl,
			"fingerprint_eint_as_int");
		if (IS_ERR(fingerprint->eint_as_int)) {
			ret = PTR_ERR(fingerprint->eint_as_int);
			fpc_log_err("fp Cannot find pinctrl eint_as_int!\n");
			return ret;
		}
		fpc_log_info("fingerprint find fp pinctrl eint_as_int!\n");
	}

	fingerprint->fp_cs_low  = pinctrl_lookup_state(fingerprint->pinctrl,
		"fingerprint_spi_cs_low");
	if (IS_ERR(fingerprint->fp_cs_low)) {
		ret = PTR_ERR(fingerprint->fp_cs_low);
		fpc_log_err("fingerprint Cannot find fp pinctrl fp_cs_low!\n");
		return ret;
	}
	fpc_log_info("fingerprint find fp pinctrl fp_cs_low!\n");

	fingerprint->fp_cs_high = pinctrl_lookup_state(fingerprint->pinctrl,
		"fingerprint_spi_cs_high");
	if (IS_ERR(fingerprint->fp_cs_high)) {
		ret = PTR_ERR(fingerprint->fp_cs_high);
		fpc_log_err("fingerprint Cannot find fp pinctrl fp_cs_high!\n");
		return ret;
	}
	fpc_log_info("fingerprint find fp pinctrl fp_cs_high!\n");

	fingerprint->fp_mo_high = pinctrl_lookup_state(fingerprint->pinctrl,
		"fingerprint_spi_mosi_high");
	if (IS_ERR(fingerprint->fp_mo_high)) {
		ret = PTR_ERR(fingerprint->fp_mo_high);
		fpc_log_err("fingerprint Cannot find fp pinctrl fp_mo_high!\n");
		return ret;
	}
	fpc_log_info("fingerprint find fp pinctrl fp_mo_high!\n");

	fingerprint->fp_mo_low = pinctrl_lookup_state(fingerprint->pinctrl,
		"fingerprint_spi_mosi_low");
	if (IS_ERR(fingerprint->fp_mo_low)) {
		ret = PTR_ERR(fingerprint->fp_mo_low);
		fpc_log_err("fingerprint Cannot find fp pinctrl fp_mo_low!\n");
		return ret;
	}
	fpc_log_info("fingerprint find fp pinctrl fp_mo_low!\n");

	fingerprint->fp_mi_high = pinctrl_lookup_state(fingerprint->pinctrl,
		"fingerprint_spi_miso_high");
	if (IS_ERR(fingerprint->fp_mi_high)) {
		ret = PTR_ERR(fingerprint->fp_mi_high);
		fpc_log_err("fingerprint Cannot find fp pinctrl fp_mi_high!\n");
		return ret;
	}
	fpc_log_info("fingerprint find fp pinctrl fp_mi_high!\n");

	fingerprint->fp_mi_low = pinctrl_lookup_state(fingerprint->pinctrl,
		"fingerprint_spi_miso_low");
	if (IS_ERR(fingerprint->fp_mi_low)) {
		ret = PTR_ERR(fingerprint->fp_mi_low);
		fpc_log_err("fingerprint Cannot find fp pinctrl fp_mi_low!\n");
		return ret;
	}
	fpc_log_info("fingerprint find fp pinctrl fp_mi_low!\n");

	fingerprint->fp_ck_high = pinctrl_lookup_state(fingerprint->pinctrl,
		"fingerprint_spi_mclk_high");
	if (IS_ERR(fingerprint->fp_ck_high)) {
		ret = PTR_ERR(fingerprint->fp_ck_high);
		fpc_log_err("fingerprint Cannot find fp pinctrl fp_ck_high!\n");
		return ret;
	}
	fpc_log_info("fingerprint find fp pinctrl fp_ck_high!\n");

	fingerprint->fp_ck_low = pinctrl_lookup_state(fingerprint->pinctrl,
		"fingerprint_spi_mclk_low");
	if (IS_ERR(fingerprint->fp_ck_low)) {
		ret = PTR_ERR(fingerprint->fp_ck_low);
		fpc_log_err("fingerprint Cannot find fp pinctrl fp_ck_low!\n");
		return ret;
	}
	fpc_log_info("fingerprint find fp pinctrl fp_ck_low!\n");

	fingerprint_get_btb_dts_data(np, fingerprint);

	fingerprint_gpio_output_dts(fingerprint, FINGERPRINT_SPI_MO_PIN, 0);
	fingerprint_gpio_output_dts(fingerprint, FINGERPRINT_SPI_MI_PIN, 0);
	fingerprint_gpio_output_dts(fingerprint, FINGERPRINT_SPI_CK_PIN, 0);
	fingerprint_gpio_output_dts(fingerprint, FINGERPRINT_SPI_CS_PIN, 0);
	fingerprint_gpio_output_dts(fingerprint, FINGERPRINT_RST_PIN, 0);

	return FP_RETURN_SUCCESS;
}

static void fingerprint_custom_timing_scheme_one(struct fp_data *fingerprint)
{
	usleep_range(1000, 1000); /* delay 1ms */
	fingerprint_gpio_output_dts(fingerprint, FINGERPRINT_RST_PIN,
		FP_GPIO_HIGH_LEVEL);
	fingerprint_gpio_output_dts(fingerprint, FINGERPRINT_SPI_CS_PIN,
		FP_GPIO_HIGH_LEVEL);
}

static void fingerprint_custom_timing_scheme_two(struct fp_data *fingerprint)
{
	msleep(10); /* delay 10ms */
	fingerprint_gpio_output_dts(fingerprint, FINGERPRINT_RST_PIN,
		FP_GPIO_HIGH_LEVEL);
	fingerprint_gpio_output_dts(fingerprint, FINGERPRINT_SPI_CS_PIN,
		FP_GPIO_HIGH_LEVEL);
}
/*
 * some device need spacial timing scheme3:
 * first power on the sensor that open loadswtich(gpio), then delay 10ms
 * power on cs, and then delay 600us, power on rst
 */
static void fingerprint_custom_timing_scheme_three(struct fp_data *fingerprint)
{
	msleep(10); /* delay 10ms */
	fingerprint_gpio_output_dts(fingerprint, FINGERPRINT_SPI_CS_PIN,
		FP_GPIO_HIGH_LEVEL);
	udelay(600); /* delay 600us */
	fingerprint_gpio_output_dts(fingerprint, FINGERPRINT_RST_PIN,
		FP_GPIO_HIGH_LEVEL);
}

static void fingerprint_custom_timing_scheme_four(struct fp_data *fingerprint)
{
	fingerprint_gpio_output_dts(fingerprint, FINGERPRINT_RST_PIN,
		FP_GPIO_HIGH_LEVEL);
	fingerprint_gpio_output_dts(fingerprint, FINGERPRINT_SPI_CS_PIN,
		FP_GPIO_HIGH_LEVEL);
}

static void fingerprint_custom_timing(struct fp_data *fingerprint)
{
	fpc_log_info("%s: fingerprint custom_timing_scheme : %d\n", __func__,
		fingerprint->custom_timing_scheme);

	switch (fingerprint->custom_timing_scheme) {
	case FP_CUSTOM_TIMING_SCHEME_ONE:
		fingerprint_custom_timing_scheme_one(fingerprint);
		break;
	case FP_CUSTOM_TIMING_SCHEME_TWO:
		fingerprint_custom_timing_scheme_two(fingerprint);
		break;
	case FP_CUSTOM_TIMING_SCHEME_THREE:
		fingerprint_custom_timing_scheme_three(fingerprint);
		break;
	case FP_CUSTOM_TIMING_SCHEME_FOUR:
		fingerprint_custom_timing_scheme_four(fingerprint);
		break;
	default:
		fpc_log_err("%s timing_scheme %d, match default timing\n",
			__func__, fingerprint->custom_timing_scheme);
		fingerprint_gpio_output_dts(fingerprint,
			FINGERPRINT_RST_PIN, FP_GPIO_HIGH_LEVEL);
		fingerprint_gpio_output_dts(fingerprint,
			FINGERPRINT_SPI_CS_PIN, FP_GPIO_HIGH_LEVEL);
	}
}

int fingerprint_gpio_reset(struct fp_data* fingerprint)
{
    int error = 0;
    int counter = FP_RESET_RETRIES;

    while (counter)
    {
        counter--;

        fingerprint_gpio_output_dts(fingerprint, FINGERPRINT_RST_PIN, 1);
        mdelay(10);
        fingerprint_gpio_output_dts(fingerprint, FINGERPRINT_RST_PIN, 0);
        mdelay(10);
        fingerprint_gpio_output_dts(fingerprint, FINGERPRINT_RST_PIN, 1);
        udelay(FP_RESET_HIGH2_US);
        mdelay(10);
    }

    fpc_log_info("Exit!\n");
    return error;
}

static int fingerprint_irq_init(struct fp_data* fingerprint)
{
    int error = 0;
    struct device_node* node;

    node = of_find_compatible_node(NULL, NULL, "mediatek,mtk_finger");
    if ( node)
    {
		fingerprint->irq_gpio = of_get_named_gpio(node, "fingerprint_eint_gpio",0);
		if(fingerprint->irq_gpio<0)
		{
			fpc_log_err("[fpc] %s read fingerprint,eint_gpio faile\n",__func__);
		}
		
       fingerprint->irq = irq_of_parse_and_map(node, 0);  // get irq number
        if (fingerprint->irq < 0)
        {
			fpc_log_err("[fpc] fingerprint irq_of_parse_and_map fail!!\n");
			return -EINVAL;
        }

        fpc_log_info(" [fpc] fingerprint->irq= %d,fingerprint>irq_gpio = %d\n", fingerprint->irq, fingerprint->irq_gpio);
    }
    else
    {
        fpc_log_err("[fpc] fingerprint null irq node!!\n");
        return -EINVAL;
    }
    return error;
}

static irqreturn_t fingerprint_btb_det_irq_handler(
	int irq, void *handle)
{
	struct fp_data *dev = (struct fp_data *)handle;

	disable_irq_nosync(irq);

	fpc_log_info("%s: Interrupt occured\n", __func__);
	schedule_delayed_work(&dev->dwork,
		msecs_to_jiffies(BTB_DET_IRQ_DEBOUNCE));
	g_btb_irq_count++;

	return IRQ_HANDLED;
}

static void fingerprint_btb_det_work_func(struct work_struct *work)
{
	struct fp_data *fingerprint =
		container_of(work, struct fp_data, dwork.work);
	int gpio_value = gpio_get_value(fingerprint->btb_det_gpio);

	fpc_log_info("btb_det_gpio value: %d", gpio_value);
	if (gpio_value == FP_GPIO_HIGH_LEVEL)
		fp_btb_disconnect_dmd_report(fingerprint->btb_det_gpio,
			"irq report fingerprint btb disconnect, btb_det_gpio");

	if (g_btb_irq_count < BTB_DET_IRQ_MAX_CNT)
		enable_irq(fingerprint->btb_det_irq);
}

static int fingerprint_btb_det_irq_init(struct fp_data *fingerprint)
{
	int ret;

	if (fingerprint->btb_det_gpio < 0) {
		fpc_log_err("%s read fingerprint,btb_det_gpio failed\n", __func__);
		return -EINVAL;
	}

	ret = gpio_request(fingerprint->btb_det_gpio, "fingerprint,btb_det_gpio");
	if (ret) {
		fpc_log_err("%s gpio_request failed %d\n", __func__, ret);
		return ret;
	}

	/* request irq */
	fingerprint->btb_det_irq = gpio_to_irq(fingerprint->btb_det_gpio);
	if (fingerprint->btb_det_irq < 0) {
		fpc_log_err("%s gpio_to_irq fail, %d\n", __func__, fingerprint->btb_det_irq);
		gpio_free(fingerprint->btb_det_gpio);
		return -EINVAL;
	}
	fpc_log_info("%s btb_det_irq = %d, btb_det_gpio = %d\n", __func__,
		fingerprint->btb_det_irq, fingerprint->btb_det_gpio);

	INIT_DELAYED_WORK(&fingerprint->dwork, fingerprint_btb_det_work_func);

	ret = request_threaded_irq(fingerprint->btb_det_irq, NULL,
		fingerprint_btb_det_irq_handler,
		IRQF_TRIGGER_RISING | IRQF_ONESHOT, "fingerprint", fingerprint);
	if (ret) {
		cancel_delayed_work_sync(&fingerprint->dwork);
		fpc_log_err("failed to request btb_det_irq %d\n",
			fingerprint->btb_det_irq);
		gpio_free(fingerprint->btb_det_gpio);
		return ret;
	}
	enable_irq_wake(fingerprint->btb_det_irq);
	g_btb_irq_flag = true; /* enable remove func */

	return ret;
}

static void fingerprint_btb_det_remove(struct fp_data *fingerprint)
{
	if (g_btb_irq_flag) {
		cancel_delayed_work_sync(&fingerprint->dwork);
		free_irq(fingerprint->btb_det_irq, fingerprint);
		gpio_free(fingerprint->btb_det_gpio);
	}
}

static int fingerprint_key_remap_reverse(int key)
{
    switch (key)
    {
        case EVENT_LEFT:
            key = EVENT_RIGHT;
            break;

        case EVENT_RIGHT:
            key = EVENT_LEFT;
            break;

        default:
            break;
    }

    return key;
}

static int fingerprint_key_remap_turn90(int key)
{
    switch (key)
    {
        case EVENT_LEFT:
            key = EVENT_UP;
            break;

        case EVENT_RIGHT:
            key = EVENT_DOWN;
            break;

        case EVENT_UP:
            key = EVENT_RIGHT;
            break;

        case EVENT_DOWN:
            key = EVENT_LEFT;
            break;

        default:
            break;
    }

    return key;
}

static int fingerprint_key_remap_turn180(int key)
{
    switch (key)
    {
        case EVENT_LEFT:
            key = EVENT_RIGHT;
            break;

        case EVENT_RIGHT:
            key = EVENT_LEFT;
            break;

        case EVENT_UP:
            key = EVENT_DOWN;
            break;

        case EVENT_DOWN:
            key = EVENT_UP;
            break;

        default:
            break;
    }

    return key;
}

static int fingerprint_key_remap_turn270(int key)
{
    switch (key)
    {
        case EVENT_LEFT:
            key = EVENT_DOWN;
            break;

        case EVENT_RIGHT:
            key = EVENT_UP;
            break;

        case EVENT_UP:
            key = EVENT_LEFT;
            break;

        case EVENT_DOWN:
            key = EVENT_RIGHT;
            break;

        default:
            break;
    }

    return key;
}

static int fingerprint_key_remap(const struct fp_data* fingerprint, int key)
{
    if (key != EVENT_RIGHT && key != EVENT_LEFT && key != EVENT_UP && key != EVENT_DOWN)
    {
        return key;
    }

    if (fingerprint->navigation_adjust1 == NAVIGATION_ADJUST_REVERSE)
    {
        key = fingerprint_key_remap_reverse(key);
    }

    switch (fingerprint->navigation_adjust2)
    {
        case NAVIGATION_ADJUST_TURN90:
            key = fingerprint_key_remap_turn90(key);
            break;

        case NAVIGATION_ADJUST_TURN180:
            key = fingerprint_key_remap_turn180(key);
            break;

        case NAVIGATION_ADJUST_TURN270:
            key = fingerprint_key_remap_turn270(key);
            break;

        default:
            break;
    }

    return key;
}

static void fingerprint_input_report(struct fp_data* fingerprint, int key)
{
    key = fingerprint_key_remap(fingerprint, key);
    input_report_key(fingerprint->input_dev, key, 1);
    input_sync(fingerprint->input_dev);
    input_report_key(fingerprint->input_dev, key, 0);
    input_sync(fingerprint->input_dev);
}

static int fingerprint_open(struct inode* inode, struct file* file)
{
    struct fp_data* fingerprint;
    fingerprint = container_of(inode->i_cdev, struct fp_data, cdev);
    file->private_data = fingerprint;
    return 0;
}

static int fingerprint_get_irq_status(struct fp_data* fingerprint)
{
    int status = 0;
    status = __gpio_get_value(fingerprint->irq_gpio);
    return status;
}

static void fingerprint_spi_clk_switch(struct fp_data* fingerprint, spi_clk_state ctrl)
{
    if (NULL == fingerprint || NULL == fingerprint->spi)
    {
        fpc_log_err("the fingerprint or fingerpint->spi is NULL, clk control failed!\n");
        return;
    }

    mutex_lock(&fingerprint->mutex_lock_clk);
    if (SPI_CLK_DISABLE == ctrl)
    {
        if(fingerprint->spi_clk_counter > 0){
            fingerprint->spi_clk_counter--;
        }

        if(fingerprint->spi_clk_counter == 0){
            mt_spi_disable_master_clk(fingerprint->spi);
        }
        else {
            fpc_log_info("the disable clk is not match, the spi_clk_counter = %d\n", fingerprint->spi_clk_counter);
		}
    }
    else
    {
        if(fingerprint->spi_clk_counter == 0){
            mt_spi_enable_master_clk(fingerprint->spi);
        }
        else {
            fpc_log_info("the enable clk is not match, the spi_clk_counter = %d\n", fingerprint->spi_clk_counter);
        }
        fingerprint->spi_clk_counter++;
    }
    mutex_unlock(&fingerprint->mutex_lock_clk);
}

static void fingerprint_poweron_open_ldo(struct fp_data *fingerprint)
{
	int ret;
	int max_cnt = 100; /* max times that try to open the power */

	if ((fingerprint->avdd == NULL) || IS_ERR(fingerprint->avdd)) {
		fpc_log_err("%s:can't get fp_avdd regulator\n", __func__);
		return;
	}

	do {
		fpc_log_info("%s regulator flag:%d\n",
			__func__, regulator_is_enabled(fingerprint->avdd));
		if (!regulator_is_enabled(fingerprint->avdd)) {
			ret = regulator_enable(fingerprint->avdd);
			if (ret != 0)
				fpc_log_err("%s:regulator_enable fail, ret:%d\n",
					__func__, ret);
		}

		/* break the process when the ldo regulator is open */
		if (regulator_is_enabled(fingerprint->avdd)) {
			fpc_log_info("regulator is open and break\n");
			break;
		}
		max_cnt--;
	} while (max_cnt > 0);
}

static void fingerprint_poweron(struct fp_data *fingerprint)
{
	fpc_log_info("start to fingerprint_poweron\n");

	switch (fingerprint->poweroff_scheme) {
	case FP_POWEROFF_SCHEME_ONE:
		break;
	case FP_POWEROFF_SCHEME_TWO:
		fingerprint_poweron_open_ldo(fingerprint);
		break;
	default:
		break;
	}
}

static void fingerprint_poweroff_close_ldo(struct fp_data *fingerprint)
{
	int ret;
	int max_cnt = 100; /* max times that try to close the power */

	if ((fingerprint->avdd == NULL) || IS_ERR(fingerprint->avdd)) {
		fpc_log_err("%s:can't get fp_avdd regulator\n", __func__);
		return;
	}
	/*
	 * the power may be shared with other modules,
	 * so now close the power maybe more than one times
	 */
	do {
		fpc_log_info("%s regulator flag:%d\n",
			__func__, regulator_is_enabled(fingerprint->avdd));
		if (regulator_is_enabled(fingerprint->avdd)) {
			ret = regulator_disable(fingerprint->avdd);
			if (ret != 0)
				fpc_log_err("%s:regulator_disable fail, ret:%d\n",
					__func__, ret);
		}

		/* break the process when the ldo regulator is close */
		if (!regulator_is_enabled(fingerprint->avdd)) {
			fpc_log_info("regulator is close and break\n");
			break;
		}
		max_cnt--;
	} while (max_cnt > 0);
}

static void fingerprint_poweroff(struct fp_data *fingerprint)
{
	fpc_log_info("start to fingerprint_poweroff\n");

	switch (fingerprint->poweroff_scheme) {
	case FP_POWEROFF_SCHEME_ONE:
		break;
	case FP_POWEROFF_SCHEME_TWO:
		fingerprint_poweroff_close_ldo(fingerprint);
		break;
	default:
		break;
	}
}

static void fingerprint_poweroff_pd_charge(struct fp_data *fingerprint)
{
	fpc_log_info("start to fingerprint_poweroff_pd_charge\n");

	switch (fingerprint->poweroff_scheme) {
	case FP_POWEROFF_SCHEME_ONE:
	case FP_POWEROFF_SCHEME_TWO:
		fingerprint_poweroff_close_ldo(fingerprint);
		break;
	default:
		break;
	}
}

static int fingerprint_extern_ldo_proc(struct device *dev, struct fp_data *fingerprint)
{
	int ret;
	fpc_log_info("fingerprint_extern_ldo_proc: enter\n");
	if (dev == NULL) {
		fpc_log_err("fingerprint_extern_ldo_proc: dev is NULL\n");
		return -1;
	}
	fingerprint->avdd = devm_regulator_get(dev, "fp_vdd");
	if (IS_ERR(fingerprint->avdd)) {
		ret = PTR_ERR(fingerprint->avdd);
		fpc_log_err("can't get fp_vdd regulator: %d\n", ret);
		return -EINVAL;
	}
	ret = regulator_enable(fingerprint->avdd);
	if (ret)
		fpc_log_err("can't enable fp_vdd: %d\n", ret);

	fingerprint->vdd = devm_regulator_get(dev, "fp_vddio");
	if (IS_ERR(fingerprint->vdd)) {
		ret = PTR_ERR(fingerprint->vdd);
		fpc_log_err("can't get fp_vddio regulator: %d\n", ret);
		return -EINVAL;
	}
	ret = regulator_enable(fingerprint->vdd);
	if (ret)
		fpc_log_err("can't enable fp_vddio: %d\n", ret);

	if (get_pd_charge_flag())
		fingerprint_poweroff_pd_charge(fingerprint);

	return ret;
}

static void fp_ioc_enable_irq(struct fp_data *fp)
{
	fpc_log_info("fingerprint_ioctl FP_IOC_CMD_ENABLE_IRQ\n");
	if (fp->use_tp_irq == USE_TP_IRQ) {
		g_wait_finger_up_flag = false;
		fp->tp_event = TP_EVENT_FINGER_UP;
		g_tp_state = INIT_STATE;
		mutex_lock(&fp->mutex_lock_irq_switch);
		g_close_tp_irq = false;
		mutex_unlock(&fp->mutex_lock_irq_switch);
		return;
	}
	if (fp->irq_num == 0) {
		enable_irq(fp->irq);
		fp->irq_num = 1;
	}
}

static void fp_ioc_disable_irq(struct fp_data *fp)
{
	fpc_log_info("fingerprint_ioctl FP_IOC_CMD_DISABLE_IRQ\n");
	if (fp->use_tp_irq == USE_TP_IRQ)
		return;
	if (fp->irq_num == 1) {
		disable_irq(fp->irq);
		fp->irq_num = 0;
	}
}

static int fp_ioc_send_sensorid_ud(struct fp_data *fp, const void __user *argp)
{
	unsigned int sensor_id = 0;

	if (copy_from_user(&sensor_id, argp, sizeof(sensor_id))) {
		fpc_log_err("%s copy_from_user failed\n", __func__);
		return -EFAULT;
	}

	fp->sensor_id_ud = sensor_id;
	return FP_RETURN_SUCCESS;
}

void ud_fp_on_hbm_completed(void)
{
	struct fp_data *fp = g_fp_data;

	if (fp == NULL) {
		fpc_log_err("%s fp is null\n", __func__);
		return;
	}
	fpc_log_info("%s notify\n", __func__);
	fp->hbm_status = HBM_ON;
	wake_up(&fp->hbm_queue);
}

void fp_set_lcd_charge_time(int time)
{
	struct fp_data *fp = g_fp_data;

	if (fp == NULL) {
		fpc_log_err("%s fp is null\n", __func__);
		return;
	}

	fp->fingerprint_bigdata.lcd_charge_time = time;
}

void fp_set_lcd_light_on_time(int time)
{
	struct fp_data *fp = g_fp_data;

	if (fp == NULL) {
		fpc_log_err("%s fp is null\n", __func__);
		return;
	}

	fp->fingerprint_bigdata.lcd_on_time = time;
}

void fp_set_cpu_wake_up_time(int time)
{
	struct fp_data *fp = g_fp_data;

	if (fp == NULL) {
		fpc_log_err("%s fp is null\n", __func__);
		return;
	}

	fp->fingerprint_bigdata.cpu_wakeup_time = time;
}

static int fp_ioc_check_hbm_status(struct fp_data *fp)
{
	int ret = FP_RETURN_SUCCESS;

	if (fp->hbm_status == HBM_ON) {
		fpc_log_info("fp_ioc_check_hbm_status ok\n");
		return ret;
	}
	if (wait_event_timeout(fp->hbm_queue,
		fp->hbm_status == HBM_ON, HBM_WAIT_TIMEOUT) <= 0)
		ret = -EFAULT;
	return ret;
}

static int fp_ioc_get_bigdata(struct fp_data *fp, void __user *argp)
{
	int ret;

	ret = copy_to_user(argp, &fp->fingerprint_bigdata,
		sizeof(fp->fingerprint_bigdata));
	if (ret) {
		fpc_log_err("%s copy_to_user failed, ret=%d\n", __func__, ret);
		return -EFAULT;
	}

	return FP_RETURN_SUCCESS;
}

/* active te single when using fingeprint , fix fingeprint blink question. */
static int fp_ioc_update_te(void)
{
	return FP_RETURN_SUCCESS;
}

static int fp_config_wait_fp_up(void)
{
	int ret = FP_RETURN_SUCCESS;

	if (g_fp_data->tp_event == TP_EVENT_FINGER_UP) {
		fpc_log_info("finger is already up\n");
		return ret;
	}
	g_wait_finger_up_flag = true;
	if (wait_event_timeout(g_fp_data->wait_finger_up_queue,
		g_fp_data->tp_event == TP_EVENT_FINGER_UP, FINGER_UP_WAIT_TIMEOUT) <= 0)
			ret = -EFAULT; // the function times out and is not an exception
	return ret;
}

static int fp_get_finger_status(void __user *argp)
{
	int ret = copy_to_user(argp, &g_tp_state, sizeof(g_tp_state));

	if (ret) {
		fpc_log_err("copy_to_user failed, ret=%d\n", ret);
		return -EFAULT;
	}

	return FP_RETURN_SUCCESS;
}

static long fingerprint_base_ioctl(struct file* file, unsigned int cmd, unsigned long arg)
{

    int error = 0;

    struct fp_data* fingerprint;
    void __user *argp = (void __user *)(uintptr_t)arg;
    int key;
    int status;
    unsigned int sensor_id;

	if (file == NULL) {
		fpc_log_err("%s,file is null!\n", __func__);
		return -EINVAL;
	}
	fingerprint = (struct fp_data*)file->private_data;
	if (fingerprint == NULL) {
		fpc_log_err("%s,fingerprint is null!\n", __func__);
		return -EINVAL;
	}

    if (_IOC_TYPE(cmd) != FP_IOC_MAGIC)
    { return -ENOTTY; }

	switch (cmd) {
	case FP_IOC_CMD_ENABLE_IRQ:
		fp_ioc_enable_irq(fingerprint);
		break;
	case FP_IOC_CMD_DISABLE_IRQ:
		fp_ioc_disable_irq(fingerprint);
		break;
	case FP_IOC_CMD_SEND_UEVENT:
		if (copy_from_user(&key, argp, sizeof(key))) {
			fpc_log_err("copy_from_user failed");
			return -EFAULT;
		}
		fingerprint_input_report(fingerprint, key);
		break;
	case FP_IOC_CMD_GET_IRQ_STATUS:
		status = fingerprint_get_irq_status(fingerprint);
		error = copy_to_user(argp, &status, sizeof(status));
		if (error) {
			fpc_log_err("copy_to_user failed, error = %d", error);
			return -EFAULT;
		}
		break;
	case FP_IOC_CMD_SET_WAKELOCK_STATUS:
		if (copy_from_user(&key, argp, sizeof(key))) {
			fpc_log_err("copy_from_user failed");
			return -EFAULT;
		}
		if (key == 1) {
			fingerprint->wakeup_enabled = true;
		}
		else {
			fingerprint->wakeup_enabled = false;
		}
		break;
	case FP_IOC_CMD_SEND_SENSORID:
		if (copy_from_user(&sensor_id, argp, sizeof(sensor_id))) {
			fpc_log_err("copy_from_user failed\n");
			return -EFAULT;
		}
		fingerprint->sensor_id = sensor_id;
		fpc_log_info("sensor_id = %x\n", sensor_id);
		break;
	case FP_IOC_CMD_SET_IPC_WAKELOCKS:
		fpc_log_info("MTK do not support the CMD 7\n");
		break;
	// TODO: add reset IOCTL
	case FP_IOC_CMD_SET_POWEROFF:
		fpc_log_info("%s FP_IOC_CMD_SET_POWEROFF\n", __func__);
		fingerprint_poweroff(fingerprint);
		break;
	case FP_IOC_CMD_SET_POWERON:
		fpc_log_info("%s FP_IOC_CMD_SET_POWERON\n", __func__);
		fingerprint_poweron(fingerprint);
		break;
	case FP_IOC_CMD_ENABLE_SPI_CLK:
		fingerprint_spi_clk_switch(fingerprint, SPI_CLK_ENABLE);
		break;
	case FP_IOC_CMD_DISABLE_SPI_CLK:
		fingerprint_spi_clk_switch(fingerprint, SPI_CLK_DISABLE);
		break;
	case FP_IOC_CMD_SEND_SENSORID_UD:
		error = fp_ioc_send_sensorid_ud(fingerprint, argp);
		break;
	case FP_IOC_CMD_CHECK_HBM_STATUS:
		error = fp_ioc_check_hbm_status(fingerprint);
		break;
	case FP_IOC_CMD_RESET_HBM_STATUS:
		fingerprint->hbm_status = HBM_NONE;
		break;
	case FP_IOC_CMD_GET_BIGDATA:
		error = fp_ioc_get_bigdata(fingerprint, argp);
		break;
	case FP_IOC_CMD_NOTIFY_DISPLAY_FP_DOWN_UD:
		error = fp_ioc_update_te();
		break;
	case FP_IOC_CMD_WAIT_FINGER_UP:
		error = fp_config_wait_fp_up();
		break;
	case FP_IOC_CMD_IDENTIFY_EXIT:
		fpc_log_info("%s: FP_IOC_CMD_IDENTIFY_EXIT\n", __func__);
		g_fp_data->tp_event = TP_EVENT_FINGER_UP;
		break;
	case FP_IOC_CMD_GET_FINGER_STATUS:
		error = fp_get_finger_status(argp);
		break;
	default:
		fpc_log_err("error = -EFAULT\n");
		error = -EFAULT;
		break;
	}
	return error;
}

static long fingerprint_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    return fingerprint_base_ioctl(file, cmd, arg);
}

static long fingerprint_compat_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    return fingerprint_ioctl(file, cmd, arg);
}

static int fingerprint_release(struct inode* inode, struct file* file)
{
    fpc_log_info("Enter!\n");
    return 0;
}

static ssize_t fpsensor_read(struct file* filp, char __user* buf, size_t count, loff_t* f_pos)
{
    fpc_log_err("kp Not support read opertion in TEE version\n");
    return -EFAULT;
}

static const struct file_operations fingerprint_fops =
{
    .owner			= THIS_MODULE,
    .open			= fingerprint_open,
    .release		= fingerprint_release,
    .unlocked_ioctl	= fingerprint_ioctl,
#ifdef CONFIG_COMPAT
    .compat_ioctl   = fingerprint_compat_ioctl,
#endif
    .read =        fpsensor_read,
};

#if defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block* self, unsigned long event, void* data)
{
    struct fb_event* evdata = data;
    int* blank;
    struct fp_data* fingerprint = container_of(self, struct fp_data, fb_notify);

    if (evdata && evdata->data && fingerprint)
    {
        if (event == FB_EVENT_BLANK)
        {
            blank = evdata->data;

            if (*blank == FB_BLANK_UNBLANK)
            { atomic_set(&fingerprint->state, fp_LCD_UNBLANK); }
            else if (*blank == FB_BLANK_POWERDOWN)
            { atomic_set(&fingerprint->state, fp_LCD_POWEROFF); }
        }
    }

    return 0;
}
#endif

static void write_conf_to_tee(struct spi_device *spi, struct fp_data *fingerprint)
{
	struct arm_smccc_res res;
	uint32_t x1;
	uint32_t x2;
	uint16_t product_id = (uint16_t)fingerprint->product_id;
	uint8_t sensor_type = 0; // default
	uint8_t spi_bus = (uint8_t)spi->controller->bus_num;

	fpc_log_info("%s: spi_bus = %u\n", __func__, spi_bus);
	x1 = sensor_type;
	x1 = x1 | (spi_bus << OFFSET_8);
	x1 = x1 | (FP_CHECK_NUM << OFFSET_16);

	x2 = product_id;
	x2 = x2 | (FP_CHECK_NUM << OFFSET_16);

	// x1,x2,0,0 --> x1,x2,x3,x4
	arm_smccc_1_1_smc(
		MTK_SIP_KERNEL_FP_CONF_TO_TEE_ADDR_AARCH64, x1, x2, 0, 0, &res);
	fpc_log_info("%s: x1:0x%x x2:0x%x\n", __func__, x1, x2);
}

static int fingerprint_probe(struct spi_device* spi)
{
	struct device* dev = NULL;
	struct fp_data* fingerprint = NULL;
    int error = 0;
	int ret = 0;

	if(NULL ==spi)
	{
		fpc_log_err("spi is null\n");
	    return -EIO;
	}
    //int irqf;
	printk(KERN_ERR "%s: enter\n", __func__);
	dev = &spi->dev;
	if(dev == NULL)
	{
		printk(KERN_ERR "%s: FINGER dev is null\n", __func__);
	}

    fingerprint = devm_kzalloc(dev, sizeof(*fingerprint), GFP_KERNEL);
    if (!fingerprint)
    {
        fpc_log_err("failed to allocate memory for struct fp_data\n");
        error = -ENOMEM;
        goto exit;
    }

#if defined (CONFIG_HUAWEI_DSM)
    if (!fingerprint_dclient)
    {
	fpc_log_err("fingerprint_dclient error\n");
	fingerprint_dclient = dsm_register_client(&dsm_fingerprint);
    }
#endif

    fpc_log_info("fingerprint driver for Android P\n");
    fingerprint->dev = dev;
    dev_set_drvdata(dev, fingerprint);
    fingerprint->spi = spi;
    fingerprint->spi->mode = SPI_MODE_0;
    fingerprint->spi->bits_per_word = 8;
    fingerprint->spi->chip_select = 0;

	ret =  fingerprint_get_dts_data(&spi->dev, fingerprint);
	if (ret) {
		fpc_log_err("fingerprint get dts data failed: %d\n", ret);
		return ret;
	}

	ret = fingerprint_extern_ldo_proc(&spi->dev, fingerprint);
	if (ret < 0)
		fpc_log_err("fingerprint_poweron regulator failed");

#if defined(CONFIG_FB)
    fingerprint->fb_notify.notifier_call = NULL;
#endif
    atomic_set(&fingerprint->state, fp_UNINIT);

    fingerprint_get_navigation_adjustvalue(&spi->dev, fingerprint);
    fingerprint->class = class_create(THIS_MODULE, FP_CLASS_NAME);

    error = alloc_chrdev_region(&fingerprint->devno, 0, 1, FP_DEV_NAME);

    if (error)
    {
        fpc_log_err("alloc_chrdev_region failed, error = %d\n", error);
        goto exit;
    }
	printk(KERN_ERR "%s sssss %d\n",__func__,fingerprint->devno);
    fingerprint->device = device_create(fingerprint->class, NULL, fingerprint->devno,
                                        NULL, "%s", FP_DEV_NAME);

    cdev_init(&fingerprint->cdev, &fingerprint_fops);
    fingerprint->cdev.owner = THIS_MODULE;

    error = cdev_add(&fingerprint->cdev, fingerprint->devno, 1);

    if (error)
    {
        fpc_log_err("cdev_add failed, error = %d\n", error);
        goto exit;
    }

    fingerprint->input_dev = devm_input_allocate_device(dev);

    if (!fingerprint->input_dev)
    {
        error = -ENOMEM;
        fpc_log_err("devm_input_allocate_device failed, error = %d\n", error);
        goto exit;
    }

    fingerprint->input_dev->name = "fingerprint";
    /* Also register the key for wake up */
    input_set_capability(fingerprint->input_dev, EV_KEY, EVENT_UP);
    input_set_capability(fingerprint->input_dev, EV_KEY, EVENT_DOWN);
    input_set_capability(fingerprint->input_dev, EV_KEY, EVENT_LEFT);
    input_set_capability(fingerprint->input_dev, EV_KEY, EVENT_RIGHT);
    input_set_capability(fingerprint->input_dev, EV_KEY, EVENT_CLICK);
    input_set_capability(fingerprint->input_dev, EV_KEY, EVENT_HOLD);
    input_set_capability(fingerprint->input_dev, EV_KEY, EVENT_DCLICK);
    set_bit(EV_KEY, fingerprint->input_dev->evbit);
    set_bit(EVENT_UP, fingerprint->input_dev->evbit);
    set_bit(EVENT_DOWN, fingerprint->input_dev->evbit);
    set_bit(EVENT_LEFT, fingerprint->input_dev->evbit);
    set_bit(EVENT_RIGHT, fingerprint->input_dev->evbit);
    set_bit(EVENT_CLICK, fingerprint->input_dev->evbit);
    set_bit(EVENT_HOLD, fingerprint->input_dev->evbit);
    set_bit(EVENT_DCLICK, fingerprint->input_dev->evbit);

    error = input_register_device(fingerprint->input_dev);
    if (error)
    {
        fpc_log_err("input_register_device failed, error = %d\n", error);
        goto exit;
    }
	
    fingerprint->pf_dev = platform_device_alloc(FP_DEV_NAME, -1);
    if (!fingerprint->pf_dev)
    {
        error = -ENOMEM;
        fpc_log_err("platform_device_alloc failed, error = %d\n", error);
        goto exit;
    }

    error = platform_device_add(fingerprint->pf_dev);
    if (error)
    {
        fpc_log_err("platform_device_add failed, error = %d\n", error);
        platform_device_del(fingerprint->pf_dev);
        goto exit;
    }
    else
    {
        dev_set_drvdata(&fingerprint->pf_dev->dev, fingerprint);

        error = sysfs_create_group(&fingerprint->pf_dev->dev.kobj, &attribute_group);
        if (error)
        {
            fpc_log_err("sysfs_create_group failed, error = %d\n", error);
            goto exit;
        }
    }

    //device_init_wakeup(dev, 1);
	wakeup_source_init(&fingerprint->ttw_wl, "fpc_ttw_wl");
	init_waitqueue_head(&fingerprint->hbm_queue);
	init_waitqueue_head(&fingerprint->wait_finger_up_queue);
	mutex_init(&fingerprint->lock);
	mutex_init(&fingerprint->mutex_lock_clk);
	mutex_init(&fingerprint->mutex_lock_irq_switch);

	fingerprint_custom_timing(fingerprint);

	if (fingerprint->use_tp_irq == USE_TP_IRQ) {
		fp_ops_register(&g_ud_fp_ops);
		fpc_log_info("%s use tp irq,register ud_fp_ops\n", __func__);
		goto use_tp_irq_tag;
	}

	error = fingerprint_irq_init(fingerprint);
	if (error) {
		fpc_log_err("fingerprint_irq_init failed, error = %d\n", error);
		goto exit;
	}

    error = request_threaded_irq(fingerprint->irq, NULL, fingerprint_irq_handler,
                                 IRQF_TRIGGER_RISING | IRQF_ONESHOT, "fingerprint", fingerprint);
    if (error)
    {
        fpc_log_err("failed to request irq %d\n", fingerprint->irq);
        goto exit;
    }

	fingerprint->irq_num = 1;

	/* Request that the interrupt should be wakeable */
	enable_irq_wake(fingerprint->irq);
	fingerprint->wakeup_enabled = true;
	fingerprint->snr_stat = 0;

use_tp_irq_tag:
#if defined(CONFIG_FB)
	if (fingerprint->fb_notify.notifier_call == NULL) {
		fingerprint->fb_notify.notifier_call = fb_notifier_callback;
		fb_register_client(&fingerprint->fb_notify);
	}
#endif
	if (fingerprint_btb_det_irq_init(fingerprint))
		fpc_log_err("fp_btb_detect_irq_init failed\n");

	write_conf_to_tee(spi, fingerprint);
	fingerprint->nav_stat = 0;
	fingerprint->sensor_id = 0;
	fingerprint->hbm_status = HBM_NONE;
	fingerprint->fingerprint_bigdata.lcd_charge_time = 60; /* unit ms */
	fingerprint->fingerprint_bigdata.lcd_on_time = 50; /* unit ms */
	fingerprint->fingerprint_bigdata.cpu_wakeup_time = 80; /* unit ms */

	g_fp_data = fingerprint;
	fpc_log_info("fingerprint_probe OK\n");

	return error;

exit:
    fpc_log_info("fingerprint probe failed!\n");
#if defined (CONFIG_HUAWEI_DSM)

    if (error && !dsm_client_ocuppy(fingerprint_dclient))
    {
        dsm_client_record(fingerprint_dclient, "fingerprint_probe failed, error = %d\n", error);
        dsm_client_notify(fingerprint_dclient, DSM_FINGERPRINT_PROBE_FAIL_ERROR_NO);
    }

#endif
    return error;
}

static int fingerprint_remove(struct spi_device* spi)
{
	struct  fp_data* fingerprint = dev_get_drvdata(&spi->dev);

	sysfs_remove_group(&fingerprint->pf_dev->dev.kobj, &attribute_group);
	cdev_del(&fingerprint->cdev);
	unregister_chrdev_region(fingerprint->devno, 1);
	input_free_device(fingerprint->input_dev);
	mutex_destroy(&fingerprint->lock);
	mutex_destroy(&fingerprint->mutex_lock_clk);
	mutex_destroy(&fingerprint->mutex_lock_irq_switch);
	wakeup_source_trash(&fingerprint->ttw_wl);
	fingerprint_btb_det_remove(fingerprint);
#if defined(CONFIG_FB)
	if (fingerprint->fb_notify.notifier_call != NULL) {
		fingerprint->fb_notify.notifier_call = NULL;
		fb_unregister_client(&fingerprint->fb_notify);
	}
#endif
	fp_ops_unregister(&g_ud_fp_ops);
	return 0;
}

static int fingerprint_suspend(struct device* dev)
{
	return FP_RETURN_SUCCESS;
}

static int fingerprint_resume(struct device* dev)
{
	return FP_RETURN_SUCCESS;
}

static const struct dev_pm_ops fingerprint_pm =
{
    .suspend = fingerprint_suspend,
    .resume = fingerprint_resume
};

// TODO: All following should consider platform
static struct of_device_id fingerprint_of_match[] =
{
    { .compatible = "mediatek,mtk_finger", },
    {}
};

MODULE_DEVICE_TABLE(of, fingerprint_of_match);

static struct spi_driver fingerprint_driver =
{
    .driver = {
        .name	= "fingerprint",
        .owner	= THIS_MODULE,
        .of_match_table = fingerprint_of_match,
        .pm = &fingerprint_pm
    },
    .probe  = fingerprint_probe,
    .remove = fingerprint_remove
};

static int __init fingerprint_init(void)
{
	int status;

    status = spi_register_driver(&fingerprint_driver);
    if (status < 0)
    {
        printk(KERN_ERR "%s:status is %d\n", __func__, status);
        return -EINVAL;
    }
    return 0;
}

static void __exit fingerprint_exit(void)
{
    spi_unregister_driver(&fingerprint_driver);
}
late_initcall(fingerprint_init);
module_exit(fingerprint_exit);

MODULE_LICENSE("GPL v2");
