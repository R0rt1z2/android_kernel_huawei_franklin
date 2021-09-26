#include <linux/init.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/pinctrl/consumer.h>
#include "securec.h"
#include "antenna_boardid_gpio_detect.h"

#define antenna_detect(_name, n, m, store)				 \
{								   \
	.attr = __ATTR(_name, m, antenna_detect_show, store),	\
	.name = ANTENNA_##n,							\
}

#define antenna_detect_ro(_name, n)			\
		antenna_detect(_name, n, S_IRUGO, NULL)

static ssize_t antenna_detect_show(struct device *dev,
	struct device_attribute *attr, char *buf);

static struct antenna_detect_info antenna_detect_tb[] = {
	antenna_detect_ro(antenna_boardid_status, BOARDID_GPIO_STATUS),
};

static struct attribute *antenna_sysfs_attrs[ARRAY_SIZE(antenna_detect_tb) + 1];
static const struct attribute_group antenna_sysfs_attr_group = {
	.attrs = antenna_sysfs_attrs,
};

/* g_func_type 0:pinctrl pull function 1:get GPIO directly function */
static int g_func_type = 0;
static int g_gpio_expect = 0;
static int g_gpio_count = 0;
static int antenna_probe_flag = 0;
static struct pinctrl *antenna_pinctrl = NULL;
static struct antenna_pinctrl_type pinctrl_table[MAX_GPIO_NUM] = {0};
static struct mutex antenna_mutex;

/* product id index table: 0000 0001 0011   0100 0101 0111  1100 1101 1111 */
const unsigned char gpio_index_table[BOARD_ID_INDEX_MAX] = {
	0, 0x1, 0x3, 0x4, 0x5, 0x7, 0xc, 0xd, 0xf, 0xff
};

struct antenna_gpio_state gpio_state_table[MAX_GPIO_NUM] = {
	{"state_id0_default", "state_id0_bias_up", "state_id0_bias_down"},
	{"state_id1_default", "state_id1_bias_up", "state_id1_bias_down"},
	{"state_id2_default", "state_id2_bias_up", "state_id2_bias_down"},
};

static void antenna_sysfs_init_attrs(void)
{
	int i;
	int limit = ARRAY_SIZE(antenna_detect_tb);

	for (i = 0; i < limit; i++)
		antenna_sysfs_attrs[i] = &antenna_detect_tb[i].attr.attr;
	antenna_sysfs_attrs[limit] = NULL;
}

static struct antenna_detect_info *antenna_detect_lookup(const char *name)
{
	int i;
	int limit = ARRAY_SIZE(antenna_detect_tb);

	if (!name) {
		pr_err("%s: invalid param, fatal error.\n", __func__);
		return NULL;
	}

	for (i = 0; i < limit; i++)
		if (!strncmp(name, antenna_detect_tb[i].attr.attr.name,
			strlen(name)))
			break;
	if (i >= limit)
		return NULL;

	return &antenna_detect_tb[i];
}

static int antenna_detect_sysfs_create_group(struct antenna_device_info *di)
{
	if (!di) {
		pr_err("%s: invalid param, fatal error.\n", __func__);
		return -EINVAL;
	}

	antenna_sysfs_init_attrs();
	return sysfs_create_group(&di->dev->kobj, &antenna_sysfs_attr_group);
}

static inline void antenna_detect_sysfs_remove_group(struct antenna_device_info *di)
{
	if (!di) {
		pr_err("%s: invalid param, fatal error.\n", __func__);
		return;
	}

	sysfs_remove_group(&di->dev->kobj, &antenna_sysfs_attr_group);
	return;
}

/**
 * get_antenna_gpio_value-----Read combination of
 * values when pull down and up the gpio
 * @pinctrl_table[gpioID].: struct antenna_pinctrl_type
 * returns: combination of values
 */
static unsigned int get_antenna_gpio_value(enum gpio_id gpioID)
{
	mutex_lock(&antenna_mutex);
	int ret;
	unsigned int value_up = 0;
	unsigned int value_down = 0;
	unsigned int gipo_status = 0;

	/* config gpio bias-down state */
	ret = pinctrl_select_state(antenna_pinctrl,
		pinctrl_table[gpioID].pinctrl_down);
	if (ret) {
		pr_err("%s: set gpio:%d down state fail.\n",
			__func__, pinctrl_table[gpioID].gpio);
		mutex_unlock(&antenna_mutex);
		return -1;
	}
	msleep(DELAY_TIME);
	value_down = (unsigned int)gpio_get_value(pinctrl_table[gpioID].gpio);

	/* config gpio bias-up state */
	ret = pinctrl_select_state(antenna_pinctrl,
		pinctrl_table[gpioID].pinctrl_up);
	if (ret) {
		pr_err("%s: set gpio:%d up state fail.\n",
			__func__, pinctrl_table[gpioID].gpio);
		mutex_unlock(&antenna_mutex);
		return -1;
	}
	msleep(DELAY_TIME);
	value_up = (unsigned int)gpio_get_value(pinctrl_table[gpioID].gpio);

	/* restore gpio default configuration */
	ret = pinctrl_select_state(antenna_pinctrl,
		pinctrl_table[gpioID].pinctrl_default);
	if (ret) {
		pr_err("%s: set gpio:%d default state fail.\n",
			__func__, pinctrl_table[gpioID].gpio);
		mutex_unlock(&antenna_mutex);
		return -1;
	}

	gipo_status = value_up | (value_down << 0x01);
	pr_err("%s: gipo:%d, idStatus:0x%x = value_up:0x%x | value_down:0x%x << 1.\n",
		__func__, pinctrl_table[gpioID].gpio, gipo_status, value_up, value_down);
	mutex_unlock(&antenna_mutex);
	return gipo_status;
}

/**
 * get_antenna_row_index_value-----get the row value
 * that the product is in the antenna baord id table
**/
static int get_antenna_row_index_value(enum gpio_id ID0, enum gpio_id ID1)
{
	unsigned int gpio_id0;
	unsigned int gpio_id1;
	unsigned int row_state;
	int row_index;

	/* get gpio value from hw flage for row index */
	gpio_id0 = get_antenna_gpio_value(ID0);
	row_state = gpio_id0 << 2;
	gpio_id1 = get_antenna_gpio_value(ID1);
	row_state |= gpio_id1;
	row_state &= GPIO_BOARD_INDEX_MASK;
	pr_err("%s: row_state:0x%x = id0:0x%x << 2 | id1:0x%x.\n",
		__func__, row_state, gpio_id0, gpio_id1);

	/* find the row index in board id table */
	for (row_index = 0; row_index < GPIO_ROW_INDEX_MAX; row_index++)
		if (gpio_index_table[row_index] == row_state)
			return row_index;

	pr_err("%s: get row index error.\n", __func__);
	return -1;
}

/**
 * get_antenna_col_index_value-----get the col value
 * that the product is in the antenna baord id table
**/
static int get_antenna_col_index_value(enum gpio_id ID2)
{
	int col_index;
	unsigned int col_state;
	unsigned int gpio_id2;

	/* get gpio value from hw flage for col index */
	gpio_id2 = get_antenna_gpio_value(ID2);
	col_state = gpio_id2 & GPIO_BOARD_INDEX_MASK;
	pr_err("%s: col_state:0x%x = id2:0x%x.\n", __func__, col_state, gpio_id2);

	/* find the col index in board id table */
	for (col_index = 0; col_index < GPIO_COL_INDEX_MAX; col_index++)
		if (gpio_index_table[col_index] == col_state)
			return col_index;

	pr_err("%s: get col index error.\n", __func__);
	return -1;
}

static int get_antenna_board_id(void)
{
	int gpio[MAX_GPIO_NUM];
	int ant_boardid = 0;
	int temp = 0;
	int i = 0;

	memset_s(gpio, sizeof(gpio), 0, sizeof(gpio));
	for (i = 0; i < g_gpio_count; i++) {
		gpio[i] = gpio_get_value(pinctrl_table[i].gpio);
		if (gpio[i] < 0) {
			pr_err("%s: get gpio value error, gpio[%d] %d\n", __func__, i, gpio[i]);
			return -1;
		}
		temp |= gpio[i] << i;
	}
	ant_boardid = ANTENNA_BOARDID_BASE + temp;
	pr_err("%s: the ant_boardid is %d, g_gpio_expect is %d.\n", __func__, ant_boardid, g_gpio_expect);
	return ((ant_boardid == g_gpio_expect) ? ANTENNA_DETECT_SUCCEED : ANTENNA_DETECT_FAIL);
}
/**
 * check_antenna_board_id-----the location
 * that the product is in produt table
**/
static int check_antenna_board_id(void)
{
	int row_index;
	int col_index;
	int ant_board_id = 0;

	row_index = get_antenna_row_index_value(GPIO_ID0, GPIO_ID1);
	col_index = get_antenna_col_index_value(GPIO_ID2);
	if (row_index < 0 || col_index < 0) {
		pr_err("%s: get table index error, row_index:%d, col_index:%d.\n",
			__func__, row_index, col_index);
		return -1;
	}
	ant_board_id = ANTENNA_BOARDID_BASE +
		(((unsigned int)row_index << 4) | ((unsigned int)col_index));
	pr_err("%s: the ant_board_id is %d, g_gpio_expect is %d.\n",
		__func__, ant_board_id, g_gpio_expect);

	return (ant_board_id == g_gpio_expect ? ANTENNA_DETECT_SUCCEED : ANTENNA_DETECT_FAIL);
}

/*******************************************************************
 * Function:	   antenna_detect_show
 * Description:	echo data to create node
 * Parameters:	 default parameters
 * return value:   0: sucess -1: error
 * ****************************************************************/
static ssize_t antenna_detect_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int ant_match_state = 0;
	struct antenna_detect_info *info = NULL;
	pr_info("%s: function start.\n", __func__);

	if (NULL == dev || NULL == attr || NULL == buf) {
		pr_err("%s dev or attr is null.\n", __func__);
		return -1;
	}

	info = antenna_detect_lookup(attr->attr.name);
	if (NULL == info) {
		pr_err("%s Invalid argument.\n", __func__);
		return -1;
	}

	switch (info->name) {
	case ANTENNA_BOARDID_GPIO_STATUS: {
		if (g_func_type == 1) {
			ant_match_state = get_antenna_board_id();
		} else {
			ant_match_state = check_antenna_board_id();
		}
		break;
	}
	default:
		pr_err("%s: HAVE NO THIS NODE:%d.\n", __func__, info->name);
		break;
	}

	pr_info("%s: function end.\n", __func__);
	return snprintf(buf, PAGE_SIZE, "%d\n", ant_match_state);
}

/******************************************************************************
 * Pinctrl configuration
 *****************************************************************************/
static int antenna_pinctrl_init(struct antenna_device_info *di)
{
	int index = 0;
	pr_info("%s: function start.\n", __func__);

	/* get antenna_pinctrl */
	antenna_pinctrl = devm_pinctrl_get(di->dev);
	if (IS_ERR(antenna_pinctrl)) {
		pr_err("%s: get antenna pinctrl fail:%li.\n",
			__func__, PTR_ERR(antenna_pinctrl));
		return -1;
	}

	for (index = 0; index < g_gpio_count; index++) {
		pinctrl_table[index].pinctrl_up = pinctrl_lookup_state(antenna_pinctrl,
			gpio_state_table[index].state_bias_up);
		if (IS_ERR(pinctrl_table[index].pinctrl_up)) {
			pr_err("%s: index:%d, get pinctrl_up state:%li fail.\n",
				__func__, index, PTR_ERR(pinctrl_table[index].pinctrl_up));
			return -1;
		}

		pinctrl_table[index].pinctrl_down = pinctrl_lookup_state(antenna_pinctrl,
			gpio_state_table[index].state_bias_down);
		if (IS_ERR(pinctrl_table[index].pinctrl_down)) {
			pr_err("%s: index:%d, get pinctrl_down state:%li fail.\n",
				__func__, index, PTR_ERR(pinctrl_table[index].pinctrl_down));
			return -1;
		}

		pinctrl_table[index].pinctrl_default = pinctrl_lookup_state(antenna_pinctrl,
			gpio_state_table[index].state_dafault);
		if (IS_ERR(pinctrl_table[index].pinctrl_default)) {
			pr_err("%s: index:%d, get pinctrl_default state:%li fail.\n",
				__func__, index, PTR_ERR(pinctrl_table[index].pinctrl_default));
			return -1;
		}
	}

	pr_info("%s: function end.\n", __func__);
	return 0;
}

/********************************************************************
 * Config: ID_0: GPIO_EXT13 = GPIO176,
           ID_1: GPIO_EXT12 = GPIO175,
           ID_2: GPIO_EXT9 = GPIO172 *
 ********************************************************************/
static int antenna_dts_parse(struct antenna_device_info *di)
{
	int i = 0;
	int ret = 0;
	struct device_node* np = di->dev->of_node;
	pr_info("%s: function start.\n", __func__);

	if (NULL == np) {
		pr_err("%s: np is null!.\n", __func__);
		return -1;
	}

	if (of_property_read_u32(np, "expect_value", &g_gpio_expect)) {
		g_gpio_expect = -1;
		pr_err("%s: get expect value error.\n", __func__);
	}

	if (of_property_read_u32(np, "func_type", &g_func_type)) {
		g_func_type = 0;
		pr_err("%s: get func_type error\n", __func__);
	}

	g_gpio_count = of_gpio_named_count(np, "ant_gpio");
	if (g_gpio_count <= 0 || g_gpio_count > MAX_GPIO_NUM) {
		pr_err("%s: gpio count %d is invalid.\n", __func__, g_gpio_count);
		return -1;
	}

	for (i = 0; i < g_gpio_count; i++) {
		pinctrl_table[i].gpio = of_get_named_gpio(np, "ant_gpio", i);
		pr_err("%s: pinctrl_table[%d].gpio:%d.\n",
			__func__, i, pinctrl_table[i].gpio);

		if (!gpio_is_valid(pinctrl_table[i].gpio)) {
			pr_err("%s: gpio:%d is invalid.\n",
				__func__, pinctrl_table[i].gpio);
			return -1;
		}

		ret = gpio_request_one(pinctrl_table[i].gpio, GPIOF_IN,
			"antenna_detect");
		if (ret < 0) {
			pr_err("%s: pinctrl_table[%d].gpio:%d request one fail.\n",
				__func__, i, pinctrl_table[i].gpio);
			return -1;
		}
	}

	pr_info("%s: function end.\n", __func__);
	return 0;
}

static int antenna_boardid_detect_probe(struct platform_device *pdev)
{
	int ret;
	struct device *antenna_dev = NULL;
	struct antenna_device_info *di = NULL;
	struct class *antenna_detect_class = NULL;
	pr_info("%s: function start.\n", __func__);

	mutex_init(&antenna_mutex);
	if (antenna_probe_flag) {
		pr_err("%s: function has done.\n", __func__);
		return 0;
	}

	if (NULL == pdev) {
		pr_err("%s: invalid param, fatal error.\n", __func__);
		return -EINVAL;
	}

	di = kzalloc(sizeof(*di), GFP_KERNEL);
	if (NULL == di) {
		pr_err("alloc di failed.\n");
		return -ENOMEM;
	}
	di->dev = &pdev->dev;
	dev_set_drvdata(&(pdev->dev), di);

	/* get dts data and init g_gpio_count */
	if (antenna_dts_parse(di) != 0) {
		pr_err("antenna_dts_parse failed.\n");
		goto free_di;
	}

	/* init antenna pinctrl, g_gpio_count needed */
	if (g_func_type == 0) {
		if (antenna_pinctrl_init(di) != 0) {
			pr_err("antenna pinctrl init fail.\n");
			goto free_di;
		}
	}

	/* get new class */
	antenna_detect_class = class_create(THIS_MODULE, "hw_antenna");
	if (IS_ERR(antenna_detect_class)) {
		pr_err("hw_antenna class create fail");
		goto free_di;
	}

	antenna_dev = device_create(antenna_detect_class,
		NULL, 0, NULL, "antenna_board");
	if (IS_ERR(antenna_dev)) {
		pr_err("create antenna_dev failed!.\n");
		goto free_di;
	}

	ret = sysfs_create_link(&antenna_dev->kobj,
		&di->dev->kobj, "antenna_board_data");
	if (ret) {
		pr_err("create link to boardid_detect fail.\n");
		goto free_di;
	}

	ret = antenna_detect_sysfs_create_group(di);
	if (ret) {
		pr_err("can't create antenna_detect sysfs entries.\n");
		goto free_di;
	}

	antenna_probe_flag = ANTENNA_DETECT_SUCCESS;
	pr_info("%s: function end.\n", __func__);
	return 0;

free_di:
	kfree(di);
	di = NULL;
	return -1;
}

static int antenna_boardid_detect_remove(struct platform_device *pdev)
{
	struct antenna_device_info *di = NULL;

	if (!pdev) {
		pr_err("%s: invalid param, fatal error.\n", __func__);
		return -EINVAL;
	}

	di = dev_get_drvdata(&pdev->dev);
	if (NULL == di) {
		pr_err("[%s]di is NULL!.\n", __func__);
		return -ENODEV;
	}

	kfree(di);
	return 0;
}

/*
 * probe match table
*/
static struct of_device_id antenna_boardid_detect_table[] = {
	{
		.compatible = "huawei,antenna_boardid_detect",
		.data = NULL,
	},
	{},
};

/*
 * antenna boardid detect driver
 */
static struct platform_driver antenna_boardid_detect_driver = {
	.probe = antenna_boardid_detect_probe,
	.remove = antenna_boardid_detect_remove,
	.driver = {
		.name = "huawei,antenna_boardid_detect",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(antenna_boardid_detect_table),
	},
};

/***************************************************************
 * Function:     antenna_boardid_detect_init
 * Description:  antenna boardid gpio detect module initialization
 * Parameters:   Null
 * return value: 0-sucess or others-fail
 * **************************************************************/
static int __init antenna_boardid_detect_init(void)
{
	return platform_driver_register(&antenna_boardid_detect_driver);
}

/*******************************************************************
 * Function:       antenna_boardid_detect_exit
 * Description:    antenna boardid gpio detect module exit
 * Parameters:     NULL
 * return value:   NULL
 * ****************************************************************/
static void __exit antenna_boardid_detect_exit(void)
{
	platform_driver_unregister(&antenna_boardid_detect_driver);
}

module_init(antenna_boardid_detect_init);
module_exit(antenna_boardid_detect_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("huawei antenna boardid detect driver");
MODULE_AUTHOR("HUAWEI Inc");
