/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: als para table ams tmd3702 source file
 * Author: linjianpeng <linjianpeng1@huawei.com>
 * Create: 2020-05-25
 */

#include <linux/module.h>
#include <linux/err.h>
#include <linux/types.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/rtc.h>
#include <linux/of_device.h>
#include <linux/slab.h>
#include "xhub_route.h"
#include "xhub_boot.h"
#include "protocol.h"
#include "sensor_config.h"
#include "sensor_detect.h"
#include "sensor_sysfs.h"
#include "als_tp_color.h"
#include "als_channel.h"
#include "als_para_table_ams_tmd3702.h"
#include <securec.h>

tmd3702_als_para_table tmd3702_als_para_diff_tp_color_table[] = {
	{ OTHER, OTHER, DEFAULT_TPLCD, OTHER,
	  { 50, 800, 1500, -200, -5250, 3720, 0, 801, 2990, -4020, 2650, -2470,
	    0, 10, 8759, 0, 0, 1675, 4316, 0, 0, 2189, 8, 6747, 5269, 4084,
	    15877, 4000, 250 }
	},
	{ HIMA, V3, DEFAULT_TPLCD, GRAY,
	  { 50, 804, 1250, -1130, 1720, -1820, 0, 804, 1250, -1130, 1720, -1820,
	    0, 8, 9463, 0, 0, 1534, 5814, 0, 0, 1962, 264, 8670, 6623, 4495,
	    18640, 4000, 200 }
	},
	{ HIMA, V3, DEFAULT_TPLCD, BLACK,
	  { 50, 804, 1250, -1130, 1720, -1820, 0, 804, 1250, -1130, 1720, -1820,
	    0, 8, 9463, 0, 0, 1534, 5814, 0, 0, 1962, 264, 8670, 6623, 4495,
	    18640, 4000, 200 }
	},
	{ HIMA, V3, DEFAULT_TPLCD, BLACK2,
	  { 50, 804, 1250, -1130, 1720, -1820, 0, 804, 1250, -1130, 1720, -1820,
	    0, 8, 9463, 0, 0, 1534, 5814, 0, 0, 1962, 264, 8670, 6623, 4495,
	    18640, 4000, 200 }
	},
	{ HIMA, V3, DEFAULT_TPLCD, GOLD,
	  { 50, 800, 1820, -1930, 4600, -4020, 0, 800, 1820, -1930, 4600, -4020,
	    0, 10, 7706, 0, 0, 1519, 4187, 0, 0, 2123, 266, 5504, 4679, 3925,
	    13640, 4000, 200 }
	},

	{ HIMA, V4, DEFAULT_TPLCD, GRAY,
	  { 50, 795, 1700, -1650, 2990, -2970, 0, 795, 1700, -1650, 2990, -2970,
	    0, 10, 8653, 0, 0, 1608, 4270, 0, 0, 2195, 10, 6214, 5153, 4129,
	    14987, 4000, 250 }
	},
	{ HIMA, V4, DEFAULT_TPLCD, BLACK,
	  { 50, 795, 1700, -1650, 2990, -2970, 0, 795, 1700, -1650, 2990, -2970,
	    0, 10, 8653, 0, 0, 1608, 4270, 0, 0, 2195, 10, 6214, 5153, 4129,
	    14987, 4000, 250 }
	},
	{ HIMA, V4, DEFAULT_TPLCD, BLACK2,
	  { 50, 795, 1700, -1650, 2990, -2970, 0, 795, 1700, -1650, 2990, -2970,
	    0, 10, 8653, 0, 0, 1608, 4270, 0, 0, 2195, 10, 6214, 5153, 4129,
	    14987, 4000, 250 }
	},
	{ HIMA, V4, DEFAULT_TPLCD, GOLD,
	  { 50, 795, 1700, -1650, 2990, -2970, 0, 795, 1700, -1650, 2990, -2970,
	    0, 10, 8653, 0, 0, 1608, 4270, 0, 0, 2195, 10, 6214, 5153, 4129,
	    14987, 4000, 250 }
	},

	{ HIMA, VN1, DEFAULT_TPLCD, GRAY,
	  { 50, 799, 1610, -980, -1020, 170, 0, 802, 1660, -1740, 3410, -3610,
	    0, 8, 10610, -2342, 0, 1897, 10083, -3024, 0, 2006, 8, 6747, 5269,
	    4084, 15877, 4000, 250 }
	},
	{ HIMA, VN1, DEFAULT_TPLCD, BLACK,
	  { 50, 799, 1610, -980, -1020, 170, 0, 802, 1660, -1740, 3410, -3610,
	    0, 8, 10610, -2342, 0, 1897, 10083, -3024, 0, 2006, 8, 6747, 5269,
	    4084, 15877, 4000, 250 }
	},
	{ HIMA, VN1, DEFAULT_TPLCD, BLACK2,
	  { 50, 799, 1610, -980, -1020, 170, 0, 802, 1660, -1740, 3410, -3610,
	    0, 8, 10610, -2342, 0, 1897, 10083, -3024, 0, 2006, 8, 6747, 5269,
	    4084, 15877, 4000, 250 }
	},
	{ HIMA, VN1, DEFAULT_TPLCD, GOLD,
	  { 50, 799, 1610, -980, -1020, 170, 0, 802, 1660, -1740, 3410, -3610,
	    0, 8, 10610, -2342, 0, 1897, 10083, -3024, 0, 2006, 8, 6747, 5269,
	    4084, 15877, 4000, 250 }
	},
	{ TAURUS, V3, DEFAULT_TPLCD, GRAY,
	  { 50, 434, 5334, -1110, 228, 5572, -826, -405, -1101, 7975, 15370, 10680,
	    0, 25, 704, 0, 0, 2703, 16092, 0, 0, -2526, 281, 6749, 5128, 4232,
	    15434, 4000, 250 }
	},
	{ TAURUS, V3, DEFAULT_TPLCD, BLACK,
	  { 50, 434, 5334, -1110, 228, 5572, -826, -405, -1101, 7975, 15370, 10680,
	    0, 25, 704, 0, 0, 2703, 16092, 0, 0, -2526, 281, 6749, 5128, 4232,
	    15434, 4000, 250 }
	},
	{ TAURUS, V3, DEFAULT_TPLCD, BLACK2,
	  { 50, 434, 5334, -1110, 228, 5572, -826, -405, -1101, 7975, 15370, 10680,
	    0, 25, 704, 0, 0, 2703, 16092, 0, 0, -2526, 281, 6749, 5128, 4232,
	    15434, 4000, 250 }
	},
	{ TAURUS, V3, DEFAULT_TPLCD, GOLD,
	  { 50, 434, 5334, -1110, 228, 5572, -826, -405, -1101, 7975, 15370, 10680,
	    0, 25, 704, 0, 0, 2703, 16092, 0, 0, -2526, 281, 6749, 5128, 4232,
	    15434, 4000, 250 }
	},
	{ TAURUS, V3, DEFAULT_TPLCD, WHITE,
	  { 50, 434, 5334, -1110, 228, 5572, -826, -405, -1101, 7975, 15370, 10680,
	    0, 25, 704, 0, 0, 2703, 16092, 0, 0, -2526, 281, 6749, 5128, 4232,
	    15434, 4000, 250 }
	},

	{ TAURUS, V4, DEFAULT_TPLCD, GRAY,
	  { 50, 434, 5334, -1110, 228, 5572, -826, -405, -1101, 7975, 8015, -988,
	    0, 30, 514, 0, 0, 2794, 17154, 0, 0, -3024, 286, 7382, 5574, 4597,
	    16756, 4000, 250 }
	},
	{ TAURUS, V4, DEFAULT_TPLCD, BLACK,
	  { 50, 434, 5334, -1110, 228, 5572, -826, -405, -1101, 7975, 8015, -988,
	    0, 30, 514, 0, 0, 2794, 17154, 0, 0, -3024, 286, 7382, 5574, 4597,
	    16756, 4000, 250 }
	},
	{ TAURUS, V4, DEFAULT_TPLCD, BLACK2,
	  { 50, 434, 5334, -1110, 228, 5572, -826, -405, -1101, 7975, 8015, -988,
	    0, 30, 514, 0, 0, 2794, 17154, 0, 0, -3024, 286, 7382, 5574, 4597,
	    16756, 4000, 250 }
	},
	{ TAURUS, V4, DEFAULT_TPLCD, GOLD,
	  { 50, 434, 5334, -1110, 228, 5572, -826, -405, -1101, 7975, 8015, -988,
	    0, 30, 514, 0, 0, 2794, 17154, 0, 0, -3024, 286, 7382, 5574, 4597,
	    16756, 4000, 250 }
	},
	{ TAURUS, V4, DEFAULT_TPLCD, WHITE,
	  { 50, 434, 5334, -1110, 228, 5572, -826, -405, -1101, 7975, 8015, -988,
	    0, 30, 514, 0, 0, 2794, 17154, 0, 0, -3024, 286, 7382, 5574, 4597,
	    16756, 4000, 250 }
	},

	{ TAURUS, VN1, DEFAULT_TPLCD, GRAY,
	  { 50, 462, 5233, -1907, 270, 5584, -1779, -359, -383, 6756, 8015, -988,
	    0, 30, 514, 0, 0, 2794, 17154, 0, 0, -3024, 286, 7382, 5574, 4597,
	    16756, 4000, 250 }
	},
	{ TAURUS, VN1, DEFAULT_TPLCD, BLACK,
	  { 50, 462, 5233, -1907, 270, 5584, -1779, -359, -383, 6756, 8015, -988,
	    0, 30, 514, 0, 0, 2794, 17154, 0, 0, -3024, 286, 7382, 5574, 4597,
	    16756, 4000, 250 }
	},
	{ TAURUS, VN1, DEFAULT_TPLCD, BLACK2,
	  { 50, 462, 5233, -1907, 270, 5584, -1779, -359, -383, 6756, 8015, -988,
	    0, 30, 514, 0, 0, 2794, 17154, 0, 0, -3024, 286, 7382, 5574, 4597,
	    16756, 4000, 250 }
	},
	{ TAURUS, VN1, DEFAULT_TPLCD, GOLD,
	  { 50, 462, 5233, -1907, 270, 5584, -1779, -359, -383, 6756, 8015, -988,
	    0, 30, 514, 0, 0, 2794, 17154, 0, 0, -3024, 286, 7382, 5574, 4597,
	    16756, 4000, 250 }
	},
	{ TAURUS, VN1, DEFAULT_TPLCD, WHITE,
	  { 50, 462, 5233, -1907, 270, 5584, -1779, -359, -383, 6756, 8015, -988,
	    0, 30, 514, 0, 0, 2794, 17154, 0, 0, -3024, 286, 7382, 5574, 4597,
	    16756, 4000, 250 }
	},

	{ TETON, V3, DEFAULT_TPLCD, WHITE,
	  { 50, 462, 5233, -1907, 270, 5584, -1779, -359, -383, 6756, 8015, -988,
	    0, 30, 514, 0, 0, 2794, 17154, 0, 0, -3024, 286, 20087, 13149, 11475,
	    3794, 4000, 250 }
	},
	{ TETON, V3, DEFAULT_TPLCD, GRAY,
	  { 50, 462, 5233, -1907, 270, 5584, -1779, -359, -383, 6756, 8015, -988,
	    0, 30, 514, 0, 0, 2794, 17154, 0, 0, -3024, 286, 20087, 13149, 11475,
	    3794, 4000, 250 }
	},
	{ TETON, V3, DEFAULT_TPLCD, BLACK,
	  { 50, 462, 5233, -1907, 270, 5584, -1779, -359, -383, 6756, 8015, -988,
	    0, 30, 514, 0, 0, 2794, 17154, 0, 0, -3024, 286, 20087, 13149, 11475,
	    3794, 4000, 250 }
	},
	{ TETON, V3, DEFAULT_TPLCD, BLACK2,
	  { 50, 462, 5233, -1907, 270, 5584, -1779, -359, -383, 6756, 8015, -988,
	    0, 30, 514, 0, 0, 2794, 17154, 0, 0, -3024, 286, 20087, 13149, 11475,
	    3794, 4000, 250 }
	},
	{ TETON, V3, DEFAULT_TPLCD, GOLD,
	  { 50, 462, 5233, -1907, 270, 5584, -1779, -359, -383, 6756, 8015, -988,
	    0, 30, 514, 0, 0, 2794, 17154, 0, 0, -3024, 286, 20087, 13149, 11475,
	    3794, 4000, 250 }
	},

	{ LION, V3, DEFAULT_TPLCD, GRAY,
	  { 50, 385, 4130, -719, 197, 4347, -440, -339, -1003, 7882, 0, 0,
	    0, 25, 8359, 0, 0, 1702, 12000, 0, 0, -157, 281, 6193, 4581, 3803,
	    13268, 4000, 250 }
	},
	{ LION, V3, DEFAULT_TPLCD, BLACK,
	  { 50, 385, 4130, -719, 197, 4347, -440, -339, -1003, 7882, 0, 0,
	    0, 25, 8359, 0, 0, 1702, 12000, 0, 0, -157, 281, 6193, 4581, 3803,
	    13268, 4000, 250 }
	},
	{ LION, V3, DEFAULT_TPLCD, BLACK2,
	  { 50, 385, 4130, -719, 197, 4347, -440, -339, -1003, 7882, 0, 0,
	    0, 25, 8359, 0, 0, 1702, 12000, 0, 0, -157, 281, 6193, 4581, 3803,
	    13268, 4000, 250 }
	},
	{ LION, V3, DEFAULT_TPLCD, GOLD,
	  { 50, 385, 4130, -719, 197, 4347, -440, -339, -1003, 7882, 0, 0,
	    0, 25, 8359, 0, 0, 1702, 12000, 0, 0, -157, 281, 6193, 4581, 3803,
	    13268, 4000, 250 }
	},
	{ LION, V3, DEFAULT_TPLCD, WHITE,
	  { 50, 385, 4130, -719, 197, 4347, -440, -339, -1003, 7882, 0, 0,
	    0, 25, 8359, 0, 0, 1702, 12000, 0, 0, -157, 281, 6193, 4581, 3803,
	    13268, 4000, 250 }
	},

	{ LION, V4, DEFAULT_TPLCD, GRAY,
	  { 50, 385, 4130, -719, 197, 4347, -440, -339, -1003, 7882, 910, -3350,
	    0, 22, 272, 0, 0, 2807, 15960, 0, 0, -874, 278, 10219, 7449, 5685,
	    21574, 4000, 250 }
	},
	{ LION, V4, DEFAULT_TPLCD, BLACK,
	  { 50, 385, 4130, -719, 197, 4347, -440, -339, -1003, 7882, 910, -3350,
	    0, 22, 272, 0, 0, 2807, 15960, 0, 0, -874, 278, 10219, 7449, 5685,
	    21574, 4000, 250 }
	},
	{ LION, V4, DEFAULT_TPLCD, BLACK2,
	  { 50, 385, 4130, -719, 197, 4347, -440, -339, -1003, 7882, 910, -3350,
	    0, 22, 272, 0, 0, 2807, 15960, 0, 0, -874, 278, 10219, 7449, 5685,
	    21574, 4000, 250 }
	},
	{ LION, V4, DEFAULT_TPLCD, GOLD,
	  { 50, 385, 4130, -719, 197, 4347, -440, -339, -1003, 7882, 910, -3350,
	    0, 22, 272, 0, 0, 2807, 15960, 0, 0, -874, 278, 10219, 7449, 5685,
	    21574, 4000, 250 }
	},
	{ LION, V4, DEFAULT_TPLCD, WHITE,
	  { 50, 385, 4130, -719, 197, 4347, -440, -339, -1003, 7882, 910, -3350,
	    0, 22, 272, 0, 0, 2807, 15960, 0, 0, -874, 278, 10219, 7449, 5685,
	    21574, 4000, 250 }
	},
	{ LION, VN1, DEFAULT_TPLCD, GRAY,
	  { 50, 385, 4130, -719, 197, 4347, -440, -339, -1003, 7882, 910, -3350,
	    0, 22, 272, 0, 0, 2807, 15960, 0, 0, -874, 278, 10219, 7449, 5685,
	    21574, 4000, 250 }
	},
	{ LION, VN1, DEFAULT_TPLCD, BLACK,
	  { 50, 385, 4130, -719, 197, 4347, -440, -339, -1003, 7882, 910, -3350,
	    0, 22, 272, 0, 0, 2807, 15960, 0, 0, -874, 278, 10219, 7449, 5685,
	    21574, 4000, 250 }
	},
	{ LION, VN1, DEFAULT_TPLCD, BLACK2,
	  { 50, 385, 4130, -719, 197, 4347, -440, -339, -1003, 7882, 910, -3350,
	    0, 22, 272, 0, 0, 2807, 15960, 0, 0, -874, 278, 10219, 7449, 5685,
	    21574, 4000, 250 }
	},
	{ LION, VN1, DEFAULT_TPLCD, GOLD,
	  { 50, 385, 4130, -719, 197, 4347, -440, -339, -1003, 7882, 910, -3350,
	    0, 22, 272, 0, 0, 2807, 15960, 0, 0, -874, 278, 10219, 7449, 5685,
	    21574, 4000, 250 }
	},
	{ LION, VN1, DEFAULT_TPLCD, WHITE,
	  { 50, 385, 4130, -719, 197, 4347, -440, -339, -1003, 7882, 910, -3350,
	    0, 22, 272, 0, 0, 2807, 15960, 0, 0, -874, 278, 10219, 7449, 5685,
	    21574, 4000, 250 }
	},
	{ ELSA, V3, DEFAULT_TPLCD, GRAY,
	  { 50, 369, 7014, -3002, 208, 6989, -2582, -353, 102, 5824, 0, 0,
	    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 256, 8510, 6316, 6009, 19287,
	    4000, 250 }
	},
	{ ELSA, V3, DEFAULT_TPLCD, BLACK,
	  { 50, 369, 7014, -3002, 208, 6989, -2582, -353, 102, 5824, 0, 0,
	    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 256, 8510, 6316, 6009, 19287,
	    4000, 250 }
	},
	{ ELSA, V3, DEFAULT_TPLCD, BLACK2,
	  { 50, 369, 7014, -3002, 208, 6989, -2582, -353, 102, 5824, 0, 0,
	    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 256, 8510, 6316, 6009, 19287,
	    4000, 250 }
	},
	{ ELSA, V3, DEFAULT_TPLCD, GOLD,
	  { 50, 369, 7014, -3002, 208, 6989, -2582, -353, 102, 5824, 0, 0,
	    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 256, 8510, 6316, 6009, 19287,
	    4000, 250 }
	},
	{ ELSA, V3, DEFAULT_TPLCD, WHITE,
	  { 50, 369, 7014, -3002, 208, 6989, -2582, -353, 102, 5824, 0, 0,
	    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 256, 8510, 6316, 6009, 19287,
	    4000, 250 }
	},
	{ ELSAP, V3, DEFAULT_TPLCD, GRAY,
	  { 50, 482, 5272, -1587, 311, 5311, -1254, -244, -843, 5875, 0, 0,
	    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 256, 8510, 6316, 6009, 19287,
	    4000, 250 }
	},
	{ ELSAP, V3, DEFAULT_TPLCD, BLACK,
	  { 50, 482, 5272, -1587, 311, 5311, -1254, -244, -843, 5875, 0, 0,
	    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 256, 8510, 6316, 6009, 19287,
	    4000, 250 }
	},
	{ ELSAP, V3, DEFAULT_TPLCD, BLACK2,
	  { 50, 482, 5272, -1587, 311, 5311, -1254, -244, -843, 5875, 0, 0,
	    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 256, 8510, 6316, 6009, 19287,
	    4000, 250 }
	},
	{ ELSAP, V3, DEFAULT_TPLCD, GOLD,
	  { 50, 482, 5272, -1587, 311, 5311, -1254, -244, -843, 5875, 0, 0,
	    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 256, 8510, 6316, 6009, 19287,
	    4000, 250 }
	},
	{ ELSAP, V3, DEFAULT_TPLCD, WHITE,
	  { 50, 482, 5272, -1587, 311, 5311, -1254, -244, -843, 5875, 0, 0,
	    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 256, 8510, 6316, 6009, 19287,
	    4000, 250 }
	},
};

tmd3702_als_para_table *als_get_tmd3702_table_by_id(uint32_t id)
{
	if (id >= ARRAY_SIZE(tmd3702_als_para_diff_tp_color_table))
		return NULL;
	return &(tmd3702_als_para_diff_tp_color_table[id]);
}

tmd3702_als_para_table *als_get_tmd3702_first_table(void)
{
	return &(tmd3702_als_para_diff_tp_color_table[0]);
}

uint32_t als_get_tmd3702_table_count(void)
{
	return ARRAY_SIZE(tmd3702_als_para_diff_tp_color_table);
}
