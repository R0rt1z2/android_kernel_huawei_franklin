/*
 * hi846_ofilm_sensor.h
 *
 * Copyright (c) 2018-2019 Huawei Technologies Co., Ltd.
 *
 * hi846_ofilm image sensor config settings
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 */
#ifndef _HI846_OFILM_SENSOR_H
#define _HI846_OFILM_SENSOR_H

#include "imgsensor_sensor_common.h"
#include "kd_imgsensor.h"

#define HI846_OTP_FUNCTION

#define SENSOR_FRM_LENGTH_LINES_REG_H 0x0006
#define SENSOR_FRM_LENGTH_LINES_REG_L 0x0007
#define SENSOR_LINE_LENGTH_PCK_REG_H 0x0008
#define SENSOR_LINE_LENGTH_PCK_REG_L 0x0009
#define SENSOR_IMAGE_ORIENTATION 0x000e
#define SENSOR_ISP_EN_REG_H 0x0a04
#define SENSOR_ISP_EN_REG_L 0x0a05
#define SENSOR_TP_MODE_REG 0x020a

#define SENSOR_INTEG_TIME_REG_H 0x0074
#define SENSOR_INTEG_TIME_REG_L 0x0075
#define SENSOR_ANA_GAIN_REG 0x0076

#define SENSOR_MODULE_ID_REG_H 0x0F17
#define SENSOR_MODULE_ID_REG_L 0x0F16

static struct imgsensor_i2c_reg stream_on[] = {
	{ 0x0a00, 0x0100, 0x00 },
};

static struct imgsensor_i2c_reg stream_off[] = {
	{ 0x0a00, 0x0000, 0x00 },
};

static struct imgsensor_i2c_reg init_setting[] = {
	{ 0x0066, 0x0101, 0x00 },
	{ 0x2000, 0x98F8, 0x00 },
	{ 0x2002, 0x00FF, 0x00 },
	{ 0x2004, 0x0006, 0x00 },
	{ 0x2008, 0x3FFF, 0x00 },
	{ 0x200A, 0xC314, 0x00 },
	{ 0x2022, 0x4130, 0x00 },
	{ 0x2034, 0x1292, 0x00 },
	{ 0x2036, 0xC02E, 0x00 },
	{ 0x2038, 0x4130, 0x00 },
	{ 0x206E, 0xF0B2, 0x00 },
	{ 0x2070, 0xFFBF, 0x00 },
	{ 0x2072, 0x2004, 0x00 },
	{ 0x2074, 0x43C2, 0x00 },
	{ 0x2076, 0x82FC, 0x00 },
	{ 0x2078, 0x12B0, 0x00 },
	{ 0x207A, 0xCAB0, 0x00 },
	{ 0x207C, 0x42A2, 0x00 },
	{ 0x207E, 0x7324, 0x00 },
	{ 0x2080, 0x9382, 0x00 },
	{ 0x2082, 0x826E, 0x00 },
	{ 0x2084, 0x2006, 0x00 },
	{ 0x2086, 0x421F, 0x00 },
	{ 0x2088, 0x0A84, 0x00 },
	{ 0x208A, 0xF03F, 0x00 },
	{ 0x208C, 0x0010, 0x00 },
	{ 0x208E, 0x4F82, 0x00 },
	{ 0x2090, 0x82E6, 0x00 },
	{ 0x2092, 0x4130, 0x00 },
	{ 0x2094, 0x120B, 0x00 },
	{ 0x2096, 0x425B, 0x00 },
	{ 0x2098, 0x008C, 0x00 },
	{ 0x209A, 0x4292, 0x00 },
	{ 0x209C, 0x7300, 0x00 },
	{ 0x209E, 0x82F4, 0x00 },
	{ 0x20A0, 0x4292, 0x00 },
	{ 0x20A2, 0x7302, 0x00 },
	{ 0x20A4, 0x82F6, 0x00 },
	{ 0x20A6, 0x1292, 0x00 },
	{ 0x20A8, 0xC006, 0x00 },
	{ 0x20AA, 0x421F, 0x00 },
	{ 0x20AC, 0x0710, 0x00 },
	{ 0x20AE, 0x523F, 0x00 },
	{ 0x20B0, 0x4F82, 0x00 },
	{ 0x20B2, 0x82E2, 0x00 },
	{ 0x20B4, 0x93C2, 0x00 },
	{ 0x20B6, 0x829F, 0x00 },
	{ 0x20B8, 0x241B, 0x00 },
	{ 0x20BA, 0x403E, 0x00 },
	{ 0x20BC, 0xFFFE, 0x00 },
	{ 0x20BE, 0x40B2, 0x00 },
	{ 0x20C0, 0xEC78, 0x00 },
	{ 0x20C2, 0x82EE, 0x00 },
	{ 0x20C4, 0x40B2, 0x00 },
	{ 0x20C6, 0xEC78, 0x00 },
	{ 0x20C8, 0x82F0, 0x00 },
	{ 0x20CA, 0x40B2, 0x00 },
	{ 0x20CC, 0xEC78, 0x00 },
	{ 0x20CE, 0x82F2, 0x00 },
	{ 0x20D0, 0x934B, 0x00 },
	{ 0x20D2, 0x2402, 0x00 },
	{ 0x20D4, 0x4E82, 0x00 },
	{ 0x20D6, 0x82EE, 0x00 },
	{ 0x20D8, 0x907B, 0x00 },
	{ 0x20DA, 0x0003, 0x00 },
	{ 0x20DC, 0x200B, 0x00 },
	{ 0x20DE, 0x421F, 0x00 },
	{ 0x20E0, 0x82EE, 0x00 },
	{ 0x20E2, 0x5E0F, 0x00 },
	{ 0x20E4, 0x4F82, 0x00 },
	{ 0x20E6, 0x82F0, 0x00 },
	{ 0x20E8, 0x5E0F, 0x00 },
	{ 0x20EA, 0x4F82, 0x00 },
	{ 0x20EC, 0x82F2, 0x00 },
	{ 0x20EE, 0x3C02, 0x00 },
	{ 0x20F0, 0x432E, 0x00 },
	{ 0x20F2, 0x3FE5, 0x00 },
	{ 0x20F4, 0x413B, 0x00 },
	{ 0x20F6, 0x4130, 0x00 },
	{ 0x20F8, 0x421F, 0x00 },
	{ 0x20FA, 0x7100, 0x00 },
	{ 0x20FC, 0x503F, 0x00 },
	{ 0x20FE, 0xFFD8, 0x00 },
	{ 0x2100, 0x4F82, 0x00 },
	{ 0x2102, 0x7A04, 0x00 },
	{ 0x2104, 0x421E, 0x00 },
	{ 0x2106, 0x82EE, 0x00 },
	{ 0x2108, 0x5F0E, 0x00 },
	{ 0x210A, 0x4E82, 0x00 },
	{ 0x210C, 0x7A06, 0x00 },
	{ 0x210E, 0x0B00, 0x00 },
	{ 0x2110, 0x7304, 0x00 },
	{ 0x2112, 0x0050, 0x00 },
	{ 0x2114, 0x40B2, 0x00 },
	{ 0x2116, 0xD081, 0x00 },
	{ 0x2118, 0x0B88, 0x00 },
	{ 0x211A, 0x421E, 0x00 },
	{ 0x211C, 0x82F0, 0x00 },
	{ 0x211E, 0x5F0E, 0x00 },
	{ 0x2120, 0x4E82, 0x00 },
	{ 0x2122, 0x7A0E, 0x00 },
	{ 0x2124, 0x521F, 0x00 },
	{ 0x2126, 0x82F2, 0x00 },
	{ 0x2128, 0x4F82, 0x00 },
	{ 0x212A, 0x7A10, 0x00 },
	{ 0x212C, 0x0B00, 0x00 },
	{ 0x212E, 0x7304, 0x00 },
	{ 0x2130, 0x007A, 0x00 },
	{ 0x2132, 0x40B2, 0x00 },
	{ 0x2134, 0x0081, 0x00 },
	{ 0x2136, 0x0B88, 0x00 },
	{ 0x2138, 0x4392, 0x00 },
	{ 0x213A, 0x7A0A, 0x00 },
	{ 0x213C, 0x0800, 0x00 },
	{ 0x213E, 0x7A0C, 0x00 },
	{ 0x2140, 0x0B00, 0x00 },
	{ 0x2142, 0x7304, 0x00 },
	{ 0x2144, 0x022B, 0x00 },
	{ 0x2146, 0x40B2, 0x00 },
	{ 0x2148, 0xD081, 0x00 },
	{ 0x214A, 0x0B88, 0x00 },
	{ 0x214C, 0x0B00, 0x00 },
	{ 0x214E, 0x7304, 0x00 },
	{ 0x2150, 0x0255, 0x00 },
	{ 0x2152, 0x40B2, 0x00 },
	{ 0x2154, 0x0081, 0x00 },
	{ 0x2156, 0x0B88, 0x00 },
	{ 0x2158, 0x9382, 0x00 },
	{ 0x215A, 0x7112, 0x00 },
	{ 0x215C, 0x2405, 0x00 },
	{ 0x215E, 0x4392, 0x00 },
	{ 0x2160, 0x760E, 0x00 },
	{ 0x2162, 0xF0B2, 0x00 },
	{ 0x2164, 0xFFEF, 0x00 },
	{ 0x2166, 0x0A84, 0x00 },
	{ 0x2168, 0x4130, 0x00 },
	{ 0x216A, 0x120B, 0x00 },
	{ 0x216C, 0x120A, 0x00 },
	{ 0x216E, 0x4E0A, 0x00 },
	{ 0x2170, 0x4F0B, 0x00 },
	{ 0x2172, 0x4C0E, 0x00 },
	{ 0x2174, 0x4D0F, 0x00 },
	{ 0x2176, 0x8A0E, 0x00 },
	{ 0x2178, 0x7B0F, 0x00 },
	{ 0x217A, 0x2C02, 0x00 },
	{ 0x217C, 0x4A0C, 0x00 },
	{ 0x217E, 0x4B0D, 0x00 },
	{ 0x2180, 0x4C0E, 0x00 },
	{ 0x2182, 0x4D0F, 0x00 },
	{ 0x2184, 0x413A, 0x00 },
	{ 0x2186, 0x413B, 0x00 },
	{ 0x2188, 0x4130, 0x00 },
	{ 0x218A, 0x120B, 0x00 },
	{ 0x218C, 0x120A, 0x00 },
	{ 0x218E, 0x1209, 0x00 },
	{ 0x2190, 0x1208, 0x00 },
	{ 0x2192, 0x1207, 0x00 },
	{ 0x2194, 0x42D2, 0x00 },
	{ 0x2196, 0x82FC, 0x00 },
	{ 0x2198, 0x82A0, 0x00 },
	{ 0x219A, 0x403B, 0x00 },
	{ 0x219C, 0x00C1, 0x00 },
	{ 0x219E, 0x4B6F, 0x00 },
	{ 0x21A0, 0x4FC2, 0x00 },
	{ 0x21A2, 0x82D4, 0x00 },
	{ 0x21A4, 0x43C2, 0x00 },
	{ 0x21A6, 0x82D5, 0x00 },
	{ 0x21A8, 0x1292, 0x00 },
	{ 0x21AA, 0xC046, 0x00 },
	{ 0x21AC, 0x4292, 0x00 },
	{ 0x21AE, 0x7560, 0x00 },
	{ 0x21B0, 0x82F8, 0x00 },
	{ 0x21B2, 0x4292, 0x00 },
	{ 0x21B4, 0x7562, 0x00 },
	{ 0x21B6, 0x82FA, 0x00 },
	{ 0x21B8, 0x93CB, 0x00 },
	{ 0x21BA, 0x0000, 0x00 },
	{ 0x21BC, 0x2450, 0x00 },
	{ 0x21BE, 0x4217, 0x00 },
	{ 0x21C0, 0x7316, 0x00 },
	{ 0x21C2, 0x4218, 0x00 },
	{ 0x21C4, 0x7318, 0x00 },
	{ 0x21C6, 0x421F, 0x00 },
	{ 0x21C8, 0x0710, 0x00 },
	{ 0x21CA, 0x4F0E, 0x00 },
	{ 0x21CC, 0x430F, 0x00 },
	{ 0x21CE, 0x4709, 0x00 },
	{ 0x21D0, 0x480A, 0x00 },
	{ 0x21D2, 0x8E09, 0x00 },
	{ 0x21D4, 0x7F0A, 0x00 },
	{ 0x21D6, 0x490B, 0x00 },
	{ 0x21D8, 0x4A0C, 0x00 },
	{ 0x21DA, 0x490D, 0x00 },
	{ 0x21DC, 0x4A0E, 0x00 },
	{ 0x21DE, 0x803D, 0x00 },
	{ 0x21E0, 0x0102, 0x00 },
	{ 0x21E2, 0x730E, 0x00 },
	{ 0x21E4, 0x2803, 0x00 },
	{ 0x21E6, 0x403B, 0x00 },
	{ 0x21E8, 0x0101, 0x00 },
	{ 0x21EA, 0x430C, 0x00 },
	{ 0x21EC, 0x4B0F, 0x00 },
	{ 0x21EE, 0x832F, 0x00 },
	{ 0x21F0, 0x43D2, 0x00 },
	{ 0x21F2, 0x01B3, 0x00 },
	{ 0x21F4, 0x4F82, 0x00 },
	{ 0x21F6, 0x7324, 0x00 },
	{ 0x21F8, 0x4292, 0x00 },
	{ 0x21FA, 0x7540, 0x00 },
	{ 0x21FC, 0x82EA, 0x00 },
	{ 0x21FE, 0x4292, 0x00 },
	{ 0x2200, 0x7542, 0x00 },
	{ 0x2202, 0x82EC, 0x00 },
	{ 0x2204, 0x434B, 0x00 },
	{ 0x2206, 0x823F, 0x00 },
	{ 0x2208, 0x4F0C, 0x00 },
	{ 0x220A, 0x430D, 0x00 },
	{ 0x220C, 0x421E, 0x00 },
	{ 0x220E, 0x82EA, 0x00 },
	{ 0x2210, 0x421F, 0x00 },
	{ 0x2212, 0x82EC, 0x00 },
	{ 0x2214, 0x5E0C, 0x00 },
	{ 0x2216, 0x6F0D, 0x00 },
	{ 0x2218, 0x890C, 0x00 },
	{ 0x221A, 0x7A0D, 0x00 },
	{ 0x221C, 0x2801, 0x00 },
	{ 0x221E, 0x435B, 0x00 },
	{ 0x2220, 0x4BC2, 0x00 },
	{ 0x2222, 0x82FC, 0x00 },
	{ 0x2224, 0x93C2, 0x00 },
	{ 0x2226, 0x829A, 0x00 },
	{ 0x2228, 0x201A, 0x00 },
	{ 0x222A, 0x93C2, 0x00 },
	{ 0x222C, 0x82A0, 0x00 },
	{ 0x222E, 0x2404, 0x00 },
	{ 0x2230, 0x43B2, 0x00 },
	{ 0x2232, 0x7540, 0x00 },
	{ 0x2234, 0x43B2, 0x00 },
	{ 0x2236, 0x7542, 0x00 },
	{ 0x2238, 0x93C2, 0x00 },
	{ 0x223A, 0x82FC, 0x00 },
	{ 0x223C, 0x2410, 0x00 },
	{ 0x223E, 0x503E, 0x00 },
	{ 0x2240, 0x0003, 0x00 },
	{ 0x2242, 0x630F, 0x00 },
	{ 0x2244, 0x4E82, 0x00 },
	{ 0x2246, 0x82EA, 0x00 },
	{ 0x2248, 0x4F82, 0x00 },
	{ 0x224A, 0x82EC, 0x00 },
	{ 0x224C, 0x470C, 0x00 },
	{ 0x224E, 0x480D, 0x00 },
	{ 0x2250, 0x8E0C, 0x00 },
	{ 0x2252, 0x7F0D, 0x00 },
	{ 0x2254, 0x2C04, 0x00 },
	{ 0x2256, 0x4782, 0x00 },
	{ 0x2258, 0x82EA, 0x00 },
	{ 0x225A, 0x4882, 0x00 },
	{ 0x225C, 0x82EC, 0x00 },
	{ 0x225E, 0x4137, 0x00 },
	{ 0x2260, 0x4138, 0x00 },
	{ 0x2262, 0x4139, 0x00 },
	{ 0x2264, 0x413A, 0x00 },
	{ 0x2266, 0x413B, 0x00 },
	{ 0x2268, 0x4130, 0x00 },
	{ 0x226A, 0x403E, 0x00 },
	{ 0x226C, 0x00C2, 0x00 },
	{ 0x226E, 0x421F, 0x00 },
	{ 0x2270, 0x7314, 0x00 },
	{ 0x2272, 0xF07F, 0x00 },
	{ 0x2274, 0x000C, 0x00 },
	{ 0x2276, 0x5F4F, 0x00 },
	{ 0x2278, 0x5F4F, 0x00 },
	{ 0x227A, 0xDFCE, 0x00 },
	{ 0x227C, 0x0000, 0x00 },
	{ 0x227E, 0xF0FE, 0x00 },
	{ 0x2280, 0x000F, 0x00 },
	{ 0x2282, 0x0000, 0x00 },
	{ 0x2284, 0x4130, 0x00 },
	{ 0x2286, 0x120B, 0x00 },
	{ 0x2288, 0x120A, 0x00 },
	{ 0x228A, 0x1209, 0x00 },
	{ 0x228C, 0x1208, 0x00 },
	{ 0x228E, 0x1207, 0x00 },
	{ 0x2290, 0x1206, 0x00 },
	{ 0x2292, 0x93C2, 0x00 },
	{ 0x2294, 0x00C1, 0x00 },
	{ 0x2296, 0x249F, 0x00 },
	{ 0x2298, 0x425E, 0x00 },
	{ 0x229A, 0x00C2, 0x00 },
	{ 0x229C, 0xC35E, 0x00 },
	{ 0x229E, 0x425F, 0x00 },
	{ 0x22A0, 0x82A0, 0x00 },
	{ 0x22A2, 0xDF4E, 0x00 },
	{ 0x22A4, 0x4EC2, 0x00 },
	{ 0x22A6, 0x00C2, 0x00 },
	{ 0x22A8, 0x934F, 0x00 },
	{ 0x22AA, 0x248F, 0x00 },
	{ 0x22AC, 0x4217, 0x00 },
	{ 0x22AE, 0x7316, 0x00 },
	{ 0x22B0, 0x4218, 0x00 },
	{ 0x22B2, 0x7318, 0x00 },
	{ 0x22B4, 0x4326, 0x00 },
	{ 0x22B6, 0xB3E2, 0x00 },
	{ 0x22B8, 0x00C2, 0x00 },
	{ 0x22BA, 0x2482, 0x00 },
	{ 0x22BC, 0x0900, 0x00 },
	{ 0x22BE, 0x731C, 0x00 },
	{ 0x22C0, 0x0800, 0x00 },
	{ 0x22C2, 0x731C, 0x00 },
	{ 0x22C4, 0x421A, 0x00 },
	{ 0x22C6, 0x7300, 0x00 },
	{ 0x22C8, 0x421B, 0x00 },
	{ 0x22CA, 0x7302, 0x00 },
	{ 0x22CC, 0x421F, 0x00 },
	{ 0x22CE, 0x7304, 0x00 },
	{ 0x22D0, 0x9F82, 0x00 },
	{ 0x22D2, 0x829C, 0x00 },
	{ 0x22D4, 0x2C02, 0x00 },
	{ 0x22D6, 0x531A, 0x00 },
	{ 0x22D8, 0x630B, 0x00 },
	{ 0x22DA, 0x4A0E, 0x00 },
	{ 0x22DC, 0x4B0F, 0x00 },
	{ 0x22DE, 0x821E, 0x00 },
	{ 0x22E0, 0x82F4, 0x00 },
	{ 0x22E2, 0x721F, 0x00 },
	{ 0x22E4, 0x82F6, 0x00 },
	{ 0x22E6, 0x2C68, 0x00 },
	{ 0x22E8, 0x4A09, 0x00 },
	{ 0x22EA, 0x9339, 0x00 },
	{ 0x22EC, 0x3460, 0x00 },
	{ 0x22EE, 0x0B00, 0x00 },
	{ 0x22F0, 0x7304, 0x00 },
	{ 0x22F2, 0x0320, 0x00 },
	{ 0x22F4, 0x421E, 0x00 },
	{ 0x22F6, 0x7300, 0x00 },
	{ 0x22F8, 0x421F, 0x00 },
	{ 0x22FA, 0x7302, 0x00 },
	{ 0x22FC, 0x531E, 0x00 },
	{ 0x22FE, 0x630F, 0x00 },
	{ 0x2300, 0x4E0C, 0x00 },
	{ 0x2302, 0x4F0D, 0x00 },
	{ 0x2304, 0x821C, 0x00 },
	{ 0x2306, 0x82F8, 0x00 },
	{ 0x2308, 0x721D, 0x00 },
	{ 0x230A, 0x82FA, 0x00 },
	{ 0x230C, 0x2C0E, 0x00 },
	{ 0x230E, 0x93B2, 0x00 },
	{ 0x2310, 0x7560, 0x00 },
	{ 0x2312, 0x2003, 0x00 },
	{ 0x2314, 0x93B2, 0x00 },
	{ 0x2316, 0x7562, 0x00 },
	{ 0x2318, 0x2408, 0x00 },
	{ 0x231A, 0x4E82, 0x00 },
	{ 0x231C, 0x7540, 0x00 },
	{ 0x231E, 0x4F82, 0x00 },
	{ 0x2320, 0x7542, 0x00 },
	{ 0x2322, 0x4E82, 0x00 },
	{ 0x2324, 0x82F8, 0x00 },
	{ 0x2326, 0x4F82, 0x00 },
	{ 0x2328, 0x82FA, 0x00 },
	{ 0x232A, 0x4E82, 0x00 },
	{ 0x232C, 0x7316, 0x00 },
	{ 0x232E, 0x12B0, 0x00 },
	{ 0x2330, 0xFE6A, 0x00 },
	{ 0x2332, 0x0900, 0x00 },
	{ 0x2334, 0x730E, 0x00 },
	{ 0x2336, 0x403F, 0x00 },
	{ 0x2338, 0x7316, 0x00 },
	{ 0x233A, 0x4A09, 0x00 },
	{ 0x233C, 0x8F29, 0x00 },
	{ 0x233E, 0x478F, 0x00 },
	{ 0x2340, 0x0000, 0x00 },
	{ 0x2342, 0x460C, 0x00 },
	{ 0x2344, 0x430D, 0x00 },
	{ 0x2346, 0x421E, 0x00 },
	{ 0x2348, 0x7300, 0x00 },
	{ 0x234A, 0x421F, 0x00 },
	{ 0x234C, 0x7302, 0x00 },
	{ 0x234E, 0x9C0E, 0x00 },
	{ 0x2350, 0x23F8, 0x00 },
	{ 0x2352, 0x9D0F, 0x00 },
	{ 0x2354, 0x23F6, 0x00 },
	{ 0x2356, 0x0B00, 0x00 },
	{ 0x2358, 0x7304, 0x00 },
	{ 0x235A, 0x01F4, 0x00 },
	{ 0x235C, 0x5036, 0x00 },
	{ 0x235E, 0x0006, 0x00 },
	{ 0x2360, 0x460C, 0x00 },
	{ 0x2362, 0x430D, 0x00 },
	{ 0x2364, 0x490E, 0x00 },
	{ 0x2366, 0x4E0F, 0x00 },
	{ 0x2368, 0x5F0F, 0x00 },
	{ 0x236A, 0x7F0F, 0x00 },
	{ 0x236C, 0xE33F, 0x00 },
	{ 0x236E, 0x521E, 0x00 },
	{ 0x2370, 0x82EA, 0x00 },
	{ 0x2372, 0x621F, 0x00 },
	{ 0x2374, 0x82EC, 0x00 },
	{ 0x2376, 0x12B0, 0x00 },
	{ 0x2378, 0xFD6A, 0x00 },
	{ 0x237A, 0x4E82, 0x00 },
	{ 0x237C, 0x7540, 0x00 },
	{ 0x237E, 0x4F82, 0x00 },
	{ 0x2380, 0x7542, 0x00 },
	{ 0x2382, 0x403B, 0x00 },
	{ 0x2384, 0x7316, 0x00 },
	{ 0x2386, 0x421C, 0x00 },
	{ 0x2388, 0x82E2, 0x00 },
	{ 0x238A, 0x430D, 0x00 },
	{ 0x238C, 0x4B2F, 0x00 },
	{ 0x238E, 0x590F, 0x00 },
	{ 0x2390, 0x4F0E, 0x00 },
	{ 0x2392, 0x430F, 0x00 },
	{ 0x2394, 0x12B0, 0x00 },
	{ 0x2396, 0xFD6A, 0x00 },
	{ 0x2398, 0x4E8B, 0x00 },
	{ 0x239A, 0x0000, 0x00 },
	{ 0x239C, 0x4BA2, 0x00 },
	{ 0x239E, 0x82CE, 0x00 },
	{ 0x23A0, 0x4382, 0x00 },
	{ 0x23A2, 0x82D0, 0x00 },
	{ 0x23A4, 0x12B0, 0x00 },
	{ 0x23A6, 0xFE6A, 0x00 },
	{ 0x23A8, 0xD3D2, 0x00 },
	{ 0x23AA, 0x00C2, 0x00 },
	{ 0x23AC, 0x3C16, 0x00 },
	{ 0x23AE, 0x9329, 0x00 },
	{ 0x23B0, 0x3BC8, 0x00 },
	{ 0x23B2, 0x4906, 0x00 },
	{ 0x23B4, 0x5326, 0x00 },
	{ 0x23B6, 0x3FC5, 0x00 },
	{ 0x23B8, 0x4A09, 0x00 },
	{ 0x23BA, 0x8219, 0x00 },
	{ 0x23BC, 0x82CE, 0x00 },
	{ 0x23BE, 0x3F95, 0x00 },
	{ 0x23C0, 0x0800, 0x00 },
	{ 0x23C2, 0x731C, 0x00 },
	{ 0x23C4, 0x0900, 0x00 },
	{ 0x23C6, 0x731C, 0x00 },
	{ 0x23C8, 0x3F7D, 0x00 },
	{ 0x23CA, 0x0900, 0x00 },
	{ 0x23CC, 0x730C, 0x00 },
	{ 0x23CE, 0x0B00, 0x00 },
	{ 0x23D0, 0x7304, 0x00 },
	{ 0x23D2, 0x01F4, 0x00 },
	{ 0x23D4, 0x3FE9, 0x00 },
	{ 0x23D6, 0x0900, 0x00 },
	{ 0x23D8, 0x732C, 0x00 },
	{ 0x23DA, 0x425F, 0x00 },
	{ 0x23DC, 0x0788, 0x00 },
	{ 0x23DE, 0x4136, 0x00 },
	{ 0x23E0, 0x4137, 0x00 },
	{ 0x23E2, 0x4138, 0x00 },
	{ 0x23E4, 0x4139, 0x00 },
	{ 0x23E6, 0x413A, 0x00 },
	{ 0x23E8, 0x413B, 0x00 },
	{ 0x23EA, 0x4130, 0x00 },
	{ 0x23EC, 0x1292, 0x00 },
	{ 0x23EE, 0xC040, 0x00 },
	{ 0x23F0, 0xD292, 0x00 },
	{ 0x23F2, 0x82E6, 0x00 },
	{ 0x23F4, 0x0A84, 0x00 },
	{ 0x23F6, 0x4130, 0x00 },
	{ 0x23FE, 0xC056, 0x00 },
	{ 0x3236, 0xFC22, 0x00 },
	{ 0x3238, 0xFFEC, 0x00 },
	{ 0x323A, 0xFCF8, 0x00 },
	{ 0x323C, 0xFC94, 0x00 },
	{ 0x323E, 0xFD8A, 0x00 },
	{ 0x3246, 0xFE86, 0x00 },
	{ 0x3248, 0xFC34, 0x00 },
	{ 0x324E, 0xFC6E, 0x00 },
	{ 0x326A, 0xC374, 0x00 },
	{ 0x326C, 0xC37C, 0x00 },
	{ 0x326E, 0x0000, 0x00 },
	{ 0x3270, 0xC378, 0x00 },
	{ 0x0A00, 0x0000, 0x00 },
	{ 0x0E04, 0x0012, 0x00 },
	{ 0x002E, 0x1111, 0x00 },
	{ 0x0032, 0x1111, 0x00 },
	{ 0x0022, 0x0008, 0x00 },
	{ 0x0026, 0x0040, 0x00 },
	{ 0x0028, 0x0017, 0x00 },
	{ 0x002C, 0x09CF, 0x00 },
	{ 0x005C, 0x2101, 0x00 },
	{ 0x0006, 0x09DC, 0x00 },
	{ 0x0008, 0x0ED8, 0x00 },
	{ 0x000E, 0x0100, 0x00 },
	{ 0x000C, 0x0022, 0x00 },
	{ 0x0A22, 0x0000, 0x00 },
	{ 0x0A24, 0x0000, 0x00 },
	{ 0x0804, 0x0000, 0x00 },
	{ 0x0A12, 0x0CC0, 0x00 },
	{ 0x0A14, 0x0990, 0x00 },
	{ 0x0710, 0x09B0, 0x00 },
	{ 0x0074, 0x09CF, 0x00 },
	{ 0x0076, 0x0000, 0x00 },
	{ 0x051E, 0x0000, 0x00 },
	{ 0x0200, 0x0400, 0x00 },
	{ 0x0A1A, 0x0C00, 0x00 },
	{ 0x0A0C, 0x0010, 0x00 },
	{ 0x0A1E, 0x0CCF, 0x00 },
	{ 0x0402, 0x0110, 0x00 },
	{ 0x0404, 0x00F4, 0x00 },
	{ 0x0408, 0x0000, 0x00 },
	{ 0x0410, 0x008D, 0x00 },
	{ 0x0412, 0x011A, 0x00 },
	{ 0x0414, 0x864C, 0x00 },
	{ 0x021C, 0x0001, 0x00 },
	{ 0x0C00, 0x9950, 0x00 },
	{ 0x0C06, 0x0021, 0x00 },
	{ 0x0C10, 0x0040, 0x00 },
	{ 0x0C12, 0x0040, 0x00 },
	{ 0x0C14, 0x0040, 0x00 },
	{ 0x0C16, 0x0040, 0x00 },
	{ 0x0A02, 0x0100, 0x00 },
	{ 0x0A04, 0x014A, 0x00 },
	{ 0x0418, 0x0000, 0x00 },
	{ 0x0128, 0x0028, 0x00 },
	{ 0x012A, 0xFFFF, 0x00 },
	{ 0x0120, 0x0046, 0x00 },
	{ 0x0122, 0x0376, 0x00 },
	{ 0x012C, 0x0020, 0x00 },
	{ 0x012E, 0xFFFF, 0x00 },
	{ 0x0124, 0x0040, 0x00 },
	{ 0x0126, 0x0378, 0x00 },
	{ 0x0746, 0x0050, 0x00 },
	{ 0x0748, 0x01D5, 0x00 },
	{ 0x074A, 0x022B, 0x00 },
	{ 0x074C, 0x03B0, 0x00 },
	{ 0x0756, 0x043F, 0x00 },
	{ 0x0758, 0x3F1D, 0x00 },
	{ 0x0B02, 0xE04D, 0x00 },
	{ 0x0B10, 0x6821, 0x00 },
	{ 0x0B12, 0x0120, 0x00 },
	{ 0x0B14, 0x0001, 0x00 },
	{ 0x2008, 0x38FD, 0x00 },
	{ 0x326E, 0x0000, 0x00 },
	{ 0x0900, 0x0320, 0x00 },
	{ 0x0902, 0xC31A, 0x00 },
	{ 0x0914, 0xC109, 0x00 },
	{ 0x0916, 0x061A, 0x00 },
	{ 0x0918, 0x0306, 0x00 },
	{ 0x091A, 0x0B09, 0x00 },
	{ 0x091C, 0x0C07, 0x00 },
	{ 0x091E, 0x0A00, 0x00 },
	{ 0x090C, 0x042A, 0x00 },
	{ 0x090E, 0x005B, 0x00 },
	{ 0x0954, 0x0089, 0x00 },
	{ 0x0956, 0x0000, 0x00 },
	{ 0x0958, 0xCA00, 0x00 },
	{ 0x095A, 0x9240, 0x00 },
	{ 0x0040, 0x0200, 0x00 },
	{ 0x0042, 0x0100, 0x00 },
	{ 0x0D04, 0x0000, 0x00 },
	{ 0x0F08, 0x2F04, 0x00 },
	{ 0x0F30, 0x001F, 0x00 },
	{ 0x0F36, 0x001F, 0x00 },
	{ 0x0F04, 0x3A00, 0x00 },
	{ 0x0F32, 0x025A, 0x00 },
	{ 0x0F38, 0x0259, 0x00 },
	{ 0x0F2A, 0x0024, 0x00 },
	{ 0x006A, 0x0100, 0x00 },
	{ 0x004C, 0x0100, 0x00 },
	{ 0x0044, 0x0001, 0x00 },
};

static struct imgsensor_i2c_reg preview_setting[] = {
	{ 0x002E, 0x3311, 0x00 },
	{ 0x0032, 0x3311, 0x00 },
	{ 0x0026, 0x0040, 0x00 },
	{ 0x002C, 0x09CF, 0x00 },
	{ 0x005C, 0x4202, 0x00 },
	{ 0x0006, 0x09C5, 0x00 },
	{ 0x0008, 0x0ED8, 0x00 },
	{ 0x000C, 0x0122, 0x00 },
	{ 0x0A22, 0x0100, 0x00 },
	{ 0x0A24, 0x0000, 0x00 },
	{ 0x0804, 0x0000, 0x00 },
	{ 0x0A12, 0x0660, 0x00 },
	{ 0x0A14, 0x04C8, 0x00 },
	{ 0x0074, 0x09BF, 0x00 },
	{ 0x021C, 0x0001, 0x00 },
	{ 0x0A04, 0x016A, 0x00 },
	{ 0x0418, 0x0000, 0x00 },
	{ 0x0128, 0x0028, 0x00 },
	{ 0x012A, 0xFFFF, 0x00 },
	{ 0x0120, 0x0046, 0x00 },
	{ 0x0122, 0x0376, 0x00 },
	{ 0x012C, 0x0020, 0x00 },
	{ 0x012E, 0xFFFF, 0x00 },
	{ 0x0124, 0x0040, 0x00 },
	{ 0x0126, 0x0378, 0x00 },
	{ 0x0B02, 0xE04D, 0x00 },
	{ 0x0B10, 0x6C21, 0x00 },
	{ 0x0B12, 0x0120, 0x00 },
	{ 0x0B14, 0x0005, 0x00 },
	{ 0x2008, 0x38FD, 0x00 },
	{ 0x326E, 0x0000, 0x00 },
	{ 0x0710, 0x04E0, 0x00 },
	/* MIPI 2lane 712bps */
	{ 0x0900, 0x0300, 0x00 },
	{ 0x0902, 0x4319, 0x00 },
	{ 0x0914, 0xC109, 0x00 },
	{ 0x0916, 0x061A, 0x00 },
	{ 0x0918, 0x0407, 0x00 },
	{ 0x091A, 0x0A0B, 0x00 },
	{ 0x091C, 0x0E08, 0x00 },
	{ 0x091E, 0x0A00, 0x00 },
	{ 0x090C, 0x0427, 0x00 },
	{ 0x090E, 0x0059, 0x00 },
	{ 0x0954, 0x0089, 0x00 },
	{ 0x0956, 0x0000, 0x00 },
	{ 0x0958, 0xCA80, 0x00 },
	{ 0x095A, 0x9240, 0x00 },
	{ 0x0F32, 0x025A, 0x00 },
	{ 0x0F38, 0x0259, 0x00 },
	{ 0x0F2A, 0x4124, 0x00 },
	{ 0x004C, 0x0100, 0x00 },
};

static struct imgsensor_i2c_reg capture_setting[] = {
	{ 0x002E, 0x1111, 0x00 },
	{ 0x0032, 0x1111, 0x00 },
	{ 0x0026, 0x0040, 0x00 },
	{ 0x002C, 0x09CF, 0x00 },
	{ 0x005C, 0x2101, 0x00 },
	{ 0x0006, 0x09DC, 0x00 },
	{ 0x0008, 0x0ED8, 0x00 },
	{ 0x000C, 0x0022, 0x00 },
	{ 0x0A22, 0x0000, 0x00 },
	{ 0x0A24, 0x0000, 0x00 },
	{ 0x0804, 0x0000, 0x00 },
	{ 0x0A12, 0x0CC0, 0x00 },
	{ 0x0A14, 0x0990, 0x00 },
	{ 0x0074, 0x09D6, 0x00 },
	{ 0x021C, 0x0001, 0x00 },
	{ 0x0A04, 0x014A, 0x00 },
	{ 0x0418, 0x0000, 0x00 },
	{ 0x0128, 0x0028, 0x00 },
	{ 0x012A, 0xFFFF, 0x00 },
	{ 0x0120, 0x0046, 0x00 },
	{ 0x0122, 0x0376, 0x00 },
	{ 0x012C, 0x0020, 0x00 },
	{ 0x012E, 0xFFFF, 0x00 },
	{ 0x0124, 0x0040, 0x00 },
	{ 0x0126, 0x0378, 0x00 },
	{ 0x0B02, 0xE04D, 0x00 },
	{ 0x0B10, 0x6821, 0x00 },
	{ 0x0B12, 0x0120, 0x00 },
	{ 0x0B14, 0x0001, 0x00 },
	{ 0x2008, 0x38FD, 0x00 },
	{ 0x326E, 0x0000, 0x00 },
	{ 0x0710, 0x09B0, 0x00 },
	/* MIPI 2lane 1424bps */
	{ 0x0900, 0x0320, 0x00 },
	{ 0x0902, 0xC31A, 0x00 },
	{ 0x0914, 0xC109, 0x00 },
	{ 0x0916, 0x061A, 0x00 },
	{ 0x0918, 0x0306, 0x00 },
	{ 0x091A, 0x0B09, 0x00 },
	{ 0x091C, 0x0C07, 0x00 },
	{ 0x091E, 0x0A00, 0x00 },
	{ 0x090C, 0x042A, 0x00 },
	{ 0x090E, 0x005B, 0x00 },
	{ 0x0954, 0x0089, 0x00 },
	{ 0x0956, 0x0000, 0x00 },
	{ 0x0958, 0xCA00, 0x00 },
	{ 0x095A, 0x9240, 0x00 },
	{ 0x0F32, 0x025A, 0x00 },
	{ 0x0F38, 0x0259, 0x00 },
	{ 0x0F2A, 0x0024, 0x00 },
	{ 0x004C, 0x0100, 0x00 },
};

static struct imgsensor_i2c_reg video_setting[] = {
	{ 0x002E, 0x1111, 0x00 },
	{ 0x0032, 0x1111, 0x00 },
	{ 0x0026, 0x0040, 0x00 },
	{ 0x002C, 0x09CF, 0x00 },
	{ 0x005C, 0x2101, 0x00 },
	{ 0x0006, 0x09DC, 0x00 },
	{ 0x0008, 0x0ED8, 0x00 },
	{ 0x000C, 0x0022, 0x00 },
	{ 0x0A22, 0x0000, 0x00 },
	{ 0x0A24, 0x0000, 0x00 },
	{ 0x0804, 0x0000, 0x00 },
	{ 0x0A12, 0x0CC0, 0x00 },
	{ 0x0A14, 0x0990, 0x00 },
	{ 0x0074, 0x09D6, 0x00 },
	{ 0x021C, 0x0001, 0x00 },
	{ 0x0A04, 0x014A, 0x00 },
	{ 0x0418, 0x0000, 0x00 },
	{ 0x0128, 0x0028, 0x00 },
	{ 0x012A, 0xFFFF, 0x00 },
	{ 0x0120, 0x0046, 0x00 },
	{ 0x0122, 0x0376, 0x00 },
	{ 0x012C, 0x0020, 0x00 },
	{ 0x012E, 0xFFFF, 0x00 },
	{ 0x0124, 0x0040, 0x00 },
	{ 0x0126, 0x0378, 0x00 },
	{ 0x0B02, 0xE04D, 0x00 },
	{ 0x0B10, 0x6821, 0x00 },
	{ 0x0B12, 0x0120, 0x00 },
	{ 0x0B14, 0x0001, 0x00 },
	{ 0x2008, 0x38FD, 0x00 },
	{ 0x326E, 0x0000, 0x00 },
	{ 0x0710, 0x09B0, 0x00 },
	/* MIPI 2lane 1424Mbps */
	{ 0x0900, 0x0320, 0x00 },
	{ 0x0902, 0xC31A, 0x00 },
	{ 0x0914, 0xC109, 0x00 },
	{ 0x0916, 0x061A, 0x00 },
	{ 0x0918, 0x0306, 0x00 },
	{ 0x091A, 0x0B09, 0x00 },
	{ 0x091C, 0x0C07, 0x00 },
	{ 0x091E, 0x0A00, 0x00 },
	{ 0x090C, 0x042A, 0x00 },
	{ 0x090E, 0x005B, 0x00 },
	{ 0x0954, 0x0089, 0x00 },
	{ 0x0956, 0x0000, 0x00 },
	{ 0x0958, 0xCA00, 0x00 },
	{ 0x095A, 0x9240, 0x00 },
	{ 0x0F32, 0x025A, 0x00 },
	{ 0x0F38, 0x0259, 0x00 },
	{ 0x0F2A, 0x0024, 0x00 },
	{ 0x004C, 0x0100, 0x00 },
};


static struct imgsensor_i2c_reg_table dump_setting[] = {
	{ 0x0934, 0x0000, IMGSENSOR_I2C_WORD_DATA, IMGSENSOR_I2C_READ, 0 },
	{ 0x0a00, 0x0000, IMGSENSOR_I2C_WORD_DATA, IMGSENSOR_I2C_READ, 0 },
};

static struct imgsensor_info_t imgsensor_info = {
	.sensor_id_reg = 0x0F16,
	.sensor_id = HI846_OFILM_SENSOR_ID,
	.checksum_value = 0xdf4593fd,

	.pre = {
		.pclk = 288000000,
		.linelength = 3800,
		.framelength = 2501,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1632,
		.grabwindow_height = 1224,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
		.mipi_pixel_rate = 142400000,
	},
	.cap = {
		.pclk = 288000000,
		.linelength = 3800,
		.framelength = 2524,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 3264,
		.grabwindow_height = 2448,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
		.mipi_pixel_rate = 284800000,
	},
	.normal_video = {
		.pclk = 288000000,
		.linelength = 3800,
		.framelength = 2524,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 3264,
		.grabwindow_height = 2448,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
		.mipi_pixel_rate = 284800000,
	},
	.init_setting = {
		.setting = init_setting,
		.size = IMGSENSOR_ARRAY_SIZE(init_setting),
		.addr_type = IMGSENSOR_I2C_WORD_ADDR,
		.data_type = IMGSENSOR_I2C_WORD_DATA,
		.delay = 0,
	},
	.pre_setting = {
		.setting = preview_setting,
		.size = IMGSENSOR_ARRAY_SIZE(preview_setting),
		.addr_type = IMGSENSOR_I2C_WORD_ADDR,
		.data_type = IMGSENSOR_I2C_WORD_DATA,
		.delay = 0,
	},
	.cap_setting = {
		.setting = capture_setting,
		.size = IMGSENSOR_ARRAY_SIZE(capture_setting),
		.addr_type = IMGSENSOR_I2C_WORD_ADDR,
		.data_type = IMGSENSOR_I2C_WORD_DATA,
		.delay = 0,
	},
	.normal_video_setting = {
		.setting = video_setting,
		.size = IMGSENSOR_ARRAY_SIZE(video_setting),
		.addr_type = IMGSENSOR_I2C_WORD_ADDR,
		.data_type = IMGSENSOR_I2C_WORD_DATA,
		.delay = 0,
	},
	.streamon_setting = {
		.setting = stream_on,
		.size = IMGSENSOR_ARRAY_SIZE(stream_on),
		.addr_type = IMGSENSOR_I2C_WORD_ADDR,
		.data_type = IMGSENSOR_I2C_WORD_DATA,
		.delay = 0,
	},
	.streamoff_setting = {
		.setting = stream_off,
		.size = IMGSENSOR_ARRAY_SIZE(stream_off),
		.addr_type = IMGSENSOR_I2C_WORD_ADDR,
		.data_type = IMGSENSOR_I2C_WORD_DATA,
		.delay = 0,
	},
	.dump_info = {
		.setting = dump_setting,
		.size = IMGSENSOR_ARRAY_SIZE(dump_setting),
	},

	.margin = 6,
	.min_shutter = 6,
	.max_frame_length = 0xffff,
	.ae_shut_delay_frame = 0,
	.ae_sensor_gain_delay_frame = 0,
	.ae_ispGain_delay_frame = 2,
	.ihdr_support = 0,
	.ihdr_le_firstline = 0,
	.sensor_mode_num = 3,

	.cap_delay_frame = 3,
	.pre_delay_frame = 3,
	.video_delay_frame = 3,
	.hs_video_delay_frame = 3,
	.slim_video_delay_frame = 3,

	.isp_driving_current = ISP_DRIVING_4MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	.mipi_sensor_type = MIPI_OPHY_NCSI2,
	.mipi_settle_delay_mode = 1,
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_Gb,
	.mclk = 24,
	.mipi_lane_num = SENSOR_MIPI_2_LANE,
	.i2c_addr_table = { 0x40, 0xff },
	.addr_type = IMGSENSOR_I2C_WORD_ADDR,
};

static struct imgsensor_t imgsensor = {
	.mirror = IMAGE_H_MIRROR,
	.sensor_mode = IMGSENSOR_MODE_INIT,
	.shutter = 0x0100,
	.gain = 0xe0,
	.dummy_pixel = 0,
	.dummy_line = 0,
	.current_fps = 300,
	.autoflicker_en = KAL_FALSE,
	.test_pattern = KAL_FALSE,
	.current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,
	.ihdr_en = 0,
	.i2c_write_id = 0x40,
};


/* Sensor output window information */
static struct SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[3] = {
	/* preview */
	{ 3264, 2448, 0, 0, 3264, 2448, 1632, 1224, 0, 0, 1632, 1224, 0, 0, 1632, 1224 },
	/* capture */
	{ 3264, 2448, 0, 0, 3264, 2448, 3264, 2448, 0, 0, 3264, 2448, 0, 0, 3264, 2448 },
	/* video */
	{ 3264, 2448, 0, 0, 3264, 2448, 3264, 2448, 0, 0, 3264, 2448, 0, 0, 3264, 2448 },
};

#endif
