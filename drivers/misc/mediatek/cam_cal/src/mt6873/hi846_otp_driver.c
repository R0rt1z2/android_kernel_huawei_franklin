/*
 * hi846_otp_driver.c
 *
 * Copyright (c) 2020-2020 Huawei Technologies Co., Ltd.
 *
 * camera hi846_otp_drive
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

#define PFX "CAM_CAL"
#define pr_fmt(fmt) PFX "[%s] " fmt, __func__


#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/of.h>
#include "cam_cal.h"
#include "cam_cal_define.h"
#include "cam_cal_list.h"
#include <linux/dma-mapping.h>
#ifdef CONFIG_COMPAT
/* 64 bit */
#include <linux/fs.h>
#include <linux/compat.h>
#endif

#include <securec.h>

#include "eeprom_hw_map.h"
#include "hi846_otp_driver.h"

#define Sleep(ms) mdelay(ms)

#define LOG_INF(format, args...)    \
	pr_info(PFX "[%s] " format, __func__, ##args)

// Refer to CalLayoutTbl
#define OFFSET_AWB 	0x07
#define OFFSET_LSC	 0x17

#define SWAP16(x)  (((x)&0xFF)<<8) | ((x)>>8)
static struct i2c_client *g_pstI2CclientG;

static int init_flag = 0;

struct hi846_otp_t hi846_otp_info = {0};

static int iReadRegI2C(u8 *a_pSendData, u16 a_sizeSendData,
		u8 *a_pRecvData, u16 a_sizeRecvData){
	int  i4RetValue = 0;

	i4RetValue = i2c_master_send(g_pstI2CclientG,
		a_pSendData, a_sizeSendData);
	if (i4RetValue != a_sizeSendData) {
		pr_debug("I2C send failed!!, Addr = 0x%x\n", a_pSendData[0]);
		return -1;
	}
	i4RetValue = i2c_master_recv(g_pstI2CclientG,
		(char *)a_pRecvData, a_sizeRecvData);
	if (i4RetValue != a_sizeRecvData) {
		pr_debug("I2C read failed!!\n");
		return -1;
	}
	return 0;
}
static int iWriteRegI2C(u8 *a_pSendData, u16 a_sizeSendData){
	int  i4RetValue = 0;

	i4RetValue = i2c_master_send(g_pstI2CclientG,
		a_pSendData, a_sizeSendData);
	if (i4RetValue != a_sizeSendData) {
		pr_debug("I2C send failed!!, Addr = 0x%x\n", a_pSendData[0]);
		return -1;
	}
	return 0;

}
static kal_uint16 read_cmos_sensor(kal_uint32 addr){
	kal_uint16 get_byte = 0;
	char pu_send_cmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };
	iReadRegI2C(pu_send_cmd, 2, (u8 *)&get_byte, 1);
	return get_byte;
}
static void write_cmos_sensor_8(kal_uint32 addr, kal_uint32 para){
	char pu_send_cmd[4] = {(char)(addr >> 8),
		(char)(addr & 0xFF), (char)(para & 0xFF)};

	iWriteRegI2C(pu_send_cmd, 3);
}
static void hi846_otp_enable(void){
	write_cmos_sensor_8(0x0A02, 0x01);
	write_cmos_sensor_8(0x0A00, 0x00);
	Sleep(10);
	write_cmos_sensor_8(0x0F02, 0x00);
	write_cmos_sensor_8(0x071A, 0x01);
	write_cmos_sensor_8(0x071B, 0x09);
	write_cmos_sensor_8(0x0D04, 0x00);
	write_cmos_sensor_8(0x0D00, 0x07);
	write_cmos_sensor_8(0x003E, 0x10);
	write_cmos_sensor_8(0x0A00, 0x01);
}
static void hi846_otp_disable(void){
	write_cmos_sensor_8(0x0A00, 0x00);
	Sleep(10);
	write_cmos_sensor_8(0x003E, 0x00);
	write_cmos_sensor_8(0x0A00, 0x01);
	Sleep(1);
}
static kal_uint8 hi846_otp_read_byte(kal_uint16 addr){
	write_cmos_sensor_8(HI846_OTP_REG_ADDRH, addr>>8);
	write_cmos_sensor_8(HI846_OTP_REG_ADDRL, addr & 0xFF);
	write_cmos_sensor_8(HI846_OTP_REG_CMD, HI846_OTP_CMD_NORMAL);
	return (kal_uint8)read_cmos_sensor(HI846_OTP_REG_RDATA);
}
static unsigned int hi846_otp_read_data(kal_uint16 addr, kal_uint8 *buf, kal_uint16 size){
	int i = 0;
	write_cmos_sensor_8(HI846_OTP_REG_ADDRH, addr>>8);
	write_cmos_sensor_8(HI846_OTP_REG_ADDRL, addr & 0xFF);
	write_cmos_sensor_8(HI846_OTP_REG_CMD, HI846_OTP_CMD_CONTINUOUS_READ);
	for(i=0; i<size; i++){
		buf[i] = read_cmos_sensor(HI846_OTP_REG_RDATA);
	}
	return i;
}
static void hi846_otp_read_module(kal_uint16 addr){
	hi846_otp_read_data(addr, (kal_uint8 *)&hi846_otp_info.module, HI846_OTP_SIZE_MODULE);
}

static void hi846_otp_read_sn(kal_uint16 addr){
	hi846_otp_read_data(addr, (kal_uint8 *)&hi846_otp_info.sn, HI846_OTP_SIZE_SN);
}

static void hi846_otp_read_awb(kal_uint16 addr){
	hi846_otp_read_data(addr, (kal_uint8 *)&hi846_otp_info.awb, HI846_OTP_SIZE_AWB);
}

static void hi846_otp_read_lsc(kal_uint16 addr){
	hi846_otp_read_data(addr, (kal_uint8 *)&hi846_otp_info.lsc, HI846_OTP_SIZE_LSC);
}

static void hi846_read_otp(void){
	kal_uint8  flag_module = 0;
	kal_uint8  flag_sn = 0;
	kal_uint8  flag_awb = 0;
	kal_uint8  flag_lsc = 0;
#ifdef HI846_OTP_DEBUG
	kal_uint32 i = 0;
	kal_uint32 check_sum_cal = 0;
	int rc = 0;
#endif

	const kal_uint16 ADDR_MODULE[] = {HI846_OTP_OFFSET_MODULE1, HI846_OTP_OFFSET_MODULE2, HI846_OTP_OFFSET_MODULE3};
	const kal_uint16 ADDR_SN[] = {HI846_OTP_OFFSET_SN1, HI846_OTP_OFFSET_SN2, HI846_OTP_OFFSET_SN3};
	const kal_uint16 ADDR_AWB[] = {HI846_OTP_OFFSET_AWB1, HI846_OTP_OFFSET_AWB2, HI846_OTP_OFFSET_AWB3};
	const kal_uint16 ADDR_LSC[] = {HI846_OTP_OFFSET_LSC1, HI846_OTP_OFFSET_LSC2, HI846_OTP_OFFSET_LSC3};

	hi846_otp_enable();

#ifdef HI846_OTP_DEBUG
{
	unsigned char otpdata[1024] = "";
	char buf[200] = "";
	hi846_otp_read_data(HI846_OTP_OFFSET_MODULE_FLAG, otpdata, 1024);

	rc = sprintf_s(buf, sizeof(buf), "HI846_OTP Data(header): ");
	if (rc < 0) {
		LOG_INF("sprintf_s HI846_OTP Data(header) failed!!\n");
		return;
	}
	for(i=0;i<16;i++){
		rc = sprintf_s(buf, sizeof(buf), "%s%4x ", buf, i);
		if (rc < 0) {
			LOG_INF("sprintf_s buf failed!!\n");
			return;
		}
	}
	LOG_INF("%s\n", buf);

	for(i=0;i<1024;i++){
		if(i%16==0){
			rc = sprintf_s(buf, sizeof(buf), "HI846_OTP Data(0x%04x): ", i);
			if (rc < 0) {
				LOG_INF("sprintf_s HI846_OTP Data failed!!\n");
				return;
			}
		}
		rc = sprintf_s(buf, sizeof(buf), "%s 0x%02x", buf, otpdata[i]);
		if (rc < 0) {
			LOG_INF("sprintf_s buf failed!!\n");
			return;
		}
		if( (i%16==15) || (i==(1024-1))){
			LOG_INF("%s\n", buf);
		}
	}
}
#endif

	flag_module = hi846_otp_read_byte(HI846_OTP_OFFSET_MODULE_FLAG);
	flag_sn = hi846_otp_read_byte(HI846_OTP_OFFSET_SN_FLAG);
	flag_awb = hi846_otp_read_byte(HI846_OTP_OFFSET_AWB_FLAG);
	flag_lsc = hi846_otp_read_byte(HI846_OTP_OFFSET_LSC_FLAG);

	LOG_INF("HI846_OTP : flag_module = 0x%x , flag_sn = 0x%x, flag_awb = 0x%x, flag_lsc = 0x%x\n", flag_module, flag_sn, flag_awb, flag_lsc);

	//moduleinfo
	if (flag_module == HI846_GROUP3_FLAG)
		hi846_otp_read_module(ADDR_MODULE[2]);
	else if (flag_module == HI846_GROUP2_FLAG)
		hi846_otp_read_module(ADDR_MODULE[1]);
	else if (flag_module == HI846_GROUP1_FLAG)
		hi846_otp_read_module(ADDR_MODULE[0]);

	//sn
	if (flag_sn == HI846_GROUP3_FLAG)
		hi846_otp_read_sn(ADDR_SN[2]);
	else if (flag_sn == HI846_GROUP2_FLAG)
		hi846_otp_read_sn(ADDR_SN[1]);
	else if (flag_sn == HI846_GROUP1_FLAG)
		hi846_otp_read_sn(ADDR_SN[0]);

	//awb
	if (flag_awb == HI846_GROUP3_FLAG)
		hi846_otp_read_awb(ADDR_AWB[2]);
	else if (flag_awb == HI846_GROUP2_FLAG)
		hi846_otp_read_awb(ADDR_AWB[1]);
	else if (flag_awb == HI846_GROUP1_FLAG)
		hi846_otp_read_awb(ADDR_AWB[0]);

	//lsc
	if (flag_lsc == HI846_GROUP3_FLAG)
		hi846_otp_read_lsc(ADDR_LSC[2]);
	else if (flag_lsc == HI846_GROUP2_FLAG)
		hi846_otp_read_lsc(ADDR_LSC[1]);
	else if (flag_lsc == HI846_GROUP1_FLAG)
		hi846_otp_read_lsc(ADDR_LSC[0]);

#ifdef HI846_OTP_DEBUG

	if (flag_module == HI846_GROUP1_FLAG || flag_module == HI846_GROUP2_FLAG || flag_module == HI846_GROUP3_FLAG)
		hi846_otp_info.flag_module = 1;
	if (flag_sn == HI846_GROUP1_FLAG || flag_sn == HI846_GROUP2_FLAG || flag_sn == HI846_GROUP3_FLAG)
		hi846_otp_info.flag_sn = 1;
	if (flag_awb == HI846_GROUP1_FLAG || flag_awb == HI846_GROUP2_FLAG || flag_awb == HI846_GROUP3_FLAG)
		hi846_otp_info.flag_awb = 1;
	if (flag_lsc == HI846_GROUP1_FLAG || flag_lsc == HI846_GROUP2_FLAG || flag_lsc == HI846_GROUP3_FLAG)
		hi846_otp_info.flag_lsc = 1;

	if(hi846_otp_info.flag_module){
		LOG_INF("MODULE time:%4d-%2d-%2d, moduleID:%x, checksum:0x%2x", hi846_otp_info.module[4],
			hi846_otp_info.module[5], hi846_otp_info.module[6],
			hi846_otp_info.module[0], hi846_otp_info.module[7]);
		LOG_INF("lens_id:%d, vcm_id:%d", hi846_otp_info.module[2], hi846_otp_info.module[3]);
		//checksum
		check_sum_cal = 0;
		for (i = 0; i < HI846_OTP_SIZE_MODULE - 1; i++)
			check_sum_cal += hi846_otp_info.module[i];
		if (((check_sum_cal % 255) + 1) == hi846_otp_info.module[7]){
			LOG_INF("otp module info checksum sucessful!");
		} else {
			LOG_INF("otp module info checksum failed, sum:0x%2x, checksum:0x%2x!",
				((check_sum_cal % 255) + 1), hi846_otp_info.module[7]);
		}
	}else{
		LOG_INF("otp module info is not used!");
	}
	if(hi846_otp_info.flag_sn){
		LOG_INF("SN time: %4d-%2d-%2d", hi846_otp_info.sn[4], hi846_otp_info.sn[5], hi846_otp_info.sn[6]);
		LOG_INF("SN:0x%2x, 0x%2x, 0x%2x, 0x%2x", hi846_otp_info.sn[7], hi846_otp_info.sn[8],
			hi846_otp_info.sn[9], hi846_otp_info.sn[10]);
		//checksum
		check_sum_cal = 0;
		for (i = 0; i < HI846_OTP_SIZE_SN - 1; i++)
			check_sum_cal += hi846_otp_info.sn[i];
		if (((check_sum_cal % 255) + 1) == hi846_otp_info.sn[11]){
			LOG_INF("otp sn checksum sucessful!");
		} else {
			LOG_INF("otp sn checksum failed, sum:0x%2x, checksum:0x%2x!",
				((check_sum_cal % 255) + 1), hi846_otp_info.sn[11]);
		}
	}else{
		LOG_INF("otp sn is not used!");
	}
	if(hi846_otp_info.flag_awb){
		LOG_INF("r:0x%x, b:0x%x, gr:0x%x, gb:0x%x", ((hi846_otp_info.awb[1]<<8)&0xff00)|(hi846_otp_info.awb[0]&0xff),
			((hi846_otp_info.awb[3]<<8)&0xff00)|(hi846_otp_info.awb[2]&0xff),
			((hi846_otp_info.awb[5]<<8)&0xff00)|(hi846_otp_info.awb[4]&0xff),
			((hi846_otp_info.awb[7]<<8)&0xff00)|(hi846_otp_info.awb[6]&0xff));
		LOG_INF("Golden r:0x%x, b:0x%x, gr:0x%x, gb:0x%x", ((hi846_otp_info.awb[9]<<8)&0xff00) |(hi846_otp_info.awb[8]&0xff),
			((hi846_otp_info.awb[11]<<8)&0xff00)|(hi846_otp_info.awb[10]&0xff),
			((hi846_otp_info.awb[13]<<8)&0xff00)|(hi846_otp_info.awb[12]&0xff),
			((hi846_otp_info.awb[15]<<8)&0xff00)|(hi846_otp_info.awb[14]&0xff));
		//checksum
		check_sum_cal = 0;
		for (i = 0; i < HI846_OTP_SIZE_AWB - 1; i++)
			check_sum_cal += hi846_otp_info.awb[i];
		if (((check_sum_cal % 255) + 1) == hi846_otp_info.awb[16]){
			LOG_INF("otp awb checksum sucessful!");
		} else {
			LOG_INF("otp awb checksum failed, sum:0x%2x, checksum:0x%2x!",
				((check_sum_cal % 255) + 1), hi846_otp_info.awb[11]);
		}
	}else{
		LOG_INF("otp awb is not used!");
	}
	if(hi846_otp_info.flag_lsc){
		//checksum
		check_sum_cal = 0;
		LOG_INF("otp lsc: [0]:0x%2x, [1]:0x%2x, [2]:0x%2x, [3]:0x%2x, [4]:0x%2x, [5]:0x%2x!",
				hi846_otp_info.lsc[0], hi846_otp_info.lsc[1], hi846_otp_info.lsc[2], hi846_otp_info.lsc[3], hi846_otp_info.lsc[4], hi846_otp_info.lsc[5]);
		LOG_INF("otp lsc: [1863]:0x%2x, [1864]:0x%2x, [1865]:0x%2x, [1866]:0x%2x, [1867]:0x%2x, [1868]:0x%2x!",
				hi846_otp_info.lsc[1863], hi846_otp_info.lsc[1864], hi846_otp_info.lsc[1865], hi846_otp_info.lsc[1866], hi846_otp_info.lsc[1867], hi846_otp_info.lsc[1868]);
		for (i = 0; i < HI846_OTP_SIZE_LSC - 1; i++)
			check_sum_cal += hi846_otp_info.lsc[i];
		if (((check_sum_cal % 255) + 1) == hi846_otp_info.lsc[1868]) {
			LOG_INF("otp lsc checksum sucessful!");
		} else {
			LOG_INF("otp lsc checksum failed, totalsum:0x%2x, sum:0x%2x, checksum:0x%2x!",
				check_sum_cal, ((check_sum_cal % 255) + 1), hi846_otp_info.lsc[1868]);
		}
	}else{
		LOG_INF("otp lsc is not used!");
	}
#endif

	hi846_otp_disable();
}

int hi846_read_buf_region(struct i2c_client *client,
				struct stCAM_CAL_LIST_STRUCT *list,
				unsigned int addr,
				unsigned char *data,
				unsigned int size)
{
	unsigned char * buffer_temp = (unsigned char *)data;
	errno_t err;
	g_pstI2CclientG = client;
	if(g_pstI2CclientG == NULL){
		LOG_INF("g_pstI2CclientG==NULL");
		return 0;
	}

	if(init_flag == 0){
		hi846_read_otp();
		init_flag = 1;
	}
	LOG_INF("addr=0x%x, list=%p, buffer_temp=%p", addr, list, buffer_temp);

	err = memcpy_s(buffer_temp, HI846_OTP_SIZE_MODULE,
			hi846_otp_info.module, HI846_OTP_SIZE_MODULE);
	if (err != EOK) {
		LOG_INF("memcpy_s hi846_otp_info.module failed!! err is %d\n", err);
		return 0;
	}
	buffer_temp += HI846_OTP_SIZE_MODULE;

	err = memcpy_s(buffer_temp, HI846_OTP_SIZE_SN,
			hi846_otp_info.sn, HI846_OTP_SIZE_SN);
	if (err != EOK) {
		LOG_INF("memcpy_s hi846_otp_info.sn failed!! err is %d\n", err);
		return 0;
	}
	buffer_temp += HI846_OTP_SIZE_SN;

	err = memcpy_s(buffer_temp, HI846_OTP_SIZE_AWB,
			hi846_otp_info.awb, HI846_OTP_SIZE_AWB);
	if (err != EOK) {
		LOG_INF("memcpy_s hi846_otp_info.awb failed!! err is %d\n", err);
		return 0;
	}
	buffer_temp += HI846_OTP_SIZE_AWB;

	err = memcpy_s(buffer_temp, HI846_OTP_SIZE_LSC,
			hi846_otp_info.lsc, HI846_OTP_SIZE_LSC);
	if (err != EOK) {
		LOG_INF("memcpy_s hi846_otp_info.lsc failed!! err is %d\n", err);
		return 0;
	}

	buffer_temp += HI846_OTP_SIZE_LSC;

	size = HI846_OTP_SIZE_MODULE + HI846_OTP_SIZE_SN + HI846_OTP_SIZE_AWB + HI846_OTP_SIZE_LSC;

	return size;
}
