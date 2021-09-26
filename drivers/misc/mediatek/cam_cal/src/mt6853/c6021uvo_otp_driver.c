/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: c6021uvo otp driver
 * Author: mazhaung
 * Create: 2020-08-1
 */

#define PFX "CAM_CAL"
#define pr_fmt(fmt) PFX "[%s] " fmt, __func__

#include "c6021uvo_otp_driver.h"

#define sleep(ms) mdelay(ms)

#define log_err(fmt, args...) \
	pr_err(PFX "%s %d " fmt, __func__, __LINE__, ##args)

#define log_inf(fmt, args...) \
	pr_info(PFX "%s %d " fmt, __func__, __LINE__, ##args)

#define OFFSET_AWB	0x07
#define OFFSET_LSC	0x17

#define swap16(x)  (((x)&0xFF)<<8) | ((x)>>8)
static struct i2c_client *g_psti2cclientg;

static int init_flag;

struct c6021uvo_otp_t c6021uvo_otp_info = { 0 };

static int ireadregi2c(u8 *a_psenddata, u16 a_sizesenddata, u8 *a_precvdata, u16 a_sizerecvdata)
{
	int i4retvalue = i2c_master_send(g_psti2cclientg, a_psenddata, a_sizesenddata);
	if (i4retvalue != a_sizesenddata) {
		pr_err("I2C send failed!!, Addr = 0x%x\n", a_psenddata[0]);
		return -1;
	}
	i4retvalue = i2c_master_recv(g_psti2cclientg, (char *)a_precvdata, a_sizerecvdata);
	if (i4retvalue != a_sizerecvdata) {
		pr_err("I2C read failed!!\n");
		return -1;
	}
	return 0;
}

static int iwriteregi2c(u8 *a_psenddata, u16 a_sizesenddata)
{
	int i4retvalue = i2c_master_send(g_psti2cclientg, a_psenddata, a_sizesenddata);
	if (i4retvalue != a_sizesenddata) {
		pr_err("I2C send failed!!, Addr = 0x%x\n", a_psenddata[0]);
		return -1;
	}
	return 0;
}

static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte = 0;
	char pu_send_cmd[2] = { (char)(addr >> 8), (char)(addr & 0xFF) };
	ireadregi2c(pu_send_cmd, 2, (u8 *)&get_byte, 1);
	return get_byte;
}

static void write_cmos_sensor_8(kal_uint32 addr, kal_uint32 para)
{
	char pu_send_cmd[4] = { (char)(addr >> 8), (char)(addr & 0xFF), (char)(para & 0xFF) };
	iwriteregi2c(pu_send_cmd, 3);
}

static void c6021uvo_otp_enable(void)
{
	write_cmos_sensor_8(0x0A02, 0x01);
	write_cmos_sensor_8(0x0A00, 0x00);
	sleep(10);
	write_cmos_sensor_8(0x0F02, 0x00);
	write_cmos_sensor_8(0x071A, 0x01);
	write_cmos_sensor_8(0x071B, 0x09);
	write_cmos_sensor_8(0x0D04, 0x00);
	write_cmos_sensor_8(0x0D00, 0x07);
	write_cmos_sensor_8(0x003E, 0x10);
	write_cmos_sensor_8(0x0A00, 0x01);
}

static void c6021uvo_otp_disable(void)
{
	write_cmos_sensor_8(0x0A00, 0x00);
	sleep(10);
	write_cmos_sensor_8(0x003E, 0x00);
	write_cmos_sensor_8(0x0A00, 0x01);
	sleep(1);
}

static kal_uint8 c6021uvo_otp_read_byte(kal_uint16 addr)
{
	write_cmos_sensor_8(C6021UVO_OTP_REG_ADDRH, addr >> 8);
	write_cmos_sensor_8(C6021UVO_OTP_REG_ADDRL, addr & 0xFF);
	write_cmos_sensor_8(C6021UVO_OTP_REG_CMD, C6021UVO_OTP_CMD_NORMAL);
	return (kal_uint8)read_cmos_sensor(C6021UVO_OTP_REG_RDATA);
}

static void c6021uvo_otp_read_data(kal_uint16 addr, kal_uint8 *buf, kal_uint16 size)
{
	int i;
	write_cmos_sensor_8(C6021UVO_OTP_REG_ADDRH, addr >> 8);
	write_cmos_sensor_8(C6021UVO_OTP_REG_ADDRL, addr & 0xFF);
	write_cmos_sensor_8(C6021UVO_OTP_REG_CMD, C6021UVO_OTP_CMD_CONTINUOUS_READ);
	for (i = 0; i < size; i++)
		buf[i] = read_cmos_sensor(C6021UVO_OTP_REG_RDATA);
}

static void c6021uvo_otp_read_module(kal_uint16 addr)
{
	c6021uvo_otp_read_data(addr, (kal_uint8 *)&c6021uvo_otp_info.module, C6021UVO_OTP_SIZE_MODULE);
}

static void c6021uvo_otp_read_sn(kal_uint16 addr)
{
	c6021uvo_otp_read_data(addr, (kal_uint8 *)&c6021uvo_otp_info.sn, C6021UVO_OTP_SIZE_SN);
}

static void c6021uvo_otp_read_awb(kal_uint16 addr)
{
	c6021uvo_otp_read_data(addr, (kal_uint8 *)&c6021uvo_otp_info.awb, C6021UVO_OTP_SIZE_AWB);
}

static void c6021uvo_otp_read_lsc(kal_uint16 addr)
{
	c6021uvo_otp_read_data(addr, (kal_uint8 *)&c6021uvo_otp_info.lsc, C6021UVO_OTP_SIZE_LSC);
}

static void otp_get_group_data(struct c6021uvo_otp_t *otp_info)
{
	int i;
	c6021uvo_otp_enable();
	otp_info->flag_module = c6021uvo_otp_read_byte(C6021UVO_OTP_OFFSET_MODULE_FLAG);
	otp_info->flag_sn = c6021uvo_otp_read_byte(C6021UVO_OTP_OFFSET_SN_FLAG);
	otp_info->flag_awb = c6021uvo_otp_read_byte(C6021UVO_OTP_OFFSET_AWB_FLAG);
	otp_info->flag_lsc = c6021uvo_otp_read_byte(C6021UVO_OTP_OFFSET_LSC_FLAG);
	log_inf("C6021UVO_OTP:flag_module=0x%x, flag_sn=0x%x, flag_awb=0x%x, flag_lsc=0x%x\n",
		otp_info->flag_module, otp_info->flag_sn, otp_info->flag_awb, otp_info->flag_lsc);

	for (i = 0; i < C6021UVO_GROUP_FLAG_SIZE; i++) {
		if (otp_info->flag_module == otp_info->addr_flag[i])
			c6021uvo_otp_read_module(otp_info->addr_module[i]);
		if (otp_info->flag_sn == otp_info->addr_flag[i])
			c6021uvo_otp_read_sn(otp_info->addr_sn[i]);
		if (otp_info->flag_awb == otp_info->addr_flag[i])
			c6021uvo_otp_read_awb(otp_info->addr_awb[i]);
		if (otp_info->flag_lsc == otp_info->addr_flag[i])
			c6021uvo_otp_read_lsc(otp_info->addr_lsc[i]);
	}
}
#ifdef C6021UVO_OTP_DEBUG
static void flag_module_check()
{
	int i;
	kal_uint32 check_sum_cal = 0;
	if (c6021uvo_otp_info.flag_module) {
		log_inf("MODULE time:%4d-%2d-%2d, moduleID:%x, checksum:0x%2x",
			c6021uvo_otp_info.module[4],
			c6021uvo_otp_info.module[5], c6021uvo_otp_info.module[6],
			c6021uvo_otp_info.module[0], c6021uvo_otp_info.module[7]);
		log_inf("lens_id:%d, vcm_id:%d", c6021uvo_otp_info.module[2],
			c6021uvo_otp_info.module[3]);
		for (i = 0; i < C6021UVO_OTP_SIZE_MODULE - 1; i++)
			check_sum_cal += c6021uvo_otp_info.module[i];
		if (((check_sum_cal % 255) + 1) == c6021uvo_otp_info.module[7]) {
			log_inf("otp module info checksum sucessful!");
		} else {
			log_err("otp module info checksum failed, sum:0x%2x, checksum:0x%2x!",
				((check_sum_cal % 255) + 1), c6021uvo_otp_info.module[7]);
		}
	} else {
		log_inf("otp module info is not used!");
	}
}

static void flag_sn_check()
{
	int i;
	kal_uint32 check_sum_cal = 0;
	if (c6021uvo_otp_info.flag_sn) {
		log_inf("SN time: %4d-%2d-%2d",
			c6021uvo_otp_info.sn[4], c6021uvo_otp_info.sn[5],
			c6021uvo_otp_info.sn[6]);
		log_inf("SN:0x%2x, 0x%2x, 0x%2x, 0x%2x",
			c6021uvo_otp_info.sn[7], c6021uvo_otp_info.sn[8],
			c6021uvo_otp_info.sn[9], c6021uvo_otp_info.sn[10]);
		for (i = 0; i < C6021UVO_OTP_SIZE_SN - 1; i++)
			check_sum_cal += c6021uvo_otp_info.sn[i];
		if (((check_sum_cal % 255) + 1) == c6021uvo_otp_info.sn[11]) {
			log_inf("otp sn checksum sucessful!");
		} else {
			log_err("otp sn checksum failed, sum:0x%2x, checksum:0x%2x!",
				((check_sum_cal % 255) + 1), c6021uvo_otp_info.sn[11]);
		}
	} else {
		log_err("otp sn is not used!");
	}
}

static void flag_awb_check()
{
	int i;
	kal_uint32 check_sum_cal = 0;
	if (c6021uvo_otp_info.flag_awb) {
		log_inf("r:0x%x, b:0x%x, gr:0x%x, gb:0x%x",
			((c6021uvo_otp_info.awb[1] << 8) & 0xff00) | (c6021uvo_otp_info.awb[0] & 0xff),
			((c6021uvo_otp_info.awb[3] << 8) & 0xff00) | (c6021uvo_otp_info.awb[2] & 0xff),
			((c6021uvo_otp_info.awb[5] << 8) & 0xff00) | (c6021uvo_otp_info.awb[4] & 0xff),
			((c6021uvo_otp_info.awb[7] << 8) & 0xff00) | (c6021uvo_otp_info.awb[6] & 0xff));
		log_inf("Golden r:0x%x, b:0x%x, gr:0x%x, gb:0x%x",
			((c6021uvo_otp_info.awb[9] << 8) & 0xff00) | (c6021uvo_otp_info.awb[8] & 0xff),
			((c6021uvo_otp_info.awb[11] << 8) & 0xff00) | (c6021uvo_otp_info.awb[10] & 0xff),
			((c6021uvo_otp_info.awb[13] << 8) & 0xff00) | (c6021uvo_otp_info.awb[12] & 0xff),
			((c6021uvo_otp_info.awb[15] << 8) & 0xff00) | (c6021uvo_otp_info.awb[14] & 0xff));
		for (i = 0; i < C6021UVO_OTP_SIZE_AWB - 1; i++)
			check_sum_cal += c6021uvo_otp_info.awb[i];
		if (((check_sum_cal % 255) + 1) == c6021uvo_otp_info.awb[16]) {
			log_inf("otp awb checksum sucessful!");
		} else {
			log_err("otp awb checksum failed, sum:0x%2x, checksum:0x%2x!",
				((check_sum_cal % 255) + 1), c6021uvo_otp_info.awb[11]);
		}
	} else {
		log_err("otp awb is not used!");
	}
}

static void flag_lsc_check()
{
	int i;
	kal_uint32 check_sum_cal = 0;
	if (c6021uvo_otp_info.flag_lsc) {
		log_inf("otp lsc:[0]:0x%2x,[1]:0x%2x,[2]:0x%2x,[3]:0x%2x,[4]:0x%2x,[5]:0x%2x!",
			c6021uvo_otp_info.lsc[0], c6021uvo_otp_info.lsc[1],
			c6021uvo_otp_info.lsc[2], c6021uvo_otp_info.lsc[3],
			c6021uvo_otp_info.lsc[4], c6021uvo_otp_info.lsc[5]);
		for (i = 0; i < C6021UVO_OTP_SIZE_LSC - 1; i++)
			check_sum_cal += c6021uvo_otp_info.lsc[i];
		if (((check_sum_cal % 255) + 1) == c6021uvo_otp_info.lsc[C6021UVO_OTP_SIZE_LSC_DATA]) {
			log_inf("otp lsc checksum sucessful!");
		} else {
			log_err("otp lsc checksum failed,totalsum:0x%2x,sum:0x%2x,checksum:0x%2x!",
				check_sum_cal, ((check_sum_cal % 255) + 1),
				c6021uvo_otp_info.lsc[C6021UVO_OTP_SIZE_LSC_DATA]);
		}
	} else {
		log_err("otp lsc is not used!");
	}
}

static void otp_data_checksum(struct c6021uvo_otp_t otp_info)
{
	int i;
	for(i = 0; i < C6021UVO_GROUP_FLAG_SIZE; i++) {
		if(otp_info.flag_module == otp_info.addr_flag[i])
			c6021uvo_otp_info.flag_module = 1;
		if(otp_info.flag_sn == otp_info.addr_flag[i])
			c6021uvo_otp_info.flag_sn = 1;
		if(otp_info.flag_awb == otp_info.addr_flag[i])
			c6021uvo_otp_info.flag_awb = 1;
		if(otp_info.flag_lsc == otp_info.addr_flag[i])
			c6021uvo_otp_info.flag_lsc = 1;
	}
	flag_module_check();
	flag_sn_check();
	flag_awb_check();
	flag_lsc_check();
}
#endif

static void c6021uvo_read_otp(void)
{
	int i;
	struct c6021uvo_otp_t otp_info;
	struct c6021uvo_otp_addr otp_addr = { C6021UVO_GROUP1_FLAG,
		C6021UVO_OTP_OFFSET_MODULE1, C6021UVO_OTP_OFFSET_SN1,
		C6021UVO_OTP_OFFSET_AWB1, C6021UVO_OTP_OFFSET_LSC1 };
	for (i = 0; i < C6021UVO_GROUP_FLAG_SIZE; i++) {
		otp_addr.addr_flag += C6021UVO_DIFFERENCE_GROUP2_FLAG * i;
		otp_info.addr_flag[i] = otp_addr.addr_flag;
		otp_info.addr_module[i] = otp_addr.addr_flag_module;
		otp_info.addr_sn[i] = otp_addr.addr_flag_sn;
		otp_info.addr_awb[i] = otp_addr.addr_flag_awb;
		otp_info.addr_lsc[i] = otp_addr.addr_flag_lsc;
		otp_addr.addr_flag_module += C6021UVO_OTP_DIFFERENCE_MODULE;
		otp_addr.addr_flag_sn += C6021UVO_OTP_DIFFERENCE_SN;
		otp_addr.addr_flag_awb += C6021UVO_OTP_DIFFERENCE_AWB;
		otp_addr.addr_flag_lsc += C6021UVO_OTP_DIFFERENCE_LSC;
	}
	otp_get_group_data(&otp_info);
#ifdef C6021UVO_OTP_DEBUG
	otp_data_checksum(otp_info);
#endif
	c6021uvo_otp_disable();
}

int c6021uvo_read_buf_region(struct i2c_client *client,
				struct stCAM_CAL_LIST_STRUCT *list,
				unsigned int addr,
				unsigned char *data,
				unsigned int size)
{
	unsigned char *buffer_temp = (unsigned char *)data;
	errno_t err;
	g_psti2cclientg = client;
	if (g_psti2cclientg == NULL) {
		log_err("g_psti2cclientg==NULL");
		return 0;
	}
	if (init_flag == 0) {
		c6021uvo_read_otp();
		init_flag = 1;
	}
	log_inf("addr=0x%x, list=%p, buffer_temp=%p", addr, list, buffer_temp);
	err = memcpy_s(buffer_temp, C6021UVO_OTP_SIZE_MODULE,
			c6021uvo_otp_info.module, C6021UVO_OTP_SIZE_MODULE);
	if (err != EOK) {
		log_err("memcpy_s c6021uvo_otp_info.module failed!! err is %d\n", err);
		return 0;
	}
	buffer_temp += C6021UVO_OTP_SIZE_MODULE;
	err = memcpy_s(buffer_temp, C6021UVO_OTP_SIZE_SN,
			c6021uvo_otp_info.sn, C6021UVO_OTP_SIZE_SN);
	if (err != EOK) {
		log_err("memcpy_s c6021uvo_otp_info.sn failed!! err is %d\n", err);
		return 0;
	}
	buffer_temp += C6021UVO_OTP_SIZE_SN;
	err = memcpy_s(buffer_temp, C6021UVO_OTP_SIZE_AWB,
			c6021uvo_otp_info.awb, C6021UVO_OTP_SIZE_AWB);
	if (err != EOK) {
		log_err("memcpy_s c6021uvo_otp_info.awb failed!! err is %d\n", err);
		return 0;
	}
	buffer_temp += C6021UVO_OTP_SIZE_AWB;
	err = memcpy_s(buffer_temp, C6021UVO_OTP_SIZE_LSC,
			c6021uvo_otp_info.lsc, C6021UVO_OTP_SIZE_LSC);
	if (err != EOK) {
		log_err("memcpy_s c6021uvo_otp_info.lsc failed!! err is %d\n", err);
		return 0;
	}
	buffer_temp += C6021UVO_OTP_SIZE_LSC;
	size = C6021UVO_OTP_SIZE_MODULE + C6021UVO_OTP_SIZE_SN + C6021UVO_OTP_SIZE_AWB + C6021UVO_OTP_SIZE_LSC;
	return size;
}
