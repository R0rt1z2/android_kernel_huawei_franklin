/*
 * Copyright (C) 2017 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/dmi.h>
#include <linux/acpi.h>
#include <linux/thermal.h>
#include <linux/platform_device.h>
#include <mt-plat/aee.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/proc_fs.h>
#include <linux/syscalls.h>
#include <linux/sched.h>
#include <linux/writeback.h>
#include <linux/uaccess.h>
#include "mt-plat/mtk_thermal_monitor.h"
#include "mach/mtk_thermal.h"
#include "mtk_thermal_timer.h"
#include <linux/uidgid.h>
#include <tmp_bts.h>
#include <linux/slab.h>

#include <mt-plat/charger_class.h>

static kuid_t uid = KUIDT_INIT(0);
static kgid_t gid = KGIDT_INIT(1000);
static DEFINE_SEMAPHORE(sem_mutex);

static unsigned int interval = 1;	/* seconds, 0 : no auto polling */
static int trip_temp[10] = { 120000, 110000, 100000, 90000, 80000,
				70000, 65000, 60000, 55000, 50000 };

static struct thermal_zone_device *thz_dev;
static int mtkts_pmintc_debug_log;
static int kernelmode;
static int g_THERMAL_TRIP[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
static int dts_ntc_status;


static int num_trip = 1;
static char g_bind0[20] = {"mtk_pmi-sysrst"};
static char g_bind1[20] = { 0 };
static char g_bind2[20] = { 0 };
static char g_bind3[20] = { 0 };
static char g_bind4[20] = { 0 };
static char g_bind5[20] = { 0 };
static char g_bind6[20] = { 0 };
static char g_bind7[20] = { 0 };
static char g_bind8[20] = { 0 };
static char g_bind9[20] = { 0 };


/**
 * If curr_temp >= polling_trip_temp1, use interval
 * else if cur_temp >= polling_trip_temp2 && curr_temp < polling_trip_temp1,
 *	use interval*polling_factor1
 * else, use interval*polling_factor2
 */
static int polling_trip_temp1 = 40000;
static int polling_trip_temp2 = 20000;
static int polling_factor1 = 5000;
static int polling_factor2 = 10000;

#define MTKTS_PMINTC_SW_FILTER (0)
#define MTKTS_PMINTC_TEMP_CRIT 60000	/* 60.000 degree Celsius */

#define mtkts_pmintc_dprintk(fmt, args...)   \
do {                                    \
	if (mtkts_pmintc_debug_log) {                \
		pr_debug("[Thermal/TZ/PMINTC]" fmt, ##args); \
	}                                   \
} while (0)

#define mtkts_pmintc_printk(fmt, args...) \
pr_debug("[Thermal/TZ/PMINTC]" fmt, ##args)


#if defined(CONFIG_MEDIATEK_MT6577_AUXADC)
struct iio_channel *thermistor_ch7;
#endif

/* #define INPUT_PARAM_FROM_USER_AP */

/*
 * kernel fopen/fclose
 */
/*
 *static mm_segment_t oldfs;
 *
 *static void my_close(int fd)
 *{
 *	set_fs(oldfs);
 *	sys_close(fd);
 *}
 *
 *static int my_open(char *fname, int flag)
 *{
 *	oldfs = get_fs();
 *    set_fs(KERNEL_DS);
 *    return sys_open(fname, flag, 0);
 *}
 */
struct PMINTC_TEMPERATURE {
	__s32 PMINTC_Temp;
	__s32 TemperatureR;
};

static int g_RAP_pull_up_R = BTSPMINTC_RAP_PULL_UP_R;
static int g_TAP_over_critical_low = BTSPMINTC_TAP_OVER_CRITICAL_LOW;
static int g_RAP_pull_up_voltage = BTSPMINTC_RAP_PULL_UP_VOLTAGE;
static int g_RAP_ntc_table = BTSPMINTC_RAP_NTC_TABLE;
static int g_RAP_ADC_channel = BTSPMINTC_RAP_ADC_CHANNEL;

static int g_pmintc_TemperatureR;
/* struct PMINTC_TEMPERATURE PMINTC_Temperature_Table[] = {0}; */

static struct PMINTC_TEMPERATURE *PMINTC_Temperature_Table;
static int ntc_tbl_size;

/* AP_NTC_BL197 */
static struct PMINTC_TEMPERATURE PMINTC_Temperature_Table1[] = {
	{-40, 74354},		/* FIX_ME */
	{-35, 74354},		/* FIX_ME */
	{-30, 74354},		/* FIX_ME */
	{-25, 74354},		/* FIX_ME */
	{-20, 74354},
	{-15, 57626},
	{-10, 45068},
	{-5, 35548},
	{0, 28267},
	{5, 22650},
	{10, 18280},
	{15, 14855},
	{20, 12151},
	{25, 10000},		/* 10K */
	{30, 8279},
	{35, 6892},
	{40, 5768},
	{45, 4852},
	{50, 4101},
	{55, 3483},
	{60, 2970},		/* FIX_ME */
	{60, 2970},		/* FIX_ME */
	{60, 2970},		/* FIX_ME */
	{60, 2970},		/* FIX_ME */
	{60, 2970},		/* FIX_ME */
	{60, 2970},		/* FIX_ME */
	{60, 2970},		/* FIX_ME */
	{60, 2970},		/* FIX_ME */
	{60, 2970},		/* FIX_ME */
	{60, 2970},		/* FIX_ME */
	{60, 2970},		/* FIX_ME */
	{60, 2970},		/* FIX_ME */
	{60, 2970},		/* FIX_ME */
	{60, 2970}		/* FIX_ME */
};

/* AP_NTC_TSM_1 */
static struct PMINTC_TEMPERATURE PMINTC_Temperature_Table2[] = {
	{-40, 70603},		/* FIX_ME */
	{-35, 70603},		/* FIX_ME */
	{-30, 70603},		/* FIX_ME */
	{-25, 70603},		/* FIX_ME */
	{-20, 70603},
	{-15, 55183},
	{-10, 43499},
	{-5, 34569},
	{0, 27680},
	{5, 22316},
	{10, 18104},
	{15, 14773},
	{20, 12122},
	{25, 10000},		/* 10K */
	{30, 8294},
	{35, 6915},
	{40, 5795},
	{45, 4882},
	{50, 4133},
	{55, 3516},
	{60, 3004},		/* FIX_ME */
	{60, 3004},		/* FIX_ME */
	{60, 3004},		/* FIX_ME */
	{60, 3004},		/* FIX_ME */
	{60, 3004},		/* FIX_ME */
	{60, 3004},		/* FIX_ME */
	{60, 3004},		/* FIX_ME */
	{60, 3004},		/* FIX_ME */
	{60, 3004},		/* FIX_ME */
	{60, 3004},		/* FIX_ME */
	{60, 3004},		/* FIX_ME */
	{60, 3004},		/* FIX_ME */
	{60, 3004},		/* FIX_ME */
	{60, 3004}		/* FIX_ME */
};

/* AP_NTC_10_SEN_1 */
static struct PMINTC_TEMPERATURE PMINTC_Temperature_Table3[] = {
	{-40, 74354},		/* FIX_ME */
	{-35, 74354},		/* FIX_ME */
	{-30, 74354},		/* FIX_ME */
	{-25, 74354},		/* FIX_ME */
	{-20, 74354},
	{-15, 57626},
	{-10, 45068},
	{-5, 35548},
	{0, 28267},
	{5, 22650},
	{10, 18280},
	{15, 14855},
	{20, 12151},
	{25, 10000},		/* 10K */
	{30, 8279},
	{35, 6892},
	{40, 5768},
	{45, 4852},
	{50, 4101},
	{55, 3483},
	{60, 2970},
	{60, 2970},		/* FIX_ME */
	{60, 2970},		/* FIX_ME */
	{60, 2970},		/* FIX_ME */
	{60, 2970},		/* FIX_ME */
	{60, 2970},		/* FIX_ME */
	{60, 2970},		/* FIX_ME */
	{60, 2970},		/* FIX_ME */
	{60, 2970},		/* FIX_ME */
	{60, 2970},		/* FIX_ME */
	{60, 2970},		/* FIX_ME */
	{60, 2970},		/* FIX_ME */
	{60, 2970},		/* FIX_ME */
	{60, 2970}		/* FIX_ME */
};

#if 0
/* AP_NTC_10 */
static struct PMINTC_TEMPERATURE PMINTC_Temperature_Table4[] = {
	{-20, 68237},
	{-15, 53650},
	{-10, 42506},
	{-5, 33892},
	{0, 27219},
	{5, 22021},
	{10, 17926},
	{15, 14674},
	{20, 12081},
	{25, 10000},
	{30, 8315},
	{35, 6948},
	{40, 5834},
	{45, 4917},
	{50, 4161},
	{55, 3535},
	{60, 3014}
};
#else
/* AP_NTC_10(TSM0A103F34D1RZ) */
static struct PMINTC_TEMPERATURE PMINTC_Temperature_Table4[] = {
	{-40, 188500},
	{-35, 144290},
	{-30, 111330},
	{-25, 86560},
	{-20, 67790},
	{-15, 53460},
	{-10, 42450},
	{-5, 33930},
	{0, 27280},
	{5, 22070},
	{10, 17960},
	{15, 14700},
	{20, 12090},
	{25, 10000},		/* 10K */
	{30, 8310},
	{35, 6940},
	{40, 5830},
	{45, 4910},
	{50, 4160},
	{55, 3540},
	{60, 3020},
	{65, 2590},
	{70, 2230},
	{75, 1920},
	{80, 1670},
	{85, 1450},
	{90, 1270},
	{95, 1110},
	{100, 975},
	{105, 860},
	{110, 760},
	{115, 674},
	{120, 599},
	{125, 534}
};
#endif

/* AP_NTC_47 */
static struct PMINTC_TEMPERATURE PMINTC_Temperature_Table5[] = {
	{-40, 483954},		/* FIX_ME */
	{-35, 483954},		/* FIX_ME */
	{-30, 483954},		/* FIX_ME */
	{-25, 483954},		/* FIX_ME */
	{-20, 483954},
	{-15, 360850},
	{-10, 271697},
	{-5, 206463},
	{0, 158214},
	{5, 122259},
	{10, 95227},
	{15, 74730},
	{20, 59065},
	{25, 47000},		/* 47K */
	{30, 37643},
	{35, 30334},
	{40, 24591},
	{45, 20048},
	{50, 16433},
	{55, 13539},
	{60, 11210},
	{60, 11210},		/* FIX_ME */
	{60, 11210},		/* FIX_ME */
	{60, 11210},		/* FIX_ME */
	{60, 11210},		/* FIX_ME */
	{60, 11210},		/* FIX_ME */
	{60, 11210},		/* FIX_ME */
	{60, 11210},		/* FIX_ME */
	{60, 11210},		/* FIX_ME */
	{60, 11210},		/* FIX_ME */
	{60, 11210},		/* FIX_ME */
	{60, 11210},		/* FIX_ME */
	{60, 11210},		/* FIX_ME */
	{60, 11210}		/* FIX_ME */
};


/* NTCG104EF104F(100K) */
static struct PMINTC_TEMPERATURE PMINTC_Temperature_Table6[] = {
	{-40, 4251000},
	{-35, 3005000},
	{-30, 2149000},
	{-25, 1554000},
	{-20, 1135000},
	{-15, 837800},
	{-10, 624100},
	{-5, 469100},
	{0, 355600},
	{5, 271800},
	{10, 209400},
	{15, 162500},
	{20, 127000},
	{25, 100000},		/* 100K */
	{30, 79230},
	{35, 63180},
	{40, 50680},
	{45, 40900},
	{50, 33190},
	{55, 27090},
	{60, 22220},
	{65, 18320},
	{70, 15180},
	{75, 12640},
	{80, 10580},
	{85, 8887},
	{90, 7500},
	{95, 6357},
	{100, 5410},
	{105, 4623},
	{110, 3965},
	{115, 3415},
	{120, 2951},
	{125, 2560}
};

/* NCP15WF104F03RC(100K) */
static struct PMINTC_TEMPERATURE PMINTC_Temperature_Table7[] = {
	{-40, 4397119},
	{-35, 3088599},
	{-30, 2197225},
	{-25, 1581881},
	{-20, 1151037},
	{-15, 846579},
	{-10, 628988},
	{-5, 471632},
	{0, 357012},
	{5, 272500},
	{10, 209710},
	{15, 162651},
	{20, 127080},
	{25, 100000},		/* 100K */
	{30, 79222},
	{35, 63167},
#if defined(APPLY_PRECISE_NTC_TABLE)
	{40, 50677},
	{41, 48528},
	{42, 46482},
	{43, 44533},
	{44, 42675},
	{45, 40904},
	{46, 39213},
	{47, 37601},
	{48, 36063},
	{49, 34595},
	{50, 33195},
	{51, 31859},
	{52, 30584},
	{53, 29366},
	{54, 28203},
	{55, 27091},
	{56, 26028},
	{57, 25013},
	{58, 24042},
	{59, 23113},
	{60, 22224},
	{61, 21374},
	{62, 20560},
	{63, 19782},
	{64, 19036},
	{65, 18322},
	{66, 17640},
	{67, 16986},
	{68, 16360},
	{69, 15759},
	{70, 15184},
	{71, 14631},
	{72, 14100},
	{73, 13591},
	{74, 13103},
	{75, 12635},
	{76, 12187},
	{77, 11756},
	{78, 11343},
	{79, 10946},
	{80, 10565},
	{81, 10199},
	{82,  9847},
	{83,  9509},
	{84,  9184},
	{85,  8872},
	{86,  8572},
	{87,  8283},
	{88,  8005},
	{89,  7738},
	{90,  7481},
#else
	{40, 50677},
	{45, 40904},
	{50, 33195},
	{55, 27091},
	{60, 22224},
	{65, 18323},
	{70, 15184},
	{75, 12635},
	{80, 10566},
	{85, 8873},
	{90, 7481},
#endif
	{95, 6337},
	{100, 5384},
	{105, 4594},
	{110, 3934},
	{115, 3380},
	{120, 2916},
	{125, 2522}
};

static struct PMINTC_TEMPERATURE PMINTC_Temperature_Table8[] = {
	{-40, 188020},
	{-35, 142787},
	{-30, 113347},
	{-25, 84822},
	{-20, 66269},
	{-15, 52228},
	{-10, 41476},
	{-5, 33146},
	{0, 26678},
	{5, 21629},
	{10, 17643},
	{15, 14472},
	{20, 11937},
	{25, 9900},
	{30, 8246},
	{35, 6853},
	{40, 5744},
	{41, 5547},
	{42, 5358},
	{43, 5176},
	{44, 5001},
	{45, 4833},
	{46, 4671},
	{47, 4515},
	{48, 4366},
	{49, 4222},
	{50, 4083},
	{51, 3949},
	{52, 3821},
	{53, 3697},
	{54, 3578},
	{55, 3463},
	{56, 3352},
	{57, 3241},
	{58, 3143},
	{59, 3044},
	{60, 2948},
	{61, 2857},
	{62, 2770},
	{63, 2685},
	{64, 2604},
	{65, 2525},
	{66, 2450},
	{67, 2377},
	{68, 2306},
	{69, 2238},
	{70, 2172},
	{71, 2108},
	{72, 2046},
	{73, 1987},
	{74, 1929},
	{75, 1874},
	{76, 1820},
	{77, 1768},
	{78, 1718},
	{79, 1669},
	{80, 1622},
	{81, 1577},
	{82, 1533},
	{83, 1490},
	{84, 1449},
	{85, 1410},
	{86, 1371},
	{87, 1334},
	{88, 1298},
	{89, 1263},
	{90, 1196},
	{95, 1074},
	{100, 941},
	{105, 828},
	{110, 731},
	{115, 647},
	{120, 574},
	{125, 510}
};

void get_ntc_in_dts(void)
{
	struct device_node *node = NULL;
	node = of_find_compatible_node(NULL, NULL, "mediatek,therm_ntc_pmic");
	if (node != NULL) {
		if (of_property_read_u32(node, "ntc_enable", &dts_ntc_status) >= 0) {
			pr_err("pmic NTC enable is %d\n", dts_ntc_status);
		} else {
		pr_err("get ntc_enable in dts fail\n");}
	} else {
		pr_err("get pmic dts fail\n");}
}

/* convert register to temperature  */
static __s32 mtkts_pmintc_thermistor_conver_temp(__s32 Res)
{
	int i = 0;
	int asize = 0;
	__s32 RES1 = 0, RES2 = 0;
	__s32 TAP_Value = -200, TMP1 = 0, TMP2 = 0;

	asize = (ntc_tbl_size / sizeof(struct PMINTC_TEMPERATURE));
	/* mtkts_pmintc_dprintk("%s() :
	 * asize = %d, Res = %d\n", __func__,asize,Res);
	 */
	if (Res >= PMINTC_Temperature_Table[0].TemperatureR) {
		TAP_Value = -40;	/* min */
	} else if (Res <= PMINTC_Temperature_Table[asize - 1].TemperatureR) {
		TAP_Value = 125;	/* max */
	} else {
		RES1 = PMINTC_Temperature_Table[0].TemperatureR;
		TMP1 = PMINTC_Temperature_Table[0].PMINTC_Temp;
		/* mtkts_pmintc_dprintk("%d : RES1 = %d,TMP1 = %d\n",__LINE__,
		 * RES1,TMP1);
		 */

		for (i = 0; i < asize; i++) {
			if (Res >= PMINTC_Temperature_Table[i].TemperatureR) {
				RES2 = PMINTC_Temperature_Table[i]
					.TemperatureR;

				TMP2 = PMINTC_Temperature_Table[i]
					.PMINTC_Temp;
				/* mtkts_pmintc_dprintk("%d :i=%d, RES2 = %d,
				 * TMP2 = %d\n",__LINE__,i,RES2,TMP2);
				 */
				break;
			}
			RES1 = PMINTC_Temperature_Table[i].TemperatureR;
			TMP1 = PMINTC_Temperature_Table[i].PMINTC_Temp;
			/* mtkts_pmintc_dprintk("%d :i=%d, RES1 = %d,
			 * TMP1 = %d\n",__LINE__,i,RES1,TMP1);
			 */
		}

		TAP_Value = (((Res - RES2) * TMP1) + ((RES1 - Res) * TMP2))
								/ (RES1 - RES2);
	}

#if 0
	mtkts_pmintc_dprintk(
			"%s() : TAP_Value = %d\n", __func__,
			TAP_Value);
	mtkts_pmintc_dprintk(
			"%s() : Res = %d\n", __func__,
			Res);
	mtkts_pmintc_dprintk(
			"%s() : RES1 = %d\n", __func__,
			RES1);
	mtkts_pmintc_dprintk(
			"%s() : RES2 = %d\n", __func__,
			RES2);
	mtkts_pmintc_dprintk(
			"%s() : TMP1 = %d\n", __func__,
			TMP1);
	mtkts_pmintc_dprintk(
			"%s() : TMP2 = %d\n", __func__,
			TMP2);
#endif

	return TAP_Value;
}

/* convert ADC_AP_temp_volt to register */
/*Volt to Temp formula same with 6589*/
static __s32 mtk_ts_pmintc_volt_to_temp(__u32 dwVolt)
{
	__s32 TRes;
	__u64 dwVCriAP = 0;
	__u64 dwVCriAP2 = 0;
	__s32 PMINTC_TMP = -100;

	/* SW workaround-----------------------------------------------------
	 * dwVCriAP = (TAP_OVER_CRITICAL_LOW * 1800) /
	 * (TAP_OVER_CRITICAL_LOW + 39000);
	 * dwVCriAP = (TAP_OVER_CRITICAL_LOW * RAP_PULL_UP_VOLT) /
	 * (TAP_OVER_CRITICAL_LOW + RAP_PULL_UP_R);
	 */

	dwVCriAP = ((__u64)g_TAP_over_critical_low *
		(__u64)g_RAP_pull_up_voltage);
	dwVCriAP2 = (g_TAP_over_critical_low + g_RAP_pull_up_R);
	do_div(dwVCriAP, dwVCriAP2);


	if (dwVolt > ((__u32)dwVCriAP)) {
		TRes = g_TAP_over_critical_low;
	} else {
		/* TRes = (39000*dwVolt) / (1800-dwVolt);
		 * TRes = (RAP_PULL_UP_R*dwVolt) / (RAP_PULL_UP_VOLT-dwVolt);
		 */
		TRes = (g_RAP_pull_up_R * dwVolt)
				/ (g_RAP_pull_up_voltage - dwVolt);
	}
	/* ------------------------------------------------------------------ */

	g_pmintc_TemperatureR = TRes;

	/* convert register to temperature */
	PMINTC_TMP = mtkts_pmintc_thermistor_conver_temp(TRes);

	return PMINTC_TMP;
}

static int get_hw_pmintc_temp(void)
{
	int ret = 0, output;
	unsigned int ntctemp = 0;
	struct charger_device *pmi_ntc = NULL;

	pmi_ntc = get_charger_by_name("primary_chg");
	if(!pmi_ntc){
		pr_err("get_hw_pmintc_temp fail to get charger_device of primary_chg\n");
		return 0;
	}
	ret = charger_dev_get_ts_adc(pmi_ntc,&ntctemp);
	mtkts_pmintc_dprintk("get_hw_pmintc_temp ntctemp=%d\n", ntctemp);

	if (ret < 0) {
		mtkts_pmintc_printk("PMI get temp failed %d\n", ret);
		return ret;
	}

	ntctemp = ntctemp / 1000;
	mtkts_pmintc_dprintk("APtery output mV = %d\n", ntctemp);
	output = mtk_ts_pmintc_volt_to_temp(ntctemp);
	mtkts_pmintc_dprintk("PMINTC output temperature = %d\n", output);
	return output;
}

static DEFINE_MUTEX(PMINTC_lock);
/*int ts_pmintc_at_boot_time = 0;*/
int mtkts_pmintc_get_hw_temp(void)
{
	int t_ret = 0;

#if MTKTS_PMINTC_SW_FILTER
	static int pre_temp1;
#define DELTA_TEMP 2000
#endif
	mutex_lock(&PMINTC_lock);

	/* get HW AP temp (TSAP) */
	/* cat /sys/class/power_supply/AP/AP_temp */
	t_ret = get_hw_pmintc_temp();
	t_ret = t_ret * 1000;

#if MTKTS_PMINTC_SW_FILTER
	if ((t_ret > 100000) || (t_ret < -30000)) {
		mtkts_pmintc_printk(
			"[Power/PMINTC_Thermal] drop this data\n");
		t_ret = pre_temp1;
	} else if ((pre_temp1 != 0)
		&& (((pre_temp1 - t_ret) >= DELTA_TEMP)
			|| ((t_ret - pre_temp1) >= DELTA_TEMP))) {
		mtkts_pmintc_dprintk(
			"[Power/PMINTC_Thermal] t_ret=%d,(t_ret-pre_temp1)=%d,(pre_temp1-t_ret)=%d\n",
			t_ret, (t_ret - pre_temp1), (pre_temp1 - t_ret));
		t_ret = pre_temp1;
	} else {
		/* update previous temp */
		pre_temp1 = t_ret;
		mtkts_pmintc_dprintk("[Power/PMINTC_Thermal] pre_temp1=%d\n",
								pre_temp1);
	}
#endif

	mutex_unlock(&PMINTC_lock);

	if (t_ret > 70000)	/* abnormal high temp */
		mtkts_pmintc_printk("T_pmintc=%d\n", t_ret);

	mtkts_pmintc_dprintk("[%s] T_pmintc, %d\n", __func__,
									t_ret);
	return t_ret;
}

static int mtkts_pmintc_get_temp(struct thermal_zone_device *thermal, int *t)
{
	*t = mtkts_pmintc_get_hw_temp();

	if ((int)*t > 52000)
		mtkts_pmintc_dprintk("T=%d\n", (int)*t);

	if ((int)*t >= polling_trip_temp1)
		thermal->polling_delay = interval * 1000;
	else if ((int)*t < polling_trip_temp2)
		thermal->polling_delay = interval * polling_factor2;
	else
		thermal->polling_delay = interval * polling_factor1;

	return 0;
}

static int mtkts_pmintc_bind(struct thermal_zone_device *thermal,
			      struct thermal_cooling_device *cdev)
{
	int table_val = 0;

	if (!strcmp(cdev->type, g_bind0)) {
		table_val = 0;
		mtkts_pmintc_dprintk("[%s] %s\n", __func__, cdev->type);
	} else if (!strcmp(cdev->type, g_bind1)) {
		table_val = 1;
		mtkts_pmintc_dprintk("[%s] %s\n", __func__, cdev->type);
	} else if (!strcmp(cdev->type, g_bind2)) {
		table_val = 2;
		mtkts_pmintc_dprintk("[%s] %s\n", __func__, cdev->type);
	} else if (!strcmp(cdev->type, g_bind3)) {
		table_val = 3;
		mtkts_pmintc_dprintk("[%s] %s\n", __func__, cdev->type);
	} else if (!strcmp(cdev->type, g_bind4)) {
		table_val = 4;
		mtkts_pmintc_dprintk("[%s] %s\n", __func__, cdev->type);
	} else if (!strcmp(cdev->type, g_bind5)) {
		table_val = 5;
		mtkts_pmintc_dprintk("[%s] %s\n", __func__, cdev->type);
	} else if (!strcmp(cdev->type, g_bind6)) {
		table_val = 6;
		mtkts_pmintc_dprintk("[%s] %s\n", __func__, cdev->type);
	} else if (!strcmp(cdev->type, g_bind7)) {
		table_val = 7;
		mtkts_pmintc_dprintk("[%s] %s\n", __func__, cdev->type);
	} else if (!strcmp(cdev->type, g_bind8)) {
		table_val = 8;
		mtkts_pmintc_dprintk("[%s] %s\n", __func__, cdev->type);
	} else if (!strcmp(cdev->type, g_bind9)) {
		table_val = 9;
		mtkts_pmintc_dprintk("[%s] %s\n", __func__, cdev->type);
	} else {
		return 0;
	}

	if (mtk_thermal_zone_bind_cooling_device(thermal, table_val, cdev)) {
		mtkts_pmintc_dprintk(
			"[%s] error binding cooling dev\n", __func__);
		return -EINVAL;
	}

	mtkts_pmintc_dprintk("[%s] binding OK, %d\n", __func__,
								table_val);
	return 0;
}

static int mtkts_pmintc_unbind(struct thermal_zone_device *thermal,
				struct thermal_cooling_device *cdev)
{
	int table_val = 0;

	if (!strcmp(cdev->type, g_bind0)) {
		table_val = 0;
		mtkts_pmintc_dprintk("[%s] %s\n", __func__,
								cdev->type);
	} else if (!strcmp(cdev->type, g_bind1)) {
		table_val = 1;
		mtkts_pmintc_dprintk("[%s] %s\n", __func__,
								cdev->type);
	} else if (!strcmp(cdev->type, g_bind2)) {
		table_val = 2;
		mtkts_pmintc_dprintk("[%s] %s\n", __func__,
								cdev->type);
	} else if (!strcmp(cdev->type, g_bind3)) {
		table_val = 3;
		mtkts_pmintc_dprintk("[%s] %s\n", __func__,
								cdev->type);
	} else if (!strcmp(cdev->type, g_bind4)) {
		table_val = 4;
		mtkts_pmintc_dprintk("[%s] %s\n", __func__,
								cdev->type);
	} else if (!strcmp(cdev->type, g_bind5)) {
		table_val = 5;
		mtkts_pmintc_dprintk("[%s] %s\n", __func__,
								cdev->type);
	} else if (!strcmp(cdev->type, g_bind6)) {
		table_val = 6;
		mtkts_pmintc_dprintk("[%s] %s\n", __func__,
								cdev->type);
	} else if (!strcmp(cdev->type, g_bind7)) {
		table_val = 7;
		mtkts_pmintc_dprintk("[%s] %s\n", __func__,
								cdev->type);
	} else if (!strcmp(cdev->type, g_bind8)) {
		table_val = 8;
		mtkts_pmintc_dprintk("[%s] %s\n", __func__,
								cdev->type);
	} else if (!strcmp(cdev->type, g_bind9)) {
		table_val = 9;
		mtkts_pmintc_dprintk("[%s] %s\n", __func__,
								cdev->type);
	} else
		return 0;

	if (thermal_zone_unbind_cooling_device(thermal, table_val, cdev)) {
		mtkts_pmintc_dprintk(
			"[%s] error unbinding cooling dev\n", __func__);
		return -EINVAL;
	}

	mtkts_pmintc_dprintk("[%s] unbinding OK\n", __func__);
	return 0;
}

static int mtkts_pmintc_get_mode(struct thermal_zone_device *thermal,
				  enum thermal_device_mode *mode)
{
	*mode = (kernelmode) ? THERMAL_DEVICE_ENABLED : THERMAL_DEVICE_DISABLED;
	return 0;
}

static int mtkts_pmintc_set_mode(struct thermal_zone_device *thermal,
				  enum thermal_device_mode mode)
{
	kernelmode = mode;
	return 0;
}

static int mtkts_pmintc_get_trip_type(
struct thermal_zone_device *thermal, int trip, enum thermal_trip_type *type)
{
	*type = g_THERMAL_TRIP[trip];
	return 0;
}

static int mtkts_pmintc_get_trip_temp(
struct thermal_zone_device *thermal, int trip, int *temp)
{
	*temp = trip_temp[trip];
	return 0;
}

static int mtkts_pmintc_get_crit_temp(
struct thermal_zone_device *thermal, int *temperature)
{
	*temperature = MTKTS_PMINTC_TEMP_CRIT;
	return 0;
}

/* bind callback functions to thermalzone */
static struct thermal_zone_device_ops mtkts_pmintc_dev_ops = {
	.bind = mtkts_pmintc_bind,
	.unbind = mtkts_pmintc_unbind,
	.get_temp = mtkts_pmintc_get_temp,
	.get_mode = mtkts_pmintc_get_mode,
	.set_mode = mtkts_pmintc_set_mode,
	.get_trip_type = mtkts_pmintc_get_trip_type,
	.get_trip_temp = mtkts_pmintc_get_trip_temp,
	.get_crit_temp = mtkts_pmintc_get_crit_temp,
};



static int mtkts_pmintc_read(struct seq_file *m, void *v)
{

	seq_printf(m,
		"[%s] trip_0_temp=%d,trip_1_temp=%d,trip_2_temp=%d,trip_3_temp=%d,\n",
		__func__,
		trip_temp[0], trip_temp[1], trip_temp[2], trip_temp[3]);

	seq_printf(m,
		"trip_4_temp=%d,trip_5_temp=%d,trip_6_temp=%d,trip_7_temp=%d,trip_8_temp=%d,trip_9_temp=%d,\n",
		trip_temp[4], trip_temp[5], trip_temp[6],
		trip_temp[7], trip_temp[8], trip_temp[9]);

	seq_printf(m,
		"g_THERMAL_TRIP_0=%d,g_THERMAL_TRIP_1=%d,g_THERMAL_TRIP_2=%d,g_THERMAL_TRIP_3=%d,\n",
		g_THERMAL_TRIP[0], g_THERMAL_TRIP[1],
		g_THERMAL_TRIP[2], g_THERMAL_TRIP[3]);

	seq_printf(m,
		"g_THERMAL_TRIP_4=%d,g_THERMAL_TRIP_5=%d,g_THERMAL_TRIP_6=%d,g_THERMAL_TRIP_7=%d,\n",
		g_THERMAL_TRIP[4], g_THERMAL_TRIP[5],
		g_THERMAL_TRIP[6], g_THERMAL_TRIP[7]);

	seq_printf(m, "g_THERMAL_TRIP_8=%d,g_THERMAL_TRIP_9=%d,\n",
					g_THERMAL_TRIP[8], g_THERMAL_TRIP[9]);

	seq_printf(m,
		"cooldev0=%s,cooldev1=%s,cooldev2=%s,cooldev3=%s,cooldev4=%s,\n",
		g_bind0, g_bind1, g_bind2, g_bind3, g_bind4);

	seq_printf(m,
		"cooldev5=%s,cooldev6=%s,cooldev7=%s,cooldev8=%s,cooldev9=%s,time_ms=%d\n",
		g_bind5, g_bind6, g_bind7, g_bind8, g_bind9, interval * 1000);

	return 0;
}

static int mtkts_pmintc_register_thermal(void);
static void mtkts_pmintc_unregister_thermal(void);

static ssize_t mtkts_pmintc_write(
struct file *file, const char __user *buffer, size_t count, loff_t *data)
{
	int len = 0;
	int i;
	struct pmintc_data {
		int trip[10];
		int t_type[10];
		char bind0[20], bind1[20], bind2[20], bind3[20], bind4[20];
		char bind5[20], bind6[20], bind7[20], bind8[20], bind9[20];
		int time_msec;
		char desc[512];
	};

	struct pmintc_data *ptr_pmintc_data = kmalloc(
					sizeof(*ptr_pmintc_data), GFP_KERNEL);

	if (ptr_pmintc_data == NULL)
		return -ENOMEM;

	len = (count < (sizeof(ptr_pmintc_data->desc) - 1)) ?
				count : (sizeof(ptr_pmintc_data->desc) - 1);

	if (copy_from_user(ptr_pmintc_data->desc, buffer, len)) {
		kfree(ptr_pmintc_data);
		return 0;
	}

	ptr_pmintc_data->desc[len] = '\0';

	if (sscanf
	    (ptr_pmintc_data->desc,
	     "%d %d %d %19s %d %d %19s %d %d %19s %d %d %19s %d %d %19s %d %d %19s %d %d %19s %d %d %19s %d %d %19s %d %d %19s %d",
		&num_trip,
		&ptr_pmintc_data->trip[0], &ptr_pmintc_data->t_type[0],
		ptr_pmintc_data->bind0,
		&ptr_pmintc_data->trip[1], &ptr_pmintc_data->t_type[1],
		ptr_pmintc_data->bind1,
		&ptr_pmintc_data->trip[2], &ptr_pmintc_data->t_type[2],
		ptr_pmintc_data->bind2,
		&ptr_pmintc_data->trip[3], &ptr_pmintc_data->t_type[3],
		ptr_pmintc_data->bind3,
		&ptr_pmintc_data->trip[4], &ptr_pmintc_data->t_type[4],
		ptr_pmintc_data->bind4,
		&ptr_pmintc_data->trip[5], &ptr_pmintc_data->t_type[5],
		ptr_pmintc_data->bind5,
		&ptr_pmintc_data->trip[6], &ptr_pmintc_data->t_type[6],
		ptr_pmintc_data->bind6,
		&ptr_pmintc_data->trip[7], &ptr_pmintc_data->t_type[7],
		ptr_pmintc_data->bind7,
		&ptr_pmintc_data->trip[8], &ptr_pmintc_data->t_type[8],
		ptr_pmintc_data->bind8,
		&ptr_pmintc_data->trip[9], &ptr_pmintc_data->t_type[9],
		ptr_pmintc_data->bind9,
		&ptr_pmintc_data->time_msec) == 32) {

		down(&sem_mutex);
		mtkts_pmintc_dprintk(
			"[%s] mtkts_pmintc_unregister_thermal\n", __func__);

		mtkts_pmintc_unregister_thermal();

		if (num_trip < 0 || num_trip > 10) {
			#ifdef CONFIG_MTK_AEE_FEATURE
			aee_kernel_warning_api(__FILE__, __LINE__,
					DB_OPT_DEFAULT, "mtkts_pmintc_write",
					"Bad argument");
			#endif
			mtkts_pmintc_dprintk(
				"[%s] bad argument\n", __func__);

			kfree(ptr_pmintc_data);
			up(&sem_mutex);
			return -EINVAL;
		}

		for (i = 0; i < num_trip; i++)
			g_THERMAL_TRIP[i] = ptr_pmintc_data->t_type[i];

		g_bind0[0] = g_bind1[0] = g_bind2[0] = g_bind3[0]
			= g_bind4[0] = g_bind5[0] = g_bind6[0]
			= g_bind7[0] = g_bind8[0] = g_bind9[0] = '\0';

		for (i = 0; i < 20; i++) {
			g_bind0[i] = ptr_pmintc_data->bind0[i];
			g_bind1[i] = ptr_pmintc_data->bind1[i];
			g_bind2[i] = ptr_pmintc_data->bind2[i];
			g_bind3[i] = ptr_pmintc_data->bind3[i];
			g_bind4[i] = ptr_pmintc_data->bind4[i];
			g_bind5[i] = ptr_pmintc_data->bind5[i];
			g_bind6[i] = ptr_pmintc_data->bind6[i];
			g_bind7[i] = ptr_pmintc_data->bind7[i];
			g_bind8[i] = ptr_pmintc_data->bind8[i];
			g_bind9[i] = ptr_pmintc_data->bind9[i];
		}

		mtkts_pmintc_dprintk(
			"[%s] g_THERMAL_TRIP_0=%d,g_THERMAL_TRIP_1=%d,",
			__func__,
			g_THERMAL_TRIP[0], g_THERMAL_TRIP[1]);

		mtkts_pmintc_dprintk(
			"g_THERMAL_TRIP_2=%d,g_THERMAL_TRIP_3=%d,g_THERMAL_TRIP_4=%d,",
			g_THERMAL_TRIP[2], g_THERMAL_TRIP[3],
			g_THERMAL_TRIP[4]);

		mtkts_pmintc_dprintk(
			"g_THERMAL_TRIP_5=%d,g_THERMAL_TRIP_6=%d,g_THERMAL_TRIP_7=%d,",
			g_THERMAL_TRIP[5], g_THERMAL_TRIP[6],
			g_THERMAL_TRIP[7]);

		mtkts_pmintc_dprintk(
			"g_THERMAL_TRIP_8=%d,g_THERMAL_TRIP_9=%d,\n",
			g_THERMAL_TRIP[8], g_THERMAL_TRIP[9]);

		mtkts_pmintc_dprintk(
			"[%s] cooldev0=%s,cooldev1=%s,cooldev2=%s,cooldev3=%s,",
			__func__,
			g_bind0, g_bind1, g_bind2, g_bind3);

		mtkts_pmintc_dprintk(
			"cooldev4=%s,cooldev5=%s,cooldev6=%s,cooldev7=%s,cooldev8=%s,cooldev9=%s\n",
			g_bind4, g_bind5, g_bind6, g_bind7, g_bind8, g_bind9);

		for (i = 0; i < num_trip; i++)
			trip_temp[i] = ptr_pmintc_data->trip[i];

		interval = ptr_pmintc_data->time_msec / 1000;

		mtkts_pmintc_dprintk(
			"[%s] trip_0_temp=%d,trip_1_temp=%d,trip_2_temp=%d,",
			__func__,
			trip_temp[0], trip_temp[1], trip_temp[2]);

		mtkts_pmintc_dprintk(
			"trip_3_temp=%d,trip_4_temp=%d,trip_5_temp=%d,trip_6_temp=%d,trip_7_temp=%d,",
			trip_temp[3], trip_temp[4], trip_temp[5],
			trip_temp[6], trip_temp[7]);

		mtkts_pmintc_dprintk(
			"trip_8_temp=%d,trip_9_temp=%d,time_ms=%d\n",
			trip_temp[8], trip_temp[9], interval * 1000);

		mtkts_pmintc_dprintk(
			"[%s] mtkts_pmintc_register_thermal\n", __func__);
		mtkts_pmintc_register_thermal();
		up(&sem_mutex);

		kfree(ptr_pmintc_data);

		/* AP_write_flag=1; */
		return count;
	}

	mtkts_pmintc_dprintk("[%s] bad argument\n", __func__);
	#ifdef CONFIG_MTK_AEE_FEATURE
	aee_kernel_warning_api(__FILE__, __LINE__, DB_OPT_DEFAULT,
					"mtkts_pmintc_write", "Bad argument");
	#endif
	kfree(ptr_pmintc_data);
	return -EINVAL;
}

void mtkts_pmintc_prepare_table(int table_num)
{

	switch (table_num) {
	case 1:		/* AP_NTC_BL197 */
		PMINTC_Temperature_Table = PMINTC_Temperature_Table1;
		ntc_tbl_size = sizeof(PMINTC_Temperature_Table1);
		break;
	case 2:		/* AP_NTC_TSM_1 */
		PMINTC_Temperature_Table = PMINTC_Temperature_Table2;
		ntc_tbl_size = sizeof(PMINTC_Temperature_Table2);
		break;
	case 3:		/* AP_NTC_10_SEN_1 */
		PMINTC_Temperature_Table = PMINTC_Temperature_Table3;
		ntc_tbl_size = sizeof(PMINTC_Temperature_Table3);
		break;
	case 4:		/* AP_NTC_10 */
		PMINTC_Temperature_Table = PMINTC_Temperature_Table4;
		ntc_tbl_size = sizeof(PMINTC_Temperature_Table4);
		break;
	case 5:		/* AP_NTC_47 */
		PMINTC_Temperature_Table = PMINTC_Temperature_Table5;
		ntc_tbl_size = sizeof(PMINTC_Temperature_Table5);
		break;
	case 6:		/* NTCG104EF104F */
		PMINTC_Temperature_Table = PMINTC_Temperature_Table6;
		ntc_tbl_size = sizeof(PMINTC_Temperature_Table6);
		break;
	case 7:		/* NCP15WF104F03RC */
		PMINTC_Temperature_Table = PMINTC_Temperature_Table7;
		ntc_tbl_size = sizeof(PMINTC_Temperature_Table7);
		break;
	case 8:		/* NCP15XH103F03RC */
		PMINTC_Temperature_Table = PMINTC_Temperature_Table8;
		ntc_tbl_size = sizeof(PMINTC_Temperature_Table8);
		break;
	default:		/* AP_NTC_10 */
		PMINTC_Temperature_Table = PMINTC_Temperature_Table4;
		ntc_tbl_size = sizeof(PMINTC_Temperature_Table4);
		break;
	}

	pr_notice("[Thermal/TZ/PMINTC] %s table_num=%d\n",
						__func__, table_num);

#if 0
	{
		int i = 0;

		for (i = 0; i < (ntc_tbl_size
					/ sizeof(struct PMINTC_TEMPERATURE));
		     i++) {
			pr_notice(
				"PMINTC_Temperature_Table[%d].APteryTemp =%d\n",
				i, PMINTC_Temperature_Table[i].PMINTC_Temp);
			pr_notice(
				"PMINTC_Temperature_Table[%d].TemperatureR=%d\n",
				i, PMINTC_Temperature_Table[i].TemperatureR);
		}
	}
#endif
}

static int mtkts_pmintc_param_read(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n", g_RAP_pull_up_R);
	seq_printf(m, "%d\n", g_RAP_pull_up_voltage);
	seq_printf(m, "%d\n", g_TAP_over_critical_low);
	seq_printf(m, "%d\n", g_RAP_ntc_table);
	seq_printf(m, "%d\n", g_RAP_ADC_channel);

	return 0;
}


static ssize_t mtkts_pmintc_param_write(
struct file *file, const char __user *buffer, size_t count, loff_t *data)
{
	int len = 0;
	struct param_data {
		char desc[512];
		char pull_R[10], pull_V[10];
		char overcrilow[16];
		char NTC_TABLE[10];
		unsigned int valR, valV, over_cri_low, ntc_table;
	};
	/* external pin: 0/1/12/13/14/15, can't use pin:2/3/4/5/6/7/8/9/10/11,
	 * choose "adc_channel=11" to check if there is any param input
	 */
	unsigned int adc_channel = 11;

	struct param_data *ptr_param_data = kmalloc(
					sizeof(*ptr_param_data), GFP_KERNEL);

	if (ptr_param_data == NULL)
		return -ENOMEM;

	len = (count < (sizeof(ptr_param_data->desc) - 1)) ?
				count : (sizeof(ptr_param_data->desc) - 1);

	if (copy_from_user(ptr_param_data->desc, buffer, len)) {
		kfree(ptr_param_data);
		return 0;
	}
	ptr_param_data->desc[len] = '\0';


	mtkts_pmintc_dprintk("[%s]\n", __func__);



	if (sscanf
	    (ptr_param_data->desc, "%9s %d %9s %d %15s %d %9s %d %d",
			ptr_param_data->pull_R, &ptr_param_data->valR,
			ptr_param_data->pull_V, &ptr_param_data->valV,
			ptr_param_data->overcrilow,
			&ptr_param_data->over_cri_low,
			ptr_param_data->NTC_TABLE,
			&ptr_param_data->ntc_table, &adc_channel) >= 8) {

		if (!strcmp(ptr_param_data->pull_R, "PUP_R")) {
			g_RAP_pull_up_R = ptr_param_data->valR;
			mtkts_pmintc_dprintk("g_RAP_pull_up_R=%d\n",
							g_RAP_pull_up_R);
		} else {
			kfree(ptr_param_data);
			mtkts_pmintc_printk(
					"[mtkts_pmintc_write] bad PUP_R argument\n");
			return -EINVAL;
		}

		if (!strcmp(ptr_param_data->pull_V, "PUP_VOLT")) {
			g_RAP_pull_up_voltage = ptr_param_data->valV;
			mtkts_pmintc_dprintk("g_Rat_pull_up_voltage=%d\n",
							g_RAP_pull_up_voltage);
		} else {
			kfree(ptr_param_data);
			mtkts_pmintc_printk(
					"[mtkts_pmintc_write] bad PUP_VOLT argument\n");
			return -EINVAL;
		}

		if (!strcmp(ptr_param_data->overcrilow, "OVER_CRITICAL_L")) {
			g_TAP_over_critical_low = ptr_param_data->over_cri_low;
			mtkts_pmintc_dprintk("g_TAP_over_critical_low=%d\n",
					      g_TAP_over_critical_low);
		} else {
			kfree(ptr_param_data);
			mtkts_pmintc_printk(
					"[mtkts_pmintc_write] bad OVERCRIT_L argument\n");
			return -EINVAL;
		}

		if (!strcmp(ptr_param_data->NTC_TABLE, "NTC_TABLE")) {
			g_RAP_ntc_table = ptr_param_data->ntc_table;
			mtkts_pmintc_dprintk("g_RAP_ntc_table=%d\n",
							g_RAP_ntc_table);
		} else {
			kfree(ptr_param_data);
			mtkts_pmintc_printk(
					"[mtkts_pmintc_write] bad NTC_TABLE argument\n");
			return -EINVAL;
		}

		/* external pin: 0/1/12/13/14/15,
		 * can't use pin:2/3/4/5/6/7/8/9/10/11,
		 * choose "adc_channel=11" to check if there is any param input
		 */
		if ((adc_channel >= 2) && (adc_channel <= 11))
			/* check unsupport pin value, if unsupport,
			 * set channel = 1 as default setting.
			 */
			g_RAP_ADC_channel = AUX_IN1_NTC;
		else {
			g_RAP_ADC_channel = adc_channel;
		}
		mtkts_pmintc_dprintk("adc_channel=%d\n", adc_channel);
		mtkts_pmintc_dprintk("g_RAP_ADC_channel=%d\n",
						g_RAP_ADC_channel);

		mtkts_pmintc_prepare_table(g_RAP_ntc_table);

		kfree(ptr_param_data);
		return count;
	}

	mtkts_pmintc_printk("[mtkts_pmintc_write] bad argument\n");
	kfree(ptr_param_data);
	return -EINVAL;
}

/* int  mtkts_pmintc_register_cooler(void)
 * {
 *	cooling devices
 *	cl_dev_sysrst = mtk_thermal_cooling_device_register(
 *				"mtktspmintctery-sysrst", NULL,
 *				&mtkts_pmintc_cooling_sysrst_ops);
 *	return 0;
 * }
 */

#if 0
static void mtkts_pmintc_cancel_thermal_timer(void)
{
	/* cancel timer
	 * mtkts_pmintc_printk("mtkts_pmintc_cancel_thermal_timer\n");
	 *
	 * stop thermal framework polling when entering deep idle
	 * if (thz_dev)
	 *	cancel_delayed_work(&(thz_dev->poll_queue));
	 */
}


static void mtkts_pmintc_start_thermal_timer(void)
{
	/* mtkts_pmintc_printk("mtkts_pmintc_start_thermal_timer\n");
	 * resume thermal framework polling when leaving deep idle
	 * if (thz_dev != NULL && interval != 0)
	 *	mod_delayed_work(system_freezable_power_efficient_wq,
	 *			&(thz_dev->poll_queue), round_jiffies(
	 *			msecs_to_jiffies(3000)));
	 */
}
#endif

static int mtkts_pmintc_register_thermal(void)
{
	mtkts_pmintc_dprintk("[%s]\n", __func__);

	/* trips : trip 0~1 */
	thz_dev = mtk_thermal_zone_device_register("charger1", num_trip,
					NULL, &mtkts_pmintc_dev_ops, 0, 0, 0,
					interval * 1000);

	return 0;
}

/* void mtkts_pmintc_unregister_cooler(void) */
/* { */
	/* if (cl_dev_sysrst) { */
	/* mtk_thermal_cooling_device_unregister(cl_dev_sysrst); */
	/* cl_dev_sysrst = NULL; */
	/* } */
/* } */
static void mtkts_pmintc_unregister_thermal(void)
{
	mtkts_pmintc_dprintk("[%s]\n", __func__);

	if (thz_dev) {
		mtk_thermal_zone_device_unregister(thz_dev);
		thz_dev = NULL;
	}
}

static int mtkts_pmintc_open(struct inode *inode, struct file *file)
{
	return single_open(file, mtkts_pmintc_read, NULL);
}

static const struct file_operations mtkts_pmintc_fops = {
	.owner = THIS_MODULE,
	.open = mtkts_pmintc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.write = mtkts_pmintc_write,
	.release = single_release,
};


static int mtkts_pmintc_param_open(struct inode *inode, struct file *file)
{
	return single_open(file, mtkts_pmintc_param_read, NULL);
}

static const struct file_operations mtkts_pmintc_param_fops = {
	.owner = THIS_MODULE,
	.open = mtkts_pmintc_param_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.write = mtkts_pmintc_param_write,
	.release = single_release,
};

static int __init mtkts_pmintc_init(void)
{
	struct proc_dir_entry *entry = NULL;
	struct proc_dir_entry *mtkts_pmintc_dir = NULL;

	mtkts_pmintc_dprintk("[%s]\n", __func__);

	get_ntc_in_dts();
	if (!dts_ntc_status)
		return 0;

	/* setup default table */
	mtkts_pmintc_prepare_table(g_RAP_ntc_table);

	mtkts_pmintc_dir = mtk_thermal_get_proc_drv_therm_dir_entry();
	if (!mtkts_pmintc_dir) {
		mtkts_pmintc_dprintk(
			"[%s]: mkdir /proc/driver/thermal failed\n", __func__);
	} else {
		entry = proc_create("tz6360ntc", 0664, mtkts_pmintc_dir,
				&mtkts_pmintc_fops);
		if (entry)
			proc_set_user(entry, uid, gid);

		entry = proc_create("tz6360ntc_param", 0664, mtkts_pmintc_dir,
				&mtkts_pmintc_param_fops);
		if (entry)
			proc_set_user(entry, uid, gid);
	}

	mtkts_pmintc_register_thermal();
#if 0
	mtkTTimer_register("tz6360ntc", mtkts_pmintc_start_thermal_timer,
					mtkts_pmintc_cancel_thermal_timer);
#endif
	return 0;
}

static void __exit mtkts_pmintc_exit(void)
{
	mtkts_pmintc_dprintk("[%s]\n", __func__);
	mtkts_pmintc_unregister_thermal();
#if 0
	mtkTTimer_unregister("tz6360ntc");
#endif
	/* mtkts_pmintc_unregister_cooler(); */
}

module_init(mtkts_pmintc_init);
module_exit(mtkts_pmintc_exit);
