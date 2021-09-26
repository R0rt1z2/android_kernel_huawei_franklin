/*
 * sgm41511h_charger.h
 *
 * sgm41511h driver
 *
 * Copyright (c) 2019-2019 Huawei Technologies Co., Ltd.
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

#ifndef SGM41511H_CHARGER_H
#define SGM41511H_CHARGER_H

#include <linux/i2c.h>
#include <linux/device.h>
#include <linux/workqueue.h>
#include "mtk_charger_intf.h"

struct sgm41511h_device_info {
	struct i2c_client *client;
	struct charger_device *chg_dev;
	struct charger_properties chg_props;
	struct device *dev;
	struct work_struct irq_work;
	struct power_supply *psy;
	enum charger_type chg_type;
	bool chg_det_enable;
	int gpio_cd;
	int gpio_int;
	int irq_int;
	int irq_active;
	int hiz_iin_limit;
	int fcp_support;
	int gpio_fcp;
	bool charge_enabled;
	int eoc;
};

#define SGM41511H_REG_NUM                            13
/* Register 00h */
#define SGM41511H_REG_ISC                            0x00

#define SGM41511H_REG_ISC_EN_HIZ_MASK                0x80
#define SGM41511H_REG_ISC_EN_HIZ_SHIFT               7
#define REG00_HIZ_ENABLE                             1
#define REG00_HIZ_DISABLE                            0

#define SGM41511H_REG_ISC_STAT_CTRL_MASK             0x60
#define SGM41511H_REG_ISC_STAT_CTRL_SHIFT            5
#define REG00_STAT_CTRL_STAT                         0
#define REG00_STAT_CTRL_ICHG                         1
#define REG00_STAT_CTRL_IINDPM                       2
#define REG00_STAT_CTRL_DISABLE                      3

#define SGM41511H_REG_ISC_IINLIM_MASK                0x1F
#define SGM41511H_REG_ISC_IINLIM_SHIFT               0
#define IINLIM_LSB                                   100
#define IINLIM_BASE                                  100
#define IINLIM_INIT                                  500000

/* Register 01h */
#define SGM41511H_REG_POC                            0x01

#define SGM41511H_REG_POC_PFM_DIS_MASK               0x80
#define SGM41511H_REG_POC_PFM_DIS_SHIFT              7
#define REG01_PFM_ENABLE                             0
#define REG01_PFM_DISABLE                            1

#define SGM41511H_REG_POC_WDT_RESET_MASK             0x40
#define SGM41511H_REG_POC_WDT_RESET_SHIFT            6
#define REG01_WDT_RESET                              1

#define SGM41511H_REG_POC_OTG_CONFIG_MASK            0x20
#define SGM41511H_REG_POC_OTG_CONFIG_SHIFT           5
#define REG01_OTG_ENABLE                             1
#define REG01_OTG_DISABLE                            0

#define SGM41511H_REG_POC_CHG_CONFIG_MASK            0x10
#define SGM41511H_REG_POC_CHG_CONFIG_SHIFT           4
#define REG01_CHG_DISABLE                            0
#define REG01_CHG_ENABLE                             1

#define SGM41511H_REG_POC_SYS_MIN_MASK               0x0E
#define SGM41511H_REG_POC_SYS_MIN_SHIFT              1

#define SGM41511H_REG_POC_MIN_VBAT_SEL_MASK          0x01
#define SGM41511H_REG_POC_MIN_VBAT_SEL_SHIFT         0
#define REG01_MIN_VBAT_2P8V                          0
#define REG01_MIN_VBAT_2P5V                          1

/* Register 02h */
#define SGM41511H_REG_CCC                            0x02

#define SGM41511H_REG_CCC_BOOST_LIM_MASK             0x80
#define SGM41511H_REG_CCC_BOOST_LIM_SHIFT            7
#define REG02_BOOST_LIM_0P5A                         0
#define REG02_BOOST_LIM_1P2A                         1
#define BOOST_LIM_0P5A                               500

#define SGM41511H_REG_CCC_Q1_FULLON_MASK             0x40
#define SGM41511H_REG_CCC_Q1_FULLON_SHIFT            6
#define SGM41511H_REG_CCC_Q1_FULLON_ENABLE           1
#define REG02_Q1_FULLON_DISABLE                      0

#define SGM41511H_REG_CCC_ICHG_MASK                  0x3F
#define SGM41511H_REG_CCC_ICHG_SHIFT                 0
#define ICHG_BASE                                    0
#define ICHG_LSB                                     60
#define ICHG_INIT                                    1000000

#define SGM41511H_REG02_INIT                         0xA1

/* Register 03h */
#define SGM41511H_REG_PCTCC                          0x03

#define SGM41511H_REG03_INIT                         0x42

#define SGM41511H_REG_PCTCC_IPRECHG_MASK             0xF0
#define SGM41511H_REG_PCTCC_IPRECHG_SHIFT            4
#define IPRECHG_BASE                                 60
#define IPRECHG_LSB                                  60

#define SGM41511H_REG_PCTCC_ITERM_MASK               0x0F
#define SGM41511H_REG_PCTCC_ITERM_SHIFT              0
#define ITERM_BASE                                   60
#define ITERM_LSB                                    60

/* Register 04h */
#define SGM41511H_REG_CVC                            0x04

#define SGM41511H_REG_CVC_VREG_MASK                  0xF8
#define SGM41511H_REG_CVC_VREG_SHIFT                 3
#define VREG_BASE                                    3856
#define VREG_LSB                                     32

#define SGM41511H_REG_CVC_TOPOFF_TIMER_MASK          0x06
#define SGM41511H_REG_CVC_TOPOFF_TIMER_SHIFT         1
#define REG04_TOPOFF_TIMER_DISABLE                   0
#define REG04_TOPOFF_TIMER_15M                       1
#define REG04_TOPOFF_TIMER_30M                       2
#define REG04_TOPOFF_TIMER_45M                       3

#define SGM41511H_REG_CVC_VRECHG_MASK                0x01
#define SGM41511H_REG_CVC_VRECHG_SHIFT               0
#define REG04_VRECHG_100MV                           0
#define REG04_VRECHG_200MV                           1

/* Register 05h */
#define SGM41511H_REG_CTTC                           0x05

#define SGM41511H_REG05_INIT                         0xAF

#define SGM41511H_REG_CTTC_EN_TERM_MASK              0x80
#define SGM41511H_REG_CTTC_EN_TERM_SHIFT             7
#define REG05_TERM_ENABLE                            1
#define REG05_TERM_DISABLE                           0

#define SGM41511H_REG_CTTC_WDT_MASK                  0x30
#define SGM41511H_REG_CTTC_WDT_SHIFT                 4
#define REG05_WDT_DISABLE                            0
#define REG05_WDT_40S                                1
#define REG05_WDT_80S                                2
#define REG05_WDT_160S                               3
#define WDT_BASE                                     0
#define WDT_STEP1                                    40
#define WDT_STEP2                                    80
#define WDT_STEP3                                    160

#define SGM41511H_REG_CTTC_EN_TIMER_MASK             0x08
#define SGM41511H_REG_CTTC_EN_TIMER_SHIFT            3
#define REG05_CHG_TIMER_ENABLE                       1
#define REG05_CHG_TIMER_DISABLE                      0
#define SAFE_TIME_RESET                              18000 /* 5h */
#define ENABLE_TIMES                                 3

#define SGM41511H_REG_CTTC_CHG_TIMER_MASK            0x04
#define SGM41511H_REG_CTTC_CHG_TIMER_SHIFT           2
#define REG05_CHG_TIMER_5HOURS                       0
#define REG05_CHG_TIMER_10HOURS                      1

#define SGM41511H_REG_CTTC_TREG_MASK                 0x02
#define SGM41511H_REG_CTTC_TREG_SHIFT                1
#define REG05_TREG_90C                               0
#define REG05_TREG_110C                              1

#define SGM41511H_REG_CTTC_JEITA_ISET_MASK           0x01
#define SGM41511H_REG_CTTC_JEITA_ISET_SHIFT          0
#define REG05_JEITA_ISET_50PCT                       0
#define REG05_JEITA_ISET_20PCT                       1

/* Register 06h */
#define SGM41511H_REG_BVTRC                          0x06

#define SGM41511H_REG_BVTRC_OVP_MASK                 0xC0
#define SGM41511H_REG_BVTRC_OVP_SHIFT                0x6
#define REG06_OVP_5P5V                               0
#define REG06_OVP_6P2V                               1
#define REG06_OVP_10P5V                              2
#define REG06_OVP_14P3V                              3

#define SGM41511H_REG_BVTRC_BOOSTV_MASK              0x30
#define SGM41511H_REG_BVTRC_BOOSTV_SHIFT             4
#define REG06_BOOSTV_4P85V                           0
#define REG06_BOOSTV_5V                              1
#define REG06_BOOSTV_5P15V                           2
#define REG06_BOOSTV_5P3V                            3

#define BOOSTV_STEP1                                 4850
#define BOOSTV_STEP2                                 5000
#define BOOSTV_STEP3                                 5150
#define BOOSTV_STEP4                                 5300

#define SGM41511H_REG_BVTRC_VINDPM_MASK              0x0F
#define SGM41511H_REG_BVTRC_VINDPM_SHIFT             0
#define VINDPM_BASE                                  3900
#define VINDPM_LSB                                   100
#define VINDPM_INIT                                  4400

/* Register 07h */
#define SGM41511H_REG_MOC                            0x07

#define SGM41511H_REG_MOC_DPDM_EN_MASK               0x80
#define SGM41511H_REG_MOC_DPDM_EN_SHIFT              7
#define REG07_FORCE_DPDM                             1

#define SGM41511H_REG_MOC_TMR2X_EN_MASK              0x40
#define SGM41511H_REG_MOC_TMR2X_EN_SHIFT             6
#define REG07_TMR2X_ENABLE                           1
#define REG07_TMR2X_DISABLE                          0

#define SGM41511H_REG_MOC_BATFET_DISABLE_MASK        0x20
#define SGM41511H_REG_MOC_BATFET_DISABLE_SHIFT       5
#define REG07_BATFET_OFF                             1
#define REG07_BATFET_ON                              0

#define SGM41511H_REG_MOC_JEITA_VSET_MASK            0x10
#define SGM41511H_REG_MOC_JEITA_VSET_SHIFT           4
#define REG07_JEITA_VSET_4100                        0
#define REG07_JEITA_VSET_VREG                        1

#define SGM41511H_REG_MOC_BATFET_DLY_MASK            0x08
#define SGM41511H_REG_MOC_BATFET_DLY_SHIFT           3
#define REG07_BATFET_DLY_0S                          0
#define REG07_BATFET_DLY_10S                         1

#define SGM41511H_REG_MOC_BATFET_RST_EN_MASK         0x04
#define SGM41511H_REG_MOC_BATFET_RST_EN_SHIFT        2
#define REG07_BATFET_RST_DISABLE                     0
#define REG07_BATFET_RST_ENABLE                      1

#define SGM41511H_REG_MOC_VDPM_BAT_TRACK_MASK        0x03
#define SGM41511H_REG_MOC_VDPM_BAT_TRACK_SHIFT       0
#define REG07_VDPM_BAT_TRACK_DISABLE                 0
#define REG07_VDPM_BAT_TRACK_200MV                   1
#define REG07_VDPM_BAT_TRACK_250MV                   2
#define REG07_VDPM_BAT_TRACK_300MV                   3

/* Register 08h */
#define SGM41511H_REG_SS                             0x08

#define SGM41511H_REG_SS_VBUS_STAT_MASK              0xE0
#define SGM41511H_REG_SS_VBUS_STAT_SHIFT             5
#define REG08_VBUS_TYPE_NONE                         0
#define REG08_VBUS_TYPE_USB                          1
#define REG08_VBUS_TYPE_CDP                          2
#define REG08_VBUS_TYPE_ADAPTER                      3
#define REG08_VBUS_TYPE_OTG                          7

#define SGM41511H_REG_SS_CHRG_STAT_MASK              0x18
#define SGM41511H_REG_SS_CHRG_STAT_SHIFT             3
#define REG08_CHRG_STAT_IDLE                         0
#define REG08_CHRG_STAT_PRECHG                       1
#define REG08_CHRG_STAT_FASTCHG                      2
#define REG08_CHRG_STAT_CHGDONE                      3

#define SGM41511H_REG_SS_PG_STAT_MASK                0x04
#define SGM41511H_REG_SS_PG_STAT_SHIFT               2
#define REG08_POWER_GOOD                             1

#define SGM41511H_REG_SS_THERM_STAT_MASK             0x02
#define SGM41511H_REG_SS_THERM_STAT_SHIFT            1

#define SGM41511H_REG_SS_VSYS_STAT_MASK              0x01
#define SGM41511H_REG_SS_VSYS_STAT_SHIFT             0
#define REG08_IN_VSYS_STAT                           1

/* Register 09h */
#define SGM41511H_REG_F                              0x09

#define SGM41511H_REG_F_FAULT_WDT_MASK               0x80
#define SGM41511H_REG_F_FAULT_WDT_SHIFT              7
#define REG09_FAULT_WDT                              1

#define SGM41511H_REG_F_FAULT_BOOST_MASK             0x40
#define SGM41511H_REG_F_FAULT_BOOST_SHIFT            6

#define SGM41511H_REG_F_FAULT_CHRG_MASK              0x30
#define SGM41511H_REG_F_FAULT_CHRG_SHIFT             4
#define REG09_FAULT_CHRG_NORMAL                      0
#define REG09_FAULT_CHRG_INPUT                       1
#define REG09_FAULT_CHRG_THERMAL                     2
#define REG09_FAULT_CHRG_TIMER                       3

#define SGM41511H_REG_F_FAULT_BAT_MASK               0x08
#define SGM41511H_REG_F_FAULT_BAT_SHIFT              3
#define REG09_FAULT_BAT_OVP                          1

#define SGM41511H_REG_F_FAULT_NTC_MASK               0x07
#define SGM41511H_REG_F_FAULT_NTC_SHIFT              0
#define REG09_FAULT_NTC_NORMAL                       0
#define REG09_FAULT_NTC_WARM                         2
#define REG09_FAULT_NTC_COOL                         3
#define REG09_FAULT_NTC_COLD                         5
#define REG09_FAULT_NTC_HOT                          6

/* Register 0Ah */
#define SGM41511H_REG_VINS                           0x0A

#define SGM41511H_REG_VINS_VBUS_GD_MASK              0x80
#define SGM41511H_REG_VINS_VBUS_GD_SHIFT             7
#define REG0A_VBUS_GD                                1

#define SGM41511H_REG_VINS_VINDPM_STAT_MASK          0x40
#define SGM41511H_REG_VINS_VINDPM_STAT_SHIFT         6
#define REG0A_VINDPM_ACTIVE                          1

#define SGM41511H_REG_VINS_IINDPM_STAT_MASK          0x20
#define SGM41511H_REG_VINS_IINDPM_STAT_SHIFT         5
#define REG0A_IINDPM_ACTIVE                          1

#define SGM41511H_REG_VINS_TOPOFF_ACTIVE_MASK        0x08
#define SGM41511H_REG_VINS_TOPOFF_ACTIVE_SHIFT       3
#define REG0A_TOPOFF_ACTIVE                          1

#define SGM41511H_REG_VINS_ACOV_STAT_MASK            0x04
#define SGM41511H_REG_VINS_ACOV_STAT_SHIFT           2
#define REG0A_ACOV_ACTIVE                            1

#define SGM41511H_REG_VINS_VINDPM_INT_MASK           0x02
#define SGM41511H_REG_VINS_VINDPM_INT_SHIFT          1
#define REG0A_VINDPM_INT_ENABLE                      0
#define REG0A_VINDPM_INT_DISABLE                     1

#define SGM41511H_REG_VINS_IINDPM_INT_MASK           0x01
#define SGM41511H_REG_VINS_IINDPM_INT_SHIFT          0
#define REG0A_IINDPM_INT_ENABLE                      0
#define REG0A_IINDPM_INT_DISABLE                     1

#define SGM41511H_REG_VINS_INT_MASK_MASK             0x03
#define SGM41511H_REG_VINS_INT_MASK_SHIFT            0

/* Register 0Bh */
#define SGM41511H_REG_VPRS                           0x0B

#define SGM41511H_REG_VPRS_REG_RESET_MASK            0x80
#define SGM41511H_REG_VPRS_REG_RESET_SHIFT           7
#define REG0B_REG_RESET                              1

#define SGM41511H_REG_VPRS_PN_MASK                   0x78
#define SGM41511H_REG_VPRS_PN_SHIFT                  3

#define SGM41511H_REG_VPRS_PART_MASK                 0x04
#define SGM41511H_REG_VPRS_PART_SHIFT                2

#define SGM41511H_REG_VPRS_DEV_REV_MASK              0x03
#define SGM41511H_REG_VPRS_DEV_REV_SHIFT             0

#define SGM41511H_VENDOR_ID                          0x2
#define SGM41512_VENDOR_ID                           0x5
#define SY6974B_VENDOR_ID                            0x8
#define BQ25601_VENDOR_ID                            0x7

#define SGM41511H_REG_SS_VBUS_PLUGGED                1

#define HIZ_IIN_FLAG_TRUE                            1
#define HIZ_IIN_FLAG_FALSE                           0

#define AC_IIN_MAX_CURRENT                           3200
#define EX_AC_IIN_MAX_CURRENT                        2100

#define CHARGE_IC_BAD                                (-1)
#define CHARGE_IC_GOOD                               0
#define WATCHDOG_TIMER_DISABLE                       0

/* Register 0Ch */
#define SGM_ETA_ID                                   0x0C
#define I2C_ERR_COUNT_ZERO                           0
#define I2C_ERR_MAX_COUNT                            5

#define CH_ENABLE_MAX_COUNT                          10

#endif /* SGM41511H_CHARGER_H */
