/*
 * eta6937_reg.h
 *
 * eta6937 driver
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
#ifndef ETA6937_REG_H
#define ETA6937_REG_H

enum eta_input_limit_current {
	ETA6937_ILIMI_CURR300 = 0,
	ETA6937_ILIMI_CURR500,
	ETA6937_ILIMI_CURR800,
	ETA6937_ILIMI_CURR1200,
	ETA6937_ILIMI_CURR1500,
	ETA6937_ILIMI_CURR2000,
};

enum eta_charging_current {
	CURRENT_LEVEL_1 = 100,
	CURRENT_LEVEL_2 = 300,
	CURRENT_LEVEL_3 = 500,
	CURRENT_LEVEL_4 = 550,
	CURRENT_LEVEL_5 = 680,
	CURRENT_LEVEL_6 = 800,
	CURRENT_LEVEL_7 = 1200,
	CURRENT_LEVEL_8 = 1350,
	CURRENT_LEVEL_9 = 1500,
	CURRENT_LEVEL_10 = 1950,
	CURRENT_LEVEL_11 = 2000,
};

#define RESISTANCE                      68
#define READ_TIMES                      3
#define UBF_SIZE                        100
#define ADDR_FIRST                      0x00
#define ADDR_END                        0x06
#define ADDR_ALL                        0x07
#define ETA_ID                          0x04
#define SY_ID                           0x01
#define I2C_ADDR                        0x6A
#define CURRENT_CONVERT                 100
#define UNIT_CONVERT                    1000
#define LOWCHG_EN                       0xff
#define LOWCHG_DISEN                    0
#define MIN_CURRENT                     1
#define SUCCESS                         0
#define ERROR                           (-1)
#define MAX_CURREN_VALUE                0x7
#define RESET_OVER_TIME                 18000 /* unit s */
#define ENABLE_TIMES                    2

/* Register 00h */
#define ETA6937_REG_00                  0x00
#define ETA6937_TMR_RST_MASK            0x80
#define ETA6937_OTG_MASK                0x80
#define ETA6937_TMR_RST_SHIFT           7
#define ETA6937_TMR_RST                 1
#define ETA6937_EN_STAT_MASK            0x40
#define ETA6937_EN_STAT_SHIFT           6
#define ETA6937_STAT_MASK               0x30
#define ETA6937_STAT_SHIFT              4
#define ETA6937_STAT_READY              0
#define ETA6937_STAT_CHARGING           1
#define ETA6937_STAT_CHGDONE            2
#define ETA6937_STAT_FAULT              3
#define ETA6937_BOOST_MASK              0x08
#define ETA6937_BOOST_SHIFT             3
#define ETA6937_FAULT_MASK              07
#define ETA6937_FAULT_SHIFT             0
#define ETA6937_OVER_LOAD               2

/* Register 01h */
#define ETA6937_REG_01                  0x01
#define ETA6937_IINLIM_MASK             0xC0
#define ETA6937_IINLIM_SHIFT            6
#define ETA6937_IINLIM_100MA            0
#define ETA6937_IINLIM_500MA            1
#define ETA6937_IINLIM_800MA            2
#define ETA6937_IINLIM_NOLIM            3
#define ETA6937_WEAK_BATT_VOLT_MASK     0x30
#define ETA6937_WEAK_BATT_VOLT_SHIFT    4
#define ETA6937_WEAK_BATT_VOLT_BASE     3400
#define ETA6937_WEAK_BATT_VOLT_LSB      100
#define ETA6937_TERM_ENABLE_MASK        0x08
#define ETA6937_TERM_ENABLE_SHIFT       3
#define ETA6937_TERM_ENABLE             1
#define ETA6937_TERM_DISABLE            0
#define ETA6937_CHARGE_ENABLE_MASK      0x04
#define ETA6937_CHARGE_ENABLE_SHIFT     2
#define ETA6937_CHARGE_ENABLE           0
#define ETA6937_CHARGE_DISABLE          1
#define ETA6937_HZ_MODE_MASK            0x02
#define ETA6937_HZ_MODE_SHIFT           1
#define ETA6937_HZ_MODE_ENABLE          1
#define ETA6937_HZ_MODE_DISABLE         0
#define ETA6937_OPA_MODE_MASK           0x01
#define ETA6937_OPA_MODE_SHIFT          0
#define ETA6937_BOOST_MODE              1
#define ETA6937_CHARGER_MODE            0

/* Register 02h */
#define ETA6937_REG_02                  0x02
#define ETA6937_VREG_MASK               0xFC
#define ETA6937_VREG_SHIFT              2
#define ETA6937_VREG_BASE               3500
#define ETA6937_VREG_LSB                20
#define ETA6937_OTG_PL_MASK             0x02
#define ETA6937_OTG_PL_SHIFT            1
#define ETA6937_OTG_PL_HIGH             1
#define ETA6937_OTG_PL_LOW              0
#define ETA6937_OTG_EN_MASK             0x01
#define ETA6937_OTG_EN_SHIFT            0
#define ETA6937_OTG_ENABLE              1
#define ETA6937_OTG_DISABLE             0

/* Register 03h */
#define ETA6937_REG_03                  0x03
#define ETA6937_VENDOR_MASK             0xE0
#define ETA6937_VENDOR_SHIFT            5
#define ETA6937_PN_MASK                 0x18
#define ETA6937_PN_SHIFT                3
#define ETA6937_REVISION_MASK           0x07
#define ETA6937_REVISION_SHIFT          0

/* Register 04h */
#define ETA6937_REG_04                  0x04
#define ETA6937_RESET_MASK              0x80
#define ETA6937_RESET_SHIFT             7
#define ETA6937_RESET                   1
#define ETA6937_ICHG_MASK               0x70
#define ETA6937_ICHG_SHIFT              4
#define ETA6937_ICHG_BASE               374 /* unit 0.1mV */
#define ETA6937_ICHG_LSB                68 /* unit 0.1mV */
#define ETA6937_ICHG_OFFSET_MASK        0x08
#define ETA6937_ICHG_OFFSET_SHIFT       3
#define ETA6937_ICHG_OFFSET_LEVET0      550 /* mA */
#define ETA6937_ICHG_OFFSET_LEVET1      650 /* mA */
#define ETA6937_ICHG_OFFSET_STEP        100 /* mA */
#define ETA6937_ICHG_OFFSET_LVL_EN      0
#define ETA6937_ICHG_OFFSET_LVL1_EN     1
#define ETA6937_ITERM_MASK              0x07
#define ETA6937_ITERM_SHIFT             0
#define ETA6937_ITERM_BASE              34 /* unit 0.1mV */
#define ETA6937_ITERM_LSB               34 /* unit 0.1mV */

/* Register 05h */
#define ETA6937_REG_05                  0x05
#define ETA6937_LOW_CHG_MASK            0x20
#define ETA6937_LOW_CHG_SHIFT           5
#define ETA6937_LOW_CHG                 1
#define ETA6937_ADD_CURR_800_MASK       0x40
#define ETA6937_ADD_CURR_800_SHIFT      6
#define ETA6937_ADD_CURR_800_ENABLE     1
#define ETA6937_ADD_CURR_800_DISABLE    0
#define ETA6937_DPM_STATUS_MASK         0x10
#define ETA6937_CD_STATUS_MASK          0x08
#define ETA6937_VSREG_MASK              0x07
#define ETA6937_VSREG_SHIFT             0
#define ETA6937_VSREG_BASE              4200
#define ETA6937_VSREG_LSB               80

/* Register 06h */
#define ETA6937_REG_06                  0x06
#define ETA6937_MAX_ICHG_MASK           0xF0
#define ETA6937_MAX_ICHG_SHIFT          4
#define ETA6937_MAX_ICHG_BASE           374
#define ETA6937_MAX_ICHG_LSB            68
#define ETA6937_MAX_VREG_MASK           0x0F
#define ETA6937_MAX_VREG_SHIFT          0
#define ETA6937_MAX_VREG_BASE           4200
#define ETA6937_MAX_VREG_LSB            20

/* Register 07h */
#define ETA6937_REG_07                  0x07
#define ETA6937_MAX_ID_SHIFT            0
#define ETA6937_EXTRA_ILIMT_EN_MASK     0x08
#define ETA6937_EXTRA_ILIMT_EN_SHIFT    3
#define EXTRA_ILIMT_ENABLE              1
#define EXTRA_ILIMT_DISABLE             0
#define ETA6937_EXTRA_ILIMT_CHG_MASK    0x07
#define ETA6937_EXTRA_ILIMT_CHG_SHIFT   0
#define ETA6937_VINDPM_HIGH_MASK        0xF0
#define ETA6937_VINDPM_HIGH_SHIFT       4

#endif /* ETA6937_REG_H */
