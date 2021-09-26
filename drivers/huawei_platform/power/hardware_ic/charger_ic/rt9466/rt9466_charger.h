/*
 * rt9466_charger.h
 *
 * rt9466 driver
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

#ifndef _RT9466_CHARGER_H_
#define _RT9466_CHARGER_H_

#define CHG_IRQGPIO_NAME_LEN               10
#define CHG_IRQ_NAME_LEN                   5
#define RT9466_IRQ_GROUP_LEN               8
#define IEOC_WORK_AROUND                   100000
#define UNIT_CONVERT                       1000
#define CHG_CTRL_LEN                       2
#define RT9466_RESERVED                    0
#define RETRY_MAX_CNT                      5
#define RT9466_SLAVE_ADDR                  0x53
#define RT9466_BEGIN_INFO                  0x00
#define RT9466_VENDOR_ID                   0x80
#define RT9466_CHIP_REV_E1                 0x01
#define RT9466_CHIP_REV_E3                 0x03
#define RT9466_IRQ_COUNT_FOR_ONE_BYTE      8
#define RT9466_ADC_REG_LEN                 2
#define RT9466_COEF_AICR_MAX               400000
#define RT9466_COEF_AICR_RATE              67
#define RT9466_COEF_ICHG_MIN_L             100000
#define RT9466_COEF_ICHG_MAX_L             450000
#define RT9466_COEF_ICHG_RATE_L            57
#define RT9466_COEF_ICHG_MIN_H             500000
#define RT9466_COEF_ICHG_MAX_H             850000
#define RT9466_COEF_ICHG_RATE_H            63
#define RT9466_REG_CHG_IRQX_LEN            5

#define RT9466_ICHG_MIN                    100000
#define RT9466_ICHG_MAX                    5000000
#define RT9466_ICHG_STEP                   100000
#define RT9466_ICHG                        2000000

#define RT9466_IEOC_MIN                    100000
#define RT9466_IEOC_MAX                    800000
#define RT9466_IEOC_STEP                   50000
#define RT9466_IEOC                        250000

#define RT9466_MIVR_MIN                    3900000
#define RT9466_MIVR_MAX                    13400000
#define RT9466_MIVR_STEP                   100000
#define RT9466_MIVR                        4500

#define RT9466_AICR_MIN                    100000
#define RT9466_AICR_MAX                    3250000
#define RT9466_AICR_STEP                   50000
#define RT9466_AICR                        500000

#define RT9466_CV_MIN                      3900000
#define RT9466_CV_MAX                      4710000
#define RT9466_CV_STEP                     10000
#define RT9466_CV                          4350000

#define RT9466_IPREC_MIN                   100000
#define RT9466_IPREC_MAX                   850000
#define RT9466_IPREC_STEP                  50000

/* IR compensation */
#define RT9466_IRCMP_RES_MIN               0
#define RT9466_IRCMP_RES_MAX               175000
#define RT9466_IRCMP_RES_STEP              25000

/* IR compensation maximum voltage clamp */
#define RT9466_IRCMP_VCLAMP_MIN            0
#define RT9466_IRCMP_VCLAMP_MAX            224000
#define RT9466_IRCMP_VCLAMP_STEP           32000

/* ADC unit/offset */
#define RT9466_ADC_UNIT_VBUS_DIV5          25000
#define RT9466_ADC_UNIT_VBUS_DIV2          10000
#define RT9466_ADC_UNIT_VBAT               5000
#define RT9466_ADC_UNIT_VSYS               5000
#define RT9466_ADC_UNIT_REGN               5000
#define RT9466_ADC_UNIT_TS_BAT             25
#define RT9466_ADC_UNIT_IBUS               50000
#define RT9466_ADC_UNIT_IBAT               50000
#define RT9466_ADC_UNIT_TEMP_JC            2

#define RT9466_ADC_OFFSET_VBUS_DIV5        0
#define RT9466_ADC_OFFSET_VBUS_DIV2        0
#define RT9466_ADC_OFFSET_VBAT             0
#define RT9466_ADC_OFFSET_VSYS             0
#define RT9466_ADC_OFFSET_REGN             0
#define RT9466_ADC_OFFSET_TS_BAT           0
#define RT9466_ADC_OFFSET_IBUS             0
#define RT9466_ADC_OFFSET_IBAT             0
#define RT9466_ADC_OFFSET_TEMP_JC          (-40)

#define RT9466_SAFETMR                     10
#ifdef CONFIG_MTK_BIF_SUPPORT
#define IRCMP_RESISTOR                     0
#define IRCMP_VCLAMP                       0
#else
#define IRCMP_RESISTOR                     25000
#define IRCMP_VCLAMP                       32000
#endif

/* CORE_CTRL0 0x00 */
#define RT9466_SHIFT_RST                   7
#define RT9466_MASK_RST                    (1 << RT9466_SHIFT_RST)

/* CHG_CTRL1 0x01 */
#define RT9466_SHIFT_OPA_MODE              0
#define RT9466_SHIFT_HZ_EN                 2
#define RT9466_SHIFT_IRQ_PULSE             3
#define RT9466_MASK_OPA_MODE               (1 << RT9466_SHIFT_OPA_MODE)
#define RT9466_MASK_HZ_EN                  (1 << RT9466_SHIFT_HZ_EN)
#define RT9466_MASK_IRQ_PULSE              (1 << RT9466_SHIFT_IRQ_PULSE)

/* CHG_CTRL2 0x02 */
#define RT9466_SHIFT_CHG_EN                0
#define RT9466_SHIFT_CFO_EN                1
#define RT9466_SHIFT_IINLMTSEL             2
#define RT9466_SHIFT_TE_EN                 4
#define RT9466_MASK_CHG_EN                 (1 << RT9466_SHIFT_CHG_EN)
#define RT9466_MASK_CFO_EN                 (1 << RT9466_SHIFT_CFO_EN)
#define RT9466_MASK_IINLMTSEL              (3 << RT9466_SHIFT_IINLMTSEL)
#define RT9466_MASK_TE_EN                  (1 << RT9466_SHIFT_TE_EN)

/* CHG_CTRL3 0x03 */
#define RT9466_SHIFT_AICR                  2
#define RT9466_SHIFT_ILIM_EN               0
#define RT9466_MASK_AICR                   0xFC
#define RT9466_MASK_ILIM_EN                (1 << RT9466_SHIFT_ILIM_EN)

/* CHG_CTRL4 0x04 */
#define RT9466_SHIFT_CV                    1
#define RT9466_MASK_CV                     0xFE

/* CHG_CTRL6 0x06 */
#define RT9466_SHIFT_MIVR                  1
#define RT9466_MASK_MIVR                   0xFE

/* CHG_CTRL7 0x07 */
#define RT9466_SHIFT_ICHG                  2
#define RT9466_MASK_ICHG                   0xFC

/* CHG_CTRL8 0x08 */
#define RT9466_SHIFT_IPREC                 0
#define RT9466_MASK_IPREC                  0x0F

/* CHG_CTRL9 0x09 */
#define RT9466_SHIFT_IEOC                  4
#define RT9466_MASK_IEOC                   0xF0

/* CHG_CTRL10 0x0A */
#define RT9466_SHIFT_BOOST_OC              0
#define RT9466_MASK_BOOST_OC               0x07

/* CHG_CTRL12 0x0C */
#define RT9466_SHIFT_TMR_EN                1
#define RT9466_SHIFT_WT_FC                 5
#define RT9466_MASK_TMR_EN                 (1 << RT9466_SHIFT_TMR_EN)
#define RT9466_MASK_WT_FC                  0xE0

/* CHG_CTRL13 0x0D */
#define RT9466_SHIFT_WDT_EN                7
#define RT9466_SHIFT_IRQ_REZ               0
#define RT9466_MASK_WDT_EN                 (1 << RT9466_SHIFT_WDT_EN)
#define RT9466_MASK_IRQ_REZ                (1 << RT9466_SHIFT_IRQ_REZ)

/* CHG_CTRL16 0x10 */
#define RT9466_SHIFT_JEITA_EN              4
#define RT9466_MASK_JEITA_EN               (1 << RT9466_SHIFT_JEITA_EN)

/* CHG_ADC 0x11 */
#define RT9466_SHIFT_ADC_IN_SEL            4
#define RT9466_SHIFT_ADC_START             0
#define RT9466_MASK_ADC_IN_SEL             0xF0
#define RT9466_MASK_ADC_START              (1 << RT9466_SHIFT_ADC_START)

/* CHG_CTRL19 0x15 */
#define RT9466_SHIFT_AUTO_SENSE_EN         0
#define RT9466_AUTO_SENSE_EN               (1 << RT9466_SHIFT_AUTO_SENSE_EN)

/* CHG_CTRL19 0x18 */
#define RT9466_SHIFT_RESET_DISABLE         7
#define RT9466_MASK_RESET_DISABLE          (1 << RT9466_SHIFT_RESET_DISABLE)

/* CHG_CTRL18 0x1A */
#define RT9466_SHIFT_IRCMP_RES             3
#define RT9466_SHIFT_IRCMP_VCLAMP          0
#define RT9466_MASK_IRCMP_RES              0x38
#define RT9466_MASK_IRCMP_VCLAMP           0x07

/* CHG_STAT 0x42 */
#define RT9466_SHIFT_ADC_STAT              0
#define RT9466_SHIFT_CHG_STAT              6
#define RT9466_MASK_CHG_STAT               0xC0

/* CHG_STATC 0x50 */
#define RT9466_SHIFT_CHG_MIVR              6

/* CHG_FAULT 0x51 */
#define RT9466_SHIFT_VBUSOV                7

/* CHG_IRQ2 0x54 */
#define RT9466_SHIFT_CHG_AICLMEASI         0
#define RT9466_MASK_CHG_AICLMEASI          (1 << RT9466_SHIFT_CHG_AICLMEASI)

enum rt9466_reg_addr {
	RT9466_REG_CORE_CTRL0,
	RT9466_REG_CHG_CTRL1,
	RT9466_REG_CHG_CTRL2,
	RT9466_REG_CHG_CTRL3,
	RT9466_REG_CHG_CTRL4,
	RT9466_REG_CHG_CTRL5,
	RT9466_REG_CHG_CTRL6,
	RT9466_REG_CHG_CTRL7,
	RT9466_REG_CHG_CTRL8,
	RT9466_REG_CHG_CTRL9,
	RT9466_REG_CHG_CTRL10,
	RT9466_REG_CHG_CTRL11,
	RT9466_REG_CHG_CTRL12,
	RT9466_REG_CHG_CTRL13,
	RT9466_REG_CHG_CTRL14,
	RT9466_REG_CHG_CTRL15,
	RT9466_REG_CHG_CTRL16,
	RT9466_REG_CHG_ADC,
	RT9466_REG_CHG_CTRL19 = 0x18,
	RT9466_REG_CHG_CTRL17,
	RT9466_REG_CHG_CTRL18,
	RT9466_REG_CHG_HIDDEN_CTRL1 = 0x20,
	RT9466_REG_CHG_HIDDEN_CTRL2,
	RT9466_REG_CHG_HIDDEN_CTRL4 = 0x23,
	RT9466_REG_CHG_HIDDEN_CTRL6 = 0x25,
	RT9466_REG_CHG_HIDDEN_CTRL7,
	RT9466_REG_CHG_HIDDEN_CTRL8,
	RT9466_REG_CHG_HIDDEN_CTRL9,
	RT9466_REG_CHG_HIDDEN_CTRL15 = 0x2E,
	RT9466_REG_DEVICE_ID = 0x40,
	RT9466_REG_CHG_STAT = 0x42,
	RT9466_REG_CHG_NTC,
	RT9466_REG_ADC_DATA_H,
	RT9466_REG_ADC_DATA_L,
	RT9466_REG_CHG_STATC = 0x50,
	RT9466_REG_CHG_FAULT,
	RT9466_REG_TS_STATC,
	RT9466_REG_CHG_IRQ1,
	RT9466_REG_CHG_IRQ2,
	RT9466_REG_CHG_IRQ3,
	RT9466_REG_CHG_STATC_CTRL = 0x60,
	RT9466_REG_CHG_FAULT_CTRL,
	RT9466_REG_TS_STATC_CTRL,
	RT9466_REG_CHG_IRQ1_CTRL,
	RT9466_REG_CHG_IRQ2_CTRL,
	RT9466_REG_CHG_IRQ3_CTRL,
	RT9466_REG_HIDDEN_MODE = 0X70,
	RT9466_REG_MAX,
};

#endif /* _RT9466_CHARGER_H_ */
