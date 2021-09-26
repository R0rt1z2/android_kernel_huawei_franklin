/*
 * x aw8697.h
 *
 * code for vibrator
 *
 * Copyright (c) 2020 Huawei Technologies Co., Ltd.
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

#ifndef _AW8697_H_
#define _AW8697_H_

#if LINUX_VERSION_CODE <= KERNEL_VERSION(4, 4, 1)
#define TIMED_OUTPUT
#endif

#include <linux/regmap.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <linux/hrtimer.h>
#include <linux/mutex.h>
#include <linux/cdev.h>
#ifdef TIMED_OUTPUT
#include <../../../drivers/staging/android/timed_output.h>
#else
#include <linux/leds.h>
#endif

#define AW8697_CHIPID                       0x97

#define MAX_I2C_BUFFER_SIZE                 65536

#define AW8697_SEQUENCER_SIZE               8
#define AW8697_SEQUENCER_LOOP_SIZE          4

#define AW8697_RTP_I2C_SINGLE_MAX_NUM       512

#define HAPTIC_MAX_TIMEOUT                  10000

#define AW8697_VBAT_REFER                   4200
#define AW8697_VBAT_MIN                     3000
#define AW8697_VBAT_MAX                     4500

/* trig config */
#define AW8697_TRIG_NUM                     3
#define AW8697_TRG1_ENABLE                  1
#define AW8697_TRG2_ENABLE                  1
#define AW8697_TRG3_ENABLE                  1
#define AW8697_EFFECT_ID_MAX                100
#define AW8697_MAX_LONG_VIB_ID              20
#define AW8697_MAX_SHORT_VIB_ID             10
#define AW8697_SHORT_VIB_AVAIABLE           3
#define AW8697_DEFAULT_LONG_VB_IDX          4
#define LONG_HAPTIC_RTP_MAX_ID              4999
#define LONG_HAPTIC_RTP_MIN_ID              1010

#define SHORT_HAPTIC_RAM_MAX_ID             309
#define SHORT_HAPTIC_RTP_MAX_ID             9999

#define SHORT_HAPTIC_RAM_MIN_IDX            1
#define SHORT_HAPTIC_RAM_MAX_IDX            30
#define SHORT_HAPTIC_RTP_MAX_IDX            9999

#define SHORT_HAPTIC_AMP_DIV_COFF           10
#define LONG_TIME_AMP_DIV_COFF              100
#define BASE_INDEX                          31
/*
 * trig default high level
 * ___________           _________________
 *           |           |
 *           |           |
 *           |___________|
 *        first edge
 *                   second edge
 *
 *
 * trig default low level
 *            ___________
 *           |           |
 *           |           |
 * __________|           |_________________
 *        first edge
 *                   second edge
 */
#define AW8697_TRG1_DEFAULT_LEVEL     1 // 1: high level; 0: low level
#define AW8697_TRG2_DEFAULT_LEVEL     1 // 1: high level; 0: low level
#define AW8697_TRG3_DEFAULT_LEVEL     1 // 1: high level; 0: low level

#define AW8697_TRG1_DUAL_EDGE         1 // 1: dual edge; 0: first edge
#define AW8697_TRG2_DUAL_EDGE         1 // 1: dual edge; 0: first edge
#define AW8697_TRG3_DUAL_EDGE         1 // 1: dual edge; 0: first edge

#define AW8697_TRG1_FIRST_EDGE_SEQ    1 // trig1: first edge waveform seq
#define AW8697_TRG1_SECOND_EDGE_SEQ   2 // trig1: second edge waveform seq
#define AW8697_TRG2_FIRST_EDGE_SEQ    1 // trig2: first edge waveform seq
#define AW8697_TRG2_SECOND_EDGE_SEQ   2 // trig2: second edge waveform seq
#define AW8697_TRG3_FIRST_EDGE_SEQ    1 // trig3: first edge waveform seq
#define AW8697_TRG3_SECOND_EDGE_SEQ   2 // trig3: second edge waveform seq
#define AW8697_F0_CALI_THER_H         (1700 + 150)
#define AW8697_F0_CALI_THER_L         (1700 - 150)

#if AW8697_TRG1_ENABLE
#define AW8697_TRG1_DEFAULT_ENABLE    AW8697_BIT_TRGCFG2_TRG1_ENABLE
#else
#define AW8697_TRG1_DEFAULT_ENABLE    AW8697_BIT_TRGCFG2_TRG1_DISABLE
#endif

#if AW8697_TRG2_ENABLE
#define AW8697_TRG2_DEFAULT_ENABLE    AW8697_BIT_TRGCFG2_TRG2_ENABLE
#else
#define AW8697_TRG2_DEFAULT_ENABLE    AW8697_BIT_TRGCFG2_TRG2_DISABLE
#endif

#if AW8697_TRG3_ENABLE
#define AW8697_TRG3_DEFAULT_ENABLE    AW8697_BIT_TRGCFG2_TRG3_ENABLE
#else
#define AW8697_TRG3_DEFAULT_ENABLE    AW8697_BIT_TRGCFG2_TRG3_DISABLE
#endif

#if AW8697_TRG1_DEFAULT_LEVEL
#define AW8697_TRG1_DEFAULT_POLAR     AW8697_BIT_TRGCFG1_TRG1_POLAR_POS
#else
#define AW8697_TRG1_DEFAULT_POLAR     AW8697_BIT_TRGCFG1_TRG1_POLAR_NEG
#endif

#if AW8697_TRG2_DEFAULT_LEVEL
#define AW8697_TRG2_DEFAULT_POLAR     AW8697_BIT_TRGCFG1_TRG2_POLAR_POS
#else
#define AW8697_TRG2_DEFAULT_POLAR     AW8697_BIT_TRGCFG1_TRG2_POLAR_NEG
#endif

#if AW8697_TRG3_DEFAULT_LEVEL
#define AW8697_TRG3_DEFAULT_POLAR     AW8697_BIT_TRGCFG1_TRG3_POLAR_POS
#else
#define AW8697_TRG3_DEFAULT_POLAR     AW8697_BIT_TRGCFG1_TRG3_POLAR_NEG
#endif

#if AW8697_TRG1_DUAL_EDGE
#define AW8697_TRG1_DEFAULT_EDGE      AW8697_BIT_TRGCFG1_TRG1_EDGE_POS_NEG
#else
#define AW8697_TRG1_DEFAULT_EDGE      AW8697_BIT_TRGCFG1_TRG1_EDGE_POS
#endif

#if AW8697_TRG2_DUAL_EDGE
#define AW8697_TRG2_DEFAULT_EDGE       AW8697_BIT_TRGCFG1_TRG2_EDGE_POS_NEG
#else
#define AW8697_TRG2_DEFAULT_EDGE       AW8697_BIT_TRGCFG1_TRG2_EDGE_POS
#endif

#if AW8697_TRG3_DUAL_EDGE
#define AW8697_TRG3_DEFAULT_EDGE       AW8697_BIT_TRGCFG1_TRG3_EDGE_POS_NEG
#else
#define AW8697_TRG3_DEFAULT_EDGE       AW8697_BIT_TRGCFG1_TRG3_EDGE_POS
#endif

#define AW8697_DRV_WAV              20
#define AW8697_BRK_WAV              21
#define AW8697_LONG_WAV_1_CYL       22
#define AW8697_LONG_WAV_2_CYL       23
#define AW8697_LONG_WAV_4_CYL       24
#define AW8697_LONG_WAV_8_CYL       25
#define AW8697_LONG_SEQ_MAX_TIME    90 // 6 seqs * 15 times
#define AW8697_MAX_PLAY_TIME        14

enum seq_long_ccyle_type_t {
	SEQ_LONG_ONE_CYCLE = 1,
	SEQ_LONG_TWO_CYCLE = 2,
	SEQ_LONG_FOUR_CYCLE = 4,
	SEQ_LONG_EIGHT_CYCLE = 8,
};

enum vib_mode_type_t {
	SHORT_VIB_RAM_MODE = 0,
	LONG_VIB_RAM_MODE = 1,
	RTP_VIB_MODE = 2,
	VIB_MODE_MAX,
};

enum aw8697_flags {
	AW8697_FLAG_NONR = 0,
	AW8697_FLAG_SKIP_INTERRUPTS = 1,
};

enum aw8697_haptic_read_write {
	AW8697_HAPTIC_CMD_READ_REG = 0,
	AW8697_HAPTIC_CMD_WRITE_REG = 1,
};

enum aw8697_haptic_work_mode {
	AW8697_HAPTIC_STANDBY_MODE = 0,
	AW8697_HAPTIC_RAM_MODE = 1,
	AW8697_HAPTIC_RTP_MODE = 2,
	AW8697_HAPTIC_TRIG_MODE = 3,
	AW8697_HAPTIC_CONT_MODE = 4,
	AW8697_HAPTIC_RAM_LOOP_MODE = 5,
};

enum aw8697_haptic_bst_mode {
	AW8697_HAPTIC_BYPASS_MODE = 0,
	AW8697_HAPTIC_BOOST_MODE = 1,
};

enum aw8697_haptic_activate_mode {
	AW8697_HAPTIC_ACTIVATE_RAM_MODE = 0,
	AW8697_HAPTIC_ACTIVATE_CONT_MODE = 1,
};

enum aw8697_haptic_cont_vbat_comp_mode {
	AW8697_HAPTIC_CONT_VBAT_SW_COMP_MODE = 0,
	AW8697_HAPTIC_CONT_VBAT_HW_COMP_MODE = 1,
};

enum aw8697_haptic_ram_vbat_comp_mode {
	AW8697_HAPTIC_RAM_VBAT_COMP_DISABLE = 0,
	AW8697_HAPTIC_RAM_VBAT_COMP_ENABLE = 1,
};

enum aw8697_haptic_f0_flag {
	AW8697_HAPTIC_LRA_F0 = 0,
	AW8697_HAPTIC_CALI_F0 = 1,
};

enum aw8697_haptic_pwm_mode {
	AW8697_PWM_48K = 0,
	AW8697_PWM_24K = 1,
	AW8697_PWM_12K = 2,
};

enum aw8697_haptic_play {
	AW8697_HAPTIC_PLAY_NULL = 0,
	AW8697_HAPTIC_PLAY_ENABLE = 1,
	AW8697_HAPTIC_PLAY_STOP = 2,
	AW8697_HAPTIC_PLAY_GAIN = 8,
};

enum aw8697_haptic_cmd {
	AW8697_HAPTIC_CMD_NULL = 0,
	AW8697_HAPTIC_CMD_ENABLE = 1,
	AW8697_HAPTIC_CMD_HAPTIC = 0x0f,
	AW8697_HAPTIC_CMD_TP = 0x10,
	AW8697_HAPTIC_CMD_SYS = 0xf0,
	AW8697_HAPTIC_CMD_STOP = 255,
};

enum aw8697_haptic_cali_lra {
	AW8697_HAPTIC_F0_CALI_LRA = 1,
	AW8697_HAPTIC_RTP_CALI_LRA = 2,
};

struct fileops {
	unsigned char cmd;
	unsigned char reg;
	unsigned char ram_addrh;
	unsigned char ram_addrl;
};

struct ram {
	unsigned int len;
	unsigned int check_sum;
	unsigned int base_addr;
	unsigned char version;
	unsigned char ram_shift;
	unsigned char baseaddr_shift;
};

struct haptic_ctr {
	unsigned char cnt;
	unsigned char cmd;
	unsigned char play;
	unsigned char wavseq;
	unsigned char loop;
	unsigned char gain;
	struct list_head list;
};

struct haptic_audio {
	struct mutex lock;
	struct hrtimer timer;
	struct work_struct work;
	int delay_val;
	int timer_val;
	struct haptic_ctr ctr;
	struct list_head ctr_list;
	struct list_head list;
	int tz_num;
	int tz_high_num;
	int tz_cnt_thr;
	int tz_cnt_max;
	unsigned int uevent_report_flag;
	unsigned int hap_cnt_outside_tz;
	unsigned int hap_cnt_max_outside_tz;
};

struct trig {
	unsigned char enable;
	unsigned char default_level;
	unsigned char dual_edge;
	unsigned char frist_seq;
	unsigned char second_seq;
};

struct aw8697_dts_info {
	unsigned int mode;
	unsigned int f0_pre;
	unsigned int f0_cali_percen;
	unsigned int cont_drv_lvl;
	unsigned int cont_drv_lvl_ov;
	unsigned int cont_td;
	unsigned int cont_zc_thr;
	unsigned int cont_num_brk;
	unsigned int f0_coeff;
	unsigned int f0_trace_parameter[4];
	unsigned int bemf_config[4];
	unsigned int sw_brake;
	unsigned int tset;
	unsigned int r_spare;
	unsigned int bstdbg[6];
	unsigned int parameter1;
};

struct aw8697 {
	struct regmap *regmap;
	struct i2c_client *i2c;
	struct device *dev;
	struct input_dev *input;

	struct mutex lock;
	struct mutex rtp_lock;
	struct hrtimer timer;
	struct work_struct vibrator_work;
	struct work_struct rtp_work;
	struct delayed_work ram_work;
#ifdef TIMED_OUTPUT
	struct timed_output_dev to_dev;
#else
	struct led_classdev cdev;
#endif
	struct fileops fileops;
	struct ram ram;
	bool haptic_ready;
	bool audio_ready;
	int pre_haptic_number;
	int f0_val_err;
	struct timeval current_time;
	struct timeval pre_enter_time;
	struct timeval start, end;
	unsigned int timeval_flags;
	unsigned int osc_cali_flag;
	unsigned long int microsecond;
	unsigned int sys_frequency;
	unsigned int rtp_len;
	unsigned int reg_real_addr;
	int reset_gpio;
	int irq_gpio;

	unsigned char hwen_flag;
	unsigned char flags;
	unsigned char chipid;

	unsigned char play_mode;

	unsigned char activate_mode;

	unsigned char auto_boost;

	int state;
	int duration;
	int amplitude;
	int index;
	int rtp_idx;
	int vmax;
	int gain;

	unsigned char seq[AW8697_SEQUENCER_SIZE];
	unsigned char loop[AW8697_SEQUENCER_SIZE];

	unsigned int rtp_cnt;
	unsigned int rtp_file_num;

	unsigned char rtp_init;
	unsigned char ram_init;
	unsigned char rtp_routine_on;

	unsigned int f0;
	unsigned int cont_f0;
	unsigned char max_pos_beme;
	unsigned char max_neg_beme;
	unsigned char f0_cali_flag;
	unsigned int theory_time;

	unsigned char ram_vbat_comp;
	unsigned int vbat;
	unsigned int lra;
	unsigned int interval_us;

	struct trig trig[AW8697_TRIG_NUM];

	struct haptic_audio haptic_audio;
	struct aw8697_dts_info info;
	unsigned int ramupdate_flag;
	unsigned int rtpupdate_flag;
	unsigned int osc_cali_run;
	unsigned int lra_calib_data;
	unsigned int f0_calib_data;
	unsigned int effect_id;
	enum vib_mode_type_t effect_mode;
};

struct aw8697_container {
	int len;
	unsigned char data[];
};

struct aw8697_seq_loop {
	unsigned char loop[AW8697_SEQUENCER_SIZE];
};

struct aw8697_que_seq {
	unsigned char index[AW8697_SEQUENCER_SIZE];
};

#define AW8697_HAPTIC_IOCTL_MAGIC         'h'

#define AW8697_HAPTIC_SET_QUE_SEQ         _IOWR(AW8697_HAPTIC_IOCTL_MAGIC, 1, struct aw8697_que_seq*)
#define AW8697_HAPTIC_SET_SEQ_LOOP        _IOWR(AW8697_HAPTIC_IOCTL_MAGIC, 2, struct aw8697_seq_loop*)
#define AW8697_HAPTIC_PLAY_QUE_SEQ        _IOWR(AW8697_HAPTIC_IOCTL_MAGIC, 3, unsigned int)
#define AW8697_HAPTIC_SET_BST_VOL         _IOWR(AW8697_HAPTIC_IOCTL_MAGIC, 4, unsigned int)
#define AW8697_HAPTIC_SET_BST_PEAK_CUR    _IOWR(AW8697_HAPTIC_IOCTL_MAGIC, 5, unsigned int)
#define AW8697_HAPTIC_SET_GAIN            _IOWR(AW8697_HAPTIC_IOCTL_MAGIC, 6, unsigned int)
#define AW8697_HAPTIC_PLAY_REPEAT_SEQ     _IOWR(AW8697_HAPTIC_IOCTL_MAGIC, 7, unsigned int)

#endif