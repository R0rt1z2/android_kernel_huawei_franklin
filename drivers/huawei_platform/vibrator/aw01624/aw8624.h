/*
 * z aw8624.h
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

#ifndef _AW8624_H_
#define _AW8624_H_

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

#define MAX_I2C_BUFFER_SIZE                     65536
#define AW8624_REG_MAX                          0xff
#define AW8624_SEQUENCER_SIZE                   8
#define AW8624_SEQUENCER_LOOP_SIZE              4
#define AW8624_RTP_I2C_SINGLE_MAX_NUM           512
#define HAPTIC_MAX_TIMEOUT                      10000
#define AW8624_VBAT_REFER                       5000
#define AW8624_VBAT_MIN                         3000
#define AW8624_VBAT_MAX                         4500
#define AW6824_MAX_HAPTIC_CNT                   256

#define AW8624_CONT_NUM_BRK                     7
#define AW8624_DEFAULT_DELAY_VAL                23
#define AW8624_DEFAULT_TIMER_VAL                23
#define AW8624_F0_CALI_THER_H                   (2350 + 150)
#define AW8624_F0_CALI_THER_L                   (2350 - 150)
#define RTP_WORK_HZ_12K                         2
#define EFFECT_ID_MAX                           100
#define EFFECT_MAX_SHORT_VIB_ID                 10
#define EFFECT_SHORT_VIB_AVAIABLE               3
#define EFFECT_MAX_LONG_VIB_ID                  20
#define LONG_VIB_EFFECT_ID                      4
#define LONG_HAPTIC_RTP_MAX_ID                  4999
#define LONG_HAPTIC_RTP_MIN_ID                  1010

#define SHORT_HAPTIC_RAM_MAX_ID                 309
#define SHORT_HAPTIC_RTP_MAX_ID                 9999

#define SHORT_HAPTIC_RAM_MIN_IDX                1
#define SHORT_HAPTIC_RAM_MAX_IDX                30
#define SHORT_HAPTIC_RTP_MAX_IDX                9999

#define SHORT_HAPTIC_AMP_DIV_COFF               10
#define LONG_TIME_AMP_DIV_COFF                  100
#define BASE_INDEX                              31
#define AW8624_LONG_HAPTIC_RUNNING              4253
#define AW8624_INTM_EXCEP_FLAG      0x20
#define AW8624_DBG_EXCEP_FLAG       0x7f
#define AW8624_DRV_WAV              10
#define AW8624_BRK_WAV              11
#define AW8624_LONG_WAV_1_CYL       12
#define AW8624_LONG_WAV_2_CYL       13
#define AW8624_LONG_WAV_4_CYL       14
#define AW8624_LONG_WAV_8_CYL       15
#define AW8624_LONG_SEQ_MAX_TIME    90 // 6 seqs * 15 times
#define AW8624_MAX_PLAY_TIME        14

enum seq_long_cycle_type_t {
	SEQ_LONG_ONE_CYCLE = 1,
	SEQ_LONG_TWO_CYCLE = 2,
	SEQ_LONG_FOUR_CYCLE = 4,
	SEQ_LONG_EIGHT_CYCLE = 8,
};

struct aw8624_dts_info {
	int aw8624_mode;
	int aw8624_f0_pre;
	int aw8624_f0_cali_percen;
	int aw8624_cont_drv_lvl;
	int aw8624_cont_drv_lvl_ov;
	int aw8624_cont_td;
	int aw8624_cont_zc_thr;
	int aw8624_cont_num_brk;
	int aw8624_f0_coeff;
	int aw8624_duration_time[5];
	int aw8624_cont_brake[3][8];
	int aw8624_f0_trace_parameter[4];
	int aw8624_bemf_config[4];
	int aw8624_sw_brake[2];
	int aw8624_wavseq[16];
	int aw8624_wavloop[10];
	int aw8624_td_brake[3];
	int aw8624_tset;
	int aw8624_parameter1;
};

enum aw8624_flags {
	AW8624_FLAG_NONR = 0,
	AW8624_FLAG_SKIP_INTERRUPTS = 1,
};

enum aw8624_chipids {
	AW8624_ID = 1,
};

enum aw8624_haptic_read_write {
	AW8624_HAPTIC_CMD_READ_REG = 0,
	AW8624_HAPTIC_CMD_WRITE_REG = 1,
};

enum aw8624_haptic_work_mode {
	AW8624_HAPTIC_STANDBY_MODE = 0,
	AW8624_HAPTIC_RAM_MODE = 1,
	AW8624_HAPTIC_RTP_MODE = 2,
	AW8624_HAPTIC_TRIG_MODE = 3,
	AW8624_HAPTIC_CONT_MODE = 4,
	AW8624_HAPTIC_RAM_LOOP_MODE = 5,
};

enum aw8624_haptic_bst_mode {
	AW8624_HAPTIC_BYPASS_MODE = 0,
	AW8624_HAPTIC_BOOST_MODE = 1,
};

enum aw8624_haptic_activate_mode {
	AW8624_HAPTIC_ACTIVATE_RAM_MODE = 0,
	AW8624_HAPTIC_ACTIVATE_CONT_MODE = 1,
};

enum aw8624_haptic_vbat_comp_mode {
	AW8624_HAPTIC_VBAT_SW_COMP_MODE = 0,
	AW8624_HAPTIC_VBAT_HW_COMP_MODE = 1,
};

enum aw8624_haptic_ram_vbat_comp_mode {
	AW8624_HAPTIC_RAM_VBAT_COMP_DISABLE = 0,
	AW8624_HAPTIC_RAM_VBAT_COMP_ENABLE = 1,
};

enum aw8624_haptic_f0_flag {
	AW8624_HAPTIC_LRA_F0 = 0,
	AW8624_HAPTIC_CALI_F0 = 1,
};

enum aw8624_haptic_pwm_mode {
	AW8624_PWM_48K = 0,
	AW8624_PWM_24K = 1,
	AW8624_PWM_12K = 2,
};

enum z_vib_mode_type {
	SHORT_VIB_RAM_MODE = 0,
	LONG_VIB_RAM_MODE = 1,
	RTP_VIB_MODE = 2,
	VIB_MODE_MAX,
};

enum aw8624_haptic_cali_lra {
	AW8624_HAPTIC_F0_CALI_LRA = 1,
	AW8624_HAPTIC_RTP_CALI_LRA = 2,
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
	unsigned char cnt;
	struct haptic_ctr ctr[AW6824_MAX_HAPTIC_CNT];
};

struct aw8624 {
	struct regmap *regmap;
	struct i2c_client *i2c;
	struct device *dev;
	struct input_dev *input;

	struct mutex lock;
	struct mutex rtp_lock;
	struct wakeup_source *ws;
	unsigned char wk_lock_flag;
	struct hrtimer timer;
	struct work_struct vibrator_work;
	struct work_struct irq_work;
	struct work_struct rtp_work;
	struct delayed_work ram_work;
	struct delayed_work stop_work;
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
	struct timeval current_time;
	struct timeval pre_enter_time;
	struct timeval start, end;
	unsigned int timeval_flags;
	unsigned int osc_cali_flag;
	unsigned long int microsecond;
	unsigned int theory_time;
	unsigned int rtp_len;
	int reset_gpio;
	int irq_gpio;
	int reset_gpio_ret;
	int irq_gpio_ret;

	unsigned char hwen_flag;
	unsigned char flags;
	unsigned char chipid;
	unsigned char chipid_flag;
	unsigned char singlecycle;
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
	int f0_value;

	unsigned char seq[AW8624_SEQUENCER_SIZE];
	unsigned char loop[AW8624_SEQUENCER_SIZE];

	unsigned int rtp_cnt;
	unsigned int rtp_file_num;

	unsigned char rtp_init;
	unsigned char ram_init;
	unsigned char rtp_routine_on;

	unsigned int f0;
	unsigned int f0_pre;
	unsigned int cont_td;
	unsigned int cont_f0;
	unsigned int cont_zc_thr;
	unsigned char cont_drv_lvl;
	unsigned char cont_drv_lvl_ov;
	unsigned char cont_num_brk;
	unsigned char max_pos_beme;
	unsigned char max_neg_beme;
	unsigned char f0_cali_flag;
	bool is_used_irq;
	bool ram_update_delay;
	bool dts_add_real;
	unsigned int reg_real_addr;
	struct haptic_audio haptic_audio;

	unsigned char ram_vbat_comp;
	unsigned int vbat;
	unsigned int lra;
	unsigned int interval_us;
	unsigned int ramupdate_flag;
	unsigned int rtpupdate_flag;
	unsigned int osc_cali_run;
	unsigned int lra_calib_data;
	unsigned int f0_calib_data;

	unsigned char  ram_update_going;
	unsigned short ram_checksum_chip;
	unsigned int effect_id;
	enum z_vib_mode_type effect_mode;
};

struct aw8624_container {
	int len;
	unsigned char data[];
};

struct aw8624_seq_loop {
	unsigned char loop[AW8624_SEQUENCER_SIZE];
};

struct aw8624_que_seq {
	unsigned char index[AW8624_SEQUENCER_SIZE];
};

#define AW8624_HAPTIC_IOCTL_MAGIC             'h'
#define AW8624_HAPTIC_SET_QUE_SEQ \
	_IOWR(AW8624_HAPTIC_IOCTL_MAGIC, 1, struct aw8624_que_seq*)
#define AW8624_HAPTIC_SET_SEQ_LOOP \
	_IOWR(AW8624_HAPTIC_IOCTL_MAGIC, 2, struct aw8624_seq_loop*)
#define AW8624_HAPTIC_PLAY_QUE_SEQ \
	_IOWR(AW8624_HAPTIC_IOCTL_MAGIC, 3, unsigned int)
#define AW8624_HAPTIC_SET_BST_VOL \
	_IOWR(AW8624_HAPTIC_IOCTL_MAGIC, 4, unsigned int)
#define AW8624_HAPTIC_SET_BST_PEAK_CUR \
	_IOWR(AW8624_HAPTIC_IOCTL_MAGIC, 5, unsigned int)
#define AW8624_HAPTIC_SET_GAIN \
	_IOWR(AW8624_HAPTIC_IOCTL_MAGIC, 6, unsigned int)
#define AW8624_HAPTIC_PLAY_REPEAT_SEQ \
	_IOWR(AW8624_HAPTIC_IOCTL_MAGIC, 7, unsigned int)

#endif
