/*
 * drivers/inputhub/sensor_detect.h
 *
 * sensors detection header file
 *
 * Copyright (c) 2012-2019 Huawei Technologies Co., Ltd.
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

#ifndef __SENSOR_DETECT_H
#define __SENSOR_DETECT_H

#include "protocol.h"

#define AOD_FEATURE_NUM 2

#define MAX_CHIP_INFO_LEN 50
#define MAX_STR_SIZE 1024
#define MAX_PHONE_COLOR_NUM 15
#define CYPRESS_CHIPS 2
#define SENSOR_PLATFORM_EXTEND_DATA_SIZE 50
#define SENSOR_PLATFORM_EXTEND_ALS_DATA_SIZE 68

#define DEF_SENSOR_COM_SETTING \
{ \
	.bus_type = TAG_I2C, \
	.bus_num = 0, \
	.disable_sample_thread = 0, \
	{ .data = 0 } \
}

typedef uint16_t gpio_num_type;

typedef enum {
	ACC,
	MAG,
	GYRO,
	ALS,
	PS,
	AIRPRESS,
	CAP_PROX,
	CONNECTIVITY,
	RPC,
	SH_AOD,
	ACC1,
	GYRO1,
	ALS1,
	MAG1,
	ALS2,
	CAP_PROX1,
	MOTION,
	SOUND,
	THERMOMETER,
	SENSOR_MAX
} sensor_detect_list;

#define SENSOR_DETECT_RETRY 2
typedef enum {
	BOOT_DETECT,
	DETECT_RETRY,
	BOOT_DETECT_END = DETECT_RETRY + SENSOR_DETECT_RETRY - 1,
	REDETECT_LATER
} detect_mode;

enum ALS_UD_TYPE_T {
	ALS_UD_NONE,
	ALS_UD_MINUS_TP_RGB,
	ALS_UD_MINUS_DSS_NOISE,
};

struct sleeve_detect_pare {
	unsigned int tp_color;
	unsigned int sleeve_detect_threshhold;
};

struct sensor_combo_cfg {
	uint8_t bus_type;
	uint8_t bus_num;
	uint8_t disable_sample_thread;
	union {
		uint32_t data;
		uint32_t i2c_address;
		union spi_ctrl ctrl;
	};
} __packed;

struct tof_platform_data {
	struct sensor_combo_cfg cfg;
	int tof_calib_zero_threshold;
	int tof_calib_6cm_threshold;
	int tof_calib_10cm_threshold;
	int tof_calib_60cm_threshold;
};

struct handpress_platform_data {
	struct sensor_combo_cfg cfg;
	uint8_t bootloader_type;
	uint8_t id[CYPRESS_CHIPS];
	uint8_t i2c_address[CYPRESS_CHIPS];
	uint8_t t_pionts[CYPRESS_CHIPS];
	uint16_t poll_interval;
	uint32_t irq[CYPRESS_CHIPS];
	uint8_t handpress_extend_data[SENSOR_PLATFORM_EXTEND_DATA_SIZE];
};

struct connectivity_platform_data {
	struct sensor_combo_cfg cfg;
	uint16_t poll_interval;
	gpio_num_type gpio1_gps_cmd_ap;
	gpio_num_type gpio1_gps_cmd_sh;
	gpio_num_type gpio2_gps_ready_ap;
	gpio_num_type gpio2_gps_ready_sh;
	gpio_num_type gpio3_wakeup_gps_ap;
	gpio_num_type gpio3_wakeup_gps_sh;
	uint32_t i3c_frequency;
	uint16_t gpio1_gps_cmd_pinmux;
	uint16_t gpio2_gps_ready_pinmux;
	uint16_t gpio3_wakeup_gps_pinmux;
};

struct motion_platform_data {
	uint8_t motion_horizontal_pickup_flag;
	uint8_t angle_gap;
};

struct aod_platform_data {
	struct sensor_combo_cfg cfg;
	uint32_t feature_set[AOD_FEATURE_NUM];
};

struct rpc_platform_data {
	uint16_t table[32];
	uint16_t mask[32];
	uint16_t default_value;
	uint16_t mask_enable;
	uint16_t sar_choice;
	uint16_t sim_type_swtich_flag;
	uint16_t fusion_type;
};

#define MAX_TX_RX_LEN 32
struct detect_word {
	struct sensor_combo_cfg cfg;
	uint32_t tx_len;
	uint8_t tx[MAX_TX_RX_LEN];
	uint32_t rx_len;
	uint8_t rx_msk[MAX_TX_RX_LEN];
	uint32_t exp_n;
	uint8_t rx_exp[MAX_TX_RX_LEN];
};

#define MAX_SENSOR_NAME_LENGTH 20
struct sensor_detect_manager {
	char sensor_name_str[MAX_SENSOR_NAME_LENGTH];
	sensor_detect_list sensor_id;
	uint8_t detect_result;
	int tag;
	const void *spara;
	int cfg_data_length;
};

#define MAX_REDETECT_NUM 100
struct sensor_redetect_state {
	uint8_t need_redetect_sensor;
	uint8_t need_recovery;
	uint8_t detect_fail_num;
	uint8_t redetect_num;
};

#define DETECTED_DEVICE_ID_NUM 4
struct detect_device_para {
	int i2c_address;
	int i2c_bus_num;
	int register_add;
	u32 commu_len;
	uint32_t register_add_len;
	uint32_t rx_len;
	uint32_t device_id;
	uint8_t detected_device_id[DETECTED_DEVICE_ID_NUM];
	uint8_t tag;
};

struct sensorlist_info {
	/*
	 * Name of this sensor.
	 * All sensors of the same "type" must have a different "name".
	 */
	char name[50];

	/* vendor of the hardware part */
	char vendor[50];
	/*
	 * version of the hardware part + driver. The value of this field
	 * must increase when the driver is updated in a way that changes the
	 * output of this sensor. This is important for fused sensors when the
	 * fusion algorithm is updated.
	 */
	int32_t version;

	/* maximum range of this sensor's value in SI units */
	int32_t max_range;

	/* smallest difference between two values reported by this sensor */
	int32_t resolution;

	/* rough estimate of this sensor's power consumption in mA */
	int32_t power;

	/*
	 * this value depends on the reporting mode:
	 * continuous: minimum sample period allowed in microseconds
	 * on-change : 0
	 * one-shot  :-1
	 * special   : 0, unless otherwise noted
	 */
	int32_t min_delay;

	/*
	 * number of events reserved for this sensor in the batch mode FIFO.
	 * If there is a dedicated FIFO for this sensor, then this is the
	 * size of this FIFO. If the FIFO is shared with other sensors,
	 * this is the size reserved for that sensor and it can be zero.
	 */
	uint32_t fifo_reserved_event_count;

	/*
	 * maximum number of events of this sensor that could be batched.
	 * This is especially relevant when the FIFO is shared between
	 * several sensors; this value is then set to the size of that FIFO.
	 */
	uint32_t fifo_max_event_count;
	/*
	 * This value is defined only for continuous mode and on-change sensors.
	 * It is the delay between two sensor events corresponding to the
	 * lowest frequency that this sensor supports.
	 * When lower frequencies are requested through batch()/setDelay()
	 * the events will be generated at this frequency instead.
	 * It can be used by the framework or applications to estimate
	 * when the batch FIFO may be full.
	 *
	 * @note
	 *   1) period_ns is in nanoseconds
	 *      where as maxDelay/minDelay are in microseconds.
	 *         continuous, on-change: maximum sampling period
	 *                                allowed in microseconds.
	 *         one-shot, special : 0
	 *   2) maxDelay should always fit within a 32 bit signed integer.
	 *      It is declared as 64 bit
	 *      on 64 bit architectures only for binary compatibility reasons.
	 * Availability: SENSORS_DEVICE_API_VERSION_1_3
	 */
	int32_t max_delay;

	/*
	 * Flags for sensor. See SENSOR_FLAG_* above.
	 * Only the least significant 32 bits are used here.
	 * It is declared as 64 bit on 64 bit architectures
	 * only for binary compatibility reasons.
	 * Availability: SENSORS_DEVICE_API_VERSION_1_3
	 */
	uint32_t flags;
};

extern int iom3_power_state;

extern int mag_opend;
extern int sonic_ps_use_rcv;

extern struct sensor_detect_manager sensor_manager[SENSOR_MAX];
extern struct sensorlist_info sensorlist_info[SENSOR_MAX];
extern struct sleeve_detect_pare sleeve_detect_paremeter[MAX_PHONE_COLOR_NUM];

extern u8 phone_color;
extern char sensor_chip_info[SENSOR_MAX][MAX_CHIP_INFO_LEN];
extern int hifi_supported;
extern struct config_on_ddr *g_config_on_ddr;

#ifdef CONFIG_HUAWEI_DSM
extern struct dsm_dev dsm_sensorhub;
extern struct dsm_client *shb_dclient;
#endif

void send_parameter_to_mcu(sensor_detect_list s_id, int cmd);
void read_aux_file_list(uint16_t fileid, uint16_t tag);
void read_dyn_file_list(uint16_t fileid);
void read_chip_info(struct device_node *dn, sensor_detect_list sname);
void read_sensorlist_info(struct device_node *dn, int sensor);
int _device_detect(struct device_node *dn, int index,
	struct sensor_combo_cfg *p_succ_ret);
int get_combo_bus_tag(const char *bus, uint8_t *tag);
sensor_detect_list get_id_by_sensor_tag(int tag);
int sensor_set_cfg_data(void);
int send_fileid_to_mcu(void);
void sensor_redetect_enter(void);
void sensor_redetect_init(void);
int sensor_set_fw_load(void);
int motion_set_cfg_data(void);

#endif /* __SENSOR_DETECT_H */
