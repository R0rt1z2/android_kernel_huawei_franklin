#ifndef __SENSOR_PARA_H
#define __SENSOR_PARA_H

struct als_para_t {
	uint8_t tp_type;
	uint8_t tp_color;
	int cali_target[2];
	uint8_t als_extend_para[50];
	uint8_t product_name;
	int32_t lcd_brightness;
	uint8_t cali_oder;
	char lcd_type[20];
	int32_t als_rawdata[4];
};

struct ps_para_t {
	uint8_t pwave;
	uint8_t pwindow;
};

struct acc_para_t {
	uint8_t lsm6dsm_direction;
	uint8_t bmi160_direction;
	uint8_t lis2dwl_direction;
	uint8_t bma4xy_direction;
	uint8_t kx022_direction;
	uint8_t bma253_direction;
	uint8_t da718_direction;
};

struct sar_para_t {
 	uint8_t temp;
};

struct mag_para_t {
	uint8_t product_id;
	uint8_t usb_status;
	unsigned int charging_current;
};

struct finger_para_t {
	short fg_data[256]; // fg sense need data size
	int head;
	int tail;
	int finger_ready;
};

struct extend_data_t {
	uint8_t ar_record;
	uint8_t step_record;
	uint8_t level_record;
	uint8_t reserved;
};

struct extend_step_para_t {
	uint64_t begin_rtc_time;
	uint64_t gap_time;
	uint32_t begin_time;
	uint16_t record_count;
	uint16_t capacity;
	uint32_t total_step_count;
	uint32_t total_floor_ascend;
	uint32_t total_calorie;
	uint32_t total_distance;
	uint16_t step_pace;
	uint16_t step_length;
	uint16_t speed;
	uint16_t touchdown_ratio;
	uint16_t reserved1;
	uint16_t reserved2; // 36
	struct extend_data_t action_record[120];
};

struct sensor_para_t{
	struct als_para_t als_para;
	struct ps_para_t ps_para;
	struct acc_para_t acc_para;
	struct sar_para_t sar_para;
	struct mag_para_t mag_para;
	struct finger_para_t fg_para;
	struct extend_step_para_t extend_step;
	uint8_t product_id;
};

enum sh_mem_type_t {
	SHR_MEM_TYPE_ALL,
	SHR_MEM_TYPE_ACC,
	SHR_MEM_TYPE_GYR,
	SHR_MEM_TYPE_ALS,
	SHR_MEM_TYPE_PROX,
	SHR_MEM_TYPE_FINGER,
	SHR_MEM_TYPE_PEDO,
	SHR_MEM_TYPE_MAG,
	SHR_MEM_TYPE_MAX,
};

void *get_sensor_share_mem_addr(enum sh_mem_type_t type);
void send_sensor_scp_brightness(int brightness);
#endif
