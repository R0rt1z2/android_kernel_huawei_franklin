/*
 * Copyright (C) 2017 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */

#ifndef __VOW_H__
#define __VOW_H__

#include <linux/compat.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include "vow_hw.h"
#include "vow_assert.h"

/*****************************************************************************
 * VOW Type Define
 *****************************************************************************/
#define DEBUG_VOWDRV 1

#if DEBUG_VOWDRV
#define VOWDRV_DEBUG(format, args...) pr_debug(format, ##args)
#else
#define VOWDRV_DEBUG(format, args...)
#endif


#define VOW_DEVNAME                    "vow"
#define VOW_IOC_MAGIC                  'V'
#define VOW_PRE_LEARN_MODE             1
#ifdef CONFIG_MTK_VOW_2E2K_SUPPORT
#define MAX_VOW_SPEAKER_MODEL          2
#else
#define MAX_VOW_SPEAKER_MODEL          4
#endif
#define VOW_WAITCHECK_INTERVAL_MS      1
#define MAX_VOW_INFO_LEN               9
#define VOW_VOICE_RECORD_THRESHOLD     2560 /* 80ms */
#define VOW_VOICE_RECORD_BIG_THRESHOLD 8320 /* 260ms */
#define VOW_IPI_SEND_CNT_TIMEOUT       500 /* 500ms */
/* UBM_V1:0xA000, UBM_V2:0xDC00, UBM_V3: 2*0x11000 */
#define VOW_TRANS_MODEL_SIZE           (220 * 1024)
#define VOW_MODEL_SIZE                 0x14000
#define VOW_VOICEDATA_OFFSET           (VOW_MODEL_SIZE * MAX_VOW_SPEAKER_MODEL)
#define VOW_VOICEDATA_SIZE             0x12500 /* 74880, need over 2.3sec */
/* IPI return value definition */
#define WORD_H                         16
#define WORD_L                         0
#define WORD_H_MASK                    0xFFFF0000
#define WORD_L_MASK                    0x0000FFFF
/* multiplier of cycle to ns in 13m clock */
#define CYCLE_TO_NS                    77
#define VOW_STOP_DUMP_WAIT             50
#define FRAME_BUF_SIZE                 8192
#define RESERVED_DATA                  4
#define VOW_RECOVERY_WAIT              100

//#define DUAL_CH_TRANSFER

/* length limitation sync by audio hal */
#if (defined CONFIG_MTK_VOW_DUAL_MIC_SUPPORT && defined DUAL_CH_TRANSFER)
#define VOW_VBUF_LENGTH      (0x12E80 * 2)  /*(0x12480 + 0x0A00) * 2*/
#else
#define VOW_VBUF_LENGTH      (0x12E80)  /* 0x12480 + 0x0A00 */
#endif

#define VOW_RECOGDATA_SIZE             0x2800
#define VOW_PCM_DUMP_BYTE_SIZE         0xA00 /* 320 * 8 */
#define VOW_EXTRA_DATA_SIZE            0x100 /* 256 */

#if (defined CONFIG_MTK_VOW_DUAL_MIC_SUPPORT && defined DUAL_CH_TRANSFER)
#define VOW_RECOGDATA_OFFSET    (VOW_VOICEDATA_OFFSET + 2 * VOW_VOICEDATA_SIZE)
#else
#define VOW_RECOGDATA_OFFSET        (VOW_VOICEDATA_OFFSET + VOW_VOICEDATA_SIZE)
#endif
#define VOW_EXTRA_DATA_OFFSET       (VOW_RECOGDATA_OFFSET + VOW_RECOGDATA_SIZE)

/* below is control message */
#define VOW_SET_CONTROL               _IOW(VOW_IOC_MAGIC, 0x03, unsigned int)
#define VOW_SET_SPEAKER_MODEL         _IOW(VOW_IOC_MAGIC, 0x04, unsigned int)
#define VOW_CLR_SPEAKER_MODEL         _IOW(VOW_IOC_MAGIC, 0x05, unsigned int)
#define VOW_SET_APREG_INFO            _IOW(VOW_IOC_MAGIC, 0x09, unsigned int)
#define VOW_BARGEIN_ON                _IOW(VOW_IOC_MAGIC, 0x0A, unsigned int)
#define VOW_BARGEIN_OFF               _IOW(VOW_IOC_MAGIC, 0x0B, unsigned int)
#define VOW_CHECK_STATUS              _IOW(VOW_IOC_MAGIC, 0x0C, unsigned int)
#define VOW_RECOG_ENABLE              _IOW(VOW_IOC_MAGIC, 0x0D, unsigned int)
#define VOW_RECOG_DISABLE             _IOW(VOW_IOC_MAGIC, 0x0E, unsigned int)
#define VOW_MODEL_START               _IOW(VOW_IOC_MAGIC, 0x0F, unsigned int)
#define VOW_MODEL_STOP                _IOW(VOW_IOC_MAGIC, 0x10, unsigned int)
#define VOW_SET_SKIP_SAMPLE_COUNT     _IOW(VOW_IOC_MAGIC, 0x11, unsigned int)
#define VOW_SET_PARAM                 _IOW(VOW_IOC_MAGIC, 0x12, unsigned int)

#ifdef CONFIG_MTK_VOW_BARGE_IN_SUPPORT

#ifdef VOW_ECHO_SW_SRC
#define VOW_BARGEIN_DUMP_OFFSET 0x1E00
#else
#define VOW_BARGEIN_DUMP_OFFSET 0xA00
#endif
#define VOW_BARGEIN_DUMP_SIZE    0x3C00
#endif  /* #ifdef CONFIG_MTK_VOW_BARGE_IN_SUPPORT */

#define KERNEL_VOW_DRV_VER "2.0.11"
struct dump_package_t {
	uint32_t dump_data_type;
	uint32_t mic_offset;
	uint32_t mic_data_size;
	uint32_t recog_data_offset;
	uint32_t recog_data_size;
#ifdef CONFIG_MTK_VOW_DUAL_MIC_SUPPORT
	uint32_t mic_offset_R;
	uint32_t mic_data_size_R;
	uint32_t recog_data_offset_R;
	uint32_t recog_data_size_R;
#endif  /* #ifdef CONFIG_MTK_VOW_DUAL_MIC_SUPPORT */
	uint32_t echo_offset;
	uint32_t echo_data_size;
};

struct dump_queue_t {
	struct dump_package_t dump_package[256];
	uint8_t idx_r;
	uint8_t idx_w;
};

struct dump_work_t {
	struct work_struct work;
	uint32_t mic_offset;
	uint32_t mic_data_size;
	uint32_t recog_data_offset;
	uint32_t recog_data_size;
#ifdef CONFIG_MTK_VOW_DUAL_MIC_SUPPORT
	uint32_t mic_offset_R;
	uint32_t mic_data_size_R;
	uint32_t recog_data_offset_R;
	uint32_t recog_data_size_R;
#endif  /* #ifdef CONFIG_MTK_VOW_DUAL_MIC_SUPPORT */
	uint32_t echo_offset;
	uint32_t echo_data_size;
};

enum { /* dump_data_t */
	DUMP_RECOG = 0,
#ifdef CONFIG_MTK_VOW_BARGE_IN_SUPPORT
	DUMP_BARGEIN,
	DUMP_INPUT,
#endif  /* #ifdef CONFIG_MTK_VOW_BARGE_IN_SUPPORT */
	NUM_DUMP_DATA,
};


/*****************************************************************************
 * VOW Enum
 *****************************************************************************/
enum vow_control_cmd_t {
	VOWControlCmd_Init = 0,
	VOWControlCmd_ReadVoiceData,
	VOWControlCmd_EnableDebug,
	VOWControlCmd_DisableDebug,
	VOWControlCmd_EnableSeamlessRecord,
	VOWControlCmd_EnableDump,
	VOWControlCmd_DisableDump,
	VOWControlCmd_Reset,
};

enum vow_ipi_msgid_t {
	IPIMSG_VOW_ENABLE = 0,
	IPIMSG_VOW_DISABLE = 1,
	IPIMSG_VOW_SETMODE = 2,
	IPIMSG_VOW_APREGDATA_ADDR = 3,
	IPIMSG_VOW_SET_MODEL = 4,
	IPIMSG_VOW_SET_FLAG = 5,
	IPIMSG_VOW_SET_SMART_DEVICE = 6,
#ifdef CONFIG_MTK_VOW_BARGE_IN_SUPPORT
	IPIMSG_VOW_SET_BARGEIN_ON = 10,
	IPIMSG_VOW_SET_BARGEIN_OFF = 11,
#endif /* #ifdef CONFIG_MTK_VOW_BARGE_IN_SUPPORT */
	IPIMSG_VOW_PCM_DUMP_ON = 12,
	IPIMSG_VOW_PCM_DUMP_OFF = 13,
	IPIMSG_VOW_COMBINED_INFO = 17,
	IPIMSG_VOW_MODEL_START = 18,
	IPIMSG_VOW_MODEL_STOP = 19,
	IPIMSG_VOW_RETURN_VALUE = 20,
	IPIMSG_VOW_SET_SKIP_SAMPLE_COUNT = 21,
	IPIMSG_VOW_SET_PARAM = 22,
};

enum vow_eint_status_t {
	VOW_EINT_DISABLE = -2,
	VOW_EINT_FAIL = -1,
	VOW_EINT_PASS = 0,
	VOW_EINT_RETRY = 1,
	NUM_OF_VOW_EINT_STATUS = 4
};

enum vow_flag_type_t {
	VOW_FLAG_DEBUG = 0,
	VOW_FLAG_PRE_LEARN,
	VOW_FLAG_DMIC_LOWPOWER,
	VOW_FLAG_PERIODIC_ENABLE,
	VOW_FLAG_FORCE_PHASE1_DEBUG,
	VOW_FLAG_FORCE_PHASE2_DEBUG,
	VOW_FLAG_SWIP_LOG_PRINT,
	VOW_FLAG_MTKIF_TYPE,
	VOW_FLAG_SEAMLESS,
	VOW_FLAG_DUAL_MIC_SWITCH,
	VOW_FLAG_MCPS,
	NUM_OF_VOW_FLAG_TYPE
};

enum vow_pwr_status_t {
	VOW_PWR_OFF = 0,
	VOW_PWR_ON = 1,
	NUM_OF_VOW_PWR_STATUS
};

enum vow_ipi_result_t {
	VOW_IPI_SUCCESS = 0,
	VOW_IPI_CLR_SMODEL_ID_NOTMATCH,
	VOW_IPI_SET_SMODEL_NO_FREE_SLOT,
};

enum vow_force_phase_t {
	NO_FORCE = 0,
	FORCE_PHASE1,
	FORCE_PHASE2,
};

enum vow_model_type_t {
	VOW_MODEL_INIT = 0,    /* no use */
	VOW_MODEL_SPEAKER = 1,
	VOW_MODEL_NOISE = 2,   /* no use */
	VOW_MODEL_FIR = 3,      /* no use */
	VOW_MODEL_CLEAR = 4
};

enum vow_mtkif_type_t {
	VOW_MTKIF_NONE = 0,
	VOW_MTKIF_AMIC = 1,
	VOW_MTKIF_DMIC = 2,
	VOW_MTKIF_DMIC_LP = 3,
};

enum vow_channel_t {
	VOW_MONO = 0,
	VOW_STEREO = 1,
};

enum vow_model_status_t {
	VOW_MODEL_STATUS_START = 1,
	VOW_MODEL_STATUS_STOP = 0
};

enum vow_model_control_t {
	VOW_SET_MODEL = 0,
	VOW_CLEAN_MODEL = 1
};

enum {
	VENDOR_ID_MTK = 77,     //'M'
	VENDOR_ID_AMAZON = 65,  //'A'
	VENDOR_ID_HUAWEI = 72,  //'huawei'
	VENDOR_ID_OTHERS = 71,  //'G'
	VENDOR_ID_NONE = 0
};

enum {
	VOW_ENABLE_DUAL_MIC = 2,
	VOW_ENABLE_SINGLE_MAIN_MIC = 1,
	VOW_ENABLE_SINGLE_REF_MIC = 0
};

/*****************************************************************************
 * VOW Structure Define
 *****************************************************************************/
struct vow_ipi_msg_t {
	short id;
	short size;
	short *buf;
};

struct vow_eint_data_struct_t {
	int size;        /* size of data section */
	int eint_status; /* eint status */
	int id;          /* user id */
	char data[RESERVED_DATA];    /* reserved for future extension */
};

typedef struct vow_setparam_t{
	int alg_type;
	int handle;
	int param_key;
	int param_value;
} vow_setparam_t;

#ifdef CONFIG_COMPAT

struct vow_speaker_model_t {
	void *model_ptr;
	int  id;
	int  uuid;
	int  flag;
	int  enabled;
	unsigned int model_size;
	unsigned int confidence_lv;
	unsigned long rx_inform_addr;
	unsigned long rx_inform_size_addr;
	int alg_type;
	unsigned int scene;
	unsigned int send_model_size;
};

struct vow_model_info_t {
	long  id;
	long  addr;
	long  size;
	long  return_size_addr;
	long  uuid;
	long  alg_type;
	long  scene;
	long  send_size;
	void *data;
};

struct vow_model_start_t {
	long handle;
	long confidence_level;
	long dsp_inform_addr;
	long dsp_inform_size_addr;
	int alg_type;
};

struct vow_model_info_kernel_t {
	compat_size_t  id;
	compat_size_t  addr;
	compat_size_t  size;
	compat_size_t  return_size_addr;
	compat_size_t  uuid;
	compat_size_t  alg_type;
	compat_size_t  scene;
	compat_size_t  send_size;
	compat_uptr_t *data;
};

struct vow_model_start_kernel_t {
	compat_size_t handle;
	compat_size_t confidence_level;
	compat_size_t dsp_inform_addr;
	compat_size_t dsp_inform_size_addr;
	compat_size_t alg_type;
};

typedef struct vow_setparam_kernel_t{
	compat_size_t alg_type;
	compat_size_t handle;
	compat_size_t param_key;
	compat_size_t param_value;
} vow_setparam_kernel_t;

struct vow_set_skip_sample_count_t {
	unsigned int skip_sample_count; // Unit: sample
	unsigned int sampling_rate;     // Unit: Hz (samples per second)
};

struct vow_set_skip_sample_count_kernel_t {
	compat_size_t skip_sample_count;
	compat_size_t sampling_rate;
};

#else  /* #ifdef CONFIG_COMPAT */

struct vow_speaker_model_t {
	void *model_ptr;
	int  id;
	int  uuid;
	int  flag;
	int  enabled;
	unsigned int model_size;
	unsigned int confidence_lv;
	unsigned long rx_inform_addr;
	unsigned long rx_inform_size_addr;
	int alg_type;
	unsigned int scene;
	unsigned int send_model_size;
};

struct vow_model_info_t {
	long  id;
	long  addr;
	long  size;
	long  return_size_addr;
	long  uuid;
	long  alg_type;
	long  scene;
	long  send_size;
	void *data;
};

struct vow_model_start_t {
	long handle;
	long confidence_level;
	long dsp_inform_addr;
	long dsp_inform_size_addr;
	int alg_type;
};

struct vow_set_skip_sample_count_t {
	unsigned int skip_sample_count; // Unit: sample
	unsigned int sampling_rate;     // Unit: Hz (samples per second)
};
#endif  /* #ifdef CONFIG_COMPAT */

enum ipi_type_flag_t {
	RECOG_OK_IDX = 0,
	DEBUG_DUMP_IDX = 1,
	RECOG_DUMP_IDX = 2,
	BARGEIN_DUMP_INFO_IDX = 3,
	BARGEIN_DUMP_IDX = 4,
	INPUT_DUMP_IDX = 5
};

#define RECOG_OK_IDX_MASK           (0x01 << RECOG_OK_IDX)
#define DEBUG_DUMP_IDX_MASK         (0x01 << DEBUG_DUMP_IDX)
#define RECOG_DUMP_IDX_MASK         (0x01 << RECOG_DUMP_IDX)
#define BARGEIN_DUMP_INFO_IDX_MASK  (0x01 << BARGEIN_DUMP_INFO_IDX)
#define BARGEIN_DUMP_IDX_MASK       (0x01 << BARGEIN_DUMP_IDX)
#define INPUT_DUMP_IDX_MASK         (0x01 << INPUT_DUMP_IDX)

struct vow_ipi_combined_info_t {
	unsigned short ipi_type_flag;
	/* IPIMSG_VOW_RECOGNIZE_OK */
	unsigned short recog_ok_uuid;
	/* unsigned int recog_ret_info; */
	unsigned int confidence_lv;
	unsigned long long recog_ok_os_timer;
	unsigned int extra_data_len;
	/* IPIMSG_VOW_DATAREADY */
	unsigned int voice_buf_offset;
	unsigned int voice_length;
#ifdef CONFIG_MTK_VOW_BARGE_IN_SUPPORT
	/* IPIMSG_VOW_BARGEIN_DUMP_INFO */
	unsigned int dump_frm_cnt;
	unsigned int voice_sample_delay;
	/* IPIMSG_VOW_BARGEIN_PCMDUMP_OK */
	unsigned int mic_dump_size;
	unsigned int mic_offset;
#ifdef CONFIG_MTK_VOW_DUAL_MIC_SUPPORT
//	unsigned int mic_dump_size_R;
	unsigned int mic_offset_R;
#endif  /* #ifdef CONFIG_MTK_VOW_DUAL_MIC_SUPPORT */
	unsigned int echo_dump_size;
	unsigned int echo_offset;
#endif  /* #ifdef CONFIG_MTK_VOW_BARGE_IN_SUPPORT */
	unsigned int recog_dump_size;
	unsigned int recog_dump_offset;
#ifdef CONFIG_MTK_VOW_DUAL_MIC_SUPPORT
//	unsigned int recog_dump_size_R;
	unsigned int recog_dump_offset_R;
#endif  /* #ifdef CONFIG_MTK_VOW_DUAL_MIC_SUPPORT */
};



/*****************************************************************************
 * VOW Function Declaration
 *****************************************************************************/
bool vow_service_GetScpRecoverStatus(void);
bool vow_service_GetVowRecoverStatus(void);
void vow_ipi_rx_internal(unsigned int msg_id, void *msg_data);
bool vow_ipi_rceive_ack(unsigned int msg_id, unsigned int msg_data);

#endif /*__VOW_H__ */
