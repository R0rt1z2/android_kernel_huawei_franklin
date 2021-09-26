#ifndef __AW882XX_MISC_H_
#define __AW882XX_MISC_H_

#include <linux/miscdevice.h>

#define AFE_PARAM_ID_AWDSP_RX_SET_ENABLE		(0x10013D11)
#define AFE_PARAM_ID_AWDSP_RX_PARAMS			(0x10013D12)
#define AFE_PARAM_ID_AWDSP_TX_SET_ENABLE		(0x10013D13)
#define AFE_PARAM_ID_AWDSP_RX_VMAX_L			(0X10013D17)
#define AFE_PARAM_ID_AWDSP_RX_VMAX_R			(0X10013D18)
#define AFE_PARAM_ID_AWDSP_RX_CALI_CFG_L		(0X10013D19)
#define AFE_PARAM_ID_AWDSP_RX_CALI_CFG_R		(0x10013d1A)
#define AFE_PARAM_ID_AWDSP_RX_RE_L			(0x10013d1B)
#define AFE_PARAM_ID_AWDSP_RX_RE_R			(0X10013D1C)
#define AFE_PARAM_ID_AWDSP_RX_NOISE_L			(0X10013D1D)
#define AFE_PARAM_ID_AWDSP_RX_NOISE_R			(0X10013D1E)
#define AFE_PARAM_ID_AWDSP_RX_F0_L			(0X10013D1F)
#define AFE_PARAM_ID_AWDSP_RX_F0_R			(0X10013D20)
#define AFE_PARAM_ID_AWDSP_RX_REAL_DATA_L		(0X10013D21)
#define AFE_PARAM_ID_AWDSP_RX_REAL_DATA_R		(0X10013D22)


enum afe_param_id_awdsp {
	INDEX_PARAMS_ID_RX_PARAMS = 0,
	INDEX_PARAMS_ID_RX_ENBALE,
	INDEX_PARAMS_ID_TX_ENABLE,
	INDEX_PARAMS_ID_RX_VMAX,
	INDEX_PARAMS_ID_RX_CALI_CFG,
	INDEX_PARAMS_ID_RX_RE,
	INDEX_PARAMS_ID_RX_NOISE,
	INDEX_PARAMS_ID_RX_F0,
	INDEX_PARAMS_ID_RX_REAL_DATA,
	INDEX_PARAMS_ID_MAX
};

enum {
	MTK_DSP_MSG_TYPE_DATA =0,
	MTK_DSP_MSG_TYPE_CMD = 1,
};

typedef struct mtk_dsp_msg_header{
	int32_t type;
	int32_t params_id;
	int32_t reserver[4];
}mtk_dsp_hdr_t;


/*********misc device ioctl fo cali**********/
#define AW882XX_CALI_CFG_NUM 3
#define AW882XX_CALI_DATA_NUM 6
#define AW882XX_PARAMS_NUM  (1752)

struct cali_cfg {
	int32_t data[AW882XX_CALI_CFG_NUM];
};
struct cali_data {
	int32_t data[AW882XX_CALI_DATA_NUM];
};
struct params_data {
	char data[AW882XX_PARAMS_NUM];
};

#define AW882XX_IOCTL_MAGIC					'a'
#define AW882XX_IOCTL_SET_CALI_CFG			_IOWR(AW882XX_IOCTL_MAGIC, 1, struct cali_cfg)
#define AW882XX_IOCTL_GET_CALI_CFG			_IOWR(AW882XX_IOCTL_MAGIC, 2, struct cali_cfg)
#define AW882XX_IOCTL_GET_CALI_DATA			_IOWR(AW882XX_IOCTL_MAGIC, 3, struct cali_data)
#define AW882XX_IOCTL_SET_NOISE				_IOWR(AW882XX_IOCTL_MAGIC, 4, int32_t)
#define AW882XX_IOCTL_GET_F0				_IOWR(AW882XX_IOCTL_MAGIC, 5, int32_t)
#define AW882XX_IOCTL_SET_CALI_RE			_IOWR(AW882XX_IOCTL_MAGIC, 6, int32_t)
#define AW882XX_IOCTL_GET_CALI_RE			_IOWR(AW882XX_IOCTL_MAGIC, 7, int32_t)
#define AW882XX_IOCTL_SET_VMAX				_IOWR(AW882XX_IOCTL_MAGIC, 8, int32_t)
#define AW882XX_IOCTL_GET_VMAX				_IOWR(AW882XX_IOCTL_MAGIC, 9, int32_t)
#define AW882XX_IOCTL_SET_PARAMS			_IOWR(AW882XX_IOCTL_MAGIC, 10, struct params_data)
#define AW882XX_IOCTL_ENABLE_CALI			_IOWR(AW882XX_IOCTL_MAGIC, 11,int8_t)
#define AW882XX_IOCTL_SET_RX_BYPASS         _IOWR(AW882XX_IOCTL_MAGIC, 12,uint32_t)


int aw_write_data_to_dsp(int index, void *data, int len, int channel);
int aw_read_data_to_dsp(int index, void *data, int len, int channel);




#define I2C_TRANSFER_MAX_SIZE    (1024)
#define UNUSED_PARAM(x) ((void)(x))

void aw882xx_misc_init(struct miscdevice *misc_device);
void aw882xx_misc_deinit(struct miscdevice *misc_device);
#endif
