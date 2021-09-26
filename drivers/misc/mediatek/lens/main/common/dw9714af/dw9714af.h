#ifndef DW9714AF_H
#define DW9714AF_H

#define DW9714V_STATUS_REG    0x05
#define DW9714V_VCM_CFG_REG    0x06
#define DW9714V_VCM_IC_INFO_REG    0x00
#define DW9714V_VCM_IC_INFO_DEFAULT    0xFE
#define DW9714V_BUSY_STATE_MASK    0x01
#define AF_REINIT_ESD_CHECK    10
#define DW9714V_STATUS_DELAY    2000

struct af_i2c_reg {
	unsigned short addr;
	unsigned short data;
	unsigned short delay;
};

struct af_move_reg {
	unsigned short data;
	unsigned short delay;
};

#endif