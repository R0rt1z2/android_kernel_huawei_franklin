/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#ifndef __LOG_STORE_H__
#define __LOG_STORE_H__

#include <linux/types.h>

#define SRAM_HEADER_SIG (0xabcd1234)
#define DRAM_HEADER_SIG (0x5678ef90)
#define LOG_STORE_SIG (0xcdab3412)
#define LOG_EMMC_SIG (0x785690ef)
#define FLAG_DISABLE 0X44495341 // acsii-DISA
#define FLAG_ENABLE 0X454E454E // acsii-ENEN
#define KEDUMP_ENABLE (1)
#define KEDUMP_DISABLE (0)

#define MAX_DRAM_COUNT	2

#define LOG_STORE_SIZE 0x40000	/*  DRAM buff 256KB*/

/*  log flag */
#define BUFF_VALID      0x01
#define CAN_FREE		0x02
#define	NEED_SAVE_TO_EMMC	0x04
#define RING_BUFF		0x08
/* ring buf, if buf_full, buf_point is the start of the buf, */
/* else buf_point is the buf end, other buf is not used */
#define BUFF_FULL		0x10	/* buf is full */
/* array buf type, buf_point is the used buf end */
#define ARRAY_BUFF		0X20
#define BUFF_ALLOC_ERROR	0X40
#define BUFF_ERROR		0x80
#define BUFF_NOT_READY		0x100
#define BUFF_READY		0x200
/* pl or lk can printk the early printk information to uart cable */
#define BUFF_EARLY_PRINTK	0x400
#define	LOG_PL_LK  0x0	/* Preloader and lk log buff */

#ifdef CONFIG_BOOT_DETECTOR
#define ROFA_MEM_SIZE   32
#endif

#ifndef CRBROI_MEM_LEN
#define CRBROI_MEM_LEN                  4
#endif
enum bootdevice_type {
	BOOT_DEVICE_EMMC = 0,
	BOOT_DEVICE_UFS = 1,
	BOOT_DEVICE_MAX
};

/* total 32 bytes <= u32(4 bytes) * 8 = 32 bytes */
struct pl_lk_log {
	u32 sig;            // default 0xabcd1234
	u32 buff_size;      // total buf size
	u32 off_pl;         // pl offset, sizeof(struct pl_lk_log)
	u32 sz_pl;          // preloader size
	u32 pl_flag;        // pl log flag
	u32 off_lk;         // lk offset, sizeof((struct pl_lk_log) + sz_pl
	u32 sz_lk;          // lk log size
	u32 lk_flag;        // lk log flag
};

/* total 40 bytes <= u32(4 bytes) * 10 = 40 bytes */
struct dram_buf_header {
	u32 sig;
	u32 flag;
	u32 buf_addr;
	u32 buf_size;
	u32 buf_offsize;
	u32 buf_point;
	u32 klog_addr;
	u32 klog_size;
	u32 atf_log_addr;
	u32 atf_log_len;
};

struct boot_context {
	u32 boot_magic;        /* must be initialized in slb1 */
	u32 boot_stage;        /* must be initialized in every stage */
	u32 last_boot_stage;
	u32 boot_error_no;     /* set this via set_boot_error */
	u32 last_boot_error_no;
	u32 hash_code;
};

/* total 256 bytes */
struct sram_log_header {
	u32 sig;
	u32 reboot_count;
	u32 save_to_emmc;
	struct dram_buf_header dram_buf;        // 40 bytes
	struct pl_lk_log dram_curlog_header;    // 32 bytes
	u32 gz_log_addr;
	u32 gz_log_len;
	struct boot_context boot_cxt;           // 24 bytes
	u32 bopd_ro_info[CRBROI_MEM_LEN];
	u32 ro_inject;
	u32 reserve[30];                        // reserve 35 * 4 char size
};
#define SRAM_RECORD_LOG_SIZE 0X00
#define SRAM_BLOCK_SIZE 0x01
#define SRAM_PMIC_BOOT_PHASE 0x02


/* emmc last block struct */
struct log_emmc_header {
	u32 sig;
	u32 offset;
	//u32 uart_flag;
	u32 reserve_flag[11];
	/* [0] used to save uart flag */
	/* [1] used to save emmc_log index */
	/* [2] used to save printk ratalimit  flag */
	/* [3] used to save kedump contrl flag */
	/* [4] used to save boot step */
};

enum EMMC_STORE_FLAG_TYPE {
	UART_LOG = 0x00,
	LOG_INDEX = 0X01,
	PRINTK_RATELIMIT = 0X02,
	KEDUMP_CTL = 0x03,
	BOOT_STEP = 0x04,
	EMMC_STORE_FLAG_TYPE_NR,
};

#define BOOT_PHASE_MASK	0xf		// b1111
#define NOW_BOOT_PHASE_SHIFT 0x0
#define LAST_BOOT_PHASE_SHIFT 0x4
#define PMIC_BOOT_PHASE_SHIFT 0x8
#define PMIC_LAST_BOOT_PHASE_SHIFT 0Xc

#define HEADER_INDEX_MAX 0x10

/* emmc store log */
struct emmc_log {
	u32 type;
	u32 start;
	u32 end;
};

#define LOG_PLLK 0x01
#define LOG_PL 0x02
#define LOG_KERNEL 0x03
#define LOG_ATF 0x04
#define LOG_GZ 0x05
#define LOG_LAST_KERNEL 0x06
#define BOOT_PHASE_PL 0x01
#define BOOT_PHASE_LK 0x02
#define BOOT_PHASE_KERNEL 0x03
#define BOOT_PHASE_ANDROID 0x04
#define BOOT_PHASE_PL_COLD_REBOOT 0X05
#define BOOT_PHASE_SUSPEND 0x06
#define BOOT_PHASE_RESUME 0X07
#ifdef CONFIG_BOOT_DETECTOR
#define BOOT_PHASE_NATIVE 0x08
#define BOOT_PHASE_FRAMEWORK 0x09
#endif

#ifdef CONFIG_MTK_DRAM_LOG_STORE
void log_store_bootup(void);
void store_log_to_emmc_enable(bool value);
void disable_early_log(void);
void *get_sram_header(void);
void log_store_to_emmc(void);
int set_emmc_config(int type, int value);
int read_emmc_config(struct log_emmc_header *log_header);
u32 get_last_boot_phase(void);
void set_boot_phase(u32 step);
#ifdef CONFIG_BOOT_DETECTOR
u32 get_boot_phase(void);
char *get_phys_mem_base(void);
char *get_rofa_mem_base(void);
char *get_klog_addr(void);
u32 get_klog_size(void);
u32 get_total_klog_size(void);
char *get_pl_lk_log_addr(void);
u32 get_pl_lk_log_size(void);
u32 get_total_pl_lk_log_size(void);
#endif
#else

static inline void  log_store_bootup(void)
{

}

static inline void store_log_to_emmc_enable(bool value)
{

}

static inline void disable_early_log(void)
{
}

static inline void log_store_to_emmc(void)
{
}

static inline int set_emmc_config(int type, int value)
{
	return 0;
}

static inline int read_emmc_config(struct log_emmc_header *log_header)
{
	return 0;
}
static inline u32 get_last_boot_phase(void)
{
	return 0;
}
static inline void set_boot_phase(u32 step)
{
}

#ifdef CONFIG_BOOT_DETECTOR
static inline u32 get_boot_phase(void)
{
	return 0;
}

static char *get_phys_mem_base(void)
{
	return NULL;
}

char *get_rofa_mem_base(void)
{
	return NULL;
}

static inline char *get_klog_addr(void)
{
	return NULL;
}

static inline u32 get_total_klog_size(void)
{
	return 0;
}

static inline u32 get_klog_size(void)
{
	return 0;
}

static inline char *get_pl_lk_log_addr(void)
{
	return NULL;
}

static inline u32 get_pl_lk_log_size(void)
{
	return 0;
}

static inline u32 get_total_pl_lk_log_size(void)
{
	return 0;
}
#endif
#endif
#endif
