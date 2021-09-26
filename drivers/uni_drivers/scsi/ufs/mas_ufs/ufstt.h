#ifndef __UFSTT_H
#define __UFSTT_H

#include "ufshcd.h"

/* BYTE SHIFT */
#define ZERO_BYTE_SHIFT			0
#define ONE_BYTE_SHIFT			8
#define TWO_BYTE_SHIFT			16
#define THREE_BYTE_SHIFT		24
#define FOUR_BYTE_SHIFT			32
#define FIVE_BYTE_SHIFT			40
#define SIX_BYTE_SHIFT			48
#define SEVEN_BYTE_SHIFT		56

#define SHIFT_BYTE_0(num)		((num) << ZERO_BYTE_SHIFT)
#define SHIFT_BYTE_1(num)		((num) << ONE_BYTE_SHIFT)
#define SHIFT_BYTE_2(num)		((num) << TWO_BYTE_SHIFT)
#define SHIFT_BYTE_3(num)		((num) << THREE_BYTE_SHIFT)
#define SHIFT_BYTE_4(num)		((num) << FOUR_BYTE_SHIFT)
#define SHIFT_BYTE_5(num)		((num) << FIVE_BYTE_SHIFT)
#define SHIFT_BYTE_6(num)		((num) << SIX_BYTE_SHIFT)
#define SHIFT_BYTE_7(num)		((num) << SEVEN_BYTE_SHIFT)

#define GET_BYTE_0(num)			(((num) >> ZERO_BYTE_SHIFT) & 0xff)
#define GET_BYTE_1(num)			(((num) >> ONE_BYTE_SHIFT) & 0xff)
#define GET_BYTE_2(num)			(((num) >> TWO_BYTE_SHIFT) & 0xff)
#define GET_BYTE_3(num)			(((num) >> THREE_BYTE_SHIFT) & 0xff)
#define GET_BYTE_4(num)			(((num) >> FOUR_BYTE_SHIFT) & 0xff)
#define GET_BYTE_5(num)			(((num) >> FIVE_BYTE_SHIFT) & 0xff)
#define GET_BYTE_6(num)			(((num) >> SIX_BYTE_SHIFT) & 0xff)
#define GET_BYTE_7(num)			(((num) >> SEVEN_BYTE_SHIFT) & 0xff)

#define UFSTT_READ_BUFFER		0xF9
#define UFSTT_READ_BUFFER_ID		0x01
#define UFSTT_READ_BUFFER_CONTROL	0x00

/* Device descriptor parameters offsets in bytes */
enum ttunit_desc_param {
	TTUNIT_DESC_PARAM_LEN			= 0x0,
	TTUNIT_DESC_PARAM_DESCRIPTOR_IDN	= 0x1,
	TTUNIT_DESC_PARAM_TURBO_TABLE_EN	= 0x2,
	TTUNIT_DESC_PARAM_L2P_SIZE		= 0x3,
};

#define SAM_STAT_GOOD_NEED_UPDATE 0x80

/* turbo table parameters offsets in bytes */
enum ufstt_desc_id {
	TURBO_TABLE_READ_BITMAP		= 0x4,
};

#define ufstt_hex_dump(prefix_str, buf, len)                                   \
	print_hex_dump(KERN_ERR, prefix_str, DUMP_PREFIX_OFFSET, 16, 4, buf,   \
		       len, false)

void ufstt_set_sdev(struct scsi_device *sdev);
void ufstt_probe(struct ufs_hba *hba);
void ufstt_remove(struct ufs_hba *hba);
void ufstt_prep_fn(struct ufs_hba *hba, struct ufshcd_lrb *lrbp);
void ufstt_unprep_fn(struct ufs_hba *hba, struct ufshcd_lrb *lrbp);
void ufstt_idle_handler(struct ufs_hba *hba, ktime_t now_time);
void ufstt_unprep_handler(struct ufs_hba *hba, struct ufshcd_lrb *lrbp,
			  ktime_t now_time);
void ufstt_node_update(void);
bool is_ufstt_batch_mode(void);
#endif
