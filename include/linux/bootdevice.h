#ifndef BOOTDEVICE_H
#define BOOTDEVICE_H
#include <linux/device.h>

enum bootdevice_type {
	BOOT_DEVICE_EMMC = 0,
	BOOT_DEVICE_UFS = 1,
	BOOT_DEVICE_MAX
};
#define UFS_VENDOR_HYNIX       0x1AD

#define MAX_PARTITION_NAME_LENGTH       36
struct flash_find_index_user {
	char name[MAX_PARTITION_NAME_LENGTH];
	int index;
};
#define INDEXEACCESSDATA _IOWR('M', 26, struct flash_find_index_user)

void set_bootdevice_type(enum bootdevice_type type);
enum bootdevice_type get_bootdevice_type(void);
unsigned int get_bootdevice_manfid(void);
void set_bootdevice_name(struct device *dev);
void set_bootdevice_size(sector_t size);
void set_bootdevice_cid(u32 *cid);
void set_bootdevice_product_name(char *product_name);
void set_bootdevice_pre_eol_info(u8 pre_eol_info);
void set_bootdevice_life_time_est_typ_a(u8 life_time_est_typ_a);
void set_bootdevice_life_time_est_typ_b(u8 life_time_est_typ_b);
void set_bootdevice_manfid(unsigned int manfid);
void set_bootdevice_rev_handler(int (*get_rev_func)(const struct device *, char *));

int __init proc_bootdevice_init(void);

#endif
