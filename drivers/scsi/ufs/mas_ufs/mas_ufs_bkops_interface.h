#ifndef __MAS_UFS_BKOPS_INTERFACE_H__
#define __MAS_UFS_BKOPS_INTERFACE_H__

int ufshcd_bkops_status_query(void *bkops_data, u32 *status);
int __ufshcd_bkops_status_query(void *bkops_data, u32 *status);
int ufshcd_bkops_start_stop(void *bkops_data, int start);
int __ufshcd_bkops_start_stop(void *bkops_data, int start);
#endif /*  __MAS_UFS_BKOPS_INTERFACE_H__ */
