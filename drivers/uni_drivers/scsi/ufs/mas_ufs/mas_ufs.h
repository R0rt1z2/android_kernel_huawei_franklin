#ifndef __MAS_UFS_H__
#define __MAS_UFS_H__

#define QUUEUE_CMD_TIMEOUT (2 * MSEC_PER_SEC)

void mas_ufshcd_prepare_req_desc_hdr(
	struct ufshcd_lrb *lrbp, u32 *upiu_flags);

void ufshcd_dump_status(
	struct Scsi_Host *host, enum blk_dump_scene dump_type);
int ufshcd_send_scsi_sync_cache_init(void);
void ufshcd_send_scsi_sync_cache_deinit(void);
int ufshcd_direct_flush(struct scsi_device *sdev);
void ufshcd_send_scsi_sync_cache_deinit(void);
void mas_ufshcd_slave_config(
	struct request_queue *q, struct scsi_device *sdev);
int ufshcd_custom_upiu_order(struct utp_upiu_req *ucd_req_ptr,
				     struct request *req,
				     struct scsi_cmnd *scmd,
				     struct ufs_hba *hba);
void ufshcd_compose_scsi_cmd(struct scsi_cmnd *cmd,
				    struct scsi_device *device,
				    unsigned char *cdb,
				    unsigned char *sense_buffer,
				    enum dma_data_direction sc_data_direction,
				    struct scatterlist *sglist,
				    unsigned int nseg,
				    unsigned int sg_len);
int ufshcd_send_scsi_request_sense(struct ufs_hba *hba,
		struct scsi_device *sdp, unsigned int timeout, bool eh_handle);
int ufshcd_send_scsi_ssu(struct ufs_hba *hba,
				struct scsi_device *sdp,
				unsigned char *cmd,
				unsigned int timeout,
				struct scsi_sense_hdr *sshdr);
int ufshcd_send_scsi_sync_cache(struct ufs_hba *hba,
				       struct scsi_device *sdp);
void ufshcd_mas_mq_init(struct Scsi_Host *host);
void ufshcd_read_vendor_feature(struct ufs_hba *hba, u8 *desc_buf);
int ufshcd_get_health_info(struct scsi_device *sdev,
	u8 *pre_eol_info, u8 *life_time_est_a, u8 *life_time_est_b);
#endif /* __MAS_UFS_H__ */
