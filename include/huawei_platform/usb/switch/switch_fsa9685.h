#ifndef _SWITCH_FSA9685_H_
#define _SWITCH_FSA9685_H_

#include <huawei_platform/usb/switch/usbswitch_common.h>
#include <mt-plat/charger_type.h>

#define FSA9685_RD_BUF_SIZE               (32)
#define FSA9685_WR_BUF_SIZE               (64)

/* usb state */
#define FSA9685_OPEN                      (0)
#define FSA9685_USB1_ID_TO_IDBYPASS       (1)
#define FSA9685_USB2_ID_TO_IDBYPASS       (2)
#define FSA9685_UART_ID_TO_IDBYPASS       (3)
#define FSA9685_USB1_ID_TO_VBAT           (5)

#define DELAY_FOR_RESET                   300 /* ms */
#define DELAY_FOR_BC12_REG                300 /* ms */
#define DELAY_FOR_SDP_RETRY               300 /* ms */

enum err_oprt_reg_num {
	ERR_FSA9685_REG_MANUAL_SW_1 = 1,
	ERR_FSA9685_READ_REG_CONTROL = 2,
	ERR_FSA9685_WRITE_REG_CONTROL = 3,
};

enum rt8979_usb_status {
	RT8979_USB_STATUS_NOVBUS,
	RT8979_USB_STATUS_UNDER_GOING,
	RT8979_USB_STATUS_SDP,
	RT8979_USB_STATUS_SDP_NSTD,
	RT8979_USB_STATUS_DCP,
	RT8979_USB_STATUS_CDP,
};

struct fsa9685_device_info {
	struct device *dev;
	struct i2c_client *client;
	int device_id;

	struct mutex accp_detect_lock;
	struct mutex accp_adaptor_reg_lock;
	struct wakeup_source usb_switch_lock;

	struct work_struct g_intb_work;
	struct delayed_work detach_delayed_work;

#ifdef CONFIG_BOOST_5V
	struct notifier_block usb_nb;
#endif

	u32 fcp_support;
	u32 scp_support;
	u32 two_switch_flag; /* disable for two switch */
	u32 pd_support;
	u32 dcd_timeout_force_enable;
	u32 power_by_5v;
	u32 ignore_force_ac;
	struct delayed_work chg_det_work;
	struct delayed_work update_type_work;

	/* rt8979 */
	struct rt8979_desc *desc;
	u32 intr_gpio;
	int irq;
	u8 irq_mask;
	bool attach;
	int hidden_mode_cnt;
	struct mutex io_lock;
	struct mutex hidden_mode_lock;
#ifdef CONFIG_MTK_EXTERNAL_CHARGER_TYPE_DETECT
	struct mutex bc12_lock;
	struct mutex bc12_en_lock;
	struct delayed_work psy_dwork;
	struct power_supply *psy;
	enum charger_type chg_type;
	enum rt8979_usb_status us;
	struct wakeup_source *bc12_en_ws;
	struct task_struct *bc12_en_kthread;
	int bc12_en_buf[2];
	int bc12_en_buf_idx;
	bool nstd_retrying;
	u8 sdp_retried;
	atomic_t chg_type_det;
	atomic_t bc12_en_req_cnt;
	wait_queue_head_t bc12_en_req;
#endif /* CONFIG_MTK_EXTERNAL_CHARGER_TYPE_DETECT */
};

struct fsa9685_device_ops {
	int (*dump_regs)(char *buf);
	int (*switchctrl_store)(struct i2c_client *client, int action);
	int (*switchctrl_show)(char *buf);
	int (*manual_switch)(int input_select);
	void (*detach_work)(void);
};

extern struct fsa9685_device_ops* usbswitch_fsa9685_get_device_ops(void);
extern struct fsa9685_device_ops* usbswitch_rt8979_get_device_ops(void);

struct fsa9685_device_info *switch_get_dev_info(void);
/* bc12 type detect */
extern int rt8979_init_early(struct fsa9685_device_info *chip);
extern int rt8979_init_later(struct fsa9685_device_info *chip);
extern void rt8979_re_init(struct fsa9685_device_info *chip);
extern int rt8979_reset(struct fsa9685_device_info *chip);
extern void rt8979_set_usbsw_state(int state);
extern int rt8979_enable_chg_type_det(struct fsa9685_device_info *chip, bool en);
extern void rt8979_dcp_accp_init(void);
extern void hw_usb_psy_force_change_type_to_ac(void);

#define ADAPTOR_BC12_TYPE_MAX_CHECK_TIME 100
#define WAIT_FOR_BC12_DELAY 5
#define ACCP_NOT_PREPARE_OK -1
#define ACCP_PREPARE_OK 0
#define BOOST_5V_CLOSE_FAIL -1
#define SET_DCDTOUT_FAIL -1
#define SET_DCDTOUT_SUCC 0

#endif /* end of _SWITCH_FSA9685_H_ */
