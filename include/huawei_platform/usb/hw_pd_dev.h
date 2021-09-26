

#ifndef __HW_PD_DEV_H__
#define __HW_PD_DEV_H__

#include <linux/device.h>
#include <huawei_platform/log/hw_log.h>
#include <linux/completion.h>
#include <linux/pm_wakeup.h>

#define CONFIG_DPM_USB_PD_CUSTOM_DBGACC
#define CONFIG_DPM_TYPEC_CAP_DBGACC_SNK
#define CONFIG_DPM_TYPEC_CAP_CUSTOM_SRC
#define PD_DPM_HW_DOCK_SVID 0x12d1

#define PD_DPM_CC_CHANGE_COUNTER_THRESHOLD     (50)
#define PD_DPM_CC_CHANGE_INTERVAL              (2000) /* ms */
#define PD_DPM_CC_CHANGE_MSEC                  (1000)
#define PD_DPM_CC_CHANGE_BUF_SIZE              (10)

#define PD_DPM_CC_DMD_COUNTER_THRESHOLD        1
#define PD_DPM_CC_DMD_INTERVAL                 (24 * 60 * 60) /* s */
#define PD_DPM_CC_DMD_BUF_SIZE                 (1024)

#define PD_DPM_INVALID_VAL                     (-1)

/* discover id ack:product vdo type offset */
#define PD_DPM_PDT_OFFSET                      (12)
#define PD_DPM_PDT_MASK                        (0x7)
#define PD_DPM_PDT_VID_OFFSET                  (16)

/* huawei product id */
#define PD_PID_COVER_ONE                       0x3000
#define PD_PID_COVER_TWO                       0x3001

/* huawei vid */
#define PD_DPM_HW_VID                          (0x12d1)

/* huawei charger device pid */
#define PD_DPM_HW_CHARGER_PID                  (0x3b30)
#define PD_DPM_HW_PDO_MASK                     (0xffff)

/* cc status for data collect */
#define PD_DPM_CC_OPEN                         (0x00)
#define PD_DPM_CC_DFT                          (0x01)
#define PD_DPM_CC_1_5                          (0x02)
#define PD_DPM_CC_3_0                          (0x03)
#define PD_DPM_CC2_STATUS_OFFSET               (0x02)
#define PD_DPM_CC_STATUS_MASK                  (0x03)
#define PD_DPM_BOTH_CC_STATUS_MASK             (0x0F)

#define CC_ORIENTATION_FACTORY_SET             1
#define CC_ORIENTATION_FACTORY_UNSET           0

#define PD_PDM_RE_ENABLE_DRP                   1

/* type-c inserted plug orientation */
enum pd_cc_orient{
	PD_DPM_CC_MODE_UFP = 0,
	PD_DPM_CC_MODE_DRP,
};
enum pd_cc_mode {
	PD_CC_ORIENT_DEFAULT = 0,
	PD_CC_ORIENT_CC1,
	PD_CC_ORIENT_CC2,
	PD_CC_NOT_READY,
};

enum pd_wait_typec_complete {
	NOT_COMPLETE,
	COMPLETE_FROM_VBUS_DISCONNECT,
	COMPLETE_FROM_TYPEC_CHANGE,
};

enum pd_device_port_power_mode{
	PD_DEV_PORT_POWERMODE_SOURCE = 0,
	PD_DEV_PORT_POWERMODE_SINK,
	PD_DEV_PORT_POWERMODE_NOT_READY,
};

enum pd_device_port_data_mode{
	PD_DEV_PORT_DATAMODE_HOST = 0,
	PD_DEV_PORT_DATAMODE_DEVICE,
	PD_DEV_PORT_DATAMODE_NOT_READY,
};

enum pd_device_port_mode{
	PD_DEV_PORT_MODE_DFP = 0,
	PD_DEV_PORT_MODE_UFP,
	PD_DEV_PORT_MODE_NOT_READY,
};

enum {
	PD_DPM_PE_EVT_DIS_VBUS_CTRL,
	PD_DPM_PE_EVT_SOURCE_VCONN,
	PD_DPM_PE_EVT_SOURCE_VBUS,
	PD_DPM_PE_EVT_SINK_VBUS,
	PD_DPM_PE_EVT_PR_SWAP,
	PD_DPM_PE_EVT_DR_SWAP,
	PD_DPM_PE_EVT_VCONN_SWAP,
	PD_DPM_PE_EVT_TYPEC_STATE,
	PD_DPM_PE_EVT_PD_STATE,
	PD_DPM_PE_EVT_BC12,
	PD_DPM_PE_ABNORMAL_CC_CHANGE_HANDLER,
	PD_DPM_PE_CABLE_VDO,
};

enum pd_typec_attach_type {
	PD_DPM_TYPEC_UNATTACHED = 0,
	PD_DPM_TYPEC_ATTACHED_SNK,
	PD_DPM_TYPEC_ATTACHED_SRC,
	PD_DPM_TYPEC_ATTACHED_AUDIO,
	PD_DPM_TYPEC_ATTACHED_DEBUG,

#ifdef CONFIG_DPM_TYPEC_CAP_DBGACC_SNK
	PD_DPM_TYPEC_ATTACHED_DBGACC_SNK,		/* Rp, Rp */
#endif	/* CONFIG_TYPEC_CAP_DBGACC_SNK */

#ifdef CONFIG_DPM_TYPEC_CAP_CUSTOM_SRC
	PD_DPM_TYPEC_ATTACHED_CUSTOM_SRC,		/* Same Rp */
#endif	/* CONFIG_TYPEC_CAP_CUSTOM_SRC */
    PD_DPM_TYPEC_ATTACHED_NORP_SRC,
};

enum {
	PD_DPM_USB_TYPEC_NONE = 0,
	PD_DPM_USB_TYPEC_DETACHED,
	PD_DPM_USB_TYPEC_DEVICE_ATTACHED,
	PD_DPM_USB_TYPEC_HOST_ATTACHED,
	PD_DPM_USB_TYPEC_AUDIO_ATTACHED,
	PD_DPM_USB_TYPEC_AUDIO_DETACHED,
};

enum pd_dpm_uevent_type {
	PD_DPM_UEVENT_START = 0,
	PD_DPM_UEVENT_COMPLETE,
};

enum pd_dpm_wake_lock_type {
	PD_WAKE_LOCK = 100,
	PD_WAKE_UNLOCK,
};

struct pd_dpm_typec_state {
	uint8_t rp_level;
	uint8_t polarity;
	uint8_t old_state;
	uint8_t new_state;

};

struct pd_dpm_ops {
	void (*pd_dpm_detect_emark_cable)(void *client);
	bool (*pd_dpm_get_hw_dock_svid_exist)(void*);
	int (*pd_dpm_notify_direct_charge_status)(void*, bool mode);
	void (*pd_dpm_set_cc_mode)(int mode);
	int (*pd_dpm_get_cc_state)(void);
	int (*pd_dpm_disable_pd)(void *client, bool disable);
	bool (*pd_dpm_check_cc_vbus_short)(void);
	void (*pd_dpm_enable_drp)(int mode);
	void (*pd_dpm_reinit_chip)(void *client);
	int (*data_role_swap)(void *client);
};
struct pd_dpm_pd_state {
	uint8_t connected;
};

struct pd_dpm_swap_state {
	uint8_t new_role;
};

enum pd_dpm_vbus_type {
	PD_DPM_VBUS_TYPE_TYPEC = 20,
	PD_DPM_VBUS_TYPE_PD,
};

enum pd_dpm_cc_voltage_type {
	PD_DPM_CC_VOLT_OPEN = 0,
	PD_DPM_CC_VOLT_RA = 1,
	PD_DPM_CC_VOLT_RD = 2,

	PD_DPM_CC_VOLT_SNK_DFT = 5,
	PD_DPM_CC_VOLT_SNK_1_5 = 6,
	PD_DPM_CC_VOLT_SNK_3_0 = 7,

	PD_DPM_CC_DRP_TOGGLING = 15,
};

struct pd_dpm_vbus_state {
	int mv;
	int ma;
	uint8_t vbus_type;
	bool ext_power;
	int remote_rp_level;
};

enum abnomal_change_type {
	PD_DPM_ABNORMAL_CC_CHANGE = 0,
	PD_DPM_UNATTACHED_VBUS_ONLY,
};

struct abnomal_change_info {
	enum abnomal_change_type event_id;
	bool first_enter;
	int change_counter;
	int dmd_counter;
	struct timespec64 ts64_last;
	struct timespec64 ts64_dmd_last;
	int change_data[PD_DPM_CC_CHANGE_BUF_SIZE];
	int dmd_data[PD_DPM_CC_CHANGE_BUF_SIZE];
};

enum cur_cap {
	PD_DPM_CURR_1P5A = 0x00,
	PD_DPM_CURR_3A = 0x01,
	PD_DPM_CURR_5A = 0x02,
	PD_DPM_CURR_6A = 0x03,
};

#define CABLE_CUR_CAP_SHIFT  5
#define CABLE_CUR_CAP_MASK   (BIT(5) | BIT(6))

enum pd_product_type {
	PD_PDT_PD_ADAPTOR = 0,
	PD_PDT_PD_POWER_BANK,
	PD_PDT_WIRELESS_CHARGER,
	PD_PDT_WIRELESS_COVER,
	PD_PDT_WIRELESS_COVER_TWO,
	PD_PDT_TOTAL,
};

struct pd_dpm_info{
	struct i2c_client *client;
	struct device *dev;
	struct mutex pd_lock;
	struct mutex sink_vbus_lock;
	struct dual_role_phy_desc *desc;

	struct notifier_block usb_nb;
	struct notifier_block chrg_wake_unlock_nb;

	enum pd_dpm_uevent_type uevent_type;
	struct work_struct pd_work;

	const char *tcpc_name;

	/* usb state update */
	struct mutex usb_lock;
	int pending_usb_event;
	int last_usb_event;
	struct workqueue_struct *usb_wq;
	struct delayed_work usb_state_update_work;
	struct delayed_work cc_moisture_flag_restore_work;

	bool pd_finish_flag;
	bool pd_source_vbus;
	int cur_usb_event;
	unsigned int cable_vdo;
	bool ctc_cable_flag;
	uint8_t cur_typec_state;
};

struct cc_check_ops {
	int (*is_cable_for_direct_charge)(void);
};
extern int cc_check_ops_register(struct cc_check_ops*);

/* for chip layer to get class created by core layer */
extern struct tcpc_device *tcpc_dev_get_by_name(const char *name);
extern int pd_dpm_handle_pe_event(unsigned long event, void *data);
extern bool pd_dpm_get_pd_finish_flag(void);
extern bool pd_dpm_get_quick_charger_flag(void);
extern bool pd_dpm_get_pd_source_vbus(void);
extern void pd_dpm_get_typec_state(int *typec_detach);
extern void pd_dpm_get_charge_event(unsigned long *event, struct pd_dpm_vbus_state *state);
extern bool pd_dpm_get_cc_orientation(void);
extern int pd_dpm_get_cc_state_type(unsigned int *cc1, unsigned int *cc2);
void pd_dpm_set_cc_voltage(int type);
enum pd_dpm_cc_voltage_type pd_dpm_get_cc_voltage(void);
int pd_dpm_ops_register(struct pd_dpm_ops *ops, void *client);
int pd_dpm_disable_pd(bool disable);
void pd_dpm_set_pd_finish_flag(bool flag);
bool pd_dpm_get_hw_dock_svid_exist(void);
extern void pd_pdm_enable_drp(void);

extern int pd_dpm_get_cur_usb_event(void);
extern void pd_dpm_audioacc_sink_vbus(unsigned long event, void *data);

extern int pmic_vbus_irq_is_enabled(void);
extern enum charger_event_type pd_dpm_get_source_sink_state(void);
extern int pd_dpm_notify_direct_charge_status(bool dc);

extern bool pd_dpm_check_cc_vbus_short(void);
extern bool pd_dpm_get_cc_moisture_status(void);
extern enum cur_cap pd_dpm_get_cvdo_cur_cap(void);
extern int pd_dpm_get_emark_detect_enable(void);
extern void pd_dpm_detect_emark_cable(void);
extern void pd_dpm_detect_emark_cable_finish(void);
bool pd_dpm_get_ctc_cable_flag(void);
extern void pd_dpm_cc_dynamic_protect(void);
void pd_dpm_set_source_sink_state(enum charger_event_type type);
bool pmic_vbus_is_connected(void);
void pmic_vbus_disconnect_process(void);
void pd_dpm_set_optional_max_power_status(bool status);
bool pd_dpm_get_optional_max_power_status(void);
void pd_dpm_set_high_voltage_charging_status(bool status);
bool pd_dpm_get_high_voltage_charging_status(void);
void mtk_set_cc1_cc2_status(bool cc1_status, bool cc2_status);
void mtk_get_cc1_cc2_status(int *cc1_status, int *cc2_status);

/* for audio analog hs drivers to check usb state */
int pd_dpm_get_analog_hs_state(void);

/* for otg channel */
bool pd_dpm_use_extra_otg_channel(void);

#endif
