

#ifndef _HW_WIFI_H
#define _HW_WIFI_H

#include "gl_os.h"
#include "gl_typedef.h"
#include "precomp.h"

/* Type of Feed */
typedef enum {
    BASE10 = 10,  /* Decimal */
    BASE16 = 16,  /* Hexadecimal */
} base_type;

/* chip rx pkts stat report */
#define WIFI_FILTER_CNT 4
#define WIFI_ABNORMAL_PKTS_CNT 5

enum {
    FILTER_DEVICE = 0,
    FILTER_ARP,
    FILTER_APF,
    FILTER_ICMP
};

enum {
    SMALL_AMSDU_CNT = 0,
    LARGE_AMSDU_CNT,
    MIX_AMSDU_CNT,
    ADDBA_REQ_CNT,
    GROUP_REKEY_CNT
};

/* adapt mtk wifi.cfg for multi-product*/
#define HW_WIFICFG_MULTI_PRODUCT  1
#define WIFI_CFG_NAME_DEFAULT "wifi.cfg"
#define WIFI_CFG_NAME_MAX_LEN 128

extern int32_t hw_priv_cmds_handler(IN struct net_device *prNetDev,
        IN int8_t *pcCommand, IN int32_t i4TotalLen, OUT int32_t *i4BytesWritten);

extern const void *get_wificfg_filename_header(void);
#endif /* _HW_WIFI_H */
