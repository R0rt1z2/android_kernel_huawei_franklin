

#include "hw_wifi.h"
#include "debug.h"
#include "gl_cfg80211.h"
#include "securec.h"
#include "wlan_lib.h"

/* private command for huawei */
#define CMD_SET_AX_BLACKLIST "SET_AX_BLACKLIST "
#define CMD_PM_STATE "CMD_PM_STATE"
#define CMD_GRO_STATE "CMD_GRO_STATE"
#define CMD_GET_TRX_TOTAL_PKTS_CNT "getTotalTxRxCount"
#define CMD_GET_TX_RATE_PKTS_CNT "getRateTxCount 0"
#define CMD_GET_RX_SPEC_CNT "getSpecificRxCount"
#define CMD_GET_RX_FILTER_CNT "getTotalFilterCount"
#define CMD_GET_CHIP_WORK_TIME "getAwakeTime"
#define CMD_GET_CHIP_SLEEP_TIME "getSleepTime"
#define CMD_GET_CHIP_DEEP_SLEEP_TIME "getDeepSleepTime"
#define CMD_GET_BAND_OCCUPIED_TIME "get_chip getBandOccupiedTime"
#define CMD_GET_BW_OCCUPIED_TIME "getBwTime 0"

#define RX_FILTER_STAT_NUM 4
#define RX_SPEC_STAT_NUM 5

typedef int(*PRIV_CMD_FUNCTION) (
    IN struct net_device *prNetDev,
    IN char *pcCommand,
    IN int i4TotalLen);

struct PRIV_CMD_HANDLER {
    uint8_t *pcCmdStr;
    PRIV_CMD_FUNCTION pfHandler;
};

static int hw_priv_set_11ax_blacklist(IN struct net_device *prNetDev,
        IN char *pcCommand, IN int i4TotalLen)
{
    uint32_t setInfoLen = 0;
    uint32_t paramsOffset;
    struct GLUE_INFO *prGlueInfo = NULL;
    struct ADAPTER *prAdapter = NULL;

    // struct PARAM_AX_BLACKLIST with a more field(ucBssIdx) than HAL
    paramsOffset = strlen(CMD_SET_AX_BLACKLIST) - 1;
    if (i4TotalLen <= paramsOffset + sizeof(struct PARAM_AX_BLACKLIST)) {
        DBGLOG(REQ, WARN, "%s: invalid params: total len: %d\n",
            __func__, i4TotalLen);
        return -1; // invalid parameters
    }

    ASSERT(prNetDev);
    if (GLUE_CHK_PR2(prNetDev, pcCommand) == FALSE)
        return -1;

    prGlueInfo = *((struct GLUE_INFO **) netdev_priv(prNetDev));
    prAdapter = prGlueInfo->prAdapter;
    if (prAdapter == NULL)
        return -1; // WLAN_STATUS_ADAPTER_NOT_READY

    *(pcCommand + paramsOffset) = 0; // default bss idx
    (void)wlanoidSetAxBlacklist(prAdapter, pcCommand + paramsOffset,
        i4TotalLen - paramsOffset, &setInfoLen);
    return 0;
}

static int hw_priv_driver_set_wifi_sleep_state(IN struct net_device *prNetDev, IN char *pcCommand,
        IN int i4TotalLen)
{
    int pos;
    int timeout;
    bool enabled;
    struct GLUE_INFO *prGlueInfo = NULL;
    struct ADAPTER *prAdapter = NULL;

    ASSERT(prNetDev);
    pos = (int)strlen(CMD_PM_STATE);

    if (GLUE_CHK_PR2(prNetDev, pcCommand) == FALSE)
        return -1;
    if (prNetDev->ieee80211_ptr == NULL || prNetDev->ieee80211_ptr->wiphy == NULL)
        return -1;

    prGlueInfo = *((struct GLUE_INFO **)netdev_priv(prNetDev));
    prAdapter = prGlueInfo->prAdapter;
    if (prAdapter == NULL)
        return -1; // wlan status adapter not ready

    if (atoi(*(pcCommand + pos + 1)) == 0) { // 0 means disable, otherwise enable
        enabled = false;
        timeout = 0; // timeout is 0 for wifi sleep disabled state
    } else {
        enabled = true;
        timeout = -1; // timeout is -1 for wifi sleep enabled state
    }

    mtk_cfg80211_set_power_mgmt(prNetDev->ieee80211_ptr->wiphy, prNetDev, enabled, timeout);
    return 1;
}

static int hw_priv_driver_set_gro_state(IN struct net_device *prNetDev, IN char *pcCommand,
    IN int i4TotalLen)
{
    int pos;
    struct GLUE_INFO *prGlueInfo = NULL;
    struct ADAPTER *prAdapter = NULL;

    ASSERT(prNetDev);

    pos = (int)strlen(CMD_GRO_STATE);

    if (GLUE_CHK_PR2(prNetDev, pcCommand) == FALSE)
        return -1;

    prGlueInfo = *((struct GLUE_INFO **)netdev_priv(prNetDev));
    prAdapter = prGlueInfo->prAdapter;
    if (prAdapter == NULL)
        return -1; // wlan status adapter not ready

    wlanSetGRO(prAdapter, prNetDev, atoi(*(pcCommand + pos + 1)));
    return 1;
}

static int hw_priv_get_stat(IN struct net_device *prNetDev, IN char *pcCommand,
    IN int i4TotalLen)
{
    struct GLUE_INFO *prGlueInfo = NULL;
    struct ADAPTER *prAdapter = NULL;

    ASSERT(prNetDev);

    if (GLUE_CHK_PR2(prNetDev, pcCommand) == FALSE)
        return -1;
    prGlueInfo = *((struct GLUE_INFO **) netdev_priv(prNetDev));

    DBGLOG(REQ, LOUD, "command is %s\n", pcCommand);

    prAdapter = prGlueInfo->prAdapter;
    if (prAdapter == NULL)
        return -1; /* WLAN_STATUS_ADAPTER_NOT_READY */
    wlanChipConfig(prAdapter, pcCommand, i4TotalLen);

    return (strlen(pcCommand) + 1);
}

/* device filter, mtk filter, arp filter, icmp filter */
static int hw_priv_get_rx_filter_pkts_cnt(IN struct net_device *prNetDev, IN char *pcCommand,
    IN int i4TotalLen)
{
    struct GLUE_INFO *prGlueInfo = NULL;
    struct ADAPTER *prAdapter = NULL;
    int ret;
    int wifiFilterCnt[WIFI_FILTER_CNT] = {0};
    int filterType;
    char *nextPos = NULL;

    ASSERT(prNetDev);

    if (GLUE_CHK_PR2(prNetDev, pcCommand) == FALSE)
        return -1;
    prGlueInfo = *((struct GLUE_INFO **) netdev_priv(prNetDev));

    DBGLOG(REQ, LOUD, "command is %s\n", pcCommand);

    prAdapter = prGlueInfo->prAdapter;
    if (prAdapter == NULL)
        return -1; /* WLAN_STATUS_ADAPTER_NOT_READY */
    wlanChipConfig(prAdapter, pcCommand, i4TotalLen);

    nextPos = (char *)pcCommand;
    for (filterType = 0; filterType <= FILTER_ICMP; filterType++) {
        wifiFilterCnt[filterType] = (int)simple_strtol(nextPos, &nextPos, BASE10);
    }
    nextPos = NULL;

    wifiFilterCnt[FILTER_ICMP] = prAdapter->rPerMonitor.rRxInfo.u4TotalRxIcmpCount;

    ret = snprintf_s(pcCommand, i4TotalLen, i4TotalLen - 1, "%d %d %d %d",
        wifiFilterCnt[FILTER_DEVICE], wifiFilterCnt[FILTER_ARP],
        wifiFilterCnt[FILTER_APF], wifiFilterCnt[FILTER_ICMP]);
    if (ret < 0) {
        printk(KERN_ERR"%s snprintf fail!\n", __func__);
        return -1;
    }
    return (strlen(pcCommand) + 1);
}

/* amsdu, large amsdu, mix amsdu, addba req, group rekey */
static int hw_priv_get_rx_spec_pkts_cnt(IN struct net_device *prNetDev, IN char *pcCommand,
    IN int i4TotalLen)
{
    struct GLUE_INFO *prGlueInfo = NULL;
    struct ADAPTER *prAdapter = NULL;
    int ret;
    int wifiSpecPktsCnt[WIFI_ABNORMAL_PKTS_CNT] = {0};
    int specPktsType;
    char *nextPos = NULL;

    ASSERT(prNetDev);

    if (GLUE_CHK_PR2(prNetDev, pcCommand) == FALSE)
        return -1;
    prGlueInfo = *((struct GLUE_INFO **) netdev_priv(prNetDev));

    DBGLOG(REQ, LOUD, "command is %s\n", pcCommand);

    prAdapter = prGlueInfo->prAdapter;
    if (prAdapter == NULL)
        return -1; /* WLAN_STATUS_ADAPTER_NOT_READY */
    wlanChipConfig(prAdapter, pcCommand, i4TotalLen);

    nextPos = (char *)pcCommand;
    for (specPktsType = 0; specPktsType <= GROUP_REKEY_CNT; specPktsType++) {
        wifiSpecPktsCnt[specPktsType] = (int)simple_strtol(nextPos, &nextPos, BASE10);
    }
    nextPos = NULL;

    wifiSpecPktsCnt[SMALL_AMSDU_CNT] = prAdapter->rPerMonitor.rRxInfo.u4TotalAmsduCount;
    wifiSpecPktsCnt[GROUP_REKEY_CNT] = prAdapter->rPerMonitor.rRxInfo.u4TotalRekeyCount;

    ret = snprintf_s(pcCommand, i4TotalLen, i4TotalLen - 1, "%d %d %d %d %d",
        wifiSpecPktsCnt[SMALL_AMSDU_CNT], wifiSpecPktsCnt[LARGE_AMSDU_CNT], wifiSpecPktsCnt[MIX_AMSDU_CNT],
        wifiSpecPktsCnt[ADDBA_REQ_CNT], wifiSpecPktsCnt[GROUP_REKEY_CNT]);
    if (ret < 0) {
        printk(KERN_ERR"%s snprintf fail!\n", __func__);
        return -1;
    }
    return (strlen(pcCommand) + 1);
}

/* private command for hauwei only */
static struct PRIV_CMD_HANDLER hw_priv_cmd_handlers[] = {
    {CMD_SET_AX_BLACKLIST, hw_priv_set_11ax_blacklist},
    {CMD_PM_STATE, hw_priv_driver_set_wifi_sleep_state},
    {CMD_GRO_STATE, hw_priv_driver_set_gro_state},
    {CMD_GET_RX_SPEC_CNT, hw_priv_get_rx_spec_pkts_cnt},
    {CMD_GET_RX_FILTER_CNT, hw_priv_get_rx_filter_pkts_cnt},
    {CMD_GET_TRX_TOTAL_PKTS_CNT, hw_priv_get_stat},
    {CMD_GET_TX_RATE_PKTS_CNT, hw_priv_get_stat},
    {CMD_GET_CHIP_WORK_TIME, hw_priv_get_stat},
    {CMD_GET_CHIP_SLEEP_TIME, hw_priv_get_stat},
    {CMD_GET_CHIP_DEEP_SLEEP_TIME, hw_priv_get_stat},
    {CMD_GET_BAND_OCCUPIED_TIME, hw_priv_get_stat},
    {CMD_GET_BW_OCCUPIED_TIME, hw_priv_get_stat},
};

int32_t hw_priv_cmds_handler(IN struct net_device *prNetDev,
        IN int8_t *pcCommand, IN int32_t i4TotalLen, OUT int32_t *i4BytesWritten)
{
    int i = 0;
    int size = sizeof(hw_priv_cmd_handlers) / sizeof(struct PRIV_CMD_HANDLER);

    for (i = 0; i < size; i++) {
        if (strnicmp(pcCommand, hw_priv_cmd_handlers[i].pcCmdStr,
                strlen(hw_priv_cmd_handlers[i].pcCmdStr)) != 0) {
            continue;
        }
        if (hw_priv_cmd_handlers[i].pfHandler != NULL) {
            *i4BytesWritten = hw_priv_cmd_handlers[i].pfHandler(prNetDev, pcCommand, i4TotalLen);
        }
        return 1; // handled
    }
    return 0;
}

const void *get_wificfg_filename_header(void)
{
    struct device_node *node = NULL;

    node = of_find_node_by_path("/huawei_wifi_info");
    if (node == NULL) {
        DBGLOG(INIT, INFO, "get_wificfg_filename_header:Node is not available.\n");
        return NULL;
    }
    DBGLOG(INIT, INFO, "get_wificfg_filename_header:Node is vail.\n");

    return of_get_property(node, "wifi_cfg_type", NULL);
}
