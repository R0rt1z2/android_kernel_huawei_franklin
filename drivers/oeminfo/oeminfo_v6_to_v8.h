/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2018-2020. All rights reserved.
 * Description: this file is used to adapt oeminfo_v6 id to oeminfo_v8 for being compatible with OEMINFO_VERSION6
 * Author: tanwei
 * Create: 2018-2-27
 */

#ifndef INCLUDE_OEMINFO_V6_TO_V8_H
#define INCLUDE_OEMINFO_V6_TO_V8_H

#define OEMINFO_UNUSEED_TYPE_USER_V8_1        0
#define OEMINFO_UNUSEED_TYPE_USER_V8_3        0
#define OEMINFO_UNUSEED_TYPE_USER_V8_5        0
#define OEMINFO_UNUSEED_TYPE_USER_V8_7        0
#define OEMINFO_UNUSEED_TYPE_USER_V8_19       0
#define OEMINFO_UNUSEED_TYPE_USER_V8_21       0
#define OEMINFO_UNUSEED_TYPE_USER_V8_23       0
#define OEMINFO_UNUSEED_TYPE_USER_V8_201      0
#define OEMINFO_UNUSEED_TYPE_USER_V8_210      0
#define OEMINFO_UNUSEED_TYPE_USER_V8_213      0
#define OEMINFO_UNUSEED_TYPE_USER_V8_216      0
#define OEMINFO_UNUSEED_TYPE_USER_V8_219      0
#define OEMINFO_UNUSEED_TYPE_USER_V8_222      0
#define OEMINFO_UNUSEED_TYPE_USER_V8_225      0
#define OEMINFO_UNUSEED_TYPE_USER_V8_228      0
#define OEMINFO_UNUSEED_TYPE_USER_V8_231      0
#define OEMINFO_UNUSEED_TYPE_USER_V8_234      0
#define OEMINFO_UNUSEED_TYPE_USER_V8_237      0
#define OEMINFO_UNUSEED_TYPE_USER_V8_240      0
#define OEMINFO_UNUSEED_TYPE_USER_V8_243      0
#define OEMINFO_UNUSEED_TYPE_USER_V8_246      0
#define OEMINFO_UNUSEED_TYPE_USER_V8_249      0
#define OEMINFO_UNUSEED_TYPE_USER_V8_252      0
#define OEMINFO_UNUSEED_TYPE_USER_V8_255      0
#define OEMINFO_UNUSEED_TYPE_USER_V8_258      0
#define OEMINFO_UNUSEED_TYPE_USER_V8_261      0
#define OEMINFO_UNUSEED_TYPE_USER_V8_264      0
#define OEMINFO_UNUSEED_TYPE_USER_V8_267      0
#define OEMINFO_UNUSEED_TYPE_USER_V8_270      0
#define OEMINFO_UNUSEED_TYPE_USER_V8_273      0
#define OEMINFO_UNUSEED_TYPE_USER_V8_276      0
#define OEMINFO_UNUSEED_TYPE_USER_V8_279      0
#define OEMINFO_UNUSEED_TYPE_USER_V8_282      0
#define OEMINFO_UNUSEED_TYPE_USER_V8_285      0
#define OEMINFO_UNUSEED_TYPE_USER_V8_288      0
#define OEMINFO_UNUSEED_TYPE_USER_V8_290    0
#define OEMINFO_UNUSEED_TYPE_USER_V8_292      0
#define OEMINFO_UNUSEED_TYPE_USER_V8_295      0
#define OEMINFO_UNUSEED_TYPE_USER_V8_298      0
#define OEMINFO_UNUSEED_TYPE_USER_V8_319      0
#define OEMINFO_UNUSEED_TYPE_USER_V8_322      0
#define OEMINFO_UNUSEED_TYPE_USER_V8_332      0
#define OEMINFO_UNUSEED_TYPE_USER_V8_335      0
#define OEMINFO_UNUSEED_TYPE_USER_V8_338      0
#define OEMINFO_UNUSEED_TYPE_USER_V8_341      0
#define OEMINFO_UNUSEED_TYPE_USER_V8_344      0

typedef enum {
    /* the next id is used for oeminfo_v8 none-wp area */
    OEMINFO_NONEWP_AMSS_VER_TYPE_V8 = 1,
    OEMINFO_NONEWP_OEMSBL_VER_TYPE_V8 = 2,
    OEMINFO_NONEWP_UPDATE_TYPE_V8 = 3,
    OEMINFO_NONEWP_WIFI_TEST_V8 = 4,
    OEMINFO_NONEWP_HWSB_LOCK_STATE_V8 = 5,
    OEMINFO_NONEWP_HWFB_LOCK_STATE_V8 = 6,
    OEMINFO_NONEWP_USER_USAGE_ACTIVDATE_TYPE_V8 = 7,
    OEMINFO_NONEWP_USB_PID_INDEX_V8 = 8,
    OEMINFO_NONEWP_USB_PID_CUSTOM_TYPE_V8 = 9,
    OEMINFO_NONEWP_TPCOLOR_TYPE_V8 = 10,
    OEMINFO_NONEWP_CRYPTFS_STATE_V8 = 11,
    OEMINFO_NONEWP_ROOT_INFO_TYPE_V8 = 12,
    OEMINFO_NONEWP_RESCUEINFO_TYPE_V8 =  13,
    OEMINFO_NONEWP_CAEMRA_DF_TYPE_V8 = 14,
    OEMINFO_NONEWP_CAMERA_HALL_OVERTURN_DATA_V8 = 15,
    OEMINFO_NONEWP_AT_LOCK_PASSWORD_SHA256_V8 = 16,
    OEMINFO_NONEWP_DAP_TIME_V8 = 17,
    OEMINFO_NONEWP_PSID_DATA_V8 = 18,
    OEMINFO_NONEWP_RESOURCE_VERSION_V8 = 19,
    OEMINFO_NONEWP_CRYPTFS_DATA_V8 = 20,
    OEMINFO_NONEWP_INITRC_TRACER_V8 = 21,
    OEMINFO_NONEWP_FBLOCK_STAT_INFO_V8 = 22,
    OEMINFO_NONEWP_VENDER_AND_COUNTRY_NAME_COTA_V8 = 23,
    OEMINFO_NONEWP_ANTITHIVES_STATE_V8 = 24,
    OEMINFO_NONEWP_WIFI_CAL_INDEX_V8 = 25,
    OEMINFO_NONEWP_SD_LOCK_PASSWD_V8 = 26,
    OEMINFO_NONEWP_MAIN_VERSION_V8 = 27,
    OEMINFO_NONEWP_BOOTANIM_SWITCH_V8 = 28,
    OEMINFO_NONEWP_BOOTANIM_SHUTFLAG_V8 = 29,
    OEMINFO_NONEWP_BOOTANIM_RINGMODE_V8 = 30,
    OEMINFO_NONEWP_BOOTANIM_RINGDEX_V8 = 31,
    OEMINFO_NONEWP_DATA_PART_FAULT_TIMES_V8 = 32,
    OEMINFO_NONEWP_APPS_NO_INSTALL_TYPE_V8 = 33,
    OEMINFO_NONEWP_SYSTEM_VERSION_V8 = 34,
    OEMINFO_NONEWP_GAMMA_DATA_V8 = 35,
    OEMINFO_NONEWP_GAMMA_LEN_V8 = 36,
    OEMINFO_NONEWP_MCC_MNC_TYPE_V8 = 37,
    OEMINFO_NONEWP_OPERATOR_INFO_V8 = 38,
    OEMINFO_NONEWP_HOTA_UPDATE_AUTH_LOCK_V8 = 39,
    OEMINFO_NONEWP_FACTORY_PRODUCT_VERSION_TYPE_V8 = 40,
    OEMINFO_NONEWP_AUDIO_SMARTPA_CALIBRATION_V8 = 41,
    OEMINFO_NONEWP_WIFIONLY_IDENTITY_V8 = 42,
    OEMINFO_NONEWP_LCD_BRIGHT_COLOR_CALIBRATION_V8 = 43,
    OEMINFO_NONEWP_FOTA_MODE_V8 = 44,
    OEMINFO_NONEWP_MARKETING_NAME_V8 = 45,
    OEMINFO_NONEWP_OVMODE_STATE_V8 = 46,
    OEMINFO_NONEWP_OMADM_INFO_V8 = 47,
    OEMINFO_NONEWP_ACTIVATION_STATUS_ID_V8 = 48,
    OEMINFO_NONEWP_HWPARENTCONTROL_RECOVER_DATA_V8 = 49,
    OEMINFO_NONEWP_FIX_HOTA_INFO_INTI_V8 = 50,
    OEMINFO_NONEWP_FIX_HOTA_INFO_V8 = 51,
    OEMINFO_NONEWP_FIX_HOTA_STRING_INTI_V8 = 52,
    OEMINFO_NONEWP_FIX_HOTA_STRING_V8 = 53,
    OEMINFO_NONEWP_VENDER_AND_COUNTRY_NAME_COTA_FACTORY_V8 = 54,
    OEMINFO_NONEWP_USER_HOTA_UPDATE_V8 = 55,
    OEMINFO_NONEWP_USE_DECRESS_BAT_VOL_FLAG_V8 = 56,
    OEMINFO_NONEWP_UPDATE_UDID_V8 = 57,
    OEMINFO_NONEWP_FACTORY_SW_VERSION_V8 = 58,
    OEMINFO_NONEWP_PRELOAD_APP_BATCH_V8 = 59,
    OEMINFO_NONEWP_CUST_Y_CABLE_TYPE_V8 = 60,
    OEMINFO_NONEWP_SDUPDATE_TYPE_V8 = 61,
    OEMINFO_NONEWP_ENABLE_RETREAD_V8 = 62,
    OEMINFO_NONEWP_SYSTEM_UPDATE_AUTH_TOKEN_INFO_V8 = 63,
    OEMINFO_NONEWP_ENCRYPT_INFO_V8 = 64,
    OEMINFO_NONEWP_MALWARE_DIAGNOSE_V8 = 65,
    OEMINFO_NONEWP_RETREAD_DIAGNOSE_V8 = 66,
    OEMINFO_NONEWP_ROOT_DIAGNOSE_V8 = 67,
    OEMINFO_NONEWP_SYSTEM_VERSION_B_V8 = 68,
    OEMINFO_NONEWP_ANDROID_VERSION_V8 = 69,
    OEMINFO_NONEWP_ANDROID_VERSION_B_V8 = 70,
    OEMINFO_NONEWP_SLOT_INFO_A_V8 = 71,
    OEMINFO_NONEWP_SLOT_INFO_B_V8 = 72,
    OEMINFO_NONEWP_SLOT_STATUS_V8 = 73,
    OEMINFO_NONEWP_VERITY_MODE_UPDATE_STATUS_V8 = 74,
    OEMINFO_NONEWP_SYSTEM_UPDATE_STATE_V8 = 75,
    OEMINFO_NONEWP_BASE_VER_TYPE_V8  = 76,
    OEMINFO_NONEWP_CUST_VER_TYPE_V8 =  77,
    OEMINFO_NONEWP_PRELOAD_VER_TYPE_V8 = 78,
    OEMINFO_NONEWP_CUSTOM_VERSION_V8 = 80,
    OEMINFO_NONEWP_CUSTOM_VERSION_B_V8 = 81,
    OEMINFO_NONEWP_PRELOAD_VERSION_V8 = 82,
    OEMINFO_NONEWP_PRELOAD_VERSION_B_V8 = 83,
    OEMINFO_NONEWP_GROUP_VERSION_V8 = 84,
    OEMINFO_NONEWP_GROUP_VERSION_B_V8 = 85,
    OEMINFO_NONEWP_BASE_VERSION_V8 = 86,
    OEMINFO_NONEWP_BASE_VERSION_B_V8 = 87,
    OEMINFO_NONEWP_BDRT_TYPE_V8 = 89,
    OEMINFO_NONEWP_COTA_REBOOT_UPDATE_PATH_V8 = 90,
    OEMINFO_NONEWP_CUSTOM_C_VERSION_V8 = 91,
    OEMINFO_NONEWP_RUNTEST_TYPE_V8 = 92,
    OEMINFO_NONEWP_MMITEST_TYPE_V8 = 93,
    OEMINFO_NONEWP_OTA_SIMLOCK_ACTIVATED_V8 = 94,
    OEMINFO_NONEWP_ATLOCK_TYPE_V8 = 95,
    OEMINFO_NONEWP_RUNTEST2_TYPE_V8 = 96,
    OEMINFO_NONEWP_RUNTIME_TYPE_V8 = 97,
    OEMINFO_NONEWP_MDATE_TYPE_V8 = 98,
    OEMINFO_NONEWP_MANUFACTURE_RESET_FLAG_V8 = 99,
    OEMINFO_NONEWP_BACKCOLOR_V8 = 100,
    OEMINFO_NONEWP_FB_STP_TYPE_V8 = 101,
    OEMINFO_NONEWP_RESET_ENTER_INFO_V8 = 102,
    OEMINFO_NONEWP_UPDATE_ENTER_INFO_V8 = 103,
    OEMINFO_NONEWP_RETAIL_APK_V8 = 104,
    OEMINFO_NONEWP_CUST_OVMODE_STATE_V8 = 105,
    OEMINFO_NONEWP_PRELOAD_OVMODE_STATE_V8 = 106,
    OEMINFO_NONEWP_ESIM_EID_V8 = 107,
    OEMINFO_NONEWP_CLOUD_ROM_INSTALL_STAT_V8 = 108,
    OEMINFO_NONEWP_DEV_NG_INFO_V8 = 109,
    OEMINFO_NONEWP_VSIM_WIFIPOLL_FLAG_V8 = 110,
    OEMINFO_NONEWP_VSIM_DOWNLOAD_STATUS_V8 = 111,
    OEMINFO_NONEWP_VSIM_WIFI_CONN_STATUS_V8 = 112,
    OEMINFO_NONEWP_CAMERA_SPECS_V8 = 113,
    OEMINFO_NONEWP_UNBIND_FLAG_V8 = 114,
    OEMINFO_NONEWP_DEBUG_EFUSE_FLAG_V8 = 115,
    OEMINFO_NONEWP_WIFI_COUNTRY_CODE_V8 = 116,
    OEMINFO_NONEWP_CCFLAG_V8 = 117,
    OEMINFO_NONEWP_FACT_ODM_V8 = 118,
    OEMINFO_NONEWP_RUNTESTB_TYPE_V8 = 119,
    OEMINFO_NONEWP_POPUPUSEDCOUNT_V8 = 120,
    OEMINFO_NONEWP_MOTORCODE_V8 = 121,
    OEMINFO_NONEWP_POS_HALL_CALIBRATION_DATA = 122,
    OEMINFO_NONEWP_CUSTOMEI_V8 = 123,
    OEMINFO_NONEWP_SCCAL_CUR_V8 = 124,
    OEMINFO_NONEWP_CUR_OFFSET_V8 = 125,
    OEMINFO_NONEWP_BAT_BLIMSW = 126,
    OEMINFO_NONEWP_BAT_BBINFO = 127,
    OEMINFO_NONEWP_BAT_HWCOUL = 128,
    OEMINFO_NONEWP_PPR = 129,
    OEMINFO_NONEWP_HOTA_BOOT_MODE = 131,
    OEMINFO_NONEWP_POST_CUSTOM_TYPE_V8 = 133,
    OEMINFO_NONEWP_UTAPKAUTORUN = 134,
    /* the id from 1000 to 1100 of 8k area are reserved */
    OEMINFO_NONEWP_REUSED_ID_1101_8K_VALID_SIZE_448_BYTE_V8 = 1101,
    OEMINFO_NONEWP_REUSED_ID_1102_8K_VALID_SIZE_64_BYTE_V8 = 1102,
    OEMINFO_NONEWP_FMD_VERIFY_DATA_V8 = 1103,
    OEMINFO_NONEWP_FCDEG_V8 = 1104,
    /* the id from 1400 to 1420 of 64k area are reserved */
    OEMINFO_NONEWP_WCNSS_FIR_BAK_TYPE_V8 = 1421,
    OEMINFO_NONEWP_CAMERA_TOF_CALIBRATION1_TYPE_V8 = 1422,
    OEMINFO_NONEWP_CAMERA_TOF_CALIBRATION2_TYPE_V8 = 1423,
    OEMINFO_NONEWP_FASTBOOT_FMD_DATA_V8 = 1424,
    OEMINFO_NONEWP_BASE_CLOUD_ROM_LIST_V8 = 1425,
    OEMINFO_NONEWP_CONTROL_MODE_DATA = 1426,
    OEMINFO_NONEWP_CAMERA_TOF_CALIBRATION3_TYPE_V8 = 1427,
    OEMINFO_NONEWP_CAMERA_TOF_CALIBRATION4_TYPE_V8 = 1428,
    OEMINFO_NONEWP_CAMERA_TOF_CALIBRATION5_TYPE_V8 = 1429,
    OEMINFO_NONEWP_CAMERA_TOF_CALIBRATION6_TYPE_V8 = 1430,
    OEMINFO_NONEWP_CAMERA_TOF_CALIBRATION7_TYPE_V8 = 1431,
    OEMINFO_NONEWP_CAMERA_BLC_CALIB_V8 = 1432,
    OEMINFO_NONEWP_SPECCALIB0_V8 = 1433,
    OEMINFO_NONEWP_SPECCALIB1_V8 = 1434,
    OEMINFO_NONEWP_CAMERA_EEPROM_DATA_00_V8 = 1435,
    OEMINFO_NONEWP_CAMERA_EEPROM_DATA_01_V8 = 1436,
    OEMINFO_NONEWP_CAMERA_EEPROM_DATA_02_V8 = 1437,
    OEMINFO_NONEWP_CAMERA_EEPROM_DATA_03_V8 = 1438,
    OEMINFO_NONEWP_CAMERA_EEPROM_DATA_04_V8 = 1439,
    OEMINFO_NONEWP_CAMERA_EEPROM_DATA_05_V8 = 1440,
    OEMINFO_NONEWP_CAMERA_EEPROM_DATA_06_V8 = 1441,
    OEMINFO_NONEWP_CAMERA_EEPROM_DATA_07_V8 = 1442,
    OEMINFO_NONEWP_CAMERA_EEPROM_DATA_08_V8 = 1443,
    OEMINFO_NONEWP_CAMERA_EEPROM_DATA_09_V8 = 1444,
    OEMINFO_NONEWP_CGM_SCG_DATA_V8 = 1445,
    OEMINFO_NONEWP_CGM_WCG_DATA_V8 = 1446,
    /* the next id is used for oeminfo_v8 root-wp area */
    OEMINFO_ROOTWP_BOARD_CODE_TYPE_V8 = 1501,
    OEMINFO_ROOTWP_VENDER_AND_COUNTRY_NAME_V8 = 1502,
    OEMINFO_ROOTWP_IMEI2_NUM_V8 = 1503,
    OEMINFO_ROOTWP_FACT_INFO_V8 = 1504,
    OEMINFO_ROOTWP_HWSB_AES_PWD_V8 = 1505,
    OEMINFO_ROOTWP_SKU_TYPE_V8 = 1506,
    OEMINFO_ROOTWP_MANUFACTURE_DATE_TYPE_V8 = 1507,
    OEMINFO_ROOTWP_LGU_SPECIAL_SN_V8 = 1508,
    OEMINFO_ROOTWP_IMEI_TYPE_V8 = 1509,
    OEMINFO_ROOTWP_MEID_TYPE_V8 = 1510,
    OEMINFO_ROOTWP_ESN_TYPE_V8 = 1511,
    OEMINFO_ROOTWP_BT_TYPE_V8 = 1512,
    OEMINFO_ROOTWP_WIFI_TYPE_V8 = 1513,
    OEMINFO_ROOTWP_FACTORY_INFO_TYPE_V8 = 1514,
    OEMINFO_ROOTWP_BUILD_NUMBER_TYPE_V8 = 1516,
    OEMINFO_ROOTWP_SINGLE_SIM_V8 = 1517,
    OEMINFO_ROOTWP_PRODUCT_MODEL_TYPE_V8 = 1518,
    OEMINFO_ROOTWP_DEVICE_MODEL_V8 = 1519,
    OEMINFO_ROOTWP_FRP_OPEN_FLAG_V8 = 1520,
    OEMINFO_ROOTWP_FRP_KEY_V8 = 1521,
    OEMINFO_ROOTWP_PRODUCE_PHASE_FLAG_V8 = 1522,
    OEMINFO_ROOTWP_MMIAUTORUN_V8 = 1523,
    OEMINFO_ROOTWP_MMIAUTOCFG_V8 = 1524,
    OEMINFO_ROOTWP_RD_LOCK_STATE_V8 = 1525,
    OEMINFO_ROOTWP_ACCESS_LOCK_STATE_V8 = 1526,
    OEMINFO_ROOTWP_ACCESS_KEY_V8 = 1527,
    OEMINFO_ROOTWP_CONTROL_FLAG_V8 = 1528,
    OEMINFO_ROOTWP_QRCODE_INFO_V8 = 1529,
    OEMINFO_ROOTWP_LOG_SWITCH_TYPE_V8 = 1530,
    OEMINFO_ROOTWP_BASEBAND_VERSION_TYPE_V8 = 1531,
    OEMINFO_ROOTWP_CONTROL_BIT_V8 = 1532,
    OEMINFO_ROOTWP_UDID_CODE_TYPE_V8 = 1533,
    OEMINFO_ROOTWP_NAL_CODE_TYPE = 1534,
    OEMINFO_ROOTWP_SYSDLL_INSTALL_TYPE_V8 = 1536,
    OEMINFO_ROOTWP_TOF_EYE_SAFE_TYPE_V8 = 1538,
    OEMINFO_ROOTWP_AUTH_ELABEL_V8 = 1539,
    OEMINFO_ROOTWP_TIME_ON_TYPE = 1540,
    OEMINFO_ROOTWP_DDC_TYPE_V8 = 1542,

    /* the id from 2500 to 2600 of 8k area are reserved */
    OEMINFO_ROOTWP_DEVICE_CERTIFICATE1_V8 = 2601,
    OEMINFO_ROOTWP_DEVICE_CERTIFICATE2_V8 = 2602,
    OEMINFO_ROOTWP_KEY_ATTESTATION_V8 = 2603,

    /* the id from 2900 to 2920 of 64k area are reserved */
    OEMINFO_ROOTWP_NFC_SEID_V8 = 2921,
    OEMINFO_ROOTWP_SCREEN_DATA_V8 = 2922,
    OEMINFO_ROOTWP_CUST_LOGO_EXTERN = 2923,
    OEMINFO_ROOTWP_POWER_CHARGING_TYPE_EXTERN = 2924,
    OEMINFO_ROOTWP_LOW_POWER_TYPE_EXTERN = 2925,
    OEMINFO_ROOTWP_COLOR_SENSOR_CALI_MATRIX = 2926,

    /* the next id is used for oeminfo_v8 powerup-wp area */
    OEMINFO_PWRUPWP_SENSOR_PROXIMITY_PA2240_CALIBRATION_V8 = 3001,
    OEMINFO_PWRUPWP_OIS_GYROGAIN_V8 = 3002,
    OEMINFO_PWRUPWP_ACCEL_SENSOR_DATA_V8 = 3003,
    OEMINFO_PWRUPWP_CAP_PROX_DATA_INFO_V8 = 3004,
    OEMINFO_PWRUPWP_SENSOR_LIGHT_BH1745_CALIBRATION_V8 = 3005,
    OEMINFO_PWRUPWP_GYRO_SENSOR_DATA_V8 = 3006,
    OEMINFO_PWRUPWP_QRCODE_BSN_V8 = 3007,
    OEMINFO_PWRUPWP_CAMERA_LASERCALIB_V8 = 3008,
    OEMINFO_PWRUPWP_MOD_HASH_V8 = 3009,
    OEMINFO_PWRUPWP_TOF_EYE_SAFE_TYPE_V8 = 3010,
    OEMINFO_PWRUPWP_LANMAC_V8 = 3011,
    OEMINFO_PWRUPWP_CASTHDCP_V8 = 3012,
    OEMINFO_PWRUPWP_MODSN_CHECK_V8 = 3013,
    OEMINFO_PWRUPWP_ALSENSOR_V8 = 3014,
    OEMINFO_PWRUPWP_ALS_UNDER_TP_CALIDATA_V8 = 3015,
    /* the id from 4000 to 4100 of 8k area are reserved */
    /* the id from 4400 to 4420 of 64k area are reserved */
    OEMINFO_PWRUPWP_CAMERA_HDC_V8 = 4421,
    OEMINFO_PWRUPWP_GMP_V8 = 4422,
    OEMINFO_PWRUPWP_GMP_EXT_V8 = 4423,
    /* the next id is used for  oeminfo_v8 logo area */
    OEMINFO_NONEWP_ASCEND_TYPE_V8 = 4501,
    OEMINFO_NONEWP_LOW_POWER_TYPE_V8 = 4502,
    OEMINFO_NONEWP_POWER_CHARGING_TYPE_V8 = 4503,
} OeminfoTypeV8;

struct OeminfoV6ToV8 {
    oeminfo_type oeminfoV6IdIndex;
    OeminfoTypeV8 oeminfoV8IdIndex;
};

static struct OeminfoV6ToV8 g_oeminfoV8Table[] = {
    { /* ----------- */
        0,
        0
    }, { /* v6_to_v8 oeminfoid=1 */
        0,
        OEMINFO_UNUSEED_TYPE_USER_V8_1
    }, { /* v6_to_v8 oeminfoid=2 */
        0,
        0
    }, { /* v6_to_v8 oeminfoid=3 */
        0,
        OEMINFO_UNUSEED_TYPE_USER_V8_3
    }, { /* v6_to_v8 oeminfoid=4 */
        0,
        0
    }, { /* v6_to_v8 oeminfoid=5 */
        0,
        OEMINFO_UNUSEED_TYPE_USER_V8_5
    }, { /* v6_to_v8 oeminfoid=6 */
        0,
        0
    }, { /* v6_to_v8 oeminfoid=7 */
        0,
        OEMINFO_UNUSEED_TYPE_USER_V8_7
    }, { /* v6_to_v8 oeminfoid=8 */
        OEMINFO_AMSS_VER_TYPE,
        OEMINFO_NONEWP_AMSS_VER_TYPE_V8
    }, { /* v6_to_v8 oeminfoid=9 */
        OEMINFO_OEMSBL_VER_TYPE,
        OEMINFO_NONEWP_OEMSBL_VER_TYPE_V8
    }, { /* v6_to_v8 oeminfoid=10 */
        OEMINFO_BAT_BLIMSW,
        OEMINFO_NONEWP_BAT_BLIMSW
    }, { /* v6_to_v8 oeminfoid = 11 */
        OEMINFO_BAT_BBINFO,
        OEMINFO_NONEWP_BAT_BBINFO
    }, { /* v6_to_v8 oeminfoid=12 */
        OEMINFO_BAT_HWCOUL,
        OEMINFO_NONEWP_BAT_HWCOUL
    }, { /* v6_to_v8 oeminfoid=13 */
        OEMINFO_PPR,
        OEMINFO_NONEWP_PPR
    }, { /* v6_to_v8 oeminfoid=14 */
        OEMINFO_UPDATE_TYPE,
        OEMINFO_NONEWP_UPDATE_TYPE_V8
    }, { /* v6_to_v8 oeminfoid=15 */
        OEMINFO_BOARD_CODE_TYPE,
        OEMINFO_ROOTWP_BOARD_CODE_TYPE_V8
    }, { /* v6_to_v8 oeminfoid=16 */
        OEMINFO_ALS_UNDER_TP_CALIDATA,
        OEMINFO_PWRUPWP_ALS_UNDER_TP_CALIDATA_V8
    }, { /* v6_to_v8 oeminfoid=17 */
        0,
        0
    }, { /* v6_to_v8 oeminfoid=18 */
        OEMINFO_VENDER_AND_COUNTRY_NAME,
        OEMINFO_ROOTWP_VENDER_AND_COUNTRY_NAME_V8
    }, { /* v6_to_v8 oeminfoid=19 */
        0,
        OEMINFO_UNUSEED_TYPE_USER_V8_19
    }, { /* v6_to_v8 oeminfoid=20 */
        OEMINFO_UTAPKAUTORUN,
        OEMINFO_NONEWP_UTAPKAUTORUN
    }, { /* v6_to_v8 oeminfoid=21 */
        OEMINFO_DDC_TYPE,
        OEMINFO_ROOTWP_DDC_TYPE_V8
    }, { /* v6_to_v8 oeminfoid=22 */
        OEMINFO_POST_CUSTOM_TYPE,
        OEMINFO_NONEWP_POST_CUSTOM_TYPE_V8
    }, { /* v6_to_v8 oeminfoid=23 */
        OEMINFO_WIFI_TEST,
        OEMINFO_NONEWP_WIFI_TEST_V8
    }, { /* v6_to_v8 oeminfoid=24 */
        OEMINFO_IMEI2_NUM,
        OEMINFO_ROOTWP_IMEI2_NUM_V8
    }, { /* v6_to_v8 oeminfoid=25 */
        OEMINFO_CUSTOMEI,
        OEMINFO_NONEWP_CUSTOMEI_V8
    }, { /* v6_to_v8 oeminfoid=26 */
        OEMINFO_FACT_INFO,
        OEMINFO_ROOTWP_FACT_INFO_V8
    }, { /* v6_to_v8 oeminfoid=27 */
        OEMINFO_FACT_ODM,
        OEMINFO_NONEWP_FACT_ODM_V8
    }, { /* v6_to_v8 oeminfoid=28 */
        OEMINFO_HWSB_AES_PWD,
        OEMINFO_ROOTWP_HWSB_AES_PWD_V8
    }, { /* v6_to_v8 oeminfoid=29 */
        OEMINFO_HWSB_LOCK_STATE,
        OEMINFO_NONEWP_HWSB_LOCK_STATE_V8
    }, { /* v6_to_v8 oeminfoid=30 */
        OEMINFO_HWFB_LOCK_STATE,
        OEMINFO_NONEWP_HWFB_LOCK_STATE_V8
    }, { /* v6_to_v8 oeminfoid=31 */
        OEMINFO_CASTHDCP,
        OEMINFO_PWRUPWP_CASTHDCP_V8
    }, { /* v6_to_v8 oeminfoid=32 */
        OEMINFO_SENSOR_PROXIMITY_PA2240_CALIBRATION,
        OEMINFO_PWRUPWP_SENSOR_PROXIMITY_PA2240_CALIBRATION_V8
    }, { /* v6_to_v8 oeminfoid=33 */
        OEMINFO_MODSN_CHECK,
        OEMINFO_PWRUPWP_MODSN_CHECK_V8
    }, { /* v6_to_v8 oeminfoid=34 */
        OEMINFO_HOTA_BOOT_MODE,
        OEMINFO_NONEWP_HOTA_BOOT_MODE
    }, { /* v6_to_v8 oeminfoid=35 */
        0,
        0
    }, { /* v6_to_v8 oeminfoid=36 */
        0,
        0
    }, { /* v6_to_v8 oeminfoid=37 */
        OEMINFO_ALSENSOR,
        OEMINFO_PWRUPWP_ALSENSOR_V8
    }, { /* v6_to_v8 oeminfoid=38 */
        OEMINFO_SKU_TYPE,
        OEMINFO_ROOTWP_SKU_TYPE_V8
    }, { /* v6_to_v8 oeminfoid=39 */
        OEMINFO_MANUFACTURE_DATE_TYPE,
        OEMINFO_ROOTWP_MANUFACTURE_DATE_TYPE_V8
    }, { /* v6_to_v8 oeminfoid=40 */
        OEMINFO_VSIM_WIFIPOLL_FLAG,
        OEMINFO_NONEWP_VSIM_WIFIPOLL_FLAG_V8
    }, { /* v6_to_v8 oeminfoid=41 */
        OEMINFO_VSIM_DOWNLOAD_STATUS,
        OEMINFO_NONEWP_VSIM_DOWNLOAD_STATUS_V8
    }, { /* v6_to_v8 oeminfoid=42 */
        OEMINFO_OIS_GYROGAIN,
        OEMINFO_PWRUPWP_OIS_GYROGAIN_V8
    }, { /* v6_to_v8 oeminfoid=43 */
        OEMINFO_VSIM_WIFI_CONN_STATUS,
        OEMINFO_NONEWP_VSIM_WIFI_CONN_STATUS_V8
    }, { /* v6_to_v8 oeminfoid=44 */
        OEMINFO_LGU_SPECIAL_SN,
        OEMINFO_ROOTWP_LGU_SPECIAL_SN_V8
    }, { /* v6_to_v8 oeminfoid=45 */
        OEMINFO_USER_USAGE_ACTIVDATE_TYPE,
        OEMINFO_NONEWP_USER_USAGE_ACTIVDATE_TYPE_V8
    }, { /* v6_to_v8 oeminfoid=46 */
        OEMINFO_UNBIND_FLAG,
        OEMINFO_NONEWP_UNBIND_FLAG_V8
    }, { /* v6_to_v8 oeminfoid=47 */
        OEMINFO_USB_PID_INDEX,
        OEMINFO_NONEWP_USB_PID_INDEX_V8
    }, { /* v6_to_v8 oeminfoid=48 */
        OEMINFO_IMEI_TYPE,
        OEMINFO_ROOTWP_IMEI_TYPE_V8
    }, { /* v6_to_v8 oeminfoid=49 */
        OEMINFO_MEID_TYPE,
        OEMINFO_ROOTWP_MEID_TYPE_V8
    }, { /* v6_to_v8 oeminfoid=50 */
        OEMINFO_ESN_TYPE,
        OEMINFO_ROOTWP_ESN_TYPE_V8
    }, { /* v6_to_v8 oeminfoid=51 */
        OEMINFO_BT_TYPE,
        OEMINFO_ROOTWP_BT_TYPE_V8
    }, { /* v6_to_v8 oeminfoid=52 */
        OEMINFO_WIFI_TYPE,
        OEMINFO_ROOTWP_WIFI_TYPE_V8
    }, { /* v6_to_v8 oeminfoid=53 */
        OEMINFO_FACTORY_INFO_TYPE,
        OEMINFO_ROOTWP_FACTORY_INFO_TYPE_V8
    }, { /* v6_to_v8 oeminfoid=54 */
        OEMINFO_RUNTESTB_TYPE,
        OEMINFO_NONEWP_RUNTESTB_TYPE_V8
    }, { /* v6_to_v8 oeminfoid=55 */
        OEMINFO_POPUPUSEDCOUNT,
        OEMINFO_NONEWP_POPUPUSEDCOUNT_V8
    }, { /* v6_to_v8 oeminfoid=56 */
        OEMINFO_MOTORCODE,
        OEMINFO_NONEWP_MOTORCODE_V8
    }, { /* v6_to_v8 oeminfoid=57 */
        OEMINFO_USB_PID_CUSTOM_TYPE,
        OEMINFO_NONEWP_USB_PID_CUSTOM_TYPE_V8
    }, { /* v6_to_v8 oeminfoid=58 */
        OEMINFO_LANMAC,
        OEMINFO_PWRUPWP_LANMAC_V8
    }, { /* v6_to_v8 oeminfoid=59 */
        OEMINFO_TIME_ON_TYPE,
        OEMINFO_ROOTWP_TIME_ON_TYPE
    }, { /* v6_to_v8 oeminfoid=60 */
        OEMINFO_POS_HALL_CALIBRATION_DATA,
        OEMINFO_NONEWP_POS_HALL_CALIBRATION_DATA
    }, { /* v6_to_v8 oeminfoid=61 */
        OEMINFO_SCCAL_CUR,
        OEMINFO_NONEWP_SCCAL_CUR_V8
    }, { /* v6_to_v8 oeminfoid=62 */
        OEMINFO_CUR_OFFSET,
        OEMINFO_NONEWP_CUR_OFFSET_V8
    }, { /* v6_to_v8 oeminfoid=63 */
        0,
        0
    }, { /* v6_to_v8 oeminfoid=64 */
        OEMINFO_CCFLAG,
        OEMINFO_NONEWP_CCFLAG_V8
    }, { /* v6_to_v8 oeminfoid=65 */
        OEMINFO_TPCOLOR_TYPE,
        OEMINFO_NONEWP_TPCOLOR_TYPE_V8
    }, { /* v6_to_v8 oeminfoid=66 */
        OEMINFO_CRYPTFS_STATE,
        OEMINFO_NONEWP_CRYPTFS_STATE_V8
    }, { /* v6_to_v8 oeminfoid=67 */
        OEMINFO_ROOT_INFO_TYPE,
        OEMINFO_NONEWP_ROOT_INFO_TYPE_V8
    }, { /* v6_to_v8 oeminfoid=68 */
        OEMINFO_RESCUEINFO_TYPE,
        OEMINFO_NONEWP_RESCUEINFO_TYPE_V8
    }, { /* v6_to_v8 oeminfoid=69 */
        OEMINFO_TOF_REPAIR_EYE_SAFE_TYPE,
        OEMINFO_ROOTWP_TOF_EYE_SAFE_TYPE_V8
    }, { /* v6_to_v8 oeminfoid=70 */
        OEMINFO_ACCEL_SENSOR_DATA,
        OEMINFO_PWRUPWP_ACCEL_SENSOR_DATA_V8
    }, { /* v6_to_v8 oeminfoid=71 */
        OEMINFO_AUTH_ELABEL,
        OEMINFO_ROOTWP_AUTH_ELABEL_V8
    }, { /* v6_to_v8 oeminfoid=72 */
        OEMINFO_DEBUG_EFUSE_FLAG,
        OEMINFO_NONEWP_DEBUG_EFUSE_FLAG_V8
    }, { /* v6_to_v8 oeminfoid=73 */
        OEMINFO_WIFI_COUNTRY_CODE,
        OEMINFO_NONEWP_WIFI_COUNTRY_CODE_V8
    }, { /* v6_to_v8 oeminfoid=74 */
        OEMINFO_CUST_OVMODE_STATE,
        OEMINFO_NONEWP_CUST_OVMODE_STATE_V8
    }, { /* v6_to_v8 oeminfoid=75 */
        OEMINFO_PRELOAD_OVMODE_STATE,
        OEMINFO_NONEWP_PRELOAD_OVMODE_STATE_V8
    }, { /* v6_to_v8 oeminfoid=76 */
        OEMINFO_SYSDLL_INSTALL_TYPE,
        OEMINFO_ROOTWP_SYSDLL_INSTALL_TYPE_V8
    }, { /* v6_to_v8 oeminfoid=77 */
        OEMINFO_RETAIL_APK,
        OEMINFO_NONEWP_RETAIL_APK_V8
    }, { /* v6_to_v8 oeminfoid=78 */
        OEMINFO_BUILD_NUMBER_TYPE,
        OEMINFO_ROOTWP_BUILD_NUMBER_TYPE_V8
    }, { /* v6_to_v8 oeminfoid=79 */
        OEMINFO_CAEMRA_DF_TYPE,
        OEMINFO_NONEWP_CAEMRA_DF_TYPE_V8
    }, { /* v6_to_v8 oeminfoid=80 */
        OEM_CAP_PROX_DATA_INFO,
        OEMINFO_PWRUPWP_CAP_PROX_DATA_INFO_V8
    }, { /* v6_to_v8 oeminfoid=81 */
        OEMINFO_SINGLE_SIM,
        OEMINFO_ROOTWP_SINGLE_SIM_V8
    }, { /* v6_to_v8 oeminfoid=82 */
        OEMINFO_CAMERA_HALL_OVERTURN_DATA,
        OEMINFO_NONEWP_CAMERA_HALL_OVERTURN_DATA_V8
    }, { /* v6_to_v8 oeminfoid=83 */
        OEMINFO_AT_LOCK_PASSWORD_SHA256,
        OEMINFO_NONEWP_AT_LOCK_PASSWORD_SHA256_V8
    }, { /* v6_to_v8 oeminfoid=84 */
        OEMINFO_RESET_ENTER_INFO,
        OEMINFO_NONEWP_RESET_ENTER_INFO_V8
    }, { /* v6_to_v8 oeminfoid=85 */
        OEMINFO_UPDATE_ENTER_INFO,
        OEMINFO_NONEWP_UPDATE_ENTER_INFO_V8
    }, { /* v6_to_v8 oeminfoid=86 */
        OEMINFO_DAP_TIME,
        OEMINFO_NONEWP_DAP_TIME_V8
    }, { /* v6_to_v8 oeminfoid=87 */
        OEMINFO_PSID_DATA,
        OEMINFO_NONEWP_PSID_DATA_V8
    }, { /* v6_to_v8 oeminfoid=88 */
        OEMINFO_RESOURCE_VERSION,
        OEMINFO_NONEWP_RESOURCE_VERSION_V8
    }, { /* v6_to_v8 oeminfoid=89 */
        OEMINFO_CRYPTFS_DATA,
        OEMINFO_NONEWP_CRYPTFS_DATA_V8
    }, { /* v6_to_v8 oeminfoid=90 */
        0,
        0
    }, { /* v6_to_v8 oeminfoid=91 */
        OEMINFO_PRODUCT_MODEL_TYPE,
        OEMINFO_ROOTWP_PRODUCT_MODEL_TYPE_V8
    }, { /* v6_to_v8 oeminfoid=92 */
        OEMINFO_INITRC_TRACER,
        OEMINFO_NONEWP_INITRC_TRACER_V8
    }, { /* v6_to_v8 oeminfoid=93 */
        OEMINFO_FBLOCK_STAT_INFO,
        OEMINFO_NONEWP_FBLOCK_STAT_INFO_V8
    }, { /* v6_to_v8 oeminfoid=94 */
        OEMINFO_VENDER_AND_COUNTRY_NAME_COTA,
        OEMINFO_NONEWP_VENDER_AND_COUNTRY_NAME_COTA_V8
    }, { /* v6_to_v8 oeminfoid=95 */
        OEMINFO_ANTITHIVES_STATE,
        OEMINFO_NONEWP_ANTITHIVES_STATE_V8
    }, { /* v6_to_v8 oeminfoid=96 */
        OEMINFO_WIFI_CAL_INDEX,
        OEMINFO_NONEWP_WIFI_CAL_INDEX_V8
    }, { /* v6_to_v8 oeminfoid=97 */
        OEMINFO_DEVICE_MODEL,
        OEMINFO_ROOTWP_DEVICE_MODEL_V8
    }, { /* v6_to_v8 oeminfoid=98 */
        OEMINFO_FRP_OPEN_FLAG,
        OEMINFO_ROOTWP_FRP_OPEN_FLAG_V8
    }, { /* v6_to_v8 oeminfoid=99 */
        OEMINFO_FRP_KEY,
        OEMINFO_ROOTWP_FRP_KEY_V8
    }, { /* v6_to_v8 oeminfoid=100 */
        OEMINFO_SD_LOCK_PASSWD,
        OEMINFO_NONEWP_SD_LOCK_PASSWD_V8
    }, { /* v6_to_v8 oeminfoid=101 */
        OEMINFO_MAIN_VERSION,
        OEMINFO_NONEWP_MAIN_VERSION_V8
    }, { /* oeminfoid=102 */
        OEMINFO_TOF_EYE_SAFE_TYPE,
        OEMINFO_PWRUPWP_TOF_EYE_SAFE_TYPE_V8
    }, { /* v6_to_v8 oeminfoid=103 */
        OEMINFO_FB_STP_TYPE,
        OEMINFO_NONEWP_FB_STP_TYPE_V8
    }, { /* v6_to_v8 oeminfoid=104 */
        OEMINFO_BOOTANIM_SWITCH,
        OEMINFO_NONEWP_BOOTANIM_SWITCH_V8
    }, { /* v6_to_v8 oeminfoid=105 */
        OEMINFO_BOOTANIM_SHUTFLAG,
        OEMINFO_NONEWP_BOOTANIM_SHUTFLAG_V8
    }, { /* v6_to_v8 oeminfoid=106 */
        OEMINFO_BOOTANIM_RINGMODE,
        OEMINFO_NONEWP_BOOTANIM_RINGMODE_V8
    }, { /* v6_to_v8 oeminfoid=107 */
        OEMINFO_BOOTANIM_RINGDEX,
        OEMINFO_NONEWP_BOOTANIM_RINGDEX_V8
    }, { /* v6_to_v8 oeminfoid=108 */
        OEMINFO_DATA_PART_FAULT_TIMES,
        OEMINFO_NONEWP_DATA_PART_FAULT_TIMES_V8
    }, { /* v6_to_v8 oeminfoid=109 */
        OEMINFO_CLOUD_ROM_INSTALL_STAT,
        OEMINFO_NONEWP_CLOUD_ROM_INSTALL_STAT_V8
    }, { /* v6_to_v8 oeminfoid=110 */
        OEMINFO_APPS_NO_INSTALL_TYPE,
        OEMINFO_NONEWP_APPS_NO_INSTALL_TYPE_V8
    }, { /* v6_to_v8 oeminfoid=111 */
        OEMINFO_SYSTEM_VERSION,
        OEMINFO_NONEWP_SYSTEM_VERSION_V8
    }, { /* v6_to_v8 oeminfoid=112 */
        OEMINFO_DEV_NG_INFO,
        OEMINFO_NONEWP_DEV_NG_INFO_V8
    }, { /* v6_to_v8 oeminfoid=113 */
        OEMINFO_CAMERA_SPECS,
        OEMINFO_NONEWP_CAMERA_SPECS_V8
    }, { /* v6_to_v8 oeminfoid=114 */
        OEMINFO_GAMMA_DATA,
        OEMINFO_NONEWP_GAMMA_DATA_V8
    }, { /* v6_to_v8 oeminfoid=115 */
        OEMINFO_GAMMA_LEN,
        OEMINFO_NONEWP_GAMMA_LEN_V8
    }, { /* v6_to_v8 oeminfoid=116 */
        OEMINFO_SENSOR_LIGHT_BH1745_CALIBRATION,
        OEMINFO_PWRUPWP_SENSOR_LIGHT_BH1745_CALIBRATION_V8
    }, { /* v6_to_v8 oeminfoid=117 */
        OEMINFO_MCC_MNC_TYPE,
        OEMINFO_NONEWP_MCC_MNC_TYPE_V8
    }, { /* v6_to_v8 oeminfoid=118 */
        OEMINFO_PRODUCE_PHASE_FLAG,
        OEMINFO_ROOTWP_PRODUCE_PHASE_FLAG_V8
    }, { /* v6_to_v8 oeminfoid=119 */
        OEMINFO_GYRO_SENSOR_DATA,
        OEMINFO_PWRUPWP_GYRO_SENSOR_DATA_V8
    }, { /* v6_to_v8 oeminfoid=120 */
        OEMINFO_OPERATOR_INFO,
        OEMINFO_NONEWP_OPERATOR_INFO_V8
    }, { /* v6_to_v8 oeminfoid=121 */
        OEMINFO_HOTA_UPDATE_AUTH_LOCK,
        OEMINFO_NONEWP_HOTA_UPDATE_AUTH_LOCK_V8
    }, { /* v6_to_v8 oeminfoid=122 */
        OEMINFO_FACTORY_PRODUCT_VERSION_TYPE,
        OEMINFO_NONEWP_FACTORY_PRODUCT_VERSION_TYPE_V8
    }, { /* v6_to_v8 oeminfoid=123 */
        OEMINFO_AUDIO_SMARTPA_CALIBRATION,
        OEMINFO_NONEWP_AUDIO_SMARTPA_CALIBRATION_V8
    }, { /* v6_to_v8 oeminfoid=124 */
        OEMINFO_WIFIONLY_IDENTITY,
        OEMINFO_NONEWP_WIFIONLY_IDENTITY_V8
    }, { /* v6_to_v8 oeminfoid=125 */
        OEMINFO_MMIAUTORUN,
        OEMINFO_ROOTWP_MMIAUTORUN_V8
    }, { /* v6_to_v8 oeminfoid=126 */
        OEMINFO_LCD_BRIGHT_COLOR_CALIBRATION,
        OEMINFO_NONEWP_LCD_BRIGHT_COLOR_CALIBRATION_V8
    }, { /* v6_to_v8 oeminfoid=127 */
        OEMINFO_FOTA_MODE,
        OEMINFO_NONEWP_FOTA_MODE_V8
    }, { /* v6_to_v8 oeminfoid=128 */
        OEMINFO_MMIAUTOCFG,
        OEMINFO_ROOTWP_MMIAUTOCFG_V8
    }, { /* v6_to_v8 oeminfoid=129 */
        OEMINFO_MARKETING_NAME,
        OEMINFO_NONEWP_MARKETING_NAME_V8
    }, { /* oeminfoid=130 */
        OEMINFO_RD_LOCK_STATE,
        OEMINFO_ROOTWP_RD_LOCK_STATE_V8
    }, { /* v6_to_v8 oeminfoid=131 */
        OEMINFO_ACCESS_LOCK_STATE,
        OEMINFO_ROOTWP_ACCESS_LOCK_STATE_V8
    }, { /* v6_to_v8 oeminfoid=132 */
        OEMINFO_ACCESS_KEY,
        OEMINFO_ROOTWP_ACCESS_KEY_V8
    }, { /* v6_to_v8 oeminfoid=133 */
        OEMINFO_OVMODE_STATE,
        OEMINFO_NONEWP_OVMODE_STATE_V8
    }, { /* v6_to_v8 oeminfoid=134 */
        OEMINFO_ESIM_EID,
        OEMINFO_NONEWP_ESIM_EID_V8
    }, { /* v6_to_v8 oeminfoid=135 */
        OEMINFO_OMADM_INFO,
        OEMINFO_NONEWP_OMADM_INFO_V8
    }, { /* v6_to_v8 oeminfoid=136 */
        OEMINFO_ACTIVATION_STATUS_ID,
        OEMINFO_NONEWP_ACTIVATION_STATUS_ID_V8
    }, { /* v6_to_v8 oeminfoid=137 */
        OEMINFO_HWPARENTCONTROL_RECOVER_DATA,
        OEMINFO_NONEWP_HWPARENTCONTROL_RECOVER_DATA_V8
    }, { /* v6_to_v8 oeminfoid=138 */
        OEMINFO_CONTROL_FLAG,
        OEMINFO_ROOTWP_CONTROL_FLAG_V8
    }, { /* v6_to_v8 oeminfoid=139 */
        OEMINFO_FIX_HOTA_INFO_INTI,
        OEMINFO_NONEWP_FIX_HOTA_INFO_INTI_V8
    }, { /* v6_to_v8 oeminfoid=140 */
        OEMINFO_FIX_HOTA_INFO,
        OEMINFO_NONEWP_FIX_HOTA_INFO_V8
    }, { /* v6_to_v8 oeminfoid=141 */
        OEMINFO_FIX_HOTA_STRING_INTI,
        OEMINFO_NONEWP_FIX_HOTA_STRING_INTI_V8
    }, { /* v6_to_v8 oeminfoid=142 */
        OEMINFO_FIX_HOTA_STRING,
        OEMINFO_NONEWP_FIX_HOTA_STRING_V8
    }, { /* v6_to_v8 oeminfoid=143 */
        OEMINFO_QRCODE_INFO,
        OEMINFO_ROOTWP_QRCODE_INFO_V8
    }, { /* v6_to_v8 oeminfoid=144 */
        OEMINFO_QRCODE_BSN,
        OEMINFO_PWRUPWP_QRCODE_BSN_V8
    }, { /* v6_to_v8 oeminfoid=145 */
        OEMINFO_VENDER_AND_COUNTRY_NAME_COTA_FACTORY,
        OEMINFO_NONEWP_VENDER_AND_COUNTRY_NAME_COTA_FACTORY_V8
    }, { /* v6_to_v8 oeminfoid=146 */
        OEMINFO_USER_HOTA_UPDATE,
        OEMINFO_NONEWP_USER_HOTA_UPDATE_V8
    }, { /* v6_to_v8 oeminfoid=147 */
        OEMINFO_LOG_SWITCH_TYPE,
        OEMINFO_ROOTWP_LOG_SWITCH_TYPE_V8
    }, { /* v6_to_v8 oeminfoid=148 */
        OEMINFO_USE_DECRESS_BAT_VOL_FLAG,
        OEMINFO_NONEWP_USE_DECRESS_BAT_VOL_FLAG_V8
    }, { /* v6_to_v8 oeminfoid=149 */
        OEMINFO_CAMERA_LASERCALIB,
        OEMINFO_PWRUPWP_CAMERA_LASERCALIB_V8
    }, { /* v6_to_v8 oeminfoid=150 */
        OEMINFO_UPDATE_UDID,
        OEMINFO_NONEWP_UPDATE_UDID_V8
    }, { /* v6_to_v8 oeminfoid=151 */
        OEMINFO_BASEBAND_VERSION_TYPE,
        OEMINFO_ROOTWP_BASEBAND_VERSION_TYPE_V8
    }, { /* v6_to_v8 oeminfoid=152 */
        OEMINFO_FACTORY_SW_VERSION,
        OEMINFO_NONEWP_FACTORY_SW_VERSION_V8
    }, { /* v6_to_v8 oeminfoid=153 */
        OEMINFO_PRELOAD_APP_BATCH,
        OEMINFO_NONEWP_PRELOAD_APP_BATCH_V8
    }, { /* v6_to_v8 oeminfoid=154 */
        OEMINFO_CUST_Y_CABLE_TYPE,
        OEMINFO_NONEWP_CUST_Y_CABLE_TYPE_V8
    }, { /* v6_to_v8 oeminfoid=155 */
        OEMINFO_UDID_CODE_TYPE,
        OEMINFO_ROOTWP_UDID_CODE_TYPE_V8
    }, { /* v6_to_v8 oeminfoid=156 */
        OEMINFO_BASE_VER_TYPE,
        OEMINFO_NONEWP_BASE_VER_TYPE_V8
    }, { /* v6_to_v8 oeminfoid=157 */
        OEMINFO_CUST_VER_TYPE,
        OEMINFO_NONEWP_CUST_VER_TYPE_V8
    }, { /* v6_to_v8 oeminfoid=158 */
        OEMINFO_PRELOAD_VER_TYPE,
        OEMINFO_NONEWP_PRELOAD_VER_TYPE_V8
    }, { /* v6_to_v8 oeminfoid=159 */
        0,
        0
    }, { /* v6_to_v8 oeminfoid=160 */
        0,
        0
    }, { /* v6_to_v8 oeminfoid=161 */
        OEMINFO_SDUPDATE_TYPE,
        OEMINFO_NONEWP_SDUPDATE_TYPE_V8
    }, { /* v6_to_v8 oeminfoid=162 */
        OEMINFO_NAL_CODE_TYPE,
        OEMINFO_ROOTWP_NAL_CODE_TYPE
    }, { /* v6_to_v8 oeminfoid=163 */
        OEMINFO_ENABLE_RETREAD,
        OEMINFO_NONEWP_ENABLE_RETREAD_V8
    }, { /* v6_to_v8 oeminfoid=164 */
        OEMINFO_BDRT_TYPE,
        OEMINFO_NONEWP_BDRT_TYPE_V8
    }, { /* v6_to_v8 oeminfoid=165 */
        OEMINFO_MOD_HASH,
        OEMINFO_PWRUPWP_MOD_HASH_V8
    }, { /* v6_to_v8 oeminfoid=166 */
        OEMINFO_SYSTEM_UPDATE_AUTH_TOKEN_INFO,
        OEMINFO_NONEWP_SYSTEM_UPDATE_AUTH_TOKEN_INFO_V8
    }, { /* v6_to_v8 oeminfoid=167 */
        OEMINFO_ENCRYPT_INFO,
        OEMINFO_NONEWP_ENCRYPT_INFO_V8
    }, { /* v6_to_v8 oeminfoid=168 */
        OEMINFO_CONTROL_BIT,
        OEMINFO_ROOTWP_CONTROL_BIT_V8
    }, { /* v6_to_v8 oeminfoid=169 */
        OEMINFO_MALWARE_DIAGNOSE,
        OEMINFO_NONEWP_MALWARE_DIAGNOSE_V8
    }, { /* v6_to_v8 oeminfoid=170 */
        OEMINFO_RETREAD_DIAGNOSE,
        OEMINFO_NONEWP_RETREAD_DIAGNOSE_V8
    }, { /* v6_to_v8 oeminfoid=171 */
        OEMINFO_ROOT_DIAGNOSE,
        OEMINFO_NONEWP_ROOT_DIAGNOSE_V8
    }, { /* v6_to_v8 oeminfoid=172 */
        OEMINFO_SYSTEM_VERSION_B,
        OEMINFO_NONEWP_SYSTEM_VERSION_B_V8
    }, { /* v6_to_v8 oeminfoid=173 */
        OEMINFO_ANDROID_VERSION,
        OEMINFO_NONEWP_ANDROID_VERSION_V8
    }, { /* v6_to_v8 oeminfoid=174 */
        OEMINFO_ANDROID_VERSION_B,
        OEMINFO_NONEWP_ANDROID_VERSION_B_V8
    }, { /* v6_to_v8 oeminfoid=175 */
        OEMINFO_SLOT_INFO_A,
        OEMINFO_NONEWP_SLOT_INFO_A_V8
    }, { /* v6_to_v8 oeminfoid=176 */
        OEMINFO_SLOT_INFO_B,
        OEMINFO_NONEWP_SLOT_INFO_B_V8
    }, { /* v6_to_v8 oeminfoid=177 */
        OEMINFO_SLOT_STATUS,
        OEMINFO_NONEWP_SLOT_STATUS_V8
    }, { /* v6_to_v8 oeminfoid=178 */
        OEMINFO_VERITY_MODE_UPDATE_STATUS,
        OEMINFO_NONEWP_VERITY_MODE_UPDATE_STATUS_V8
    }, { /* v6_to_v8 oeminfoid=179 */
        OEMINFO_SYSTEM_UPDATE_STATE,
        OEMINFO_NONEWP_SYSTEM_UPDATE_STATE_V8
    }, { /* v6_to_v8 oeminfoid=180 */
        OEMINFO_CUSTOM_VERSION,
        OEMINFO_NONEWP_CUSTOM_VERSION_V8
    }, { /* v6_to_v8 oeminfoid=181 */
        OEMINFO_CUSTOM_VERSION_B,
        OEMINFO_NONEWP_CUSTOM_VERSION_B_V8
    }, { /* v6_to_v8 oeminfoid=182 */
        OEMINFO_PRELOAD_VERSION,
        OEMINFO_NONEWP_PRELOAD_VERSION_V8
    }, { /* v6_to_v8 oeminfoid=183 */
        OEMINFO_PRELOAD_VERSION_B,
        OEMINFO_NONEWP_PRELOAD_VERSION_B_V8
    }, { /* v6_to_v8 oeminfoid=184 */
        OEMINFO_GROUP_VERSION,
        OEMINFO_NONEWP_GROUP_VERSION_V8
    }, { /* v6_to_v8 oeminfoid=185 */
        OEMINFO_GROUP_VERSION_B,
        OEMINFO_NONEWP_GROUP_VERSION_B_V8
    }, { /* v6_to_v8 oeminfoid=186 */
        OEMINFO_BASE_VERSION,
        OEMINFO_NONEWP_BASE_VERSION_V8
    }, { /* v6_to_v8 oeminfoid=187 */
        OEMINFO_BASE_VERSION_B,
        OEMINFO_NONEWP_BASE_VERSION_B_V8
    }, { /* v6_to_v8 oeminfoid=188 */
        OEMINFO_COTA_REBOOT_UPDATE_PATH,
        OEMINFO_NONEWP_COTA_REBOOT_UPDATE_PATH_V8
    }, { /* oeminfoid=189 */
        OEMINFO_CUSTOM_C_VERSION,
        OEMINFO_NONEWP_CUSTOM_C_VERSION_V8
    }, { /* oeminfoid=190 */
        OEMINFO_RUNTEST_TYPE,
        OEMINFO_NONEWP_RUNTEST_TYPE_V8
    }, { /* oeminfoid=191 */
        OEMINFO_MMITEST_TYPE,
        OEMINFO_NONEWP_MMITEST_TYPE_V8
    }, { /* oeminfoid=192 */
        OEMINFO_OTA_SIMLOCK_ACTIVATED,
        OEMINFO_NONEWP_OTA_SIMLOCK_ACTIVATED_V8
    }, { /* oeminfoid=193 */
        OEMINFO_ATLOCK_TYPE,
        OEMINFO_NONEWP_ATLOCK_TYPE_V8
    }, { /* oeminfoid=194 */
        OEMINFO_RUNTEST2_TYPE,
        OEMINFO_NONEWP_RUNTEST2_TYPE_V8
    }, { /* oeminfoid=195 */
        OEMINFO_RUNTIME_TYPE,
        OEMINFO_NONEWP_RUNTIME_TYPE_V8
    }, { /* oeminfoid=196 */
        OEMINFO_MDATE_TYPE,
        OEMINFO_NONEWP_MDATE_TYPE_V8
    }, { /* oeminfoid=197 */
        OEMINFO_MANUFACTURE_RESET_FLAG,
        OEMINFO_NONEWP_MANUFACTURE_RESET_FLAG_V8
    }, { /* oeminfoid=198 */
        0,
        0
    }, { /* oeminfoid=199 */
        OEMINFO_BACKCOLOR,
        OEMINFO_NONEWP_BACKCOLOR_V8
    }, { /* oeminfoid=200 */
        0,
        0
    }, { /* oeminfoid=201 */
        0,
        OEMINFO_UNUSEED_TYPE_USER_V8_201
    }, { /* oeminfoid=202 */
        0,
        0
    }, { /* oeminfoid=203 */
        OEMINFO_REUSED_ID_203_8K_VALID_SIZE_448_BYTE,
        OEMINFO_NONEWP_REUSED_ID_1101_8K_VALID_SIZE_448_BYTE_V8
    }, { /* oeminfoid=204 */
        OEMINFO_REUSED_ID_204_8K_VALID_SIZE_64_BYTE,
        OEMINFO_NONEWP_REUSED_ID_1102_8K_VALID_SIZE_64_BYTE_V8
    }, { /* oeminfoid=205 */
        OEMINFO_FMD_VERIFY_DATA,
        OEMINFO_NONEWP_FMD_VERIFY_DATA_V8
    }, { /* oeminfoid=206 */
        OEMINFO_DEVICE_CERTIFICATE1,
        OEMINFO_ROOTWP_DEVICE_CERTIFICATE1_V8
    }, { /* oeminfoid=207 */
        OEMINFO_DEVICE_CERTIFICATE2,
        OEMINFO_ROOTWP_DEVICE_CERTIFICATE2_V8
    }, { /* oeminfoid=208 */
        OEMINFO_KEY_ATTESTATION,
        OEMINFO_ROOTWP_KEY_ATTESTATION_V8
    }, { /* oeminfoid=209 */
        OEMINFO_FCDEG,
        OEMINFO_NONEWP_FCDEG_V8
    }, { /* oeminfoid=210 */
        0,
        OEMINFO_UNUSEED_TYPE_USER_V8_210
    }, { /* oeminfoid=211 */
        0,
        0
    }, { /* oeminfoid=212 */
        0,
        0
    }, { /* oeminfoid=213 */
        0,
        OEMINFO_UNUSEED_TYPE_USER_V8_213
    }, { /* oeminfoid=214 */
        0,
        0
    }, { /* oeminfoid=215 */
        0,
        0
    }, { /* oeminfoid=216 */
        0,
        OEMINFO_UNUSEED_TYPE_USER_V8_216
    }, { /* oeminfoid=217 */
        0,
        0
    }, { /* oeminfoid=218 */
        0,
        0
    }, { /* oeminfoid=219 */
        0,
        OEMINFO_UNUSEED_TYPE_USER_V8_219
    }, { /* oeminfoid=220 */
        0,
        0
    }, { /* oeminfoid=221 */
        0,
        0
    }, { /* oeminfoid=222 */
        0,
        OEMINFO_UNUSEED_TYPE_USER_V8_222
    }, { /* oeminfoid=223 */
        0,
        0
    }, { /* oeminfoid=224 */
        0,
        0
    }, { /* oeminfoid=225 */
        0,
        OEMINFO_UNUSEED_TYPE_USER_V8_225
    }, { /* oeminfoid=226 */
        0,
        0
    }, { /* oeminfoid=227 */
        0,
        0
    }, { /* oeminfoid=228 */
        0,
        OEMINFO_UNUSEED_TYPE_USER_V8_228
    }, { /* oeminfoid=229 */
        0,
        0
    }, { /* oeminfoid=230 */
        0,
        0
    }, { /* oeminfoid=231 */
        0,
        OEMINFO_UNUSEED_TYPE_USER_V8_231
    }, { /* oeminfoid=232 */
        0,
        0
    }, { /* oeminfoid=233 */
        0,
        0
    }, { /* oeminfoid=234 */
        0,
        OEMINFO_UNUSEED_TYPE_USER_V8_234
    }, { /* oeminfoid=235 */
        0,
        0
    }, { /* oeminfoid=236 */
        0,
        0
    }, { /* oeminfoid=237 */
        0,
        OEMINFO_UNUSEED_TYPE_USER_V8_237
    }, { /* oeminfoid=238 */
        0,
        0
    }, { /* oeminfoid=239 */
        0,
        0
    }, { /* oeminfoid=240 */
        0,
        OEMINFO_UNUSEED_TYPE_USER_V8_240
    }, { /* oeminfoid=241 */
        0,
        0
    }, { /* oeminfoid=242 */
        0,
        0
    }, { /* oeminfoid=243 */
        0,
        OEMINFO_UNUSEED_TYPE_USER_V8_243
    }, { /* oeminfoid=244 */
        0,
        0
    }, { /* oeminfoid=245 */
        0,
        0
    }, { /* oeminfoid=246 */
        0,
        OEMINFO_UNUSEED_TYPE_USER_V8_246
    }, { /* oeminfoid=247 */
        0,
        0
    }, { /* oeminfoid=248 */
        0,
        0
    }, { /* oeminfoid=249 */
        0,
        OEMINFO_UNUSEED_TYPE_USER_V8_249
    }, { /* oeminfoid=250 */
        0,
        0
    }, { /* oeminfoid=251 */
        0,
        0
    }, { /* oeminfoid=252 */
        0,
        OEMINFO_UNUSEED_TYPE_USER_V8_252
    }, { /* oeminfoid=253 */
        0,
        0
    }, { /* oeminfoid=254 */
        0,
        0
    }, { /* oeminfoid=255 */
        0,
        OEMINFO_UNUSEED_TYPE_USER_V8_255
    }, { /* oeminfoid=256 */
        0,
        0
    }, { /* oeminfoid=257 */
        0,
        0
    }, { /* oeminfoid=258 */
        0,
        OEMINFO_UNUSEED_TYPE_USER_V8_258
    }, { /* oeminfoid=259 */
        0,
        0
    }, { /* oeminfoid=260 */
        0,
        0
    }, { /* oeminfoid=261 */
        0,
        OEMINFO_UNUSEED_TYPE_USER_V8_261
    }, { /* oeminfoid=262 */
        0,
        0
    }, { /* oeminfoid=263 */
        0,
        0
    }, { /* oeminfoid=264 */
        0,
        OEMINFO_UNUSEED_TYPE_USER_V8_264
    }, { /* oeminfoid=265 */
        0,
        0
    }, { /* oeminfoid=266 */
        0,
        0
    }, { /* oeminfoid=267 */
        0,
        OEMINFO_UNUSEED_TYPE_USER_V8_267
    }, { /* oeminfoid=268 */
        0,
        0
    }, { /* oeminfoid=269 */
        0,
        0
    }, { /* oeminfoid=270 */
        0,
        OEMINFO_UNUSEED_TYPE_USER_V8_270
    }, { /* oeminfoid=271 */
        0,
        0
    }, { /* oeminfoid=272 */
        0,
        0
    }, { /* oeminfoid=273 */
        0,
        OEMINFO_UNUSEED_TYPE_USER_V8_273
    }, { /* oeminfoid=274 */
        0,
        0
    }, { /* oeminfoid=275 */
        0,
        0
    }, { /* oeminfoid=276 */
        0,
        OEMINFO_UNUSEED_TYPE_USER_V8_276
    }, { /* oeminfoid=277 */
        0,
        0
    }, { /* oeminfoid=278 */
        0,
        0
    }, { /* oeminfoid=279 */
        0,
        OEMINFO_UNUSEED_TYPE_USER_V8_279
    }, { /* oeminfoid=280 */
        0,
        0
    }, { /* oeminfoid=281 */
        0,
        0
    }, { /* oeminfoid=282 */
        0,
        OEMINFO_UNUSEED_TYPE_USER_V8_282
    }, { /* oeminfoid=283 */
        0,
        0
    }, { /* oeminfoid=284 */
        0,
        0
    }, { /* oeminfoid=285 */
        0,
        OEMINFO_UNUSEED_TYPE_USER_V8_285
    }, { /* oeminfoid=286 */
        0,
        0
    }, { /* oeminfoid=287 */
        0,
        0
    }, { /* oeminfoid=288 */
        0,
        OEMINFO_UNUSEED_TYPE_USER_V8_288
    }, { /* oeminfoid=289 */
        0,
        0
    }, { /* oeminfoid=290 */
        0,
        OEMINFO_UNUSEED_TYPE_USER_V8_290
    }, { /* oeminfoid=291 */
        0,
        0
    }, { /* oeminfoid=292 */
        0,
        OEMINFO_UNUSEED_TYPE_USER_V8_292
    }, { /* oeminfoid=293 */
        0,
        0
    }, { /* oeminfoid=294 */
        0,
        0
    }, { /* oeminfoid=295 */
        0,
        OEMINFO_UNUSEED_TYPE_USER_V8_295
    }, { /* oeminfoid=296 */
        0,
        0
    }, { /* oeminfoid=297 */
        0,
        0
    }, { /* oeminfoid=298 */
        0,
        OEMINFO_UNUSEED_TYPE_USER_V8_298
    }, { /* oeminfoid=299 */
        0,
        0
    }, { /* oeminfoid=300 */
        0,
        0
    }, { /* oeminfoid=301 */
        OEMINFO_CAMERA_TOF_CALIBRATION1_TYPE,
        OEMINFO_NONEWP_CAMERA_TOF_CALIBRATION1_TYPE_V8
    }, { /* oeminfoid=302 */
        OEMINFO_CAMERA_TOF_CALIBRATION2_TYPE,
        OEMINFO_NONEWP_CAMERA_TOF_CALIBRATION2_TYPE_V8
    }, { /* oeminfoid=303 */
        OEMINFO_FASTBOOT_FMD_DATA,
        OEMINFO_NONEWP_FASTBOOT_FMD_DATA_V8
    }, { /* oeminfoid=304 */
        OEMINFO_BASE_CLOUD_ROM_LIST,
        OEMINFO_NONEWP_BASE_CLOUD_ROM_LIST_V8
    }, { /* oeminfoid=305 */
        OEMINFO_CAMERA_TOF_CALIBRATION3_TYPE,
        OEMINFO_NONEWP_CAMERA_TOF_CALIBRATION3_TYPE_V8
    }, { /* oeminfoid=306 */
        OEMINFO_CAMERA_TOF_CALIBRATION4_TYPE,
        OEMINFO_NONEWP_CAMERA_TOF_CALIBRATION4_TYPE_V8
    }, { /* oeminfoid=307 */
        OEMINFO_CAMERA_TOF_CALIBRATION5_TYPE,
        OEMINFO_NONEWP_CAMERA_TOF_CALIBRATION5_TYPE_V8
    }, { /* oeminfoid=308 */
        OEMINFO_CAMERA_TOF_CALIBRATION6_TYPE,
        OEMINFO_NONEWP_CAMERA_TOF_CALIBRATION6_TYPE_V8
    }, { /* oeminfoid=309 */
        OEMINFO_CAMERA_TOF_CALIBRATION7_TYPE,
        OEMINFO_NONEWP_CAMERA_TOF_CALIBRATION7_TYPE_V8
    }, { /* oeminfoid=310 */
        OEMINFO_CAMERA_BLC_CALIB,
        OEMINFO_NONEWP_CAMERA_BLC_CALIB_V8
    }, { /* oeminfoid=311 */
        OEMINFO_GMP,
        OEMINFO_PWRUPWP_GMP_V8
    }, { /* oeminfoid=312 */
        OEMINFO_SCREEN_DATA,
        OEMINFO_ROOTWP_SCREEN_DATA_V8
    }, { /* oeminfoid=313 */
        OEMINFO_CGM_SCG_DATA,
        OEMINFO_NONEWP_CGM_SCG_DATA_V8
    }, { /* oeminfoid=314 */
        OEMINFO_CGM_WCG_DATA,
        OEMINFO_NONEWP_CGM_WCG_DATA_V8
    }, { /* oeminfoid=315 */
        OEMINFO_GMP_EXT,
        OEMINFO_PWRUPWP_GMP_EXT_V8
    }, { /* oeminfoid=316 */
        OEMINFO_CUST_LOGO_EXTERN,
        OEMINFO_ROOTWP_CUST_LOGO_EXTERN
    }, { /* oeminfoid=317 */
        OEMINFO_POWER_CHARGING_TYPE_EXTERN,
        OEMINFO_ROOTWP_POWER_CHARGING_TYPE_EXTERN
    }, { /* oeminfoid=318 */
        OEMINFO_LOW_POWER_TYPE_EXTERN,
        OEMINFO_ROOTWP_LOW_POWER_TYPE_EXTERN
    }, { /* oeminfoid=319 */
        OEMINFO_COLOR_SENSOR_CALI_MATRIX,
        OEMINFO_ROOTWP_COLOR_SENSOR_CALI_MATRIX
    }, { /* oeminfoid=320 */
        0,
        0
    }, { /* oeminfoid=321 */
        0,
        0
    }, { /* oeminfoid=322 */
        0,
        OEMINFO_UNUSEED_TYPE_USER_V8_322
    }, { /* oeminfoid=323 */
        0,
        0
    }, { /* oeminfoid=324 */
        OEMINFO_WCNSS_FIR_BAK_TYPE,
        OEMINFO_NONEWP_WCNSS_FIR_BAK_TYPE_V8
    }, { /* oeminfoid=325 */
        OEMINFO_SPECCALIB0,
        OEMINFO_NONEWP_SPECCALIB0_V8
    }, { /* oeminfoid=326 */
        OEMINFO_SPECCALIB1,
        OEMINFO_NONEWP_SPECCALIB1_V8
    }, { /* oeminfoid=327 */
        0,
        0
    }, { /* oeminfoid=328 */
        0,
        0
    }, { /* oeminfoid=329 */
        OEMINFO_CAMERA_HDC,
        OEMINFO_PWRUPWP_CAMERA_HDC_V8
    }, { /* oeminfoid=330 */
        OEMINFO_NFC_SEID,
        OEMINFO_ROOTWP_NFC_SEID_V8
    }, { /* oeminfoid=331 */
        OEMINFO_CONTROL_MODE_DATA,
        OEMINFO_NONEWP_CONTROL_MODE_DATA
    }, { /* oeminfoid=332 */
        0,
        OEMINFO_UNUSEED_TYPE_USER_V8_332
    }, { /* oeminfoid=333 */
        0,
        0
    }, { /* oeminfoid=334 */
        0,
        0
    }, { /* oeminfoid=335 */
        0,
        OEMINFO_UNUSEED_TYPE_USER_V8_335
    }, { /* oeminfoid=336 */
        0,
        0
    }, { /* oeminfoid=337 */
        0,
        0
    }, { /* oeminfoid=338 */
        0,
        OEMINFO_UNUSEED_TYPE_USER_V8_338
    }, { /* oeminfoid=339 */
        0,
        0
    }, { /* oeminfoid=340 */
        OEMINFO_CAMERA_EEPROM_DATA_00,
        OEMINFO_NONEWP_CAMERA_EEPROM_DATA_00_V8
    }, { /* oeminfoid=341 */
        OEMINFO_CAMERA_EEPROM_DATA_01,
        OEMINFO_NONEWP_CAMERA_EEPROM_DATA_01_V8
    }, { /* oeminfoid=342 */
        OEMINFO_CAMERA_EEPROM_DATA_02,
        OEMINFO_NONEWP_CAMERA_EEPROM_DATA_02_V8
    }, { /* oeminfoid=343 */
        OEMINFO_CAMERA_EEPROM_DATA_03,
        OEMINFO_NONEWP_CAMERA_EEPROM_DATA_03_V8
    }, { /* oeminfoid=344 */
        OEMINFO_CAMERA_EEPROM_DATA_04,
        OEMINFO_NONEWP_CAMERA_EEPROM_DATA_04_V8
    }, { /* oeminfoid=345 */
        OEMINFO_CAMERA_EEPROM_DATA_05,
        OEMINFO_NONEWP_CAMERA_EEPROM_DATA_05_V8
    }, { /* oeminfoid=346 */
        OEMINFO_CAMERA_EEPROM_DATA_06,
        OEMINFO_NONEWP_CAMERA_EEPROM_DATA_06_V8
    }, { /* oeminfoid=347 */
        OEMINFO_CAMERA_EEPROM_DATA_07,
        OEMINFO_NONEWP_CAMERA_EEPROM_DATA_07_V8
    }, { /* oeminfoid=348 */
        OEMINFO_CAMERA_EEPROM_DATA_08,
        OEMINFO_NONEWP_CAMERA_EEPROM_DATA_08_V8
    }, { /* oeminfoid=349 */
        OEMINFO_CAMERA_EEPROM_DATA_09,
        OEMINFO_NONEWP_CAMERA_EEPROM_DATA_09_V8
    }, { /* oeminfoid=350 */
        0,
        0
    }, { /* oeminfoid=351 */
        OEMINFO_ASCEND_TYPE,
        OEMINFO_NONEWP_ASCEND_TYPE_V8
    }, { /* oeminfoid=352 */
        OEMINFO_LOW_POWER_TYPE,
        OEMINFO_NONEWP_LOW_POWER_TYPE_V8
    }, { /* oeminfoid=353 */
        OEMINFO_POWER_CHARGING_TYPE,
        OEMINFO_NONEWP_POWER_CHARGING_TYPE_V8
    },
};

#endif /* INCLUDE_OEMINFO_V6_TO_V8_H */
