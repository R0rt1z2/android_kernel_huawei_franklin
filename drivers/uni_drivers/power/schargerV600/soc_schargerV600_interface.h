

/*****************************************************************************
  1 其他头文件包含
*****************************************************************************/

#ifndef __SOC_SCHARGER_INTERFACE_H__
#define __SOC_SCHARGER_INTERFACE_H__

#ifdef __cplusplus
    #if __cplusplus
        extern "C" {
    #endif
#endif



/*****************************************************************************
  2 宏定义
*****************************************************************************/

/****************************************************************************
                     (1/4) REG_PD
 ****************************************************************************/
/* 寄存器说明：Vendor ID Low
   位域定义UNION结构:  SOC_SCHARGER_VENDIDL_UNION */
#define SOC_SCHARGER_VENDIDL_ADDR(base)               ((base) + (0x00))

/* 寄存器说明：Vendor ID High
   位域定义UNION结构:  SOC_SCHARGER_VENDIDH_UNION */
#define SOC_SCHARGER_VENDIDH_ADDR(base)               ((base) + (0x01))

/* 寄存器说明：Product ID Low
   位域定义UNION结构:  SOC_SCHARGER_PRODIDL_UNION */
#define SOC_SCHARGER_PRODIDL_ADDR(base)               ((base) + (0x02))

/* 寄存器说明：Product ID High
   位域定义UNION结构:  SOC_SCHARGER_PRODIDH_UNION */
#define SOC_SCHARGER_PRODIDH_ADDR(base)               ((base) + (0x03))

/* 寄存器说明：Device ID Low
   位域定义UNION结构:  SOC_SCHARGER_DEVIDL_UNION */
#define SOC_SCHARGER_DEVIDL_ADDR(base)                ((base) + (0x04))

/* 寄存器说明：Device ID High
   位域定义UNION结构:  SOC_SCHARGER_DEVIDH_UNION */
#define SOC_SCHARGER_DEVIDH_ADDR(base)                ((base) + (0x05))

/* 寄存器说明：USB Type-C Revision Low
   位域定义UNION结构:  SOC_SCHARGER_TYPECREVL_UNION */
#define SOC_SCHARGER_TYPECREVL_ADDR(base)             ((base) + (0x06))

/* 寄存器说明：USB Type-C Revision High
   位域定义UNION结构:  SOC_SCHARGER_TYPECREVH_UNION */
#define SOC_SCHARGER_TYPECREVH_ADDR(base)             ((base) + (0x07))

/* 寄存器说明：USB PD Version
   位域定义UNION结构:  SOC_SCHARGER_USBPDVER_UNION */
#define SOC_SCHARGER_USBPDVER_ADDR(base)              ((base) + (0x08))

/* 寄存器说明：USB PD Revision
   位域定义UNION结构:  SOC_SCHARGER_USBPDREV_UNION */
#define SOC_SCHARGER_USBPDREV_ADDR(base)              ((base) + (0x09))

/* 寄存器说明：USB PD Interface Revision Low
   位域定义UNION结构:  SOC_SCHARGER_PDIFREVL_UNION */
#define SOC_SCHARGER_PDIFREVL_ADDR(base)              ((base) + (0x0A))

/* 寄存器说明：USB PD Interface Revision High
   位域定义UNION结构:  SOC_SCHARGER_PDIFREVH_UNION */
#define SOC_SCHARGER_PDIFREVH_ADDR(base)              ((base) + (0x0B))

/* 寄存器说明：PD Alert Low
   位域定义UNION结构:  SOC_SCHARGER_PD_ALERT_L_UNION */
#define SOC_SCHARGER_PD_ALERT_L_ADDR(base)            ((base) + (0x10))

/* 寄存器说明：PD Alert High
   位域定义UNION结构:  SOC_SCHARGER_PD_ALERT_H_UNION */
#define SOC_SCHARGER_PD_ALERT_H_ADDR(base)            ((base) + (0x11))

/* 寄存器说明：PD Alert Mask Low
   位域定义UNION结构:  SOC_SCHARGER_PD_ALERT_MSK_L_UNION */
#define SOC_SCHARGER_PD_ALERT_MSK_L_ADDR(base)        ((base) + (0x12))

/* 寄存器说明：PD Alert Mask High
   位域定义UNION结构:  SOC_SCHARGER_PD_ALERT_MSK_H_UNION */
#define SOC_SCHARGER_PD_ALERT_MSK_H_ADDR(base)        ((base) + (0x13))

/* 寄存器说明：PD Power State Mask
   位域定义UNION结构:  SOC_SCHARGER_PD_PWRSTAT_MSK_UNION */
#define SOC_SCHARGER_PD_PWRSTAT_MSK_ADDR(base)        ((base) + (0x14))

/* 寄存器说明：PD Fault State Mask
   位域定义UNION结构:  SOC_SCHARGER_PD_FAULTSTAT_MSK_UNION */
#define SOC_SCHARGER_PD_FAULTSTAT_MSK_ADDR(base)      ((base) + (0x15))

/* 寄存器说明：PD TCPC Control
   位域定义UNION结构:  SOC_SCHARGER_PD_TCPC_CTRL_UNION */
#define SOC_SCHARGER_PD_TCPC_CTRL_ADDR(base)          ((base) + (0x19))

/* 寄存器说明：PD Role Control
   位域定义UNION结构:  SOC_SCHARGER_PD_ROLE_CTRL_UNION */
#define SOC_SCHARGER_PD_ROLE_CTRL_ADDR(base)          ((base) + (0x1A))

/* 寄存器说明：PD Fault Control
   位域定义UNION结构:  SOC_SCHARGER_PD_FAULT_CTRL_UNION */
#define SOC_SCHARGER_PD_FAULT_CTRL_ADDR(base)         ((base) + (0x1B))

/* 寄存器说明：PD Power Control
   位域定义UNION结构:  SOC_SCHARGER_PD_PWR_CTRL_UNION */
#define SOC_SCHARGER_PD_PWR_CTRL_ADDR(base)           ((base) + (0x1C))

/* 寄存器说明：PD CC Status
   位域定义UNION结构:  SOC_SCHARGER_PD_CC_STATUS_UNION */
#define SOC_SCHARGER_PD_CC_STATUS_ADDR(base)          ((base) + (0x1D))

/* 寄存器说明：PD Power Status
   位域定义UNION结构:  SOC_SCHARGER_PD_PWR_STATUS_UNION */
#define SOC_SCHARGER_PD_PWR_STATUS_ADDR(base)         ((base) + (0x1E))

/* 寄存器说明：PD Fault Status
   位域定义UNION结构:  SOC_SCHARGER_PD_FAULT_STATUS_UNION */
#define SOC_SCHARGER_PD_FAULT_STATUS_ADDR(base)       ((base) + (0x1F))

/* 寄存器说明：PD Command
   位域定义UNION结构:  SOC_SCHARGER_PD_COMMAND_UNION */
#define SOC_SCHARGER_PD_COMMAND_ADDR(base)            ((base) + (0x23))

/* 寄存器说明：PD Device Cap1 Low
   位域定义UNION结构:  SOC_SCHARGER_PD_DEVCAP1L_UNION */
#define SOC_SCHARGER_PD_DEVCAP1L_ADDR(base)           ((base) + (0x24))

/* 寄存器说明：PD Device Cap1 High
   位域定义UNION结构:  SOC_SCHARGER_PD_DEVCAP1H_UNION */
#define SOC_SCHARGER_PD_DEVCAP1H_ADDR(base)           ((base) + (0x25))

/* 寄存器说明：PD Device Cap2 Low
   位域定义UNION结构:  SOC_SCHARGER_PD_DEVCAP2L_UNION */
#define SOC_SCHARGER_PD_DEVCAP2L_ADDR(base)           ((base) + (0x26))

/* 寄存器说明：PD Device Cap2 High
   位域定义UNION结构:  SOC_SCHARGER_PD_DEVCAP2H_UNION */
#define SOC_SCHARGER_PD_DEVCAP2H_ADDR(base)           ((base) + (0x27))

/* 寄存器说明：PD Standard Input Capabilities
   位域定义UNION结构:  SOC_SCHARGER_PD_STDIN_CAP_UNION */
#define SOC_SCHARGER_PD_STDIN_CAP_ADDR(base)          ((base) + (0x28))

/* 寄存器说明：PD Standard Output Capabilities
   位域定义UNION结构:  SOC_SCHARGER_PD_STDOUT_CAP_UNION */
#define SOC_SCHARGER_PD_STDOUT_CAP_ADDR(base)         ((base) + (0x29))

/* 寄存器说明：PD Message Header
   位域定义UNION结构:  SOC_SCHARGER_PD_MSG_HEADR_UNION */
#define SOC_SCHARGER_PD_MSG_HEADR_ADDR(base)          ((base) + (0x2E))

/* 寄存器说明：PD RX Detect
   位域定义UNION结构:  SOC_SCHARGER_PD_RXDETECT_UNION */
#define SOC_SCHARGER_PD_RXDETECT_ADDR(base)           ((base) + (0x2F))

/* 寄存器说明：PD RX ByteCount
   位域定义UNION结构:  SOC_SCHARGER_PD_RXBYTECNT_UNION */
#define SOC_SCHARGER_PD_RXBYTECNT_ADDR(base)          ((base) + (0x30))

/* 寄存器说明：PD RX Type
   位域定义UNION结构:  SOC_SCHARGER_PD_RXTYPE_UNION */
#define SOC_SCHARGER_PD_RXTYPE_ADDR(base)             ((base) + (0x31))

/* 寄存器说明：PD RX Header Low
   位域定义UNION结构:  SOC_SCHARGER_PD_RXHEADL_UNION */
#define SOC_SCHARGER_PD_RXHEADL_ADDR(base)            ((base) + (0x32))

/* 寄存器说明：PD RX Header High
   位域定义UNION结构:  SOC_SCHARGER_PD_RXHEADH_UNION */
#define SOC_SCHARGER_PD_RXHEADH_ADDR(base)            ((base) + (0x33))

/* 寄存器说明：PD RX Data Payload
   位域定义UNION结构:  SOC_SCHARGER_PD_RXDATA_0_UNION */
#define SOC_SCHARGER_PD_RXDATA_0_ADDR(base)           ((base) + (k*1+0x34))

/* 寄存器说明：PD Transmit
   位域定义UNION结构:  SOC_SCHARGER_PD_TRANSMIT_UNION */
#define SOC_SCHARGER_PD_TRANSMIT_ADDR(base)           ((base) + (0x50))

/* 寄存器说明：PD TX Byte Count
   位域定义UNION结构:  SOC_SCHARGER_PD_TXBYTECNT_UNION */
#define SOC_SCHARGER_PD_TXBYTECNT_ADDR(base)          ((base) + (0x51))

/* 寄存器说明：PD TX Header Low
   位域定义UNION结构:  SOC_SCHARGER_PD_TXHEADL_UNION */
#define SOC_SCHARGER_PD_TXHEADL_ADDR(base)            ((base) + (0x52))

/* 寄存器说明：PD TX Header High
   位域定义UNION结构:  SOC_SCHARGER_PD_TXHEADH_UNION */
#define SOC_SCHARGER_PD_TXHEADH_ADDR(base)            ((base) + (0x53))

/* 寄存器说明：PD TX Payload
   位域定义UNION结构:  SOC_SCHARGER_PD_TXDATA_UNION */
#define SOC_SCHARGER_PD_TXDATA_ADDR(base)             ((base) + (k*1+0x54))

/* 寄存器说明：PD VBUS Voltage Low
   位域定义UNION结构:  SOC_SCHARGER_PD_VBUS_VOL_L_UNION */
#define SOC_SCHARGER_PD_VBUS_VOL_L_ADDR(base)         ((base) + (0x70))

/* 寄存器说明：PD VBUS Voltage High
   位域定义UNION结构:  SOC_SCHARGER_PD_VBUS_VOL_H_UNION */
#define SOC_SCHARGER_PD_VBUS_VOL_H_ADDR(base)         ((base) + (0x71))

/* 寄存器说明：VBUS Sink Disconnect Threshold Low
   位域定义UNION结构:  SOC_SCHARGER_PD_VBUS_SNK_DISCL_UNION */
#define SOC_SCHARGER_PD_VBUS_SNK_DISCL_ADDR(base)     ((base) + (0x72))

/* 寄存器说明：VBUS Sink Disconnect Threshold High
   位域定义UNION结构:  SOC_SCHARGER_PD_VBUS_SNK_DISCH_UNION */
#define SOC_SCHARGER_PD_VBUS_SNK_DISCH_ADDR(base)     ((base) + (0x73))

/* 寄存器说明：VBUS Discharge Stop Threshold Low
   位域定义UNION结构:  SOC_SCHARGER_PD_VBUS_STOP_DISCL_UNION */
#define SOC_SCHARGER_PD_VBUS_STOP_DISCL_ADDR(base)    ((base) + (0x74))

/* 寄存器说明：VBUS Discharge Stop Threshold High
   位域定义UNION结构:  SOC_SCHARGER_PD_VBUS_STOP_DISCH_UNION */
#define SOC_SCHARGER_PD_VBUS_STOP_DISCH_ADDR(base)    ((base) + (0x75))

/* 寄存器说明：Voltage High Trip Point Low
   位域定义UNION结构:  SOC_SCHARGER_PD_VALARMH_CFGL_UNION */
#define SOC_SCHARGER_PD_VALARMH_CFGL_ADDR(base)       ((base) + (0x76))

/* 寄存器说明：Voltage High Trip Point High
   位域定义UNION结构:  SOC_SCHARGER_PD_VALARMH_CFGH_UNION */
#define SOC_SCHARGER_PD_VALARMH_CFGH_ADDR(base)       ((base) + (0x77))

/* 寄存器说明：Voltage Low Trip Point Low
   位域定义UNION结构:  SOC_SCHARGER_PD_VALARML_CFGL_UNION */
#define SOC_SCHARGER_PD_VALARML_CFGL_ADDR(base)       ((base) + (0x78))

/* 寄存器说明：Voltage Low Trip Point High
   位域定义UNION结构:  SOC_SCHARGER_PD_VALARML_CFGH_UNION */
#define SOC_SCHARGER_PD_VALARML_CFGH_ADDR(base)       ((base) + (0x79))

/* 寄存器说明：PD自定义配置寄存器
   位域定义UNION结构:  SOC_SCHARGER_PD_VDM_CFG_0_UNION */
#define SOC_SCHARGER_PD_VDM_CFG_0_ADDR(base)          ((base) + (0x7A))

/* 寄存器说明：PD自定义使能寄存器
   位域定义UNION结构:  SOC_SCHARGER_PD_VDM_ENABLE_UNION */
#define SOC_SCHARGER_PD_VDM_ENABLE_ADDR(base)         ((base) + (0x7B))

/* 寄存器说明：PD自定义配置寄存器
   位域定义UNION结构:  SOC_SCHARGER_PD_VDM_CFG_1_UNION */
#define SOC_SCHARGER_PD_VDM_CFG_1_ADDR(base)          ((base) + (0x7C))

/* 寄存器说明：芯片测试用，不对产品开放。测试数据配置寄存器
   位域定义UNION结构:  SOC_SCHARGER_PD_DBG_RDATA_CFG_UNION */
#define SOC_SCHARGER_PD_DBG_RDATA_CFG_ADDR(base)      ((base) + (0x7D))

/* 寄存器说明：芯片测试用，不对产品开放。测试回读数据。
   位域定义UNION结构:  SOC_SCHARGER_PD_DBG_RDATA_UNION */
#define SOC_SCHARGER_PD_DBG_RDATA_ADDR(base)          ((base) + (0x7E))

/* 寄存器说明：Vendor Define Register, Page Select
   位域定义UNION结构:  SOC_SCHARGER_VDM_PAGE_SELECT_UNION */
#define SOC_SCHARGER_VDM_PAGE_SELECT_ADDR(base)       ((base) + (0x7F))



/****************************************************************************
                     (2/4) REG_PAGE0
 ****************************************************************************/
 #define PAGE0_BASE    (0x0080)
/* 寄存器说明：fcp slave 连接状态寄存器。
   位域定义UNION结构:  SOC_SCHARGER_STATUIS_UNION */
#define SOC_SCHARGER_STATUIS_ADDR(base)               ((base) + (0x00) + PAGE0_BASE)

/* 寄存器说明：FCP CNTL配置寄存器。
   位域定义UNION结构:  SOC_SCHARGER_CNTL_UNION */
#define SOC_SCHARGER_CNTL_ADDR(base)                  ((base) + (0x01) + PAGE0_BASE)

/* 寄存器说明：fcp 读写命令配置寄存器。
   位域定义UNION结构:  SOC_SCHARGER_CMD_UNION */
#define SOC_SCHARGER_CMD_ADDR(base)                   ((base) + (0x04) + PAGE0_BASE)

/* 寄存器说明：fcp burst读写长度配置寄存器。
   位域定义UNION结构:  SOC_SCHARGER_FCP_LENGTH_UNION */
#define SOC_SCHARGER_FCP_LENGTH_ADDR(base)            ((base) + (0x05) + PAGE0_BASE)

/* 寄存器说明：fcp 读写地址配置寄存器。
   位域定义UNION结构:  SOC_SCHARGER_ADDR_UNION */
#define SOC_SCHARGER_ADDR_ADDR(base)                  ((base) + (0x07) + PAGE0_BASE)

/* 寄存器说明：fcp 写数据寄存器。
   位域定义UNION结构:  SOC_SCHARGER_DATA0_UNION */
#define SOC_SCHARGER_DATA0_ADDR(base)                 ((base) + (0x08) + PAGE0_BASE)

/* 寄存器说明：fcp 写数据寄存器。
   位域定义UNION结构:  SOC_SCHARGER_DATA1_UNION */
#define SOC_SCHARGER_DATA1_ADDR(base)                 ((base) + (0x09) + PAGE0_BASE)

/* 寄存器说明：fcp 写数据寄存器。
   位域定义UNION结构:  SOC_SCHARGER_DATA2_UNION */
#define SOC_SCHARGER_DATA2_ADDR(base)                 ((base) + (0x0A) + PAGE0_BASE)

/* 寄存器说明：fcp 写数据寄存器。
   位域定义UNION结构:  SOC_SCHARGER_DATA3_UNION */
#define SOC_SCHARGER_DATA3_ADDR(base)                 ((base) + (0x0B) + PAGE0_BASE)

/* 寄存器说明：fcp 写数据寄存器。
   位域定义UNION结构:  SOC_SCHARGER_DATA4_UNION */
#define SOC_SCHARGER_DATA4_ADDR(base)                 ((base) + (0x0C) + PAGE0_BASE)

/* 寄存器说明：fcp 写数据寄存器。
   位域定义UNION结构:  SOC_SCHARGER_DATA5_UNION */
#define SOC_SCHARGER_DATA5_ADDR(base)                 ((base) + (0x0D) + PAGE0_BASE)

/* 寄存器说明：fcp 写数据寄存器。
   位域定义UNION结构:  SOC_SCHARGER_DATA6_UNION */
#define SOC_SCHARGER_DATA6_ADDR(base)                 ((base) + (0x0E) + PAGE0_BASE)

/* 寄存器说明：fcp 写数据寄存器。
   位域定义UNION结构:  SOC_SCHARGER_DATA7_UNION */
#define SOC_SCHARGER_DATA7_ADDR(base)                 ((base) + (0x0F) + PAGE0_BASE)

/* 寄存器说明：slave返回的数据。
   位域定义UNION结构:  SOC_SCHARGER_FCP_RDATA0_UNION */
#define SOC_SCHARGER_FCP_RDATA0_ADDR(base)            ((base) + (0x10) + PAGE0_BASE)

/* 寄存器说明：slave返回的数据。
   位域定义UNION结构:  SOC_SCHARGER_FCP_RDATA1_UNION */
#define SOC_SCHARGER_FCP_RDATA1_ADDR(base)            ((base) + (0x11) + PAGE0_BASE)

/* 寄存器说明：slave返回的数据。
   位域定义UNION结构:  SOC_SCHARGER_FCP_RDATA2_UNION */
#define SOC_SCHARGER_FCP_RDATA2_ADDR(base)            ((base) + (0x12) + PAGE0_BASE)

/* 寄存器说明：slave返回的数据。
   位域定义UNION结构:  SOC_SCHARGER_FCP_RDATA3_UNION */
#define SOC_SCHARGER_FCP_RDATA3_ADDR(base)            ((base) + (0x13) + PAGE0_BASE)

/* 寄存器说明：slave返回的数据。
   位域定义UNION结构:  SOC_SCHARGER_FCP_RDATA4_UNION */
#define SOC_SCHARGER_FCP_RDATA4_ADDR(base)            ((base) + (0x14) + PAGE0_BASE)

/* 寄存器说明：slave返回的数据。
   位域定义UNION结构:  SOC_SCHARGER_FCP_RDATA5_UNION */
#define SOC_SCHARGER_FCP_RDATA5_ADDR(base)            ((base) + (0x15) + PAGE0_BASE)

/* 寄存器说明：slave返回的数据。
   位域定义UNION结构:  SOC_SCHARGER_FCP_RDATA6_UNION */
#define SOC_SCHARGER_FCP_RDATA6_ADDR(base)            ((base) + (0x16) + PAGE0_BASE)

/* 寄存器说明：slave返回的数据。
   位域定义UNION结构:  SOC_SCHARGER_FCP_RDATA7_UNION */
#define SOC_SCHARGER_FCP_RDATA7_ADDR(base)            ((base) + (0x17) + PAGE0_BASE)

/* 寄存器说明：FCP 中断1寄存器。
   位域定义UNION结构:  SOC_SCHARGER_ISR1_UNION */
#define SOC_SCHARGER_ISR1_ADDR(base)                  ((base) + (0x19) + PAGE0_BASE)

/* 寄存器说明：FCP 中断2寄存器。
   位域定义UNION结构:  SOC_SCHARGER_ISR2_UNION */
#define SOC_SCHARGER_ISR2_ADDR(base)                  ((base) + (0x1A) + PAGE0_BASE)

/* 寄存器说明：FCP 中断屏蔽1寄存器。
   位域定义UNION结构:  SOC_SCHARGER_IMR1_UNION */
#define SOC_SCHARGER_IMR1_ADDR(base)                  ((base) + (0x1B) + PAGE0_BASE)

/* 寄存器说明：FCP 中断屏蔽2寄存器。
   位域定义UNION结构:  SOC_SCHARGER_IMR2_UNION */
#define SOC_SCHARGER_IMR2_ADDR(base)                  ((base) + (0x1C) + PAGE0_BASE)

/* 寄存器说明：FCP中断5寄存器。
   位域定义UNION结构:  SOC_SCHARGER_FCP_IRQ5_UNION */
#define SOC_SCHARGER_FCP_IRQ5_ADDR(base)              ((base) + (0x1D) + PAGE0_BASE)

/* 寄存器说明：FCP中断屏蔽5寄存器。
   位域定义UNION结构:  SOC_SCHARGER_FCP_IRQ5_MASK_UNION */
#define SOC_SCHARGER_FCP_IRQ5_MASK_ADDR(base)         ((base) + (0x1E) + PAGE0_BASE)

/* 寄存器说明：高压块充控制寄存器。
   位域定义UNION结构:  SOC_SCHARGER_FCP_CTRL_UNION */
#define SOC_SCHARGER_FCP_CTRL_ADDR(base)              ((base) + (0x1F) + PAGE0_BASE)

/* 寄存器说明：高压快充协议模式2档位控制寄存器。
   位域定义UNION结构:  SOC_SCHARGER_FCP_MODE2_SET_UNION */
#define SOC_SCHARGER_FCP_MODE2_SET_ADDR(base)         ((base) + (0x20) + PAGE0_BASE)

/* 寄存器说明：高压块充Adapter控制寄存器。
   位域定义UNION结构:  SOC_SCHARGER_FCP_ADAP_CTRL_UNION */
#define SOC_SCHARGER_FCP_ADAP_CTRL_ADDR(base)         ((base) + (0x21) + PAGE0_BASE)

/* 寄存器说明：slave返回数据采集好指示信号。
   位域定义UNION结构:  SOC_SCHARGER_RDATA_READY_UNION */
#define SOC_SCHARGER_RDATA_READY_ADDR(base)           ((base) + (0x22) + PAGE0_BASE)

/* 寄存器说明：FCP软复位控制寄存器。
   位域定义UNION结构:  SOC_SCHARGER_FCP_SOFT_RST_CTRL_UNION */
#define SOC_SCHARGER_FCP_SOFT_RST_CTRL_ADDR(base)     ((base) + (0x23) + PAGE0_BASE)

/* 寄存器说明：FCP读数据Parity有误回读寄存器
            说明：不对产品开放
   位域定义UNION结构:  SOC_SCHARGER_FCP_RDATA_PARITY_ERR_UNION */
#define SOC_SCHARGER_FCP_RDATA_PARITY_ERR_ADDR(base)  ((base) + (0x25) + PAGE0_BASE)

/* 寄存器说明：FCP初始化Retry配置寄存器
   位域定义UNION结构:  SOC_SCHARGER_FCP_INIT_RETRY_CFG_UNION */
#define SOC_SCHARGER_FCP_INIT_RETRY_CFG_ADDR(base)    ((base) + (0x26) + PAGE0_BASE)

/* 寄存器说明：HKADC软复位配置寄存器
   位域定义UNION结构:  SOC_SCHARGER_HKADC_SOFT_RST_CTRL_UNION */
#define SOC_SCHARGER_HKADC_SOFT_RST_CTRL_ADDR(base)   ((base) + (0x28) + PAGE0_BASE)

/* 寄存器说明：HKADC配置寄存器
   位域定义UNION结构:  SOC_SCHARGER_HKADC_CTRL0_UNION */
#define SOC_SCHARGER_HKADC_CTRL0_ADDR(base)           ((base) + (0x29) + PAGE0_BASE)

/* 寄存器说明：HKADC START信号配置
   位域定义UNION结构:  SOC_SCHARGER_HKADC_START_UNION */
#define SOC_SCHARGER_HKADC_START_ADDR(base)           ((base) + (0x2A) + PAGE0_BASE)

/* 寄存器说明：HKADC配置寄存器
   位域定义UNION结构:  SOC_SCHARGER_HKADC_CTRL1_UNION */
#define SOC_SCHARGER_HKADC_CTRL1_ADDR(base)           ((base) + (0x2B) + PAGE0_BASE)

/* 寄存器说明：HKADC轮询标志位高8bit（高精度）
   位域定义UNION结构:  SOC_SCHARGER_HKADC_SEQ_CH_H_UNION */
#define SOC_SCHARGER_HKADC_SEQ_CH_H_ADDR(base)        ((base) + (0x2C) + PAGE0_BASE)

/* 寄存器说明：HKADC轮询标志位低8bit（高精度）
   位域定义UNION结构:  SOC_SCHARGER_HKADC_SEQ_CH_L_UNION */
#define SOC_SCHARGER_HKADC_SEQ_CH_L_ADDR(base)        ((base) + (0x2D) + PAGE0_BASE)

/* 寄存器说明：HKADC配置寄存器
   位域定义UNION结构:  SOC_SCHARGER_HKADC_CTRL2_UNION */
#define SOC_SCHARGER_HKADC_CTRL2_ADDR(base)           ((base) + (0x2E) + PAGE0_BASE)

/* 寄存器说明：放电控制寄存器
   位域定义UNION结构:  SOC_SCHARGER_SOH_DISCHG_EN_UNION */
#define SOC_SCHARGER_SOH_DISCHG_EN_ADDR(base)         ((base) + (0x2F) + PAGE0_BASE)

/* 寄存器说明：ACR 配置寄存器
   位域定义UNION结构:  SOC_SCHARGER_ACR_CTRL_UNION */
#define SOC_SCHARGER_ACR_CTRL_ADDR(base)              ((base) + (0x30) + PAGE0_BASE)

/* 寄存器说明：HKADC循环轮询模式数据读请求信号
   位域定义UNION结构:  SOC_SCHARGER_HKADC_RD_SEQ_UNION */
#define SOC_SCHARGER_HKADC_RD_SEQ_ADDR(base)          ((base) + (0x31) + PAGE0_BASE)

/* 寄存器说明：当前ADC采数是处于充电状态还是非充电状态
   位域定义UNION结构:  SOC_SCHARGER_PULSE_NON_CHG_FLAG_UNION */
#define SOC_SCHARGER_PULSE_NON_CHG_FLAG_ADDR(base)    ((base) + (0x32) + PAGE0_BASE)

/* 寄存器说明：VUSB采样结果低8bit
   位域定义UNION结构:  SOC_SCHARGER_VUSB_ADC_L_UNION */
#define SOC_SCHARGER_VUSB_ADC_L_ADDR(base)            ((base) + (0x33) + PAGE0_BASE)

/* 寄存器说明：VUSB采样结果高6bit
   位域定义UNION结构:  SOC_SCHARGER_VUSB_ADC_H_UNION */
#define SOC_SCHARGER_VUSB_ADC_H_ADDR(base)            ((base) + (0x34) + PAGE0_BASE)

/* 寄存器说明：IBUS采样结果低8bit
   位域定义UNION结构:  SOC_SCHARGER_IBUS_ADC_L_UNION */
#define SOC_SCHARGER_IBUS_ADC_L_ADDR(base)            ((base) + (0x35) + PAGE0_BASE)

/* 寄存器说明：IBUS采样结果高5bit
   位域定义UNION结构:  SOC_SCHARGER_IBUS_ADC_H_UNION */
#define SOC_SCHARGER_IBUS_ADC_H_ADDR(base)            ((base) + (0x36) + PAGE0_BASE)

/* 寄存器说明：VBUS采样结果低8bit
   位域定义UNION结构:  SOC_SCHARGER_VBUS_ADC_L_UNION */
#define SOC_SCHARGER_VBUS_ADC_L_ADDR(base)            ((base) + (0x37) + PAGE0_BASE)

/* 寄存器说明：VBUS采样结果高6bit
   位域定义UNION结构:  SOC_SCHARGER_VBUS_ADC_H_UNION */
#define SOC_SCHARGER_VBUS_ADC_H_ADDR(base)            ((base) + (0x38) + PAGE0_BASE)

/* 寄存器说明：VOUT采样结果低8bit
   位域定义UNION结构:  SOC_SCHARGER_VOUT_ADC_L_UNION */
#define SOC_SCHARGER_VOUT_ADC_L_ADDR(base)            ((base) + (0x39) + PAGE0_BASE)

/* 寄存器说明：VOUT采样结果高5bit
   位域定义UNION结构:  SOC_SCHARGER_VOUT_ADC_H_UNION */
#define SOC_SCHARGER_VOUT_ADC_H_ADDR(base)            ((base) + (0x3A) + PAGE0_BASE)

/* 寄存器说明：VBAT采样结果低8bit
   位域定义UNION结构:  SOC_SCHARGER_VBAT_ADC_L_UNION */
#define SOC_SCHARGER_VBAT_ADC_L_ADDR(base)            ((base) + (0x3B) + PAGE0_BASE)

/* 寄存器说明：VBAT采样结果高5bit
   位域定义UNION结构:  SOC_SCHARGER_VBAT_ADC_H_UNION */
#define SOC_SCHARGER_VBAT_ADC_H_ADDR(base)            ((base) + (0x3C) + PAGE0_BASE)

/* 寄存器说明：IBAT采样结果低8bit
   位域定义UNION结构:  SOC_SCHARGER_IBAT_ADC_L_UNION */
#define SOC_SCHARGER_IBAT_ADC_L_ADDR(base)            ((base) + (0x3D) + PAGE0_BASE)

/* 寄存器说明：IBAT采样结果高6bit
   位域定义UNION结构:  SOC_SCHARGER_IBAT_ADC_H_UNION */
#define SOC_SCHARGER_IBAT_ADC_H_ADDR(base)            ((base) + (0x3E) + PAGE0_BASE)

/* 寄存器说明：通道6 ADC转换结果低8bit
   位域定义UNION结构:  SOC_SCHARGER_HKADC_DATA_6_L_UNION */
#define SOC_SCHARGER_HKADC_DATA_6_L_ADDR(base)        ((base) + (0x3F) + PAGE0_BASE)

/* 寄存器说明：通道6 ADC转换结果高4bit
   位域定义UNION结构:  SOC_SCHARGER_HKADC_DATA_6_H_UNION */
#define SOC_SCHARGER_HKADC_DATA_6_H_ADDR(base)        ((base) + (0x40) + PAGE0_BASE)

/* 寄存器说明：通道7 ADC转换结果低8bit
   位域定义UNION结构:  SOC_SCHARGER_HKADC_DATA_7_L_UNION */
#define SOC_SCHARGER_HKADC_DATA_7_L_ADDR(base)        ((base) + (0x41) + PAGE0_BASE)

/* 寄存器说明：通道7 ADC转换结果高4bit
   位域定义UNION结构:  SOC_SCHARGER_HKADC_DATA_7_H_UNION */
#define SOC_SCHARGER_HKADC_DATA_7_H_ADDR(base)        ((base) + (0x42) + PAGE0_BASE)

/* 寄存器说明：通道8 ADC转换结果低8bit
   位域定义UNION结构:  SOC_SCHARGER_HKADC_DATA_8_L_UNION */
#define SOC_SCHARGER_HKADC_DATA_8_L_ADDR(base)        ((base) + (0x43) + PAGE0_BASE)

/* 寄存器说明：通道8 ADC转换结果高4bit
   位域定义UNION结构:  SOC_SCHARGER_HKADC_DATA_8_H_UNION */
#define SOC_SCHARGER_HKADC_DATA_8_H_ADDR(base)        ((base) + (0x44) + PAGE0_BASE)

/* 寄存器说明：通道9 ADC转换结果低8bit
   位域定义UNION结构:  SOC_SCHARGER_HKADC_DATA_9_L_UNION */
#define SOC_SCHARGER_HKADC_DATA_9_L_ADDR(base)        ((base) + (0x45) + PAGE0_BASE)

/* 寄存器说明：通道9 ADC转换结果高4bit
   位域定义UNION结构:  SOC_SCHARGER_HKADC_DATA_9_H_UNION */
#define SOC_SCHARGER_HKADC_DATA_9_H_ADDR(base)        ((base) + (0x46) + PAGE0_BASE)

/* 寄存器说明：通道10 ADC转换结果低8bit
   位域定义UNION结构:  SOC_SCHARGER_HKADC_DATA_10_L_UNION */
#define SOC_SCHARGER_HKADC_DATA_10_L_ADDR(base)       ((base) + (0x47) + PAGE0_BASE)

/* 寄存器说明：通道10 ADC转换结果高4bit
   位域定义UNION结构:  SOC_SCHARGER_HKADC_DATA_10_H_UNION */
#define SOC_SCHARGER_HKADC_DATA_10_H_ADDR(base)       ((base) + (0x48) + PAGE0_BASE)

/* 寄存器说明：通道11 ADC转换结果低8bit
   位域定义UNION结构:  SOC_SCHARGER_HKADC_DATA_11_L_UNION */
#define SOC_SCHARGER_HKADC_DATA_11_L_ADDR(base)       ((base) + (0x49) + PAGE0_BASE)

/* 寄存器说明：通道11 ADC转换结果高4bit
   位域定义UNION结构:  SOC_SCHARGER_HKADC_DATA_11_H_UNION */
#define SOC_SCHARGER_HKADC_DATA_11_H_ADDR(base)       ((base) + (0x4A) + PAGE0_BASE)

/* 寄存器说明：通道12 ADC转换结果低8bit
   位域定义UNION结构:  SOC_SCHARGER_HKADC_DATA_12_L_UNION */
#define SOC_SCHARGER_HKADC_DATA_12_L_ADDR(base)       ((base) + (0x4B) + PAGE0_BASE)

/* 寄存器说明：通道12 ADC转换结果高4bit
   位域定义UNION结构:  SOC_SCHARGER_HKADC_DATA_12_H_UNION */
#define SOC_SCHARGER_HKADC_DATA_12_H_ADDR(base)       ((base) + (0x4C) + PAGE0_BASE)

/* 寄存器说明：通道13 ADC转换结果低8bit
   位域定义UNION结构:  SOC_SCHARGER_HKADC_DATA_13_L_UNION */
#define SOC_SCHARGER_HKADC_DATA_13_L_ADDR(base)       ((base) + (0x4D) + PAGE0_BASE)

/* 寄存器说明：通道13 ADC转换结果高4bit
   位域定义UNION结构:  SOC_SCHARGER_HKADC_DATA_13_H_UNION */
#define SOC_SCHARGER_HKADC_DATA_13_H_ADDR(base)       ((base) + (0x4E) + PAGE0_BASE)

/* 寄存器说明：通道14 ADC转换结果低8bit
   位域定义UNION结构:  SOC_SCHARGER_HKADC_DATA_14_L_UNION */
#define SOC_SCHARGER_HKADC_DATA_14_L_ADDR(base)       ((base) + (0x4F) + PAGE0_BASE)

/* 寄存器说明：通道14 ADC转换结果高4bit
   位域定义UNION结构:  SOC_SCHARGER_HKADC_DATA_14_H_UNION */
#define SOC_SCHARGER_HKADC_DATA_14_H_ADDR(base)       ((base) + (0x50) + PAGE0_BASE)

/* 寄存器说明：通道15 ADC转换结果低8bit
   位域定义UNION结构:  SOC_SCHARGER_HKADC_DATA_15_L_UNION */
#define SOC_SCHARGER_HKADC_DATA_15_L_ADDR(base)       ((base) + (0x51) + PAGE0_BASE)

/* 寄存器说明：通道15 ADC转换结果高4bit
   位域定义UNION结构:  SOC_SCHARGER_HKADC_DATA_15_H_UNION */
#define SOC_SCHARGER_HKADC_DATA_15_H_ADDR(base)       ((base) + (0x52) + PAGE0_BASE)

/* 寄存器说明：PD模块BMC时钟恢复电路配置寄存器
   位域定义UNION结构:  SOC_SCHARGER_PD_CDR_CFG_0_UNION */
#define SOC_SCHARGER_PD_CDR_CFG_0_ADDR(base)          ((base) + (0x58) + PAGE0_BASE)

/* 寄存器说明：PD模块BMC时钟恢复电路配置寄存器
   位域定义UNION结构:  SOC_SCHARGER_PD_CDR_CFG_1_UNION */
#define SOC_SCHARGER_PD_CDR_CFG_1_ADDR(base)          ((base) + (0x59) + PAGE0_BASE)

/* 寄存器说明：PD模块Debug用配置寄存器
   位域定义UNION结构:  SOC_SCHARGER_PD_DBG_CFG_0_UNION */
#define SOC_SCHARGER_PD_DBG_CFG_0_ADDR(base)          ((base) + (0x5A) + PAGE0_BASE)

/* 寄存器说明：PD模块Debug用配置寄存器
   位域定义UNION结构:  SOC_SCHARGER_PD_DBG_CFG_1_UNION */
#define SOC_SCHARGER_PD_DBG_CFG_1_ADDR(base)          ((base) + (0x5B) + PAGE0_BASE)

/* 寄存器说明：PD模块Debug用回读寄存器
   位域定义UNION结构:  SOC_SCHARGER_PD_DBG_RO_0_UNION */
#define SOC_SCHARGER_PD_DBG_RO_0_ADDR(base)           ((base) + (0x5C) + PAGE0_BASE)

/* 寄存器说明：PD模块Debug用回读寄存器
   位域定义UNION结构:  SOC_SCHARGER_PD_DBG_RO_1_UNION */
#define SOC_SCHARGER_PD_DBG_RO_1_ADDR(base)           ((base) + (0x5D) + PAGE0_BASE)

/* 寄存器说明：PD模块Debug用回读寄存器
   位域定义UNION结构:  SOC_SCHARGER_PD_DBG_RO_2_UNION */
#define SOC_SCHARGER_PD_DBG_RO_2_ADDR(base)           ((base) + (0x5E) + PAGE0_BASE)

/* 寄存器说明：PD模块Debug用回读寄存器
   位域定义UNION结构:  SOC_SCHARGER_PD_DBG_RO_3_UNION */
#define SOC_SCHARGER_PD_DBG_RO_3_ADDR(base)           ((base) + (0x5F) + PAGE0_BASE)

/* 寄存器说明：实际中断和伪中断选择信号
   位域定义UNION结构:  SOC_SCHARGER_IRQ_FAKE_SEL_UNION */
#define SOC_SCHARGER_IRQ_FAKE_SEL_ADDR(base)          ((base) + (0x60) + PAGE0_BASE)

/* 寄存器说明：伪中断源
   位域定义UNION结构:  SOC_SCHARGER_IRQ_FAKE_UNION */
#define SOC_SCHARGER_IRQ_FAKE_ADDR(base)              ((base) + (0x61) + PAGE0_BASE)

/* 寄存器说明：
   位域定义UNION结构:  SOC_SCHARGER_IRQ_FLAG_UNION */
#define SOC_SCHARGER_IRQ_FLAG_ADDR(base)              ((base) + (0x62) + PAGE0_BASE)

/* 寄存器说明：
   位域定义UNION结构:  SOC_SCHARGER_IRQ_FLAG_0_UNION */
#define SOC_SCHARGER_IRQ_FLAG_0_ADDR(base)            ((base) + (0x63) + PAGE0_BASE)

/* 寄存器说明：
   位域定义UNION结构:  SOC_SCHARGER_IRQ_FLAG_1_UNION */
#define SOC_SCHARGER_IRQ_FLAG_1_ADDR(base)            ((base) + (0x64) + PAGE0_BASE)

/* 寄存器说明：
   位域定义UNION结构:  SOC_SCHARGER_IRQ_FLAG_2_UNION */
#define SOC_SCHARGER_IRQ_FLAG_2_ADDR(base)            ((base) + (0x65) + PAGE0_BASE)

/* 寄存器说明：
   位域定义UNION结构:  SOC_SCHARGER_IRQ_FLAG_3_UNION */
#define SOC_SCHARGER_IRQ_FLAG_3_ADDR(base)            ((base) + (0x66) + PAGE0_BASE)

/* 寄存器说明：
   位域定义UNION结构:  SOC_SCHARGER_IRQ_FLAG_4_UNION */
#define SOC_SCHARGER_IRQ_FLAG_4_ADDR(base)            ((base) + (0x67) + PAGE0_BASE)

/* 寄存器说明：
   位域定义UNION结构:  SOC_SCHARGER_IRQ_FLAG_5_UNION */
#define SOC_SCHARGER_IRQ_FLAG_5_ADDR(base)            ((base) + (0x68) + PAGE0_BASE)

/* 寄存器说明：
   位域定义UNION结构:  SOC_SCHARGER_IRQ_FLAG_6_UNION */
#define SOC_SCHARGER_IRQ_FLAG_6_ADDR(base)            ((base) + (0x69) + PAGE0_BASE)

/* 寄存器说明：
   位域定义UNION结构:  SOC_SCHARGER_IRQ_FLAG_7_UNION */
#define SOC_SCHARGER_IRQ_FLAG_7_ADDR(base)            ((base) + (0x6A) + PAGE0_BASE)

/* 寄存器说明：看门狗软复位控制寄存器。
   位域定义UNION结构:  SOC_SCHARGER_WDT_SOFT_RST_UNION */
#define SOC_SCHARGER_WDT_SOFT_RST_ADDR(base)          ((base) + (0x6B) + PAGE0_BASE)

/* 寄存器说明：喂狗时间控制寄存器。
   位域定义UNION结构:  SOC_SCHARGER_WDT_CTRL_UNION */
#define SOC_SCHARGER_WDT_CTRL_ADDR(base)              ((base) + (0x6C) + PAGE0_BASE)

/* 寄存器说明：直充IBAT校准档位调节寄存器
   位域定义UNION结构:  SOC_SCHARGER_DC_IBAT_REGULATOR_UNION */
#define SOC_SCHARGER_DC_IBAT_REGULATOR_ADDR(base)     ((base) + (0x6D) + PAGE0_BASE)

/* 寄存器说明：直充VBAT校准档位调节寄存器
   位域定义UNION结构:  SOC_SCHARGER_DC_VBAT_REGULATOR_UNION */
#define SOC_SCHARGER_DC_VBAT_REGULATOR_ADDR(base)     ((base) + (0x6E) + PAGE0_BASE)

/* 寄存器说明：直充VOUT校准档位调节寄存器
   位域定义UNION结构:  SOC_SCHARGER_DC_VOUT_REGULATOR_UNION */
#define SOC_SCHARGER_DC_VOUT_REGULATOR_ADDR(base)     ((base) + (0x6F) + PAGE0_BASE)

/* 寄存器说明：OTG模块配置寄存器
   位域定义UNION结构:  SOC_SCHARGER_OTG_CFG_UNION */
#define SOC_SCHARGER_OTG_CFG_ADDR(base)               ((base) + (0x70) + PAGE0_BASE)

/* 寄存器说明：脉冲充电配置寄存器0
   位域定义UNION结构:  SOC_SCHARGER_PULSE_CHG_CFG0_UNION */
#define SOC_SCHARGER_PULSE_CHG_CFG0_ADDR(base)        ((base) + (0x71) + PAGE0_BASE)

/* 寄存器说明：脉冲充电配置寄存器1
   位域定义UNION结构:  SOC_SCHARGER_PULSE_CHG_CFG1_UNION */
#define SOC_SCHARGER_PULSE_CHG_CFG1_ADDR(base)        ((base) + (0x72) + PAGE0_BASE)

/* 寄存器说明：脉冲模式放电时间配置寄存器
   位域定义UNION结构:  SOC_SCHARGER_DISCHG_TIME_UNION */
#define SOC_SCHARGER_DISCHG_TIME_ADDR(base)           ((base) + (0x73) + PAGE0_BASE)

/* 寄存器说明：数字内部信号状态寄存器
   位域定义UNION结构:  SOC_SCHARGER_DIG_STATUS0_UNION */
#define SOC_SCHARGER_DIG_STATUS0_ADDR(base)           ((base) + (0x74) + PAGE0_BASE)

/* 寄存器说明：测试总线配置寄存器
   位域定义UNION结构:  SOC_SCHARGER_SC_TESTBUS_CFG_UNION */
#define SOC_SCHARGER_SC_TESTBUS_CFG_ADDR(base)        ((base) + (0x76) + PAGE0_BASE)

/* 寄存器说明：测试总线回读寄存器
   位域定义UNION结构:  SOC_SCHARGER_SC_TESTBUS_RDATA_UNION */
#define SOC_SCHARGER_SC_TESTBUS_RDATA_ADDR(base)      ((base) + (0x77) + PAGE0_BASE)

/* 寄存器说明：全局软复位控制寄存器。
   位域定义UNION结构:  SOC_SCHARGER_GLB_SOFT_RST_CTRL_UNION */
#define SOC_SCHARGER_GLB_SOFT_RST_CTRL_ADDR(base)     ((base) + (0x78) + PAGE0_BASE)

/* 寄存器说明：EFUSE软复位控制寄存器。
   位域定义UNION结构:  SOC_SCHARGER_EFUSE_SOFT_RST_CTRL_UNION */
#define SOC_SCHARGER_EFUSE_SOFT_RST_CTRL_ADDR(base)   ((base) + (0x79) + PAGE0_BASE)

/* 寄存器说明：CORE CRG时钟使能配置寄存器
   位域定义UNION结构:  SOC_SCHARGER_SC_CRG_CLK_EN_CTRL_UNION */
#define SOC_SCHARGER_SC_CRG_CLK_EN_CTRL_ADDR(base)    ((base) + (0x7A) + PAGE0_BASE)

/* 寄存器说明：BUCK模式配置寄存器
   位域定义UNION结构:  SOC_SCHARGER_BUCK_MODE_CFG_UNION */
#define SOC_SCHARGER_BUCK_MODE_CFG_ADDR(base)         ((base) + (0x7B) + PAGE0_BASE)

/* 寄存器说明：SCC充电模式配置寄存器
   位域定义UNION结构:  SOC_SCHARGER_SC_MODE_CFG_UNION */
#define SOC_SCHARGER_SC_MODE_CFG_ADDR(base)           ((base) + (0x7C) + PAGE0_BASE)

/* 寄存器说明：LVC充电模式配置寄存器
   位域定义UNION结构:  SOC_SCHARGER_LVC_MODE_CFG_UNION */
#define SOC_SCHARGER_LVC_MODE_CFG_ADDR(base)          ((base) + (0x7D) + PAGE0_BASE)

/* 寄存器说明：BUCK使能配置信号
   位域定义UNION结构:  SOC_SCHARGER_SC_BUCK_ENB_UNION */
#define SOC_SCHARGER_SC_BUCK_ENB_ADDR(base)           ((base) + (0x7E) + PAGE0_BASE)

/* 寄存器说明：OVP模块使能控制
   位域定义UNION结构:  SOC_SCHARGER_SC_OVP_MOS_EN_UNION */
#define SOC_SCHARGER_SC_OVP_MOS_EN_ADDR(base)         ((base) + (0x7F) + PAGE0_BASE)



/****************************************************************************
                     (3/4) REG_PAGE1
 ****************************************************************************/
 #define PAGE1_BASE     (0x0180)
/* 寄存器说明：FCP中断3寄存器。
   位域定义UNION结构:  SOC_SCHARGER_FCP_IRQ3_UNION */
#define SOC_SCHARGER_FCP_IRQ3_ADDR(base)              ((base) + (0x00) + PAGE1_BASE)

/* 寄存器说明：FCP中断4寄存器。
   位域定义UNION结构:  SOC_SCHARGER_FCP_IRQ4_UNION */
#define SOC_SCHARGER_FCP_IRQ4_ADDR(base)              ((base) + (0x01) + PAGE1_BASE)

/* 寄存器说明：FCP中断屏蔽3寄存器。
   位域定义UNION结构:  SOC_SCHARGER_FCP_IRQ3_MASK_UNION */
#define SOC_SCHARGER_FCP_IRQ3_MASK_ADDR(base)         ((base) + (0x02) + PAGE1_BASE)

/* 寄存器说明：FCP中断屏蔽4寄存器。
   位域定义UNION结构:  SOC_SCHARGER_FCP_IRQ4_MASK_UNION */
#define SOC_SCHARGER_FCP_IRQ4_MASK_ADDR(base)         ((base) + (0x03) + PAGE1_BASE)

/* 寄存器说明：状态机状态寄存器。
   位域定义UNION结构:  SOC_SCHARGER_MSTATE_UNION */
#define SOC_SCHARGER_MSTATE_ADDR(base)                ((base) + (0x04) + PAGE1_BASE)

/* 寄存器说明：crc 使能控制寄存器。
   位域定义UNION结构:  SOC_SCHARGER_CRC_ENABLE_UNION */
#define SOC_SCHARGER_CRC_ENABLE_ADDR(base)            ((base) + (0x05) + PAGE1_BASE)

/* 寄存器说明：crc 初始值。
   位域定义UNION结构:  SOC_SCHARGER_CRC_START_VALUE_UNION */
#define SOC_SCHARGER_CRC_START_VALUE_ADDR(base)       ((base) + (0x06) + PAGE1_BASE)

/* 寄存器说明：采样点调整寄存器。
   位域定义UNION结构:  SOC_SCHARGER_SAMPLE_CNT_ADJ_UNION */
#define SOC_SCHARGER_SAMPLE_CNT_ADJ_ADDR(base)        ((base) + (0x07) + PAGE1_BASE)

/* 寄存器说明：RX ping 最小长度高位。
   位域定义UNION结构:  SOC_SCHARGER_RX_PING_WIDTH_MIN_H_UNION */
#define SOC_SCHARGER_RX_PING_WIDTH_MIN_H_ADDR(base)   ((base) + (0x08) + PAGE1_BASE)

/* 寄存器说明：RX ping 最小长度低8位
   位域定义UNION结构:  SOC_SCHARGER_RX_PING_WIDTH_MIN_L_UNION */
#define SOC_SCHARGER_RX_PING_WIDTH_MIN_L_ADDR(base)   ((base) + (0x09) + PAGE1_BASE)

/* 寄存器说明：RX ping 最大长度高位
   位域定义UNION结构:  SOC_SCHARGER_RX_PING_WIDTH_MAX_H_UNION */
#define SOC_SCHARGER_RX_PING_WIDTH_MAX_H_ADDR(base)   ((base) + (0x0A) + PAGE1_BASE)

/* 寄存器说明：RX ping 最大长度低8位。
   位域定义UNION结构:  SOC_SCHARGER_RX_PING_WIDTH_MAX_L_UNION */
#define SOC_SCHARGER_RX_PING_WIDTH_MAX_L_ADDR(base)   ((base) + (0x0B) + PAGE1_BASE)

/* 寄存器说明：数据等待时间寄存器。
   位域定义UNION结构:  SOC_SCHARGER_DATA_WAIT_TIME_UNION */
#define SOC_SCHARGER_DATA_WAIT_TIME_ADDR(base)        ((base) + (0x0C) + PAGE1_BASE)

/* 寄存器说明：数据重新发送次数。
   位域定义UNION结构:  SOC_SCHARGER_RETRY_CNT_UNION */
#define SOC_SCHARGER_RETRY_CNT_ADDR(base)             ((base) + (0x0D) + PAGE1_BASE)

/* 寄存器说明：fcp只读预留寄存器。
   位域定义UNION结构:  SOC_SCHARGER_FCP_RO_RESERVE_UNION */
#define SOC_SCHARGER_FCP_RO_RESERVE_ADDR(base)        ((base) + (0x0E) + PAGE1_BASE)

/* 寄存器说明：FCP debug寄存器0。
   位域定义UNION结构:  SOC_SCHARGER_FCP_DEBUG_REG0_UNION */
#define SOC_SCHARGER_FCP_DEBUG_REG0_ADDR(base)        ((base) + (0x0F) + PAGE1_BASE)

/* 寄存器说明：FCP debug寄存器1。
   位域定义UNION结构:  SOC_SCHARGER_FCP_DEBUG_REG1_UNION */
#define SOC_SCHARGER_FCP_DEBUG_REG1_ADDR(base)        ((base) + (0x10) + PAGE1_BASE)

/* 寄存器说明：FCP debug寄存器2。
   位域定义UNION结构:  SOC_SCHARGER_FCP_DEBUG_REG2_UNION */
#define SOC_SCHARGER_FCP_DEBUG_REG2_ADDR(base)        ((base) + (0x11) + PAGE1_BASE)

/* 寄存器说明：FCP debug寄存器3。
   位域定义UNION结构:  SOC_SCHARGER_FCP_DEBUG_REG3_UNION */
#define SOC_SCHARGER_FCP_DEBUG_REG3_ADDR(base)        ((base) + (0x12) + PAGE1_BASE)

/* 寄存器说明：FCP debug寄存器4。
   位域定义UNION结构:  SOC_SCHARGER_FCP_DEBUG_REG4_UNION */
#define SOC_SCHARGER_FCP_DEBUG_REG4_ADDR(base)        ((base) + (0x13) + PAGE1_BASE)

/* 寄存器说明：ACK前同步头等待微调节寄存器。
   位域定义UNION结构:  SOC_SCHARGER_RX_PACKET_WAIT_ADJUST_UNION */
#define SOC_SCHARGER_RX_PACKET_WAIT_ADJUST_ADDR(base) ((base) + (0x14) + PAGE1_BASE)

/* 寄存器说明：FCP采样微调寄存器
   位域定义UNION结构:  SOC_SCHARGER_SAMPLE_CNT_TINYJUST_UNION */
#define SOC_SCHARGER_SAMPLE_CNT_TINYJUST_ADDR(base)   ((base) + (0x15) + PAGE1_BASE)

/* 寄存器说明：FCP检测RX PING CNT微调寄存器
   位域定义UNION结构:  SOC_SCHARGER_RX_PING_CNT_TINYJUST_UNION */
#define SOC_SCHARGER_RX_PING_CNT_TINYJUST_ADDR(base)  ((base) + (0x16) + PAGE1_BASE)

/* 寄存器说明：SHIFT_CNT最大值配置寄存器
   位域定义UNION结构:  SOC_SCHARGER_SHIFT_CNT_CFG_MAX_UNION */
#define SOC_SCHARGER_SHIFT_CNT_CFG_MAX_ADDR(base)     ((base) + (0x17) + PAGE1_BASE)

/* 寄存器说明：HKADC_配置寄存器_0
   位域定义UNION结构:  SOC_SCHARGER_HKADC_CFG_REG_0_UNION */
#define SOC_SCHARGER_HKADC_CFG_REG_0_ADDR(base)       ((base) + (0x18) + PAGE1_BASE)

/* 寄存器说明：HKADC_配置寄存器_1
   位域定义UNION结构:  SOC_SCHARGER_HKADC_CFG_REG_1_UNION */
#define SOC_SCHARGER_HKADC_CFG_REG_1_ADDR(base)       ((base) + (0x19) + PAGE1_BASE)

/* 寄存器说明：HKADC_配置寄存器_2
   位域定义UNION结构:  SOC_SCHARGER_HKADC_CFG_REG_2_UNION */
#define SOC_SCHARGER_HKADC_CFG_REG_2_ADDR(base)       ((base) + (0x1A) + PAGE1_BASE)

/* 寄存器说明：HKADC 采样0p1校正值
   位域定义UNION结构:  SOC_SCHARGER_HKADC_OFFSET_0P1_UNION */
#define SOC_SCHARGER_HKADC_OFFSET_0P1_ADDR(base)      ((base) + (0x1B) + PAGE1_BASE)

/* 寄存器说明：HKADC 采样2p45校正值
   位域定义UNION结构:  SOC_SCHARGER_HKADC_OFFSET_2P45_UNION */
#define SOC_SCHARGER_HKADC_OFFSET_2P45_ADDR(base)     ((base) + (0x1C) + PAGE1_BASE)

/* 寄存器说明：电池过压保护电压检测高阈值0的低8bit
   位域定义UNION结构:  SOC_SCHARGER_SOH_OVH_TH0_L_UNION */
#define SOC_SCHARGER_SOH_OVH_TH0_L_ADDR(base)         ((base) + (0x1D) + PAGE1_BASE)

/* 寄存器说明：电池过压保护电压检测高阈值0的高4bit
   位域定义UNION结构:  SOC_SCHARGER_SOH_OVH_TH0_H_UNION */
#define SOC_SCHARGER_SOH_OVH_TH0_H_ADDR(base)         ((base) + (0x1E) + PAGE1_BASE)

/* 寄存器说明：电池过压保护温度检测高阈值0的低8bit
   位域定义UNION结构:  SOC_SCHARGER_TSBAT_OVH_TH0_L_UNION */
#define SOC_SCHARGER_TSBAT_OVH_TH0_L_ADDR(base)       ((base) + (0x1F) + PAGE1_BASE)

/* 寄存器说明：电池过压保护温度检测高阈值0的高4bit
   位域定义UNION结构:  SOC_SCHARGER_TSBAT_OVH_TH0_H_UNION */
#define SOC_SCHARGER_TSBAT_OVH_TH0_H_ADDR(base)       ((base) + (0x20) + PAGE1_BASE)

/* 寄存器说明：电池过压保护电压检测高阈值1的低8bit
   位域定义UNION结构:  SOC_SCHARGER_SOH_OVH_TH1_L_UNION */
#define SOC_SCHARGER_SOH_OVH_TH1_L_ADDR(base)         ((base) + (0x21) + PAGE1_BASE)

/* 寄存器说明：电池过压保护电压检测高阈值1的高4bit
   位域定义UNION结构:  SOC_SCHARGER_SOH_OVH_TH1_H_UNION */
#define SOC_SCHARGER_SOH_OVH_TH1_H_ADDR(base)         ((base) + (0x22) + PAGE1_BASE)

/* 寄存器说明：电池过压保护温度检测高阈值1的低8bit
   位域定义UNION结构:  SOC_SCHARGER_TSBAT_OVH_TH1_L_UNION */
#define SOC_SCHARGER_TSBAT_OVH_TH1_L_ADDR(base)       ((base) + (0x23) + PAGE1_BASE)

/* 寄存器说明：电池过压保护温度检测高阈值1的高4bit
   位域定义UNION结构:  SOC_SCHARGER_TSBAT_OVH_TH1_H_UNION */
#define SOC_SCHARGER_TSBAT_OVH_TH1_H_ADDR(base)       ((base) + (0x24) + PAGE1_BASE)

/* 寄存器说明：电池过压保护电压检测高阈值2的低8bit
   位域定义UNION结构:  SOC_SCHARGER_SOH_OVH_TH2_L_UNION */
#define SOC_SCHARGER_SOH_OVH_TH2_L_ADDR(base)         ((base) + (0x25) + PAGE1_BASE)

/* 寄存器说明：电池过压保护电压检测高阈值2的高4bit
   位域定义UNION结构:  SOC_SCHARGER_SOH_OVH_TH2_H_UNION */
#define SOC_SCHARGER_SOH_OVH_TH2_H_ADDR(base)         ((base) + (0x26) + PAGE1_BASE)

/* 寄存器说明：电池过压保护温度检测高阈值2的低8bit
   位域定义UNION结构:  SOC_SCHARGER_TSBAT_OVH_TH2_L_UNION */
#define SOC_SCHARGER_TSBAT_OVH_TH2_L_ADDR(base)       ((base) + (0x27) + PAGE1_BASE)

/* 寄存器说明：电池过压保护温度检测高阈值2的高4bit
   位域定义UNION结构:  SOC_SCHARGER_TSBAT_OVH_TH2_H_UNION */
#define SOC_SCHARGER_TSBAT_OVH_TH2_H_ADDR(base)       ((base) + (0x28) + PAGE1_BASE)

/* 寄存器说明：电池过压保护电压检测低阈值的低8bit
   位域定义UNION结构:  SOC_SCHARGER_SOH_OVL_TH_L_UNION */
#define SOC_SCHARGER_SOH_OVL_TH_L_ADDR(base)          ((base) + (0x29) + PAGE1_BASE)

/* 寄存器说明：电池过压保护电压检测低阈值的高4bit
   位域定义UNION结构:  SOC_SCHARGER_SOH_OVL_TH_H_UNION */
#define SOC_SCHARGER_SOH_OVL_TH_H_ADDR(base)          ((base) + (0x2A) + PAGE1_BASE)

/* 寄存器说明：SOH OVP 高/底阈值检测实时记录
   位域定义UNION结构:  SOC_SCHARGER_SOH_OVP_REAL_UNION */
#define SOC_SCHARGER_SOH_OVP_REAL_ADDR(base)          ((base) + (0x2B) + PAGE1_BASE)

/* 寄存器说明：HKADC Testbus使能及选择信号
   位域定义UNION结构:  SOC_SCHARGER_HKADC_TB_EN_SEL_UNION */
#define SOC_SCHARGER_HKADC_TB_EN_SEL_ADDR(base)       ((base) + (0x2C) + PAGE1_BASE)

/* 寄存器说明：HKADC testbus回读数据
   位域定义UNION结构:  SOC_SCHARGER_HKADC_TB_DATA0_UNION */
#define SOC_SCHARGER_HKADC_TB_DATA0_ADDR(base)        ((base) + (0x2D) + PAGE1_BASE)

/* 寄存器说明：HKADC testbus回读数据
   位域定义UNION结构:  SOC_SCHARGER_HKADC_TB_DATA1_UNION */
#define SOC_SCHARGER_HKADC_TB_DATA1_ADDR(base)        ((base) + (0x2E) + PAGE1_BASE)

/* 寄存器说明：ACR_TOP_配置寄存器_0
   位域定义UNION结构:  SOC_SCHARGER_ACR_TOP_CFG_REG_0_UNION */
#define SOC_SCHARGER_ACR_TOP_CFG_REG_0_ADDR(base)     ((base) + (0x30) + PAGE1_BASE)

/* 寄存器说明：ACR_TOP_配置寄存器_1
   位域定义UNION结构:  SOC_SCHARGER_ACR_TOP_CFG_REG_1_UNION */
#define SOC_SCHARGER_ACR_TOP_CFG_REG_1_ADDR(base)     ((base) + (0x31) + PAGE1_BASE)

/* 寄存器说明：ACR_TOP_配置寄存器_2
   位域定义UNION结构:  SOC_SCHARGER_ACR_TOP_CFG_REG_2_UNION */
#define SOC_SCHARGER_ACR_TOP_CFG_REG_2_ADDR(base)     ((base) + (0x32) + PAGE1_BASE)

/* 寄存器说明：第n+1个acr_pulse高电平期采样时间档位
   位域定义UNION结构:  SOC_SCHARGER_ACR_SAMPLE_TIME_H_UNION */
#define SOC_SCHARGER_ACR_SAMPLE_TIME_H_ADDR(base)     ((base) + (0x33) + PAGE1_BASE)

/* 寄存器说明：第n+1个acr_pulse低电平期采样时间档位
   位域定义UNION结构:  SOC_SCHARGER_ACR_SAMPLE_TIME_L_UNION */
#define SOC_SCHARGER_ACR_SAMPLE_TIME_L_ADDR(base)     ((base) + (0x34) + PAGE1_BASE)

/* 寄存器说明：第n+1个acr_pulse高电平期间第一个采样数据低8bit。
   位域定义UNION结构:  SOC_SCHARGER_ACR_DATA0_L_UNION */
#define SOC_SCHARGER_ACR_DATA0_L_ADDR(base)           ((base) + (0x35) + PAGE1_BASE)

/* 寄存器说明：第n+1个acr_pulse高电平期间第一个采样数据高4bit。
   位域定义UNION结构:  SOC_SCHARGER_ACR_DATA0_H_UNION */
#define SOC_SCHARGER_ACR_DATA0_H_ADDR(base)           ((base) + (0x36) + PAGE1_BASE)

/* 寄存器说明：第n+1个acr_pulse高电平期间第二个采样数据低8bit。
   位域定义UNION结构:  SOC_SCHARGER_ACR_DATA1_L_UNION */
#define SOC_SCHARGER_ACR_DATA1_L_ADDR(base)           ((base) + (0x37) + PAGE1_BASE)

/* 寄存器说明：第n+1个acr_pulse高电平期间第二个采样数据高4bit。
   位域定义UNION结构:  SOC_SCHARGER_ACR_DATA1_H_UNION */
#define SOC_SCHARGER_ACR_DATA1_H_ADDR(base)           ((base) + (0x38) + PAGE1_BASE)

/* 寄存器说明：第n+1个acr_pulse高电平期间第三个采样数据低8bit。
   位域定义UNION结构:  SOC_SCHARGER_ACR_DATA2_L_UNION */
#define SOC_SCHARGER_ACR_DATA2_L_ADDR(base)           ((base) + (0x39) + PAGE1_BASE)

/* 寄存器说明：第n+1个acr_pulse高电平期间第三个采样数据高4bit。
   位域定义UNION结构:  SOC_SCHARGER_ACR_DATA2_H_UNION */
#define SOC_SCHARGER_ACR_DATA2_H_ADDR(base)           ((base) + (0x3A) + PAGE1_BASE)

/* 寄存器说明：第n+1个acr_pulse高电平期间第四个采样数据低8bit。
   位域定义UNION结构:  SOC_SCHARGER_ACR_DATA3_L_UNION */
#define SOC_SCHARGER_ACR_DATA3_L_ADDR(base)           ((base) + (0x3B) + PAGE1_BASE)

/* 寄存器说明：第n+1个acr_pulse高电平期间第四个采样数据高4bit。
   位域定义UNION结构:  SOC_SCHARGER_ACR_DATA3_H_UNION */
#define SOC_SCHARGER_ACR_DATA3_H_ADDR(base)           ((base) + (0x3C) + PAGE1_BASE)

/* 寄存器说明：第n+1个acr_pulse高电平期间第五个采样数据低8bit。
   位域定义UNION结构:  SOC_SCHARGER_ACR_DATA4_L_UNION */
#define SOC_SCHARGER_ACR_DATA4_L_ADDR(base)           ((base) + (0x3D) + PAGE1_BASE)

/* 寄存器说明：第n+1个acr_pulse高电平期间第五个采样数据高4bit。
   位域定义UNION结构:  SOC_SCHARGER_ACR_DATA4_H_UNION */
#define SOC_SCHARGER_ACR_DATA4_H_ADDR(base)           ((base) + (0x3E) + PAGE1_BASE)

/* 寄存器说明：第n+1个acr_pulse高电平期间第六个采样数据低8bit。
   位域定义UNION结构:  SOC_SCHARGER_ACR_DATA5_L_UNION */
#define SOC_SCHARGER_ACR_DATA5_L_ADDR(base)           ((base) + (0x3F) + PAGE1_BASE)

/* 寄存器说明：第n+1个acr_pulse高电平期间第六个采样数据高4bit。
   位域定义UNION结构:  SOC_SCHARGER_ACR_DATA5_H_UNION */
#define SOC_SCHARGER_ACR_DATA5_H_ADDR(base)           ((base) + (0x40) + PAGE1_BASE)

/* 寄存器说明：第n+1个acr_pulse高电平期间第七个采样数据低8bit。
   位域定义UNION结构:  SOC_SCHARGER_ACR_DATA6_L_UNION */
#define SOC_SCHARGER_ACR_DATA6_L_ADDR(base)           ((base) + (0x41) + PAGE1_BASE)

/* 寄存器说明：第n+1个acr_pulse高电平期间第七个采样数据高4bit。
   位域定义UNION结构:  SOC_SCHARGER_ACR_DATA6_H_UNION */
#define SOC_SCHARGER_ACR_DATA6_H_ADDR(base)           ((base) + (0x42) + PAGE1_BASE)

/* 寄存器说明：第n+1个acr_pulse高电平期间第八个采样数据低8bit。
   位域定义UNION结构:  SOC_SCHARGER_ACR_DATA7_L_UNION */
#define SOC_SCHARGER_ACR_DATA7_L_ADDR(base)           ((base) + (0x43) + PAGE1_BASE)

/* 寄存器说明：第n+1个acr_pulse高电平期间第八个采样数据高4bit。
   位域定义UNION结构:  SOC_SCHARGER_ACR_DATA7_H_UNION */
#define SOC_SCHARGER_ACR_DATA7_H_ADDR(base)           ((base) + (0x44) + PAGE1_BASE)

/* 寄存器说明：
   位域定义UNION结构:  SOC_SCHARGER_IRQ_MASK_UNION */
#define SOC_SCHARGER_IRQ_MASK_ADDR(base)              ((base) + (0x48) + PAGE1_BASE)

/* 寄存器说明：
   位域定义UNION结构:  SOC_SCHARGER_IRQ_MASK_0_UNION */
#define SOC_SCHARGER_IRQ_MASK_0_ADDR(base)            ((base) + (0x49) + PAGE1_BASE)

/* 寄存器说明：
   位域定义UNION结构:  SOC_SCHARGER_IRQ_MASK_1_UNION */
#define SOC_SCHARGER_IRQ_MASK_1_ADDR(base)            ((base) + (0x4A) + PAGE1_BASE)

/* 寄存器说明：
   位域定义UNION结构:  SOC_SCHARGER_IRQ_MASK_2_UNION */
#define SOC_SCHARGER_IRQ_MASK_2_ADDR(base)            ((base) + (0x4B) + PAGE1_BASE)

/* 寄存器说明：
   位域定义UNION结构:  SOC_SCHARGER_IRQ_MASK_3_UNION */
#define SOC_SCHARGER_IRQ_MASK_3_ADDR(base)            ((base) + (0x4C) + PAGE1_BASE)

/* 寄存器说明：
   位域定义UNION结构:  SOC_SCHARGER_IRQ_MASK_4_UNION */
#define SOC_SCHARGER_IRQ_MASK_4_ADDR(base)            ((base) + (0x4D) + PAGE1_BASE)

/* 寄存器说明：
   位域定义UNION结构:  SOC_SCHARGER_IRQ_MASK_5_UNION */
#define SOC_SCHARGER_IRQ_MASK_5_ADDR(base)            ((base) + (0x4E) + PAGE1_BASE)

/* 寄存器说明：
   位域定义UNION结构:  SOC_SCHARGER_IRQ_MASK_6_UNION */
#define SOC_SCHARGER_IRQ_MASK_6_ADDR(base)            ((base) + (0x4F) + PAGE1_BASE)

/* 寄存器说明：
   位域定义UNION结构:  SOC_SCHARGER_IRQ_MASK_7_UNION */
#define SOC_SCHARGER_IRQ_MASK_7_ADDR(base)            ((base) + (0x50) + PAGE1_BASE)

/* 寄存器说明：
   位域定义UNION结构:  SOC_SCHARGER_IRQ_STATUS_0_UNION */
#define SOC_SCHARGER_IRQ_STATUS_0_ADDR(base)          ((base) + (0x51) + PAGE1_BASE)

/* 寄存器说明：
   位域定义UNION结构:  SOC_SCHARGER_IRQ_STATUS_1_UNION */
#define SOC_SCHARGER_IRQ_STATUS_1_ADDR(base)          ((base) + (0x52) + PAGE1_BASE)

/* 寄存器说明：
   位域定义UNION结构:  SOC_SCHARGER_IRQ_STATUS_2_UNION */
#define SOC_SCHARGER_IRQ_STATUS_2_ADDR(base)          ((base) + (0x53) + PAGE1_BASE)

/* 寄存器说明：
   位域定义UNION结构:  SOC_SCHARGER_IRQ_STATUS_3_UNION */
#define SOC_SCHARGER_IRQ_STATUS_3_ADDR(base)          ((base) + (0x54) + PAGE1_BASE)

/* 寄存器说明：
   位域定义UNION结构:  SOC_SCHARGER_IRQ_STATUS_4_UNION */
#define SOC_SCHARGER_IRQ_STATUS_4_ADDR(base)          ((base) + (0x55) + PAGE1_BASE)

/* 寄存器说明：
   位域定义UNION结构:  SOC_SCHARGER_IRQ_STATUS_5_UNION */
#define SOC_SCHARGER_IRQ_STATUS_5_ADDR(base)          ((base) + (0x56) + PAGE1_BASE)

/* 寄存器说明：
   位域定义UNION结构:  SOC_SCHARGER_IRQ_STATUS_6_UNION */
#define SOC_SCHARGER_IRQ_STATUS_6_ADDR(base)          ((base) + (0x57) + PAGE1_BASE)

/* 寄存器说明：
   位域定义UNION结构:  SOC_SCHARGER_IRQ_STATUS_7_UNION */
#define SOC_SCHARGER_IRQ_STATUS_7_ADDR(base)          ((base) + (0x58) + PAGE1_BASE)

/* 寄存器说明：
   位域定义UNION结构:  SOC_SCHARGER_IRQ_STATUS_8_UNION */
#define SOC_SCHARGER_IRQ_STATUS_8_ADDR(base)          ((base) + (0x59) + PAGE1_BASE)

/* 寄存器说明：EFUSE选择信号
   位域定义UNION结构:  SOC_SCHARGER_EFUSE_SEL_UNION */
#define SOC_SCHARGER_EFUSE_SEL_ADDR(base)             ((base) + (0x60) + PAGE1_BASE)

/* 寄存器说明：
   位域定义UNION结构:  SOC_SCHARGER_EFUSE_CFG_0_UNION */
#define SOC_SCHARGER_EFUSE_CFG_0_ADDR(base)           ((base) + (0x61) + PAGE1_BASE)

/* 寄存器说明：
   位域定义UNION结构:  SOC_SCHARGER_EFUSE_WE_0_UNION */
#define SOC_SCHARGER_EFUSE_WE_0_ADDR(base)            ((base) + (0x62) + PAGE1_BASE)

/* 寄存器说明：
   位域定义UNION结构:  SOC_SCHARGER_EFUSE_WE_1_UNION */
#define SOC_SCHARGER_EFUSE_WE_1_ADDR(base)            ((base) + (0x63) + PAGE1_BASE)

/* 寄存器说明：
   位域定义UNION结构:  SOC_SCHARGER_EFUSE_WE_2_UNION */
#define SOC_SCHARGER_EFUSE_WE_2_ADDR(base)            ((base) + (0x64) + PAGE1_BASE)

/* 寄存器说明：
   位域定义UNION结构:  SOC_SCHARGER_EFUSE_WE_3_UNION */
#define SOC_SCHARGER_EFUSE_WE_3_ADDR(base)            ((base) + (0x65) + PAGE1_BASE)

/* 寄存器说明：
   位域定义UNION结构:  SOC_SCHARGER_EFUSE_WE_4_UNION */
#define SOC_SCHARGER_EFUSE_WE_4_ADDR(base)            ((base) + (0x66) + PAGE1_BASE)

/* 寄存器说明：
   位域定义UNION结构:  SOC_SCHARGER_EFUSE_WE_5_UNION */
#define SOC_SCHARGER_EFUSE_WE_5_ADDR(base)            ((base) + (0x67) + PAGE1_BASE)

/* 寄存器说明：
   位域定义UNION结构:  SOC_SCHARGER_EFUSE_WE_6_UNION */
#define SOC_SCHARGER_EFUSE_WE_6_ADDR(base)            ((base) + (0x68) + PAGE1_BASE)

/* 寄存器说明：
   位域定义UNION结构:  SOC_SCHARGER_EFUSE_WE_7_UNION */
#define SOC_SCHARGER_EFUSE_WE_7_ADDR(base)            ((base) + (0x69) + PAGE1_BASE)

/* 寄存器说明：
   位域定义UNION结构:  SOC_SCHARGER_EFUSE_PDOB_PRE_ADDR_WE_UNION */
#define SOC_SCHARGER_EFUSE_PDOB_PRE_ADDR_WE_ADDR(base) ((base) + (0x6A) + PAGE1_BASE)

/* 寄存器说明：
   位域定义UNION结构:  SOC_SCHARGER_EFUSE_PDOB_PRE_WDATA_UNION */
#define SOC_SCHARGER_EFUSE_PDOB_PRE_WDATA_ADDR(base)  ((base) + (0x6B) + PAGE1_BASE)

/* 寄存器说明：
   位域定义UNION结构:  SOC_SCHARGER_EFUSE1_TESTBUS_0_UNION */
#define SOC_SCHARGER_EFUSE1_TESTBUS_0_ADDR(base)      ((base) + (0x6C) + PAGE1_BASE)

/* 寄存器说明：
   位域定义UNION结构:  SOC_SCHARGER_EFUSE2_TESTBUS_0_UNION */
#define SOC_SCHARGER_EFUSE2_TESTBUS_0_ADDR(base)      ((base) + (0x6D) + PAGE1_BASE)

/* 寄存器说明：
   位域定义UNION结构:  SOC_SCHARGER_EFUSE3_TESTBUS_0_UNION */
#define SOC_SCHARGER_EFUSE3_TESTBUS_0_ADDR(base)      ((base) + (0x6E) + PAGE1_BASE)

/* 寄存器说明：
   位域定义UNION结构:  SOC_SCHARGER_EFUSE_TESTBUS_SEL_UNION */
#define SOC_SCHARGER_EFUSE_TESTBUS_SEL_ADDR(base)     ((base) + (0x6F) + PAGE1_BASE)

/* 寄存器说明：
   位域定义UNION结构:  SOC_SCHARGER_EFUSE_TESTBUS_CFG_UNION */
#define SOC_SCHARGER_EFUSE_TESTBUS_CFG_ADDR(base)     ((base) + (0x70) + PAGE1_BASE)

/* 寄存器说明：
   位域定义UNION结构:  SOC_SCHARGER_EFUSE1_TESTBUS_RDATA_UNION */
#define SOC_SCHARGER_EFUSE1_TESTBUS_RDATA_ADDR(base)  ((base) + (0x71) + PAGE1_BASE)

/* 寄存器说明：
   位域定义UNION结构:  SOC_SCHARGER_EFUSE2_TESTBUS_RDATA_UNION */
#define SOC_SCHARGER_EFUSE2_TESTBUS_RDATA_ADDR(base)  ((base) + (0x72) + PAGE1_BASE)

/* 寄存器说明：
   位域定义UNION结构:  SOC_SCHARGER_EFUSE3_TESTBUS_RDATA_UNION */
#define SOC_SCHARGER_EFUSE3_TESTBUS_RDATA_ADDR(base)  ((base) + (0x73) + PAGE1_BASE)

/* 寄存器说明：公共模块测试信号配置寄存器
   位域定义UNION结构:  SOC_SCHARGER_GLB_TESTBUS_CFG_UNION */
#define SOC_SCHARGER_GLB_TESTBUS_CFG_ADDR(base)       ((base) + (0x74) + PAGE1_BASE)

/* 寄存器说明：公共模块测试信号
   位域定义UNION结构:  SOC_SCHARGER_GLB_TEST_DATA_UNION */
#define SOC_SCHARGER_GLB_TEST_DATA_ADDR(base)         ((base) + (0x75) + PAGE1_BASE)



/****************************************************************************
                     (4/4) REG_PAGE2
 ****************************************************************************/
 #define PAGE2_BASE  (0x0280)
/* 寄存器说明：CHARGER_配置寄存器_0
   位域定义UNION结构:  SOC_SCHARGER_CHARGER_CFG_REG_0_UNION */
#define SOC_SCHARGER_CHARGER_CFG_REG_0_ADDR(base)     ((base) + (0x00) + PAGE2_BASE)

/* 寄存器说明：CHARGER_配置寄存器_1
   位域定义UNION结构:  SOC_SCHARGER_CHARGER_CFG_REG_1_UNION */
#define SOC_SCHARGER_CHARGER_CFG_REG_1_ADDR(base)     ((base) + (0x01) + PAGE2_BASE)

/* 寄存器说明：CHARGER_配置寄存器_2
   位域定义UNION结构:  SOC_SCHARGER_CHARGER_CFG_REG_2_UNION */
#define SOC_SCHARGER_CHARGER_CFG_REG_2_ADDR(base)     ((base) + (0x02) + PAGE2_BASE)

/* 寄存器说明：CHARGER_配置寄存器_3
   位域定义UNION结构:  SOC_SCHARGER_CHARGER_CFG_REG_3_UNION */
#define SOC_SCHARGER_CHARGER_CFG_REG_3_ADDR(base)     ((base) + (0x03) + PAGE2_BASE)

/* 寄存器说明：CHARGER_配置寄存器_4
   位域定义UNION结构:  SOC_SCHARGER_CHARGER_CFG_REG_4_UNION */
#define SOC_SCHARGER_CHARGER_CFG_REG_4_ADDR(base)     ((base) + (0x04) + PAGE2_BASE)

/* 寄存器说明：CHARGER_配置寄存器_5
   位域定义UNION结构:  SOC_SCHARGER_CHARGER_CFG_REG_5_UNION */
#define SOC_SCHARGER_CHARGER_CFG_REG_5_ADDR(base)     ((base) + (0x05) + PAGE2_BASE)

/* 寄存器说明：CHARGER_配置寄存器_6
   位域定义UNION结构:  SOC_SCHARGER_CHARGER_CFG_REG_6_UNION */
#define SOC_SCHARGER_CHARGER_CFG_REG_6_ADDR(base)     ((base) + (0x06) + PAGE2_BASE)

/* 寄存器说明：CHARGER_配置寄存器_7
   位域定义UNION结构:  SOC_SCHARGER_CHARGER_CFG_REG_7_UNION */
#define SOC_SCHARGER_CHARGER_CFG_REG_7_ADDR(base)     ((base) + (0x07) + PAGE2_BASE)

/* 寄存器说明：CHARGER_配置寄存器_8
   位域定义UNION结构:  SOC_SCHARGER_CHARGER_CFG_REG_8_UNION */
#define SOC_SCHARGER_CHARGER_CFG_REG_8_ADDR(base)     ((base) + (0x08) + PAGE2_BASE)

/* 寄存器说明：CHARGER_配置寄存器_9
   位域定义UNION结构:  SOC_SCHARGER_CHARGER_CFG_REG_9_UNION */
#define SOC_SCHARGER_CHARGER_CFG_REG_9_ADDR(base)     ((base) + (0x09) + PAGE2_BASE)

/* 寄存器说明：CHARGER_只读寄存器_10
   位域定义UNION结构:  SOC_SCHARGER_CHARGER_RO_REG_10_UNION */
#define SOC_SCHARGER_CHARGER_RO_REG_10_ADDR(base)     ((base) + (0x0A) + PAGE2_BASE)

/* 寄存器说明：CHARGER_只读寄存器_11
   位域定义UNION结构:  SOC_SCHARGER_CHARGER_RO_REG_11_UNION */
#define SOC_SCHARGER_CHARGER_RO_REG_11_ADDR(base)     ((base) + (0x0B) + PAGE2_BASE)

/* 寄存器说明：USB_OVP_配置寄存器_0
   位域定义UNION结构:  SOC_SCHARGER_USB_OVP_CFG_REG_0_UNION */
#define SOC_SCHARGER_USB_OVP_CFG_REG_0_ADDR(base)     ((base) + (0x0C) + PAGE2_BASE)

/* 寄存器说明：USB_OVP_配置寄存器_1
   位域定义UNION结构:  SOC_SCHARGER_USB_OVP_CFG_REG_1_UNION */
#define SOC_SCHARGER_USB_OVP_CFG_REG_1_ADDR(base)     ((base) + (0x0D) + PAGE2_BASE)

/* 寄存器说明：TCPC_配置寄存器_1
   位域定义UNION结构:  SOC_SCHARGER_TCPC_CFG_REG_1_UNION */
#define SOC_SCHARGER_TCPC_CFG_REG_1_ADDR(base)        ((base) + (0x0E) + PAGE2_BASE)

/* 寄存器说明：TCPC_配置寄存器_2
   位域定义UNION结构:  SOC_SCHARGER_TCPC_CFG_REG_2_UNION */
#define SOC_SCHARGER_TCPC_CFG_REG_2_ADDR(base)        ((base) + (0x0F) + PAGE2_BASE)

/* 寄存器说明：TCPC_配置寄存器_3
   位域定义UNION结构:  SOC_SCHARGER_TCPC_CFG_REG_3_UNION */
#define SOC_SCHARGER_TCPC_CFG_REG_3_ADDR(base)        ((base) + (0x10) + PAGE2_BASE)

/* 寄存器说明：TCPC_只读寄存器_5
   位域定义UNION结构:  SOC_SCHARGER_TCPC_RO_REG_5_UNION */
#define SOC_SCHARGER_TCPC_RO_REG_5_ADDR(base)         ((base) + (0x11) + PAGE2_BASE)

/* 寄存器说明：SCHG_LOGIC_配置寄存器_0
   位域定义UNION结构:  SOC_SCHARGER_SCHG_LOGIC_CFG_REG_0_UNION */
#define SOC_SCHARGER_SCHG_LOGIC_CFG_REG_0_ADDR(base)  ((base) + (0x12) + PAGE2_BASE)

/* 寄存器说明：SCHG_LOGIC_配置寄存器_1
   位域定义UNION结构:  SOC_SCHARGER_SCHG_LOGIC_CFG_REG_1_UNION */
#define SOC_SCHARGER_SCHG_LOGIC_CFG_REG_1_ADDR(base)  ((base) + (0x13) + PAGE2_BASE)

/* 寄存器说明：SCHG_LOGIC_配置寄存器_2
   位域定义UNION结构:  SOC_SCHARGER_SCHG_LOGIC_CFG_REG_2_UNION */
#define SOC_SCHARGER_SCHG_LOGIC_CFG_REG_2_ADDR(base)  ((base) + (0x14) + PAGE2_BASE)

/* 寄存器说明：SCHG_LOGIC_配置寄存器_3
   位域定义UNION结构:  SOC_SCHARGER_SCHG_LOGIC_CFG_REG_3_UNION */
#define SOC_SCHARGER_SCHG_LOGIC_CFG_REG_3_ADDR(base)  ((base) + (0x15) + PAGE2_BASE)

/* 寄存器说明：SCHG_LOGIC_只读寄存器_4
   位域定义UNION结构:  SOC_SCHARGER_SCHG_LOGIC_RO_REG_4_UNION */
#define SOC_SCHARGER_SCHG_LOGIC_RO_REG_4_ADDR(base)   ((base) + (0x16) + PAGE2_BASE)

/* 寄存器说明：BATFET配置寄存器
   位域定义UNION结构:  SOC_SCHARGER_CHARGER_BATFET_CTRL_UNION */
#define SOC_SCHARGER_CHARGER_BATFET_CTRL_ADDR(base)   ((base) + (0x17) + PAGE2_BASE)

/* 寄存器说明：VBAT LV寄存器
   位域定义UNION结构:  SOC_SCHARGER_VBAT_LV_REG_UNION */
#define SOC_SCHARGER_VBAT_LV_REG_ADDR(base)           ((base) + (0x18) + PAGE2_BASE)

/* 寄存器说明：VDM测试模式下配置寄存器0
   位域定义UNION结构:  SOC_SCHARGER_VDM_CFG_REG_0_UNION */
#define SOC_SCHARGER_VDM_CFG_REG_0_ADDR(base)         ((base) + (0x1A) + PAGE2_BASE)

/* 寄存器说明：VDM测试模式下配置寄存器1
   位域定义UNION结构:  SOC_SCHARGER_VDM_CFG_REG_1_UNION */
#define SOC_SCHARGER_VDM_CFG_REG_1_ADDR(base)         ((base) + (0x1B) + PAGE2_BASE)

/* 寄存器说明：VDM测试模式下配置寄存器2
   位域定义UNION结构:  SOC_SCHARGER_VDM_CFG_REG_2_UNION */
#define SOC_SCHARGER_VDM_CFG_REG_2_ADDR(base)         ((base) + (0x1C) + PAGE2_BASE)

/* 寄存器说明：DET_TOP_配置寄存器_0
   位域定义UNION结构:  SOC_SCHARGER_DET_TOP_CFG_REG_0_UNION */
#define SOC_SCHARGER_DET_TOP_CFG_REG_0_ADDR(base)     ((base) + (0x20) + PAGE2_BASE)

/* 寄存器说明：DET_TOP_配置寄存器_1
   位域定义UNION结构:  SOC_SCHARGER_DET_TOP_CFG_REG_1_UNION */
#define SOC_SCHARGER_DET_TOP_CFG_REG_1_ADDR(base)     ((base) + (0x21) + PAGE2_BASE)

/* 寄存器说明：DET_TOP_配置寄存器_2
   位域定义UNION结构:  SOC_SCHARGER_DET_TOP_CFG_REG_2_UNION */
#define SOC_SCHARGER_DET_TOP_CFG_REG_2_ADDR(base)     ((base) + (0x22) + PAGE2_BASE)

/* 寄存器说明：DET_TOP_配置寄存器_3
   位域定义UNION结构:  SOC_SCHARGER_DET_TOP_CFG_REG_3_UNION */
#define SOC_SCHARGER_DET_TOP_CFG_REG_3_ADDR(base)     ((base) + (0x23) + PAGE2_BASE)

/* 寄存器说明：DET_TOP_配置寄存器_4
   位域定义UNION结构:  SOC_SCHARGER_DET_TOP_CFG_REG_4_UNION */
#define SOC_SCHARGER_DET_TOP_CFG_REG_4_ADDR(base)     ((base) + (0x24) + PAGE2_BASE)

/* 寄存器说明：DET_TOP_配置寄存器_5
   位域定义UNION结构:  SOC_SCHARGER_DET_TOP_CFG_REG_5_UNION */
#define SOC_SCHARGER_DET_TOP_CFG_REG_5_ADDR(base)     ((base) + (0x25) + PAGE2_BASE)

/* 寄存器说明：DET_TOP_配置寄存器_6
   位域定义UNION结构:  SOC_SCHARGER_DET_TOP_CFG_REG_6_UNION */
#define SOC_SCHARGER_DET_TOP_CFG_REG_6_ADDR(base)     ((base) + (0x26) + PAGE2_BASE)

/* 寄存器说明：PSEL_配置寄存器_0
   位域定义UNION结构:  SOC_SCHARGER_PSEL_CFG_REG_0_UNION */
#define SOC_SCHARGER_PSEL_CFG_REG_0_ADDR(base)        ((base) + (0x27) + PAGE2_BASE)

/* 寄存器说明：PSEL_只读寄存器_1
   位域定义UNION结构:  SOC_SCHARGER_PSEL_RO_REG_1_UNION */
#define SOC_SCHARGER_PSEL_RO_REG_1_ADDR(base)         ((base) + (0x28) + PAGE2_BASE)

/* 寄存器说明：REF_TOP_配置寄存器_0
   位域定义UNION结构:  SOC_SCHARGER_REF_TOP_CFG_REG_0_UNION */
#define SOC_SCHARGER_REF_TOP_CFG_REG_0_ADDR(base)     ((base) + (0x29) + PAGE2_BASE)

/* 寄存器说明：REF_TOP_配置寄存器_1
   位域定义UNION结构:  SOC_SCHARGER_REF_TOP_CFG_REG_1_UNION */
#define SOC_SCHARGER_REF_TOP_CFG_REG_1_ADDR(base)     ((base) + (0x2A) + PAGE2_BASE)

/* 寄存器说明：REF_TOP_配置寄存器_2
   位域定义UNION结构:  SOC_SCHARGER_REF_TOP_CFG_REG_2_UNION */
#define SOC_SCHARGER_REF_TOP_CFG_REG_2_ADDR(base)     ((base) + (0x2B) + PAGE2_BASE)

/* 寄存器说明：REF_TOP_配置寄存器_3
   位域定义UNION结构:  SOC_SCHARGER_REF_TOP_CFG_REG_3_UNION */
#define SOC_SCHARGER_REF_TOP_CFG_REG_3_ADDR(base)     ((base) + (0x2C) + PAGE2_BASE)

/* 寄存器说明：DC_TOP_配置寄存器_0
   位域定义UNION结构:  SOC_SCHARGER_DC_TOP_CFG_REG_0_UNION */
#define SOC_SCHARGER_DC_TOP_CFG_REG_0_ADDR(base)      ((base) + (0x30) + PAGE2_BASE)

/* 寄存器说明：DC_TOP_配置寄存器_1
   位域定义UNION结构:  SOC_SCHARGER_DC_TOP_CFG_REG_1_UNION */
#define SOC_SCHARGER_DC_TOP_CFG_REG_1_ADDR(base)      ((base) + (0x31) + PAGE2_BASE)

/* 寄存器说明：DC_TOP_配置寄存器_2
   位域定义UNION结构:  SOC_SCHARGER_DC_TOP_CFG_REG_2_UNION */
#define SOC_SCHARGER_DC_TOP_CFG_REG_2_ADDR(base)      ((base) + (0x32) + PAGE2_BASE)

/* 寄存器说明：DC_TOP_配置寄存器_3
   位域定义UNION结构:  SOC_SCHARGER_DC_TOP_CFG_REG_3_UNION */
#define SOC_SCHARGER_DC_TOP_CFG_REG_3_ADDR(base)      ((base) + (0x33) + PAGE2_BASE)

/* 寄存器说明：DC_TOP_配置寄存器_4
   位域定义UNION结构:  SOC_SCHARGER_DC_TOP_CFG_REG_4_UNION */
#define SOC_SCHARGER_DC_TOP_CFG_REG_4_ADDR(base)      ((base) + (0x34) + PAGE2_BASE)

/* 寄存器说明：DC_TOP_配置寄存器_5
   位域定义UNION结构:  SOC_SCHARGER_DC_TOP_CFG_REG_5_UNION */
#define SOC_SCHARGER_DC_TOP_CFG_REG_5_ADDR(base)      ((base) + (0x35) + PAGE2_BASE)

/* 寄存器说明：DC_TOP_配置寄存器_6
   位域定义UNION结构:  SOC_SCHARGER_DC_TOP_CFG_REG_6_UNION */
#define SOC_SCHARGER_DC_TOP_CFG_REG_6_ADDR(base)      ((base) + (0x36) + PAGE2_BASE)

/* 寄存器说明：DC_TOP_配置寄存器_7
   位域定义UNION结构:  SOC_SCHARGER_DC_TOP_CFG_REG_7_UNION */
#define SOC_SCHARGER_DC_TOP_CFG_REG_7_ADDR(base)      ((base) + (0x37) + PAGE2_BASE)

/* 寄存器说明：DC_TOP_配置寄存器_8
   位域定义UNION结构:  SOC_SCHARGER_DC_TOP_CFG_REG_8_UNION */
#define SOC_SCHARGER_DC_TOP_CFG_REG_8_ADDR(base)      ((base) + (0x38) + PAGE2_BASE)

/* 寄存器说明：DC_TOP_配置寄存器_9
   位域定义UNION结构:  SOC_SCHARGER_DC_TOP_CFG_REG_9_UNION */
#define SOC_SCHARGER_DC_TOP_CFG_REG_9_ADDR(base)      ((base) + (0x39) + PAGE2_BASE)

/* 寄存器说明：DC_TOP_配置寄存器_10
   位域定义UNION结构:  SOC_SCHARGER_DC_TOP_CFG_REG_10_UNION */
#define SOC_SCHARGER_DC_TOP_CFG_REG_10_ADDR(base)     ((base) + (0x3A) + PAGE2_BASE)

/* 寄存器说明：DC_TOP_配置寄存器_11
   位域定义UNION结构:  SOC_SCHARGER_DC_TOP_CFG_REG_11_UNION */
#define SOC_SCHARGER_DC_TOP_CFG_REG_11_ADDR(base)     ((base) + (0x3B) + PAGE2_BASE)

/* 寄存器说明：DC_TOP_配置寄存器_12
   位域定义UNION结构:  SOC_SCHARGER_DC_TOP_CFG_REG_12_UNION */
#define SOC_SCHARGER_DC_TOP_CFG_REG_12_ADDR(base)     ((base) + (0x3C) + PAGE2_BASE)

/* 寄存器说明：DC_TOP_配置寄存器_13
   位域定义UNION结构:  SOC_SCHARGER_DC_TOP_CFG_REG_13_UNION */
#define SOC_SCHARGER_DC_TOP_CFG_REG_13_ADDR(base)     ((base) + (0x3D) + PAGE2_BASE)

/* 寄存器说明：DC_TOP_配置寄存器_14
   位域定义UNION结构:  SOC_SCHARGER_DC_TOP_CFG_REG_14_UNION */
#define SOC_SCHARGER_DC_TOP_CFG_REG_14_ADDR(base)     ((base) + (0x3E) + PAGE2_BASE)

/* 寄存器说明：DC_TOP_配置寄存器_15
   位域定义UNION结构:  SOC_SCHARGER_DC_TOP_CFG_REG_15_UNION */
#define SOC_SCHARGER_DC_TOP_CFG_REG_15_ADDR(base)     ((base) + (0x3F) + PAGE2_BASE)

/* 寄存器说明：DC_TOP_配置寄存器_16
   位域定义UNION结构:  SOC_SCHARGER_DC_TOP_CFG_REG_16_UNION */
#define SOC_SCHARGER_DC_TOP_CFG_REG_16_ADDR(base)     ((base) + (0x40) + PAGE2_BASE)

/* 寄存器说明：DC_TOP_配置寄存器_17
   位域定义UNION结构:  SOC_SCHARGER_DC_TOP_CFG_REG_17_UNION */
#define SOC_SCHARGER_DC_TOP_CFG_REG_17_ADDR(base)     ((base) + (0x41) + PAGE2_BASE)

/* 寄存器说明：DC_TOP_配置寄存器_18
   位域定义UNION结构:  SOC_SCHARGER_DC_TOP_CFG_REG_18_UNION */
#define SOC_SCHARGER_DC_TOP_CFG_REG_18_ADDR(base)     ((base) + (0x42) + PAGE2_BASE)

/* 寄存器说明：DC_TOP_配置寄存器_19
   位域定义UNION结构:  SOC_SCHARGER_DC_TOP_CFG_REG_19_UNION */
#define SOC_SCHARGER_DC_TOP_CFG_REG_19_ADDR(base)     ((base) + (0x43) + PAGE2_BASE)

/* 寄存器说明：DC_TOP_配置寄存器_20
   位域定义UNION结构:  SOC_SCHARGER_DC_TOP_CFG_REG_20_UNION */
#define SOC_SCHARGER_DC_TOP_CFG_REG_20_ADDR(base)     ((base) + (0x44) + PAGE2_BASE)

/* 寄存器说明：DC_TOP_配置寄存器_21
   位域定义UNION结构:  SOC_SCHARGER_DC_TOP_CFG_REG_21_UNION */
#define SOC_SCHARGER_DC_TOP_CFG_REG_21_ADDR(base)     ((base) + (0x45) + PAGE2_BASE)

/* 寄存器说明：DC_TOP_配置寄存器_22
   位域定义UNION结构:  SOC_SCHARGER_DC_TOP_CFG_REG_22_UNION */
#define SOC_SCHARGER_DC_TOP_CFG_REG_22_ADDR(base)     ((base) + (0x46) + PAGE2_BASE)

/* 寄存器说明：DC_TOP_只读寄存器_23
   位域定义UNION结构:  SOC_SCHARGER_DC_TOP_RO_REG_23_UNION */
#define SOC_SCHARGER_DC_TOP_RO_REG_23_ADDR(base)      ((base) + (0x47) + PAGE2_BASE)

/* 寄存器说明：VERSION0_只读寄存器_0
   位域定义UNION结构:  SOC_SCHARGER_VERSION0_RO_REG_0_UNION */
#define SOC_SCHARGER_VERSION0_RO_REG_0_ADDR(base)     ((base) + (0x49) + PAGE2_BASE)

/* 寄存器说明：VERSION1_只读寄存器_0
   位域定义UNION结构:  SOC_SCHARGER_VERSION1_RO_REG_0_UNION */
#define SOC_SCHARGER_VERSION1_RO_REG_0_ADDR(base)     ((base) + (0x4A) + PAGE2_BASE)

/* 寄存器说明：VERSION2_只读寄存器_0
   位域定义UNION结构:  SOC_SCHARGER_VERSION2_RO_REG_0_UNION */
#define SOC_SCHARGER_VERSION2_RO_REG_0_ADDR(base)     ((base) + (0x4B) + PAGE2_BASE)

/* 寄存器说明：VERSION3_只读寄存器_0
   位域定义UNION结构:  SOC_SCHARGER_VERSION3_RO_REG_0_UNION */
#define SOC_SCHARGER_VERSION3_RO_REG_0_ADDR(base)     ((base) + (0x4C) + PAGE2_BASE)

/* 寄存器说明：VERSION4_只读寄存器_0
   位域定义UNION结构:  SOC_SCHARGER_VERSION4_RO_REG_0_UNION */
#define SOC_SCHARGER_VERSION4_RO_REG_0_ADDR(base)     ((base) + (0x4D) + PAGE2_BASE)

/* 寄存器说明：VERSION5_只读寄存器_0
   位域定义UNION结构:  SOC_SCHARGER_VERSION5_RO_REG_0_UNION */
#define SOC_SCHARGER_VERSION5_RO_REG_0_ADDR(base)     ((base) + (0x4E) + PAGE2_BASE)

/* 寄存器说明：BUCK_配置寄存器_0
   位域定义UNION结构:  SOC_SCHARGER_BUCK_CFG_REG_0_UNION */
#define SOC_SCHARGER_BUCK_CFG_REG_0_ADDR(base)        ((base) + (0x50) + PAGE2_BASE)

/* 寄存器说明：BUCK_配置寄存器_1
   位域定义UNION结构:  SOC_SCHARGER_BUCK_CFG_REG_1_UNION */
#define SOC_SCHARGER_BUCK_CFG_REG_1_ADDR(base)        ((base) + (0x51) + PAGE2_BASE)

/* 寄存器说明：BUCK_配置寄存器_2
   位域定义UNION结构:  SOC_SCHARGER_BUCK_CFG_REG_2_UNION */
#define SOC_SCHARGER_BUCK_CFG_REG_2_ADDR(base)        ((base) + (0x52) + PAGE2_BASE)

/* 寄存器说明：BUCK_配置寄存器_3
   位域定义UNION结构:  SOC_SCHARGER_BUCK_CFG_REG_3_UNION */
#define SOC_SCHARGER_BUCK_CFG_REG_3_ADDR(base)        ((base) + (0x53) + PAGE2_BASE)

/* 寄存器说明：BUCK_配置寄存器_4
   位域定义UNION结构:  SOC_SCHARGER_BUCK_CFG_REG_4_UNION */
#define SOC_SCHARGER_BUCK_CFG_REG_4_ADDR(base)        ((base) + (0x54) + PAGE2_BASE)

/* 寄存器说明：BUCK_配置寄存器_5
   位域定义UNION结构:  SOC_SCHARGER_BUCK_CFG_REG_5_UNION */
#define SOC_SCHARGER_BUCK_CFG_REG_5_ADDR(base)        ((base) + (0x55) + PAGE2_BASE)

/* 寄存器说明：BUCK_配置寄存器_6
   位域定义UNION结构:  SOC_SCHARGER_BUCK_CFG_REG_6_UNION */
#define SOC_SCHARGER_BUCK_CFG_REG_6_ADDR(base)        ((base) + (0x56) + PAGE2_BASE)

/* 寄存器说明：BUCK_配置寄存器_7
   位域定义UNION结构:  SOC_SCHARGER_BUCK_CFG_REG_7_UNION */
#define SOC_SCHARGER_BUCK_CFG_REG_7_ADDR(base)        ((base) + (0x57) + PAGE2_BASE)

/* 寄存器说明：BUCK_配置寄存器_8
   位域定义UNION结构:  SOC_SCHARGER_BUCK_CFG_REG_8_UNION */
#define SOC_SCHARGER_BUCK_CFG_REG_8_ADDR(base)        ((base) + (0x58) + PAGE2_BASE)

/* 寄存器说明：BUCK_配置寄存器_9
   位域定义UNION结构:  SOC_SCHARGER_BUCK_CFG_REG_9_UNION */
#define SOC_SCHARGER_BUCK_CFG_REG_9_ADDR(base)        ((base) + (0x59) + PAGE2_BASE)

/* 寄存器说明：BUCK_配置寄存器_10
   位域定义UNION结构:  SOC_SCHARGER_BUCK_CFG_REG_10_UNION */
#define SOC_SCHARGER_BUCK_CFG_REG_10_ADDR(base)       ((base) + (0x5A) + PAGE2_BASE)

/* 寄存器说明：BUCK_配置寄存器_11
   位域定义UNION结构:  SOC_SCHARGER_BUCK_CFG_REG_11_UNION */
#define SOC_SCHARGER_BUCK_CFG_REG_11_ADDR(base)       ((base) + (0x5B) + PAGE2_BASE)

/* 寄存器说明：BUCK_配置寄存器_12
   位域定义UNION结构:  SOC_SCHARGER_BUCK_CFG_REG_12_UNION */
#define SOC_SCHARGER_BUCK_CFG_REG_12_ADDR(base)       ((base) + (0x5C) + PAGE2_BASE)

/* 寄存器说明：BUCK_配置寄存器_13
   位域定义UNION结构:  SOC_SCHARGER_BUCK_CFG_REG_13_UNION */
#define SOC_SCHARGER_BUCK_CFG_REG_13_ADDR(base)       ((base) + (0x5D) + PAGE2_BASE)

/* 寄存器说明：BUCK_配置寄存器_14
   位域定义UNION结构:  SOC_SCHARGER_BUCK_CFG_REG_14_UNION */
#define SOC_SCHARGER_BUCK_CFG_REG_14_ADDR(base)       ((base) + (0x5E) + PAGE2_BASE)

/* 寄存器说明：BUCK_配置寄存器_15
   位域定义UNION结构:  SOC_SCHARGER_BUCK_CFG_REG_15_UNION */
#define SOC_SCHARGER_BUCK_CFG_REG_15_ADDR(base)       ((base) + (0x5F) + PAGE2_BASE)

/* 寄存器说明：BUCK_配置寄存器_16
   位域定义UNION结构:  SOC_SCHARGER_BUCK_CFG_REG_16_UNION */
#define SOC_SCHARGER_BUCK_CFG_REG_16_ADDR(base)       ((base) + (0x60) + PAGE2_BASE)

/* 寄存器说明：BUCK_配置寄存器_17
   位域定义UNION结构:  SOC_SCHARGER_BUCK_CFG_REG_17_UNION */
#define SOC_SCHARGER_BUCK_CFG_REG_17_ADDR(base)       ((base) + (0x61) + PAGE2_BASE)

/* 寄存器说明：BUCK_配置寄存器_18
   位域定义UNION结构:  SOC_SCHARGER_BUCK_CFG_REG_18_UNION */
#define SOC_SCHARGER_BUCK_CFG_REG_18_ADDR(base)       ((base) + (0x62) + PAGE2_BASE)

/* 寄存器说明：BUCK_配置寄存器_19
   位域定义UNION结构:  SOC_SCHARGER_BUCK_CFG_REG_19_UNION */
#define SOC_SCHARGER_BUCK_CFG_REG_19_ADDR(base)       ((base) + (0x63) + PAGE2_BASE)

/* 寄存器说明：BUCK_配置寄存器_20
   位域定义UNION结构:  SOC_SCHARGER_BUCK_CFG_REG_20_UNION */
#define SOC_SCHARGER_BUCK_CFG_REG_20_ADDR(base)       ((base) + (0x64) + PAGE2_BASE)

/* 寄存器说明：BUCK_配置寄存器_21
   位域定义UNION结构:  SOC_SCHARGER_BUCK_CFG_REG_21_UNION */
#define SOC_SCHARGER_BUCK_CFG_REG_21_ADDR(base)       ((base) + (0x65) + PAGE2_BASE)

/* 寄存器说明：BUCK_配置寄存器_22
   位域定义UNION结构:  SOC_SCHARGER_BUCK_CFG_REG_22_UNION */
#define SOC_SCHARGER_BUCK_CFG_REG_22_ADDR(base)       ((base) + (0x66) + PAGE2_BASE)

/* 寄存器说明：BUCK_配置寄存器_23
   位域定义UNION结构:  SOC_SCHARGER_BUCK_CFG_REG_23_UNION */
#define SOC_SCHARGER_BUCK_CFG_REG_23_ADDR(base)       ((base) + (0x67) + PAGE2_BASE)

/* 寄存器说明：BUCK_配置寄存器_24
   位域定义UNION结构:  SOC_SCHARGER_BUCK_CFG_REG_24_UNION */
#define SOC_SCHARGER_BUCK_CFG_REG_24_ADDR(base)       ((base) + (0x68) + PAGE2_BASE)

/* 寄存器说明：BUCK_配置寄存器_25
   位域定义UNION结构:  SOC_SCHARGER_BUCK_CFG_REG_25_UNION */
#define SOC_SCHARGER_BUCK_CFG_REG_25_ADDR(base)       ((base) + (0x69) + PAGE2_BASE)

/* 寄存器说明：BUCK_只读寄存器_26
   位域定义UNION结构:  SOC_SCHARGER_BUCK_RO_REG_26_UNION */
#define SOC_SCHARGER_BUCK_RO_REG_26_ADDR(base)        ((base) + (0x6A) + PAGE2_BASE)

/* 寄存器说明：BUCK_只读寄存器_27
   位域定义UNION结构:  SOC_SCHARGER_BUCK_RO_REG_27_UNION */
#define SOC_SCHARGER_BUCK_RO_REG_27_ADDR(base)        ((base) + (0x6B) + PAGE2_BASE)

/* 寄存器说明：BUCK_只读寄存器_28
   位域定义UNION结构:  SOC_SCHARGER_BUCK_RO_REG_28_UNION */
#define SOC_SCHARGER_BUCK_RO_REG_28_ADDR(base)        ((base) + (0x6C) + PAGE2_BASE)

/* 寄存器说明：OTG_配置寄存器_0
   位域定义UNION结构:  SOC_SCHARGER_OTG_CFG_REG_0_UNION */
#define SOC_SCHARGER_OTG_CFG_REG_0_ADDR(base)         ((base) + (0x6D) + PAGE2_BASE)

/* 寄存器说明：OTG_配置寄存器_1
   位域定义UNION结构:  SOC_SCHARGER_OTG_CFG_REG_1_UNION */
#define SOC_SCHARGER_OTG_CFG_REG_1_ADDR(base)         ((base) + (0x6E) + PAGE2_BASE)

/* 寄存器说明：OTG_配置寄存器_2
   位域定义UNION结构:  SOC_SCHARGER_OTG_CFG_REG_2_UNION */
#define SOC_SCHARGER_OTG_CFG_REG_2_ADDR(base)         ((base) + (0x6F) + PAGE2_BASE)

/* 寄存器说明：OTG_配置寄存器_3
   位域定义UNION结构:  SOC_SCHARGER_OTG_CFG_REG_3_UNION */
#define SOC_SCHARGER_OTG_CFG_REG_3_ADDR(base)         ((base) + (0x70) + PAGE2_BASE)

/* 寄存器说明：OTG_配置寄存器_4
   位域定义UNION结构:  SOC_SCHARGER_OTG_CFG_REG_4_UNION */
#define SOC_SCHARGER_OTG_CFG_REG_4_ADDR(base)         ((base) + (0x71) + PAGE2_BASE)

/* 寄存器说明：OTG_配置寄存器_5
   位域定义UNION结构:  SOC_SCHARGER_OTG_CFG_REG_5_UNION */
#define SOC_SCHARGER_OTG_CFG_REG_5_ADDR(base)         ((base) + (0x72) + PAGE2_BASE)

/* 寄存器说明：OTG_配置寄存器_6
   位域定义UNION结构:  SOC_SCHARGER_OTG_CFG_REG_6_UNION */
#define SOC_SCHARGER_OTG_CFG_REG_6_ADDR(base)         ((base) + (0x73) + PAGE2_BASE)

/* 寄存器说明：OTG_只读寄存器_7
   位域定义UNION结构:  SOC_SCHARGER_OTG_RO_REG_7_UNION */
#define SOC_SCHARGER_OTG_RO_REG_7_ADDR(base)          ((base) + (0x74) + PAGE2_BASE)

/* 寄存器说明：OTG_只读寄存器_8
   位域定义UNION结构:  SOC_SCHARGER_OTG_RO_REG_8_UNION */
#define SOC_SCHARGER_OTG_RO_REG_8_ADDR(base)          ((base) + (0x75) + PAGE2_BASE)

/* 寄存器说明：OTG_只读寄存器_9
   位域定义UNION结构:  SOC_SCHARGER_OTG_RO_REG_9_UNION */
#define SOC_SCHARGER_OTG_RO_REG_9_ADDR(base)          ((base) + (0x76) + PAGE2_BASE)





/*****************************************************************************
  3 枚举定义
*****************************************************************************/



/*****************************************************************************
  4 消息头定义
*****************************************************************************/


/*****************************************************************************
  5 消息定义
*****************************************************************************/



/*****************************************************************************
  6 STRUCT定义
*****************************************************************************/



/*****************************************************************************
  7 UNION定义
*****************************************************************************/

/****************************************************************************
                     (1/4) REG_PD
 ****************************************************************************/
/*****************************************************************************
 结构名    : SOC_SCHARGER_VENDIDL_UNION
 结构说明  : VENDIDL 寄存器结构定义。地址偏移量:0x00，初值:0xD1，宽度:8
 寄存器说明: Vendor ID Low
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  pd_vend_id_l : 8;  /* bit[0-7]: Vendor ID低8比特 */
    } reg;
} SOC_SCHARGER_VENDIDL_UNION;
#endif
#define SOC_SCHARGER_VENDIDL_pd_vend_id_l_START  (0)
#define SOC_SCHARGER_VENDIDL_pd_vend_id_l_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_VENDIDH_UNION
 结构说明  : VENDIDH 寄存器结构定义。地址偏移量:0x01，初值:0x12，宽度:8
 寄存器说明: Vendor ID High
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  pd_vend_id_h : 8;  /* bit[0-7]: Vendor ID高8比特 */
    } reg;
} SOC_SCHARGER_VENDIDH_UNION;
#endif
#define SOC_SCHARGER_VENDIDH_pd_vend_id_h_START  (0)
#define SOC_SCHARGER_VENDIDH_pd_vend_id_h_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_PRODIDL_UNION
 结构说明  : PRODIDL 寄存器结构定义。地址偏移量:0x02，初值:0x26，宽度:8
 寄存器说明: Product ID Low
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  pd_prod_id_l : 8;  /* bit[0-7]: Product ID低8比特 */
    } reg;
} SOC_SCHARGER_PRODIDL_UNION;
#endif
#define SOC_SCHARGER_PRODIDL_pd_prod_id_l_START  (0)
#define SOC_SCHARGER_PRODIDL_pd_prod_id_l_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_PRODIDH_UNION
 结构说明  : PRODIDH 寄存器结构定义。地址偏移量:0x03，初值:0x65，宽度:8
 寄存器说明: Product ID High
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  pd_prod_id_h : 8;  /* bit[0-7]: Product ID高8比特 */
    } reg;
} SOC_SCHARGER_PRODIDH_UNION;
#endif
#define SOC_SCHARGER_PRODIDH_pd_prod_id_h_START  (0)
#define SOC_SCHARGER_PRODIDH_pd_prod_id_h_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_DEVIDL_UNION
 结构说明  : DEVIDL 寄存器结构定义。地址偏移量:0x04，初值:0x00，宽度:8
 寄存器说明: Device ID Low
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  pd_dev_id_l : 8;  /* bit[0-7]: Device ID低8比特 */
    } reg;
} SOC_SCHARGER_DEVIDL_UNION;
#endif
#define SOC_SCHARGER_DEVIDL_pd_dev_id_l_START  (0)
#define SOC_SCHARGER_DEVIDL_pd_dev_id_l_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_DEVIDH_UNION
 结构说明  : DEVIDH 寄存器结构定义。地址偏移量:0x05，初值:0x01，宽度:8
 寄存器说明: Device ID High
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  pd_dev_id_h : 8;  /* bit[0-7]: Device ID高8比特 */
    } reg;
} SOC_SCHARGER_DEVIDH_UNION;
#endif
#define SOC_SCHARGER_DEVIDH_pd_dev_id_h_START  (0)
#define SOC_SCHARGER_DEVIDH_pd_dev_id_h_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_TYPECREVL_UNION
 结构说明  : TYPECREVL 寄存器结构定义。地址偏移量:0x06，初值:0x12，宽度:8
 寄存器说明: USB Type-C Revision Low
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  pd_typc_revision_l : 8;  /* bit[0-7]: Type-C Version低8比特（Reference to SuperSwitch） */
    } reg;
} SOC_SCHARGER_TYPECREVL_UNION;
#endif
#define SOC_SCHARGER_TYPECREVL_pd_typc_revision_l_START  (0)
#define SOC_SCHARGER_TYPECREVL_pd_typc_revision_l_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_TYPECREVH_UNION
 结构说明  : TYPECREVH 寄存器结构定义。地址偏移量:0x07，初值:0x00，宽度:8
 寄存器说明: USB Type-C Revision High
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  pd_typc_revision_h : 8;  /* bit[0-7]: Type-C Version高8比特 */
    } reg;
} SOC_SCHARGER_TYPECREVH_UNION;
#endif
#define SOC_SCHARGER_TYPECREVH_pd_typc_revision_h_START  (0)
#define SOC_SCHARGER_TYPECREVH_pd_typc_revision_h_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_USBPDVER_UNION
 结构说明  : USBPDVER 寄存器结构定义。地址偏移量:0x08，初值:0x10，宽度:8
 寄存器说明: USB PD Version
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  pd_version : 8;  /* bit[0-7]: USB PD Version */
    } reg;
} SOC_SCHARGER_USBPDVER_UNION;
#endif
#define SOC_SCHARGER_USBPDVER_pd_version_START  (0)
#define SOC_SCHARGER_USBPDVER_pd_version_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_USBPDREV_UNION
 结构说明  : USBPDREV 寄存器结构定义。地址偏移量:0x09，初值:0x30，宽度:8
 寄存器说明: USB PD Revision
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  pd_revision : 8;  /* bit[0-7]: USB PD Revision */
    } reg;
} SOC_SCHARGER_USBPDREV_UNION;
#endif
#define SOC_SCHARGER_USBPDREV_pd_revision_START  (0)
#define SOC_SCHARGER_USBPDREV_pd_revision_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_PDIFREVL_UNION
 结构说明  : PDIFREVL 寄存器结构定义。地址偏移量:0x0A，初值:0x10，宽度:8
 寄存器说明: USB PD Interface Revision Low
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  pd_itf_revision_l : 8;  /* bit[0-7]: USB PD Interface Revision低8比特 */
    } reg;
} SOC_SCHARGER_PDIFREVL_UNION;
#endif
#define SOC_SCHARGER_PDIFREVL_pd_itf_revision_l_START  (0)
#define SOC_SCHARGER_PDIFREVL_pd_itf_revision_l_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_PDIFREVH_UNION
 结构说明  : PDIFREVH 寄存器结构定义。地址偏移量:0x0B，初值:0x10，宽度:8
 寄存器说明: USB PD Interface Revision High
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  pd_itf_revision_h : 8;  /* bit[0-7]: USB PD Interface Revision高8比特 */
    } reg;
} SOC_SCHARGER_PDIFREVH_UNION;
#endif
#define SOC_SCHARGER_PDIFREVH_pd_itf_revision_h_START  (0)
#define SOC_SCHARGER_PDIFREVH_pd_itf_revision_h_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_PD_ALERT_L_UNION
 结构说明  : PD_ALERT_L 寄存器结构定义。地址偏移量:0x10，初值:0x00，宽度:8
 寄存器说明: PD Alert Low
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  pd_int_ccstatus    : 1;  /* bit[0-0]: CC1/CC2状态发生变化
                                                             1b:CC Status发生变化中断
                                                             0b:没有中断发生
                                                             注意：这里的CC1/CC2状态发生变化指的是PD_CC_STATUS的pd_cc1_stat/pd_cc2_stat。而pd_look4con与pd_con_result的变化不会触发中断
                                                             默认0 */
        unsigned char  pd_int_port_pwr    : 1;  /* bit[1-1]: Power Status（PD_PWR_STATUS寄存器）发生变化中断
                                                             1b:Power Status发生变化中断
                                                             0b:没有中断发生
                                                             默认0 */
        unsigned char  pd_int_rxstat      : 1;  /* bit[2-2]: RX通道接收到SOP* Message中断。
                                                             1b:RX接收到SOP* Msg中断
                                                             0b:没有中断发生
                                                             默认0 */
        unsigned char  pd_int_rxhardrst   : 1;  /* bit[3-3]: RX通道接收到Hard Reset中断。
                                                             1b:RX通道接收到Hard Reset中断发生
                                                             0b:没有中断发生
                                                             默认0 */
        unsigned char  pd_int_txfail      : 1;  /* bit[4-4]: TX通道发送失败（在CRCReceive时间内没有收到有效的GoodCRC）
                                                             1b:TX通道发送失败中断
                                                             0b:没有中断发生
                                                             默认0 */
        unsigned char  pd_int_txdisc      : 1;  /* bit[5-5]: TX通道发送被丢弃。当物理层正在发送数据；或正在接受数据；或TX成功，失败，丢弃中断没有清零；或RX成功，RX Hard Reset中断没有清零；或发送时候PD_TXBYTECNT为0的时候触发TRANSMIT，则发送数据会被丢弃。

                                                             1b:TX通道发送被丢弃中断
                                                             0b:没有中断发生
                                                             默认0 */
        unsigned char  pd_int_txsucc      : 1;  /* bit[6-6]: TX通道发送成功
                                                             1b:TX通道发送成功中断
                                                             0b:没有中断发生
                                                             默认0 */
        unsigned char  pd_int_vbus_alrm_h : 1;  /* bit[7-7]: Vbus高电压中断
                                                             1b:发生Vbus高电压中断
                                                             0b:没有中断发生
                                                             默认0 */
    } reg;
} SOC_SCHARGER_PD_ALERT_L_UNION;
#endif
#define SOC_SCHARGER_PD_ALERT_L_pd_int_ccstatus_START     (0)
#define SOC_SCHARGER_PD_ALERT_L_pd_int_ccstatus_END       (0)
#define SOC_SCHARGER_PD_ALERT_L_pd_int_port_pwr_START     (1)
#define SOC_SCHARGER_PD_ALERT_L_pd_int_port_pwr_END       (1)
#define SOC_SCHARGER_PD_ALERT_L_pd_int_rxstat_START       (2)
#define SOC_SCHARGER_PD_ALERT_L_pd_int_rxstat_END         (2)
#define SOC_SCHARGER_PD_ALERT_L_pd_int_rxhardrst_START    (3)
#define SOC_SCHARGER_PD_ALERT_L_pd_int_rxhardrst_END      (3)
#define SOC_SCHARGER_PD_ALERT_L_pd_int_txfail_START       (4)
#define SOC_SCHARGER_PD_ALERT_L_pd_int_txfail_END         (4)
#define SOC_SCHARGER_PD_ALERT_L_pd_int_txdisc_START       (5)
#define SOC_SCHARGER_PD_ALERT_L_pd_int_txdisc_END         (5)
#define SOC_SCHARGER_PD_ALERT_L_pd_int_txsucc_START       (6)
#define SOC_SCHARGER_PD_ALERT_L_pd_int_txsucc_END         (6)
#define SOC_SCHARGER_PD_ALERT_L_pd_int_vbus_alrm_h_START  (7)
#define SOC_SCHARGER_PD_ALERT_L_pd_int_vbus_alrm_h_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_PD_ALERT_H_UNION
 结构说明  : PD_ALERT_H 寄存器结构定义。地址偏移量:0x11，初值:0x00，宽度:8
 寄存器说明: PD Alert High
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  pd_int_vbus_alrm_l   : 1;  /* bit[0-0]: Vbus低电压中断
                                                               1b:发生Vbus低电压中断
                                                               0b:没有中断发生
                                                               默认0 */
        unsigned char  pd_int_fault         : 1;  /* bit[1-1]: 有错误发生（Fault需要回读Fault Status寄存器去判断）
                                                               1b:错误发生中断，需要回读PD_FAULT_STATUS寄存器判断错误类型。
                                                               0b:没有中断发生
                                                               默认0 */
        unsigned char  pd_int_rx_full       : 1;  /* bit[2-2]: RX通道溢出中断。
                                                               1b:RX buffer区域写满溢出，溢出后不会回GoodCRC给对端。
                                                               0b:没有中断发生
                                                               默认0
                                                               注意：对该比特进行写1不会清除该比特。需要写PD_ALERT_H寄存器的pd_int_rxstat为1，才能清该比特为0。
                                                               默认0 */
        unsigned char  pd_int_vbus_snk_disc : 1;  /* bit[3-3]: Vbus Sink拔除中断。
                                                               1b:Vbus Sink拔除中断发生
                                                               0b:没有中断发生
                                                               默认0 */
        unsigned char  reserved             : 3;  /* bit[4-6]: reserved */
        unsigned char  pd_int_fr_swap       : 1;  /* bit[7-7]: Fast Role Swap中断
                                                               1b:发生Fast Role Swap中断；
                                                               0b:没有中断发生
                                                               默认0 */
    } reg;
} SOC_SCHARGER_PD_ALERT_H_UNION;
#endif
#define SOC_SCHARGER_PD_ALERT_H_pd_int_vbus_alrm_l_START    (0)
#define SOC_SCHARGER_PD_ALERT_H_pd_int_vbus_alrm_l_END      (0)
#define SOC_SCHARGER_PD_ALERT_H_pd_int_fault_START          (1)
#define SOC_SCHARGER_PD_ALERT_H_pd_int_fault_END            (1)
#define SOC_SCHARGER_PD_ALERT_H_pd_int_rx_full_START        (2)
#define SOC_SCHARGER_PD_ALERT_H_pd_int_rx_full_END          (2)
#define SOC_SCHARGER_PD_ALERT_H_pd_int_vbus_snk_disc_START  (3)
#define SOC_SCHARGER_PD_ALERT_H_pd_int_vbus_snk_disc_END    (3)
#define SOC_SCHARGER_PD_ALERT_H_pd_int_fr_swap_START        (7)
#define SOC_SCHARGER_PD_ALERT_H_pd_int_fr_swap_END          (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_PD_ALERT_MSK_L_UNION
 结构说明  : PD_ALERT_MSK_L 寄存器结构定义。地址偏移量:0x12，初值:0xFF，宽度:8
 寄存器说明: PD Alert Mask Low
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  pd_msk_ccstatus    : 1;  /* bit[0-0]: 1b: Interrupt unmasked
                                                             0b: Interrupt masked
                                                             Default: 1b */
        unsigned char  pd_msk_port_pwr    : 1;  /* bit[1-1]: 1b: Interrupt unmasked
                                                             0b: Interrupt masked
                                                             Default: 1b */
        unsigned char  pd_msk_rxstat      : 1;  /* bit[2-2]: 1b: Interrupt unmasked
                                                             0b: Interrupt masked
                                                             Default: 1b */
        unsigned char  pd_msk_rxhardrst   : 1;  /* bit[3-3]: 1b: Interrupt unmasked
                                                             0b: Interrupt masked
                                                             Default: 1b */
        unsigned char  pd_msk_txfail      : 1;  /* bit[4-4]: 1b: Interrupt unmasked
                                                             0b: Interrupt masked
                                                             Default: 1b */
        unsigned char  pd_msk_txdisc      : 1;  /* bit[5-5]: 1b: Interrupt unmasked
                                                             0b: Interrupt masked
                                                             Default: 1b */
        unsigned char  pd_msk_txsucc      : 1;  /* bit[6-6]: 1b: Interrupt unmasked
                                                             0b: Interrupt masked
                                                             Default: 1b */
        unsigned char  pd_msk_vbus_alrm_h : 1;  /* bit[7-7]: 1b: Interrupt unmasked
                                                             0b: Interrupt masked
                                                             Default: 1b */
    } reg;
} SOC_SCHARGER_PD_ALERT_MSK_L_UNION;
#endif
#define SOC_SCHARGER_PD_ALERT_MSK_L_pd_msk_ccstatus_START     (0)
#define SOC_SCHARGER_PD_ALERT_MSK_L_pd_msk_ccstatus_END       (0)
#define SOC_SCHARGER_PD_ALERT_MSK_L_pd_msk_port_pwr_START     (1)
#define SOC_SCHARGER_PD_ALERT_MSK_L_pd_msk_port_pwr_END       (1)
#define SOC_SCHARGER_PD_ALERT_MSK_L_pd_msk_rxstat_START       (2)
#define SOC_SCHARGER_PD_ALERT_MSK_L_pd_msk_rxstat_END         (2)
#define SOC_SCHARGER_PD_ALERT_MSK_L_pd_msk_rxhardrst_START    (3)
#define SOC_SCHARGER_PD_ALERT_MSK_L_pd_msk_rxhardrst_END      (3)
#define SOC_SCHARGER_PD_ALERT_MSK_L_pd_msk_txfail_START       (4)
#define SOC_SCHARGER_PD_ALERT_MSK_L_pd_msk_txfail_END         (4)
#define SOC_SCHARGER_PD_ALERT_MSK_L_pd_msk_txdisc_START       (5)
#define SOC_SCHARGER_PD_ALERT_MSK_L_pd_msk_txdisc_END         (5)
#define SOC_SCHARGER_PD_ALERT_MSK_L_pd_msk_txsucc_START       (6)
#define SOC_SCHARGER_PD_ALERT_MSK_L_pd_msk_txsucc_END         (6)
#define SOC_SCHARGER_PD_ALERT_MSK_L_pd_msk_vbus_alrm_h_START  (7)
#define SOC_SCHARGER_PD_ALERT_MSK_L_pd_msk_vbus_alrm_h_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_PD_ALERT_MSK_H_UNION
 结构说明  : PD_ALERT_MSK_H 寄存器结构定义。地址偏移量:0x13，初值:0x0F，宽度:8
 寄存器说明: PD Alert Mask High
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  pd_msk_vbus_alrm_l   : 1;  /* bit[0-0]: 1b: Interrupt unmasked
                                                               0b: Interrupt masked
                                                               Default: 1b */
        unsigned char  pd_msk_fault         : 1;  /* bit[1-1]: 1b: Interrupt unmasked
                                                               0b: Interrupt masked
                                                               Default: 1b */
        unsigned char  pd_msk_rx_full       : 1;  /* bit[2-2]: 1b: Interrupt unmasked
                                                               0b: Interrupt masked
                                                               Default: 1b */
        unsigned char  pd_msk_vbus_snk_disc : 1;  /* bit[3-3]: 1b: Interrupt unmasked
                                                               0b: Interrupt masked
                                                               Default: 1b */
        unsigned char  reserved             : 3;  /* bit[4-6]: reserved */
        unsigned char  pd_msk_fr_swap       : 1;  /* bit[7-7]: 1b: Interrupt unmasked
                                                               0b: Interrupt masked
                                                               Default: 0 */
    } reg;
} SOC_SCHARGER_PD_ALERT_MSK_H_UNION;
#endif
#define SOC_SCHARGER_PD_ALERT_MSK_H_pd_msk_vbus_alrm_l_START    (0)
#define SOC_SCHARGER_PD_ALERT_MSK_H_pd_msk_vbus_alrm_l_END      (0)
#define SOC_SCHARGER_PD_ALERT_MSK_H_pd_msk_fault_START          (1)
#define SOC_SCHARGER_PD_ALERT_MSK_H_pd_msk_fault_END            (1)
#define SOC_SCHARGER_PD_ALERT_MSK_H_pd_msk_rx_full_START        (2)
#define SOC_SCHARGER_PD_ALERT_MSK_H_pd_msk_rx_full_END          (2)
#define SOC_SCHARGER_PD_ALERT_MSK_H_pd_msk_vbus_snk_disc_START  (3)
#define SOC_SCHARGER_PD_ALERT_MSK_H_pd_msk_vbus_snk_disc_END    (3)
#define SOC_SCHARGER_PD_ALERT_MSK_H_pd_msk_fr_swap_START        (7)
#define SOC_SCHARGER_PD_ALERT_MSK_H_pd_msk_fr_swap_END          (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_PD_PWRSTAT_MSK_UNION
 结构说明  : PD_PWRSTAT_MSK 寄存器结构定义。地址偏移量:0x14，初值:0xDF，宽度:8
 寄存器说明: PD Power State Mask
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  pd_msk_snkvbus     : 1;  /* bit[0-0]: 1b: Interrupt unmasked
                                                             0b: Interrupt masked
                                                             Default: 1b */
        unsigned char  pd_msk_vconn_vld   : 1;  /* bit[1-1]: 1b: Interrupt unmasked
                                                             0b: Interrupt masked
                                                             Default: 1b */
        unsigned char  pd_msk_vbus_vld    : 1;  /* bit[2-2]: 1b: Interrupt unmasked
                                                             0b: Interrupt masked
                                                             Default: 1b */
        unsigned char  pd_msk_vbus_vld_en : 1;  /* bit[3-3]: 1b: Interrupt unmasked
                                                             0b: Interrupt masked
                                                             Default: 1b */
        unsigned char  pd_msk_src_vbus    : 1;  /* bit[4-4]: 1b: Interrupt unmasked
                                                             0b: Interrupt masked
                                                             Default: 1b */
        unsigned char  reserved           : 1;  /* bit[5-5]: reserved */
        unsigned char  pd_msk_init_stat   : 1;  /* bit[6-6]: 1b: Interrupt unmasked
                                                             0b: Interrupt masked
                                                             Default: 1b */
        unsigned char  pd_msk_debug_acc   : 1;  /* bit[7-7]: 1b: Interrupt unmasked
                                                             0b: Interrupt masked
                                                             Default: 1b */
    } reg;
} SOC_SCHARGER_PD_PWRSTAT_MSK_UNION;
#endif
#define SOC_SCHARGER_PD_PWRSTAT_MSK_pd_msk_snkvbus_START      (0)
#define SOC_SCHARGER_PD_PWRSTAT_MSK_pd_msk_snkvbus_END        (0)
#define SOC_SCHARGER_PD_PWRSTAT_MSK_pd_msk_vconn_vld_START    (1)
#define SOC_SCHARGER_PD_PWRSTAT_MSK_pd_msk_vconn_vld_END      (1)
#define SOC_SCHARGER_PD_PWRSTAT_MSK_pd_msk_vbus_vld_START     (2)
#define SOC_SCHARGER_PD_PWRSTAT_MSK_pd_msk_vbus_vld_END       (2)
#define SOC_SCHARGER_PD_PWRSTAT_MSK_pd_msk_vbus_vld_en_START  (3)
#define SOC_SCHARGER_PD_PWRSTAT_MSK_pd_msk_vbus_vld_en_END    (3)
#define SOC_SCHARGER_PD_PWRSTAT_MSK_pd_msk_src_vbus_START     (4)
#define SOC_SCHARGER_PD_PWRSTAT_MSK_pd_msk_src_vbus_END       (4)
#define SOC_SCHARGER_PD_PWRSTAT_MSK_pd_msk_init_stat_START    (6)
#define SOC_SCHARGER_PD_PWRSTAT_MSK_pd_msk_init_stat_END      (6)
#define SOC_SCHARGER_PD_PWRSTAT_MSK_pd_msk_debug_acc_START    (7)
#define SOC_SCHARGER_PD_PWRSTAT_MSK_pd_msk_debug_acc_END      (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_PD_FAULTSTAT_MSK_UNION
 结构说明  : PD_FAULTSTAT_MSK 寄存器结构定义。地址偏移量:0x15，初值:0xF7，宽度:8
 寄存器说明: PD Fault State Mask
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  pd_msk_i2c_err          : 1;  /* bit[0-0]: 1b: Interrupt unmasked
                                                                  0b: Interrupt masked
                                                                  Default: 1b */
        unsigned char  pd_msk_vconn_ocp        : 1;  /* bit[1-1]: no use */
        unsigned char  pd_msk_vbus_ovp         : 1;  /* bit[2-2]: no use */
        unsigned char  reserved_0              : 1;  /* bit[3-3]: no use */
        unsigned char  pd_msk_force_disch_fail : 1;  /* bit[4-4]: 1b: Interrupt unmasked
                                                                  0b: Interrupt masked
                                                                  Default: 1b */
        unsigned char  pd_msk_auto_disch_fail  : 1;  /* bit[5-5]: 1b: Interrupt unmasked
                                                                  0b: Interrupt masked
                                                                  Default: 1b */
        unsigned char  reserved_1              : 2;  /* bit[6-7]: reserved */
    } reg;
} SOC_SCHARGER_PD_FAULTSTAT_MSK_UNION;
#endif
#define SOC_SCHARGER_PD_FAULTSTAT_MSK_pd_msk_i2c_err_START           (0)
#define SOC_SCHARGER_PD_FAULTSTAT_MSK_pd_msk_i2c_err_END             (0)
#define SOC_SCHARGER_PD_FAULTSTAT_MSK_pd_msk_vconn_ocp_START         (1)
#define SOC_SCHARGER_PD_FAULTSTAT_MSK_pd_msk_vconn_ocp_END           (1)
#define SOC_SCHARGER_PD_FAULTSTAT_MSK_pd_msk_vbus_ovp_START          (2)
#define SOC_SCHARGER_PD_FAULTSTAT_MSK_pd_msk_vbus_ovp_END            (2)
#define SOC_SCHARGER_PD_FAULTSTAT_MSK_pd_msk_force_disch_fail_START  (4)
#define SOC_SCHARGER_PD_FAULTSTAT_MSK_pd_msk_force_disch_fail_END    (4)
#define SOC_SCHARGER_PD_FAULTSTAT_MSK_pd_msk_auto_disch_fail_START   (5)
#define SOC_SCHARGER_PD_FAULTSTAT_MSK_pd_msk_auto_disch_fail_END     (5)


/*****************************************************************************
 结构名    : SOC_SCHARGER_PD_TCPC_CTRL_UNION
 结构说明  : PD_TCPC_CTRL 寄存器结构定义。地址偏移量:0x19，初值:0x00，宽度:8
 寄存器说明: PD TCPC Control
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  pd_orient         : 1;  /* bit[0-0]: CC1/CC2通路选择配置寄存器。每次连接建立时，TCPC上报CC1/CC2连接中断，TCPM均需要滤抖tCCDebounce时间后判断方向重新配置pd_orient（即便方向与原设置一致仍需重新配置）
                                                            0b：选择CC1为通信通路，检测CC1是否有通信发生。当Vconn有效则置于CC2通路；
                                                            1b：选择CC2为通信通路，检测CC2是否有通信发生。当Vconn有效则置于CC1通路；
                                                            默认0 */
        unsigned char  pd_bist_mode      : 1;  /* bit[1-1]: BIST模式配置寄存器。设置为1主要用于USB兼容性测试仪测试PD物理层。当连接断开，TCPM需要写该比特为0。
                                                            0b:正常模式，PD_RXDETECT寄存器所使能检测的SOP* msg会通过中断方式告知TCPM。
                                                            1b：BIST测试模式，PD_RXDETECT寄存器所使能检测的SOP* msg不会通过中断方式告知TCPM。TCPC自身会自动存储收到的msg、校验CRC，回复GoodCRC。其中，不会置位PD_ALERT_L的pd_int_rxstat，但有可能置位PD_ALERT_H的pd_int_rx_full
                                                            默认0 */
        unsigned char  reserved_0        : 2;  /* bit[2-3]: I2C_CLK_STRECTH功能不支持。因此reserved */
        unsigned char  pd_debug_acc_ctrl : 1;  /* bit[4-4]: Debug Accessory控制寄存器
                                                            0b：由TCPC自动控制进入与退出；
                                                            1b：有TCPM控制，当设置为1，若TCPC处于Debug模式，则退出Debug模式；若TCPC不处于Debug模式，则不可进入Debug模式，接收不到VDM命令；
                                                            默认0 */
        unsigned char  reserved_1        : 3;  /* bit[5-7]: reserved */
    } reg;
} SOC_SCHARGER_PD_TCPC_CTRL_UNION;
#endif
#define SOC_SCHARGER_PD_TCPC_CTRL_pd_orient_START          (0)
#define SOC_SCHARGER_PD_TCPC_CTRL_pd_orient_END            (0)
#define SOC_SCHARGER_PD_TCPC_CTRL_pd_bist_mode_START       (1)
#define SOC_SCHARGER_PD_TCPC_CTRL_pd_bist_mode_END         (1)
#define SOC_SCHARGER_PD_TCPC_CTRL_pd_debug_acc_ctrl_START  (4)
#define SOC_SCHARGER_PD_TCPC_CTRL_pd_debug_acc_ctrl_END    (4)


/*****************************************************************************
 结构名    : SOC_SCHARGER_PD_ROLE_CTRL_UNION
 结构说明  : PD_ROLE_CTRL 寄存器结构定义。地址偏移量:0x1A，初值:0x0A，宽度:8
 寄存器说明: PD Role Control
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  pd_cc1_cfg : 2;  /* bit[0-1]: CC1通路上下拉电阻配置寄存器，TCPC上电复位后默认双Rp下拉。
                                                     00b:Ra
                                                     01b:Rp
                                                     10b:Rd
                                                     11b:Open
                                                     默认:10b */
        unsigned char  pd_cc2_cfg : 2;  /* bit[2-3]: CC2通路上下拉电阻配置寄存器，TCPC上电复位后默认双Rp下拉。
                                                     00b:Ra
                                                     01b:Rp
                                                     10b:Rd
                                                     11b:Open
                                                     默认:10b */
        unsigned char  pd_rp_cfg  : 2;  /* bit[4-5]: Rp上拉阻值设置寄存器
                                                     00b：Rp默认
                                                     01b：Rp 1.5A
                                                     10b：Rp 3.0A
                                                     11b：预留，不可配置
                                                     默认00b */
        unsigned char  pd_drp     : 1;  /* bit[6-6]: DRP功能使能配置寄存器
                                                     0b：不使能DRP toggle功能，pd_cc1/cc2_cfg寄存器直接决定上下拉电阻；
                                                     1b：使能DRP功能。使能DRP后，TCPM可以配置pd_cc1/cc2_cfg为双Rp上拉或者双Rd下拉设置初始相位（配置其他值无效），然后写PD_COMMAND寄存器Look4Connection，则TCPC开始以初始相位做toggle。Toggle至检测到有效插入后TCPC上报中断，TCPM读取信息建立连接后需要设置pd_drp为0。
                                                     默认0b */
        unsigned char  reserved   : 1;  /* bit[7-7]: reserved */
    } reg;
} SOC_SCHARGER_PD_ROLE_CTRL_UNION;
#endif
#define SOC_SCHARGER_PD_ROLE_CTRL_pd_cc1_cfg_START  (0)
#define SOC_SCHARGER_PD_ROLE_CTRL_pd_cc1_cfg_END    (1)
#define SOC_SCHARGER_PD_ROLE_CTRL_pd_cc2_cfg_START  (2)
#define SOC_SCHARGER_PD_ROLE_CTRL_pd_cc2_cfg_END    (3)
#define SOC_SCHARGER_PD_ROLE_CTRL_pd_rp_cfg_START   (4)
#define SOC_SCHARGER_PD_ROLE_CTRL_pd_rp_cfg_END     (5)
#define SOC_SCHARGER_PD_ROLE_CTRL_pd_drp_START      (6)
#define SOC_SCHARGER_PD_ROLE_CTRL_pd_drp_END        (6)


/*****************************************************************************
 结构名    : SOC_SCHARGER_PD_FAULT_CTRL_UNION
 结构说明  : PD_FAULT_CTRL 寄存器结构定义。地址偏移量:0x1B，初值:0x00，宽度:8
 寄存器说明: PD Fault Control
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  pd_vconn_ocp_en    : 1;  /* bit[0-0]: No Use。Vconn过流检测电路使能。由于该部分由模拟寄存器独立配置，因此这里没有使用，预留寄存器。 */
        unsigned char  reserved_0         : 2;  /* bit[1-2]: No Use。Vbus OCP/OVP检测电路使能。由于该部分由模拟寄存器独立配置，因此这里没有使用，预留寄存器。 */
        unsigned char  pd_disch_timer_dis : 1;  /* bit[3-3]: Vbus放电Fail检测电路控制寄存器
                                                             0b：使能Vbus放电Fail检测电路计数器；
                                                             1b：不使能Vbus放电Fail检测电路计数器；则PD_FAULT_STATUS寄存器的pd_auto_disch_fail/pd_force_disch_fail不会上报；
                                                             默认0 */
        unsigned char  reserved_1         : 4;  /* bit[4-7]: reserved */
    } reg;
} SOC_SCHARGER_PD_FAULT_CTRL_UNION;
#endif
#define SOC_SCHARGER_PD_FAULT_CTRL_pd_vconn_ocp_en_START     (0)
#define SOC_SCHARGER_PD_FAULT_CTRL_pd_vconn_ocp_en_END       (0)
#define SOC_SCHARGER_PD_FAULT_CTRL_pd_disch_timer_dis_START  (3)
#define SOC_SCHARGER_PD_FAULT_CTRL_pd_disch_timer_dis_END    (3)


/*****************************************************************************
 结构名    : SOC_SCHARGER_PD_PWR_CTRL_UNION
 结构说明  : PD_PWR_CTRL 寄存器结构定义。地址偏移量:0x1C，初值:0x60，宽度:8
 寄存器说明: PD Power Control
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  pd_vconn_en       : 1;  /* bit[0-0]: Vconn使能
                                                            0b：不使能Vconn Source
                                                            1b：使能Vconn Source，并依据PD_TCPC_CTR寄存器的pd_orient比特配置到对应的CC1/CC2通路上
                                                            默认0 */
        unsigned char  reserved_0        : 1;  /* bit[1-1]: No use。Vconn Power功率配置。由于芯片额定只提供1W的Vconn Source，因此没有使用。 */
        unsigned char  pd_force_disch_en : 1;  /* bit[2-2]: VBUS泄放通路使能信号。设置为1可直接对模拟电路使能泻放功能。
                                                            0b: 不使能VBUS泄放电阻
                                                            1b: 使能VBUS泄放电阻
                                                            默认0 */
        unsigned char  pd_bleed_disch_en : 1;  /* bit[3-3]: VBUS Bleed泄放通路使能信号。设置为1可直接对模拟电路使能泻放功能。
                                                            0b: 不使能Bleed VBUS泄放电阻
                                                            1b: 使能Bleed VBUS泄放电阻
                                                            默认0 */
        unsigned char  pd_auto_disch     : 1;  /* bit[4-4]: VBUS在拔出时自动放电使能
                                                            0b: 拔出时不使能自动放电
                                                            1b: 拔出时使能自动放电
                                                            默认0 */
        unsigned char  pd_valarm_dis     : 1;  /* bit[5-5]: VBUS电压高低阈值告警使能配置寄存器
                                                            0b：VBUS电压告警使能；当VBUS电压超过PD_VALARMH，则上报pd_int_vbus_alrm_h中断；当VBUS电压降低过PD_VALARML，则上报pd_int_vbus_alrm_l中断；另外使能前需要设置HKADC采样VBUS电压阈值；
                                                            1b：不使能VBUS电压告警功能。即不上报pd_int_vbus_alrm_h/pd_int_vbus_alrm_l中断；
                                                            默认1； */
        unsigned char  pd_vbus_mon_dis   : 1;  /* bit[6-6]: VBUS电压阈值监测控制信号
                                                            0b:使能VBUS电压阈值检测，可以通过回读VBUS_VOLTAGE读取VBUS电压阈值；
                                                            1b：不使能VBUS电压阈值检测，VBUS_VOLTAGE回读为全0.
                                                            默认1 */
        unsigned char  reserved_1        : 1;  /* bit[7-7]: reserved */
    } reg;
} SOC_SCHARGER_PD_PWR_CTRL_UNION;
#endif
#define SOC_SCHARGER_PD_PWR_CTRL_pd_vconn_en_START        (0)
#define SOC_SCHARGER_PD_PWR_CTRL_pd_vconn_en_END          (0)
#define SOC_SCHARGER_PD_PWR_CTRL_pd_force_disch_en_START  (2)
#define SOC_SCHARGER_PD_PWR_CTRL_pd_force_disch_en_END    (2)
#define SOC_SCHARGER_PD_PWR_CTRL_pd_bleed_disch_en_START  (3)
#define SOC_SCHARGER_PD_PWR_CTRL_pd_bleed_disch_en_END    (3)
#define SOC_SCHARGER_PD_PWR_CTRL_pd_auto_disch_START      (4)
#define SOC_SCHARGER_PD_PWR_CTRL_pd_auto_disch_END        (4)
#define SOC_SCHARGER_PD_PWR_CTRL_pd_valarm_dis_START      (5)
#define SOC_SCHARGER_PD_PWR_CTRL_pd_valarm_dis_END        (5)
#define SOC_SCHARGER_PD_PWR_CTRL_pd_vbus_mon_dis_START    (6)
#define SOC_SCHARGER_PD_PWR_CTRL_pd_vbus_mon_dis_END      (6)


/*****************************************************************************
 结构名    : SOC_SCHARGER_PD_CC_STATUS_UNION
 结构说明  : PD_CC_STATUS 寄存器结构定义。地址偏移量:0x1D，初值:0x10，宽度:8
 寄存器说明: PD CC Status
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  pd_cc1_stat   : 2;  /* bit[0-1]: CC1通路状态，
                                                        当CC1为Rp上拉，则值为：
                                                        00b：SRC.Open（Open, Rp）
                                                        01：SRC.Ra（在vRa最大值以下）
                                                        10b：SRC.Rd（在vRd范围之内）
                                                        11b：预留

                                                        当CC1为Rd下拉，则值为：
                                                        00b：SNK.Open（在vRa最大值以下）
                                                        01b：SNK.Default（在vRd-Connect最小值以上）
                                                        10b：SNK.Power1.5（在vRd-Connect最小值以上检测到Rp 1.5A）
                                                        11b：SNK.Power3.0（在vRd-Connect最小值以上检测到Rp 3.0A）

                                                        另外，设置pd_cc1_cfg为Ra或者Open，则为0；
                                                        在pd_look4con为1时候，该比特状态为0；
                                                        默认00b */
        unsigned char  pd_cc2_stat   : 2;  /* bit[2-3]: CC2通路状态，
                                                        当CC2为Rp上拉，则值为：
                                                        00b：SRC.Open（Open, Rp）
                                                        01：SRC.Ra（在vRa最大值以下）
                                                        10b：SRC.Rd（在vRd范围之内）
                                                        11b：预留

                                                        当CC2为Rd下拉，则值为：
                                                        00b：SNK.Open（在vRa最大值以下）
                                                        01b：SNK.Default（在vRd-Connect最小值以上）
                                                        10b：SNK.Power1.5（在vRd-Connect最小值以上检测到Rp 1.5A）
                                                        11b：SNK.Power3.0（在vRd-Connect最小值以上检测到Rp 3.0A）

                                                        另外，设置pd_cc2_cfg为Ra或者Open，则为0；
                                                        在pd_look4con为1时候，该比特状态为0；
                                                        默认00b */
        unsigned char  pd_con_result : 1;  /* bit[4-4]: 0b：TCPC目前状态为双Rp上拉；
                                                        1b：TCPC目前状态为双Rd下拉；
                                                        默认1 */
        unsigned char  pd_look4con   : 1;  /* bit[5-5]: 0b：TCPC不在检测有效连接
                                                        1b：TCPC正在检测有效连接；
                                                        当pd_look4con发生1到0的变化，意味着检测到潜在的连接。
                                                        默认0 */
        unsigned char  reserved      : 2;  /* bit[6-7]:  */
    } reg;
} SOC_SCHARGER_PD_CC_STATUS_UNION;
#endif
#define SOC_SCHARGER_PD_CC_STATUS_pd_cc1_stat_START    (0)
#define SOC_SCHARGER_PD_CC_STATUS_pd_cc1_stat_END      (1)
#define SOC_SCHARGER_PD_CC_STATUS_pd_cc2_stat_START    (2)
#define SOC_SCHARGER_PD_CC_STATUS_pd_cc2_stat_END      (3)
#define SOC_SCHARGER_PD_CC_STATUS_pd_con_result_START  (4)
#define SOC_SCHARGER_PD_CC_STATUS_pd_con_result_END    (4)
#define SOC_SCHARGER_PD_CC_STATUS_pd_look4con_START    (5)
#define SOC_SCHARGER_PD_CC_STATUS_pd_look4con_END      (5)


/*****************************************************************************
 结构名    : SOC_SCHARGER_PD_PWR_STATUS_UNION
 结构说明  : PD_PWR_STATUS 寄存器结构定义。地址偏移量:0x1E，初值:0x48，宽度:8
 寄存器说明: PD Power Status
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  pd_sinking_vbus        : 1;  /* bit[0-0]: 0b：Sink is Disconned
                                                                 1b：TCPC正在Sinking Vbus到负载；
                                                                 默认0 */
        unsigned char  pd_vconn_present       : 1;  /* bit[1-1]: 0b：Vconn没有在位；
                                                                 1b：Vconn apply到CC1或CC2上；
                                                                 若PD_PWR_CTRL寄存器的pd_vconn_en为0，则该比特为0；
                                                                 默认0 */
        unsigned char  pd_vbus_present        : 1;  /* bit[2-2]: 0b：Vbus不在位；
                                                                 1b：Vbus在位；
                                                                 当Vbus电压小于3.4V则认为Vbus不在位
                                                                 默认0 */
        unsigned char  pd_vbus_pres_detect_en : 1;  /* bit[3-3]: 0b：Vbus在位检测没有使能；
                                                                 1b：Vbus在位检测已经使能；
                                                                 标记TCPC内部Vbus在位检测电路是否使能
                                                                 默认1，使能 */
        unsigned char  pd_source_vbus         : 1;  /* bit[4-4]: 0b：TCPC没有在Source Vbus；
                                                                 1b：TCPC在Source Vbus
                                                                 该比特仅作为TCPC是否在Source Vbus的监测，不做控制。
                                                                 默认0 */
        unsigned char  reserved               : 1;  /* bit[5-5]: Sourcing High Voltage没有使用，预留 */
        unsigned char  pd_tcpc_init_stat      : 1;  /* bit[6-6]: 0b：TCPC已经完成初始化且所有寄存器是配置有效的；
                                                                 1b：TCPC尚在或者尚未完成初始化，寄存器仅保证00h~0Fh的回读值正确；
                                                                 默认1 */
        unsigned char  pd_debug_acc_connect   : 1;  /* bit[7-7]: 0b：没有Debug Accessory连接；
                                                                 1b：有Debug Accessory连接，本芯片在Debug模式下仅作为Debug.Snk，不作为Debug.Src。
                                                                 默认0 */
    } reg;
} SOC_SCHARGER_PD_PWR_STATUS_UNION;
#endif
#define SOC_SCHARGER_PD_PWR_STATUS_pd_sinking_vbus_START         (0)
#define SOC_SCHARGER_PD_PWR_STATUS_pd_sinking_vbus_END           (0)
#define SOC_SCHARGER_PD_PWR_STATUS_pd_vconn_present_START        (1)
#define SOC_SCHARGER_PD_PWR_STATUS_pd_vconn_present_END          (1)
#define SOC_SCHARGER_PD_PWR_STATUS_pd_vbus_present_START         (2)
#define SOC_SCHARGER_PD_PWR_STATUS_pd_vbus_present_END           (2)
#define SOC_SCHARGER_PD_PWR_STATUS_pd_vbus_pres_detect_en_START  (3)
#define SOC_SCHARGER_PD_PWR_STATUS_pd_vbus_pres_detect_en_END    (3)
#define SOC_SCHARGER_PD_PWR_STATUS_pd_source_vbus_START          (4)
#define SOC_SCHARGER_PD_PWR_STATUS_pd_source_vbus_END            (4)
#define SOC_SCHARGER_PD_PWR_STATUS_pd_tcpc_init_stat_START       (6)
#define SOC_SCHARGER_PD_PWR_STATUS_pd_tcpc_init_stat_END         (6)
#define SOC_SCHARGER_PD_PWR_STATUS_pd_debug_acc_connect_START    (7)
#define SOC_SCHARGER_PD_PWR_STATUS_pd_debug_acc_connect_END      (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_PD_FAULT_STATUS_UNION
 结构说明  : PD_FAULT_STATUS 寄存器结构定义。地址偏移量:0x1F，初值:0x80，宽度:8
 寄存器说明: PD Fault Status
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  pd_i2c_err           : 1;  /* bit[0-0]: I2C总线Error
                                                               0b：没有错误发生；
                                                               1b：I2C配置错误发生；
                                                               当配置Transmit触发发送SOP*类型msg的时候，若TX buffer cnt小于2，则上报I2C Error
                                                               默认0 */
        unsigned char  pd_vconn_ocp         : 1;  /* bit[1-1]: VCONN OCP由系统中断上报，这里不上报。 */
        unsigned char  pd_vusb_ovp          : 1;  /* bit[2-2]: VBUS OVP由系统中断上报，这里不上报。 */
        unsigned char  reserved_0           : 1;  /* bit[3-3]: VBUS OCP no use */
        unsigned char  pd_force_disch_fail  : 1;  /* bit[4-4]: 0b：force放电没有失败；
                                                               1b：force放电失败。当PD_PWR_CTRL中的pd_force_disch_en设置为1且pd_disch_timer_dis设置为0，则在tSave0v（250ms）内VBUS电压没有下降到vSafe0v，则上报自动放电失败。 */
        unsigned char  pd_auto_disch_fail   : 1;  /* bit[5-5]: 0b：自动放电没有失败；
                                                               1b：自动放电失败。当PD_PWR_CTRL中的pd_auto_disch设置为1，且pd_disch_timer_dis设置为0，则在拔出时候，如果在tSave0v（250ms）内VBUS电压没有下降到vSafe0v，则上报自动放电失败。
                                                               默认0 */
        unsigned char  reserved_1           : 1;  /* bit[6-6]: 原协议为Force Off Vbus，本芯片不支持。 */
        unsigned char  pd_reg_reset_default : 1;  /* bit[7-7]: 当TCPC芯片所有与PD协议相关的寄存器被复位至默认值，该比特置1。否则为0。不可屏蔽。这会在power up初始化或者异常上电复位的时候发生。 */
    } reg;
} SOC_SCHARGER_PD_FAULT_STATUS_UNION;
#endif
#define SOC_SCHARGER_PD_FAULT_STATUS_pd_i2c_err_START            (0)
#define SOC_SCHARGER_PD_FAULT_STATUS_pd_i2c_err_END              (0)
#define SOC_SCHARGER_PD_FAULT_STATUS_pd_vconn_ocp_START          (1)
#define SOC_SCHARGER_PD_FAULT_STATUS_pd_vconn_ocp_END            (1)
#define SOC_SCHARGER_PD_FAULT_STATUS_pd_vusb_ovp_START           (2)
#define SOC_SCHARGER_PD_FAULT_STATUS_pd_vusb_ovp_END             (2)
#define SOC_SCHARGER_PD_FAULT_STATUS_pd_force_disch_fail_START   (4)
#define SOC_SCHARGER_PD_FAULT_STATUS_pd_force_disch_fail_END     (4)
#define SOC_SCHARGER_PD_FAULT_STATUS_pd_auto_disch_fail_START    (5)
#define SOC_SCHARGER_PD_FAULT_STATUS_pd_auto_disch_fail_END      (5)
#define SOC_SCHARGER_PD_FAULT_STATUS_pd_reg_reset_default_START  (7)
#define SOC_SCHARGER_PD_FAULT_STATUS_pd_reg_reset_default_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_PD_COMMAND_UNION
 结构说明  : PD_COMMAND 寄存器结构定义。地址偏移量:0x23，初值:0x00，宽度:8
 寄存器说明: PD Command
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  pd_command : 8;  /* bit[0-7]: 命令寄存器，TCPM通过I2C接口对该寄存器配置为不同的值从而触发不同的命令。命令列表如下所示。
                                                     8'h11：WakeI2C，不支持；
                                                     8'h22：DisableVbusDetect，支持，关闭Vbus电压检测功能。当TCPC在sourcing或者sinking power的时候建议不关闭。
                                                     8'h33：EnableVbusDetect，支持，使能Vbus电压检测功能
                                                     8'h44：DisableSinkVbus，不支持，软件通过配置sc_ovp_mos_en寄存器为0来停止Sink Vbus；
                                                     8'h55：SinkVbus，不支持，软件通过配置sc_ovp_mos_en寄存器为1来使能Sink Vbus；
                                                     8'h66：DisableSourceVbus，不支持，软件通过配置关闭内部OTG使能（具体看otg编程指南部分）来停止Source Vbus
                                                     6'h77：SourceVbusDefaultVol，不支持，软件通过配置开启内部OTG使能（具体看otg编程指南部分）来Source Vbus；
                                                     6'h88：SourceVbusHighVol，不支持；
                                                     6'h99：Look4Connection，支持，当PD_ROLE_CTRL的pd_cc2_cfg/pd_cc1_cfg为双Rp或者双Rd，且pd_drp为1，则TCPC开始DRP Toggle，若条件不成立，则没有动作。
                                                     6'hAA：RxOneMore，支持，使得PD接收端在发送完下一次GoodCRC后自动清PD_RXDETECT寄存器。这便于在某一特定点可以关闭msg的接收，分包或RX FIFO的深度不限制该功能的使用。
                                                     其他：不支持 */
    } reg;
} SOC_SCHARGER_PD_COMMAND_UNION;
#endif
#define SOC_SCHARGER_PD_COMMAND_pd_command_START  (0)
#define SOC_SCHARGER_PD_COMMAND_pd_command_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_PD_DEVCAP1L_UNION
 结构说明  : PD_DEVCAP1L 寄存器结构定义。地址偏移量:0x24，初值:0x00，宽度:8
 寄存器说明: PD Device Cap1 Low
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  pd_dev_cap1_l : 8;  /* bit[0-7]: Device Capabilities1低8比特 */
    } reg;
} SOC_SCHARGER_PD_DEVCAP1L_UNION;
#endif
#define SOC_SCHARGER_PD_DEVCAP1L_pd_dev_cap1_l_START  (0)
#define SOC_SCHARGER_PD_DEVCAP1L_pd_dev_cap1_l_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_PD_DEVCAP1H_UNION
 结构说明  : PD_DEVCAP1H 寄存器结构定义。地址偏移量:0x25，初值:0x00，宽度:8
 寄存器说明: PD Device Cap1 High
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  pd_dev_cap1_h : 8;  /* bit[0-7]: Device Capabilities1高8比特 */
    } reg;
} SOC_SCHARGER_PD_DEVCAP1H_UNION;
#endif
#define SOC_SCHARGER_PD_DEVCAP1H_pd_dev_cap1_h_START  (0)
#define SOC_SCHARGER_PD_DEVCAP1H_pd_dev_cap1_h_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_PD_DEVCAP2L_UNION
 结构说明  : PD_DEVCAP2L 寄存器结构定义。地址偏移量:0x26，初值:0x00，宽度:8
 寄存器说明: PD Device Cap2 Low
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  pd_dev_cap2_l : 8;  /* bit[0-7]: Device Capabilities2低8比特 */
    } reg;
} SOC_SCHARGER_PD_DEVCAP2L_UNION;
#endif
#define SOC_SCHARGER_PD_DEVCAP2L_pd_dev_cap2_l_START  (0)
#define SOC_SCHARGER_PD_DEVCAP2L_pd_dev_cap2_l_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_PD_DEVCAP2H_UNION
 结构说明  : PD_DEVCAP2H 寄存器结构定义。地址偏移量:0x27，初值:0x00，宽度:8
 寄存器说明: PD Device Cap2 High
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  pd_dev_cap2_h : 8;  /* bit[0-7]: Device Capabilities2高8比特 */
    } reg;
} SOC_SCHARGER_PD_DEVCAP2H_UNION;
#endif
#define SOC_SCHARGER_PD_DEVCAP2H_pd_dev_cap2_h_START  (0)
#define SOC_SCHARGER_PD_DEVCAP2H_pd_dev_cap2_h_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_PD_STDIN_CAP_UNION
 结构说明  : PD_STDIN_CAP 寄存器结构定义。地址偏移量:0x28，初值:0x00，宽度:8
 寄存器说明: PD Standard Input Capabilities
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  pd_stdin_cap : 8;  /* bit[0-7]: Standard Input Capabilities */
    } reg;
} SOC_SCHARGER_PD_STDIN_CAP_UNION;
#endif
#define SOC_SCHARGER_PD_STDIN_CAP_pd_stdin_cap_START  (0)
#define SOC_SCHARGER_PD_STDIN_CAP_pd_stdin_cap_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_PD_STDOUT_CAP_UNION
 结构说明  : PD_STDOUT_CAP 寄存器结构定义。地址偏移量:0x29，初值:0x00，宽度:8
 寄存器说明: PD Standard Output Capabilities
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  pd_stdout_cap : 8;  /* bit[0-7]: Standard Output Capabilities */
    } reg;
} SOC_SCHARGER_PD_STDOUT_CAP_UNION;
#endif
#define SOC_SCHARGER_PD_STDOUT_CAP_pd_stdout_cap_START  (0)
#define SOC_SCHARGER_PD_STDOUT_CAP_pd_stdout_cap_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_PD_MSG_HEADR_UNION
 结构说明  : PD_MSG_HEADR 寄存器结构定义。地址偏移量:0x2E，初值:0x02，宽度:8
 寄存器说明: PD Message Header
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  pd_msg_header : 8;  /* bit[0-7]: TCPM需要在TCPC完成初始化后依据业务需求配置该寄存器。
                                                        TCPM在检测到有效连接后，在配置PD_RXDETECT寄存器使能接收前，需要刷新该寄存器。
                                                        TCPC在回GoodCRC时会从该寄存器取对应信息。
                                                        Bit[7:5]：reserved
                                                        bit[4]：Cable Plug
                                                        0b:Msg oriented from Source, Sink, or DRP
                                                        1b:Msg oriented from a Cable Plug
                                                        bit[3]:Data Role
                                                        0b:UFP
                                                        1b:DFP
                                                        bit[2:1]:USB PD spec. Revision
                                                        00b:Revision 1.0
                                                        01b:Revision 2.0
                                                        Other:reserved
                                                        bit[0] Power Role
                                                        0b: Sink
                                                        1b: Source */
    } reg;
} SOC_SCHARGER_PD_MSG_HEADR_UNION;
#endif
#define SOC_SCHARGER_PD_MSG_HEADR_pd_msg_header_START  (0)
#define SOC_SCHARGER_PD_MSG_HEADR_pd_msg_header_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_PD_RXDETECT_UNION
 结构说明  : PD_RXDETECT 寄存器结构定义。地址偏移量:0x2F，初值:0x00，宽度:8
 寄存器说明: PD RX Detect
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  pd_en_sop        : 1;  /* bit[0-0]: 0b:TCPC不检测SOP msg
                                                           1b：TCPC检测SOP msg
                                                           默认0 */
        unsigned char  pd_en_sop1       : 1;  /* bit[1-1]: 0b:TCPC不检测SOP' msg
                                                           1b：TCPC检测SOP' msg
                                                           默认0 */
        unsigned char  pd_en_sop2       : 1;  /* bit[2-2]: 0b:TCPC不检测SOP'' msg
                                                           1b：TCPC检测SOP'' msg
                                                           默认0 */
        unsigned char  pd_en_sop1_debug : 1;  /* bit[3-3]: 0b:TCPC不检测SOP_DBG' msg
                                                           1b：TCPC检测SOP_DBG' msg
                                                           默认0 */
        unsigned char  pd_en_sop2_debug : 1;  /* bit[4-4]: 0b:TCPC不检测SOPDBG'' msg
                                                           1b：TCPC检测SOPDBG'' msg
                                                           默认0 */
        unsigned char  pd_hard_rst      : 1;  /* bit[5-5]: 0b:TCPC不检测Hard Reset
                                                           1b：TCPC检测Hard Reset
                                                           默认0 */
        unsigned char  pd_cable_rst     : 1;  /* bit[6-6]: 0b:TCPC不检测Cable Reset
                                                           1b：TCPC检测Cable Reset
                                                           默认0 */
        unsigned char  reserved         : 1;  /* bit[7-7]:  */
    } reg;
} SOC_SCHARGER_PD_RXDETECT_UNION;
#endif
#define SOC_SCHARGER_PD_RXDETECT_pd_en_sop_START         (0)
#define SOC_SCHARGER_PD_RXDETECT_pd_en_sop_END           (0)
#define SOC_SCHARGER_PD_RXDETECT_pd_en_sop1_START        (1)
#define SOC_SCHARGER_PD_RXDETECT_pd_en_sop1_END          (1)
#define SOC_SCHARGER_PD_RXDETECT_pd_en_sop2_START        (2)
#define SOC_SCHARGER_PD_RXDETECT_pd_en_sop2_END          (2)
#define SOC_SCHARGER_PD_RXDETECT_pd_en_sop1_debug_START  (3)
#define SOC_SCHARGER_PD_RXDETECT_pd_en_sop1_debug_END    (3)
#define SOC_SCHARGER_PD_RXDETECT_pd_en_sop2_debug_START  (4)
#define SOC_SCHARGER_PD_RXDETECT_pd_en_sop2_debug_END    (4)
#define SOC_SCHARGER_PD_RXDETECT_pd_hard_rst_START       (5)
#define SOC_SCHARGER_PD_RXDETECT_pd_hard_rst_END         (5)
#define SOC_SCHARGER_PD_RXDETECT_pd_cable_rst_START      (6)
#define SOC_SCHARGER_PD_RXDETECT_pd_cable_rst_END        (6)


/*****************************************************************************
 结构名    : SOC_SCHARGER_PD_RXBYTECNT_UNION
 结构说明  : PD_RXBYTECNT 寄存器结构定义。地址偏移量:0x30，初值:0x00，宽度:8
 寄存器说明: PD RX ByteCount
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  pd_rx_bytecnt : 8;  /* bit[0-7]:  */
    } reg;
} SOC_SCHARGER_PD_RXBYTECNT_UNION;
#endif
#define SOC_SCHARGER_PD_RXBYTECNT_pd_rx_bytecnt_START  (0)
#define SOC_SCHARGER_PD_RXBYTECNT_pd_rx_bytecnt_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_PD_RXTYPE_UNION
 结构说明  : PD_RXTYPE 寄存器结构定义。地址偏移量:0x31，初值:0x00，宽度:8
 寄存器说明: PD RX Type
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  pd_rx_type : 3;  /* bit[0-2]:  */
        unsigned char  reserved   : 5;  /* bit[3-7]:  */
    } reg;
} SOC_SCHARGER_PD_RXTYPE_UNION;
#endif
#define SOC_SCHARGER_PD_RXTYPE_pd_rx_type_START  (0)
#define SOC_SCHARGER_PD_RXTYPE_pd_rx_type_END    (2)


/*****************************************************************************
 结构名    : SOC_SCHARGER_PD_RXHEADL_UNION
 结构说明  : PD_RXHEADL 寄存器结构定义。地址偏移量:0x32，初值:0x00，宽度:8
 寄存器说明: PD RX Header Low
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  pd_rx_head_l : 8;  /* bit[0-7]:  */
    } reg;
} SOC_SCHARGER_PD_RXHEADL_UNION;
#endif
#define SOC_SCHARGER_PD_RXHEADL_pd_rx_head_l_START  (0)
#define SOC_SCHARGER_PD_RXHEADL_pd_rx_head_l_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_PD_RXHEADH_UNION
 结构说明  : PD_RXHEADH 寄存器结构定义。地址偏移量:0x33，初值:0x00，宽度:8
 寄存器说明: PD RX Header High
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  pd_rx_head_h : 8;  /* bit[0-7]:  */
    } reg;
} SOC_SCHARGER_PD_RXHEADH_UNION;
#endif
#define SOC_SCHARGER_PD_RXHEADH_pd_rx_head_h_START  (0)
#define SOC_SCHARGER_PD_RXHEADH_pd_rx_head_h_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_PD_RXDATA_0_UNION
 结构说明  : PD_RXDATA_0 寄存器结构定义。地址偏移量:k*1+0x34，初值:0x00，宽度:8
 寄存器说明: PD RX Data Payload
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  pd_rx_data : 8;  /* bit[0-7]:  */
    } reg;
} SOC_SCHARGER_PD_RXDATA_0_UNION;
#endif
#define SOC_SCHARGER_PD_RXDATA_0_pd_rx_data_START  (0)
#define SOC_SCHARGER_PD_RXDATA_0_pd_rx_data_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_PD_TRANSMIT_UNION
 结构说明  : PD_TRANSMIT 寄存器结构定义。地址偏移量:0x50，初值:0x00，宽度:8
 寄存器说明: PD Transmit
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  pd_transmit  : 3;  /* bit[0-2]:  */
        unsigned char  reserved_0   : 1;  /* bit[3-3]:  */
        unsigned char  pd_retry_cnt : 2;  /* bit[4-5]:  */
        unsigned char  reserved_1   : 2;  /* bit[6-7]:  */
    } reg;
} SOC_SCHARGER_PD_TRANSMIT_UNION;
#endif
#define SOC_SCHARGER_PD_TRANSMIT_pd_transmit_START   (0)
#define SOC_SCHARGER_PD_TRANSMIT_pd_transmit_END     (2)
#define SOC_SCHARGER_PD_TRANSMIT_pd_retry_cnt_START  (4)
#define SOC_SCHARGER_PD_TRANSMIT_pd_retry_cnt_END    (5)


/*****************************************************************************
 结构名    : SOC_SCHARGER_PD_TXBYTECNT_UNION
 结构说明  : PD_TXBYTECNT 寄存器结构定义。地址偏移量:0x51，初值:0x00，宽度:8
 寄存器说明: PD TX Byte Count
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  pd_tx_bytecnt : 8;  /* bit[0-7]:  */
    } reg;
} SOC_SCHARGER_PD_TXBYTECNT_UNION;
#endif
#define SOC_SCHARGER_PD_TXBYTECNT_pd_tx_bytecnt_START  (0)
#define SOC_SCHARGER_PD_TXBYTECNT_pd_tx_bytecnt_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_PD_TXHEADL_UNION
 结构说明  : PD_TXHEADL 寄存器结构定义。地址偏移量:0x52，初值:0x00，宽度:8
 寄存器说明: PD TX Header Low
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  pd_tx_head_l : 8;  /* bit[0-7]:  */
    } reg;
} SOC_SCHARGER_PD_TXHEADL_UNION;
#endif
#define SOC_SCHARGER_PD_TXHEADL_pd_tx_head_l_START  (0)
#define SOC_SCHARGER_PD_TXHEADL_pd_tx_head_l_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_PD_TXHEADH_UNION
 结构说明  : PD_TXHEADH 寄存器结构定义。地址偏移量:0x53，初值:0x00，宽度:8
 寄存器说明: PD TX Header High
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  pd_tx_head_h : 8;  /* bit[0-7]:  */
    } reg;
} SOC_SCHARGER_PD_TXHEADH_UNION;
#endif
#define SOC_SCHARGER_PD_TXHEADH_pd_tx_head_h_START  (0)
#define SOC_SCHARGER_PD_TXHEADH_pd_tx_head_h_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_PD_TXDATA_UNION
 结构说明  : PD_TXDATA 寄存器结构定义。地址偏移量:k*1+0x54，初值:0x00，宽度:8
 寄存器说明: PD TX Payload
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  pd_tx_data : 8;  /* bit[0-7]:  */
    } reg;
} SOC_SCHARGER_PD_TXDATA_UNION;
#endif
#define SOC_SCHARGER_PD_TXDATA_pd_tx_data_START  (0)
#define SOC_SCHARGER_PD_TXDATA_pd_tx_data_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_PD_VBUS_VOL_L_UNION
 结构说明  : PD_VBUS_VOL_L 寄存器结构定义。地址偏移量:0x70，初值:0x00，宽度:8
 寄存器说明: PD VBUS Voltage Low
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  pd_vbus_vol_l : 8;  /* bit[0-7]:  */
    } reg;
} SOC_SCHARGER_PD_VBUS_VOL_L_UNION;
#endif
#define SOC_SCHARGER_PD_VBUS_VOL_L_pd_vbus_vol_l_START  (0)
#define SOC_SCHARGER_PD_VBUS_VOL_L_pd_vbus_vol_l_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_PD_VBUS_VOL_H_UNION
 结构说明  : PD_VBUS_VOL_H 寄存器结构定义。地址偏移量:0x71，初值:0x00，宽度:8
 寄存器说明: PD VBUS Voltage High
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  pd_vbus_vol_h : 8;  /* bit[0-7]:  */
    } reg;
} SOC_SCHARGER_PD_VBUS_VOL_H_UNION;
#endif
#define SOC_SCHARGER_PD_VBUS_VOL_H_pd_vbus_vol_h_START  (0)
#define SOC_SCHARGER_PD_VBUS_VOL_H_pd_vbus_vol_h_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_PD_VBUS_SNK_DISCL_UNION
 结构说明  : PD_VBUS_SNK_DISCL 寄存器结构定义。地址偏移量:0x72，初值:0x90，宽度:8
 寄存器说明: VBUS Sink Disconnect Threshold Low
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  pd_vbus_snk_disc_l : 8;  /* bit[0-7]:  */
    } reg;
} SOC_SCHARGER_PD_VBUS_SNK_DISCL_UNION;
#endif
#define SOC_SCHARGER_PD_VBUS_SNK_DISCL_pd_vbus_snk_disc_l_START  (0)
#define SOC_SCHARGER_PD_VBUS_SNK_DISCL_pd_vbus_snk_disc_l_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_PD_VBUS_SNK_DISCH_UNION
 结构说明  : PD_VBUS_SNK_DISCH 寄存器结构定义。地址偏移量:0x73，初值:0x00，宽度:8
 寄存器说明: VBUS Sink Disconnect Threshold High
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  pd_vbus_snk_disc_h : 8;  /* bit[0-7]:  */
    } reg;
} SOC_SCHARGER_PD_VBUS_SNK_DISCH_UNION;
#endif
#define SOC_SCHARGER_PD_VBUS_SNK_DISCH_pd_vbus_snk_disc_h_START  (0)
#define SOC_SCHARGER_PD_VBUS_SNK_DISCH_pd_vbus_snk_disc_h_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_PD_VBUS_STOP_DISCL_UNION
 结构说明  : PD_VBUS_STOP_DISCL 寄存器结构定义。地址偏移量:0x74，初值:0x1C，宽度:8
 寄存器说明: VBUS Discharge Stop Threshold Low
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  pd_vbus_stop_disc_l : 8;  /* bit[0-7]:  */
    } reg;
} SOC_SCHARGER_PD_VBUS_STOP_DISCL_UNION;
#endif
#define SOC_SCHARGER_PD_VBUS_STOP_DISCL_pd_vbus_stop_disc_l_START  (0)
#define SOC_SCHARGER_PD_VBUS_STOP_DISCL_pd_vbus_stop_disc_l_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_PD_VBUS_STOP_DISCH_UNION
 结构说明  : PD_VBUS_STOP_DISCH 寄存器结构定义。地址偏移量:0x75，初值:0x00，宽度:8
 寄存器说明: VBUS Discharge Stop Threshold High
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  pd_vbus_stop_disc_h : 8;  /* bit[0-7]:  */
    } reg;
} SOC_SCHARGER_PD_VBUS_STOP_DISCH_UNION;
#endif
#define SOC_SCHARGER_PD_VBUS_STOP_DISCH_pd_vbus_stop_disc_h_START  (0)
#define SOC_SCHARGER_PD_VBUS_STOP_DISCH_pd_vbus_stop_disc_h_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_PD_VALARMH_CFGL_UNION
 结构说明  : PD_VALARMH_CFGL 寄存器结构定义。地址偏移量:0x76，初值:0x00，宽度:8
 寄存器说明: Voltage High Trip Point Low
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  pd_valarm_high_l : 8;  /* bit[0-7]:  */
    } reg;
} SOC_SCHARGER_PD_VALARMH_CFGL_UNION;
#endif
#define SOC_SCHARGER_PD_VALARMH_CFGL_pd_valarm_high_l_START  (0)
#define SOC_SCHARGER_PD_VALARMH_CFGL_pd_valarm_high_l_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_PD_VALARMH_CFGH_UNION
 结构说明  : PD_VALARMH_CFGH 寄存器结构定义。地址偏移量:0x77，初值:0x00，宽度:8
 寄存器说明: Voltage High Trip Point High
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  pd_valarm_high_h : 8;  /* bit[0-7]:  */
    } reg;
} SOC_SCHARGER_PD_VALARMH_CFGH_UNION;
#endif
#define SOC_SCHARGER_PD_VALARMH_CFGH_pd_valarm_high_h_START  (0)
#define SOC_SCHARGER_PD_VALARMH_CFGH_pd_valarm_high_h_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_PD_VALARML_CFGL_UNION
 结构说明  : PD_VALARML_CFGL 寄存器结构定义。地址偏移量:0x78，初值:0x00，宽度:8
 寄存器说明: Voltage Low Trip Point Low
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  pd_valarm_low_l : 8;  /* bit[0-7]:  */
    } reg;
} SOC_SCHARGER_PD_VALARML_CFGL_UNION;
#endif
#define SOC_SCHARGER_PD_VALARML_CFGL_pd_valarm_low_l_START  (0)
#define SOC_SCHARGER_PD_VALARML_CFGL_pd_valarm_low_l_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_PD_VALARML_CFGH_UNION
 结构说明  : PD_VALARML_CFGH 寄存器结构定义。地址偏移量:0x79，初值:0x00，宽度:8
 寄存器说明: Voltage Low Trip Point High
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  pd_valarm_low_h : 8;  /* bit[0-7]:  */
    } reg;
} SOC_SCHARGER_PD_VALARML_CFGH_UNION;
#endif
#define SOC_SCHARGER_PD_VALARML_CFGH_pd_valarm_low_h_START  (0)
#define SOC_SCHARGER_PD_VALARML_CFGH_pd_valarm_low_h_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_PD_VDM_CFG_0_UNION
 结构说明  : PD_VDM_CFG_0 寄存器结构定义。地址偏移量:0x7A，初值:0x60，宽度:8
 寄存器说明: PD自定义配置寄存器
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  da_frs_en          : 1;  /* bit[0-0]: 外部Switch芯片（供5V电压）使能信号：
                                                             0b：不使能
                                                             1b：使能
                                                             默认0 */
        unsigned char  da_vconn_dis_en    : 1;  /* bit[1-1]: VCONN泄放通路使能信号：
                                                             0b：不使能VCONN泄放电阻
                                                             1b：使能VCONN泄放电阻
                                                             默认0 */
        unsigned char  pd_drp_thres       : 2;  /* bit[2-3]: TCPC DRP Toggling时间配置。DRP Toggling总体时间额定为75ms，可以通过配置pd_drp_thres来设置作为Src的时间；
                                                             00:dcSRC.DRP为22.5ms；
                                                             01:dcSRC.DRP为30ms；
                                                             10:dcSRC.DRP为37.5ms；
                                                             11:dcSRC.DRP为52.5ms；
                                                             默认为00 */
        unsigned char  pd_try_snk_en      : 1;  /* bit[4-4]: TCPC状态机TrySnk使能。若使能TrySnk机制，则在Unattach.SRC状态时检测到可能有器件插入后，状态机会跳转至Try.Snk以尝试作为Snk进行连接；若不使能TrySnk机制，则Unattach.SRC状态时检测到可能有器件插入后，状态机会跳转到Attached.SRC，不进行TrySnk连接。
                                                             0b: 不使能TrySnk机制；
                                                             1b：使能TrySnk机制；
                                                             默认0 */
        unsigned char  da_force_unplug_en : 1;  /* bit[5-5]: Rd在系统复位时自动断开使能信号：
                                                             0b：不使能自动断开功能
                                                             1b：使能自动断开功能
                                                             da_force_unplug_en只受pwr_rst_n复位，不受管脚复位reset_n和软复位影响；
                                                             默认1 */
        unsigned char  pd_bmc_cdr_sel     : 1;  /* bit[6-6]: PD物理层RX通路BMC编码CDR（时钟恢复电路）方案选择：
                                                             0b:选择counter四次平均预估方案采数；
                                                             1b:选择锁相环锁定方案；
                                                             默认1 */
        unsigned char  pd_cc_orient_sel   : 1;  /* bit[7-7]: Type-C协议CC1/CC2通道orient配置选择。
                                                             0b：在Debug模式下默认由硬件自动在连接确立后自动配置orient；
                                                             1b：在Debug模式下在连接确立后由软件配置orient；
                                                             默认0 */
    } reg;
} SOC_SCHARGER_PD_VDM_CFG_0_UNION;
#endif
#define SOC_SCHARGER_PD_VDM_CFG_0_da_frs_en_START           (0)
#define SOC_SCHARGER_PD_VDM_CFG_0_da_frs_en_END             (0)
#define SOC_SCHARGER_PD_VDM_CFG_0_da_vconn_dis_en_START     (1)
#define SOC_SCHARGER_PD_VDM_CFG_0_da_vconn_dis_en_END       (1)
#define SOC_SCHARGER_PD_VDM_CFG_0_pd_drp_thres_START        (2)
#define SOC_SCHARGER_PD_VDM_CFG_0_pd_drp_thres_END          (3)
#define SOC_SCHARGER_PD_VDM_CFG_0_pd_try_snk_en_START       (4)
#define SOC_SCHARGER_PD_VDM_CFG_0_pd_try_snk_en_END         (4)
#define SOC_SCHARGER_PD_VDM_CFG_0_da_force_unplug_en_START  (5)
#define SOC_SCHARGER_PD_VDM_CFG_0_da_force_unplug_en_END    (5)
#define SOC_SCHARGER_PD_VDM_CFG_0_pd_bmc_cdr_sel_START      (6)
#define SOC_SCHARGER_PD_VDM_CFG_0_pd_bmc_cdr_sel_END        (6)
#define SOC_SCHARGER_PD_VDM_CFG_0_pd_cc_orient_sel_START    (7)
#define SOC_SCHARGER_PD_VDM_CFG_0_pd_cc_orient_sel_END      (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_PD_VDM_ENABLE_UNION
 结构说明  : PD_VDM_ENABLE 寄存器结构定义。地址偏移量:0x7B，初值:0x01，宽度:8
 寄存器说明: PD自定义使能寄存器
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  enable_pd : 1;  /* bit[0-0]: PD协议使能寄存器。使能后才能接收与发送PD信息。不影响Type-C端口检测电路与对应配置。
                                                    0b：不使能PD协议
                                                    1b: 使能PD协议。默认当Type-C端口检测电路检测到有效连接后，PD工作时钟会开启，PD协议处于Ready状态，可以由TCPM配置接收与发送PD信息。
                                                    默认1 */
        unsigned char  reserved  : 7;  /* bit[1-7]: reserved */
    } reg;
} SOC_SCHARGER_PD_VDM_ENABLE_UNION;
#endif
#define SOC_SCHARGER_PD_VDM_ENABLE_enable_pd_START  (0)
#define SOC_SCHARGER_PD_VDM_ENABLE_enable_pd_END    (0)


/*****************************************************************************
 结构名    : SOC_SCHARGER_PD_VDM_CFG_1_UNION
 结构说明  : PD_VDM_CFG_1 寄存器结构定义。地址偏移量:0x7C，初值:0x00，宽度:8
 寄存器说明: PD自定义配置寄存器
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  tc_fsm_reset         : 1;  /* bit[0-0]: Type-C状态机复位信号，高电平有效
                                                               0b：不复位Type-C状态机；
                                                               1b：复位Type-C状态机；
                                                               默认0 */
        unsigned char  pd_fsm_reset         : 1;  /* bit[1-1]: PD状态机复位信号，高电平有效
                                                               0b：不复位PD状态机；
                                                               1b：复位PD状态机；
                                                               默认0 */
        unsigned char  pd_tx_phy_soft_reset : 1;  /* bit[2-2]: PD TX物理层软复位，高电平有效；
                                                               0b：不复位PD TX PHY；
                                                               1b：复位PD TX PHY；
                                                               默认0 */
        unsigned char  pd_rx_phy_soft_reset : 1;  /* bit[3-3]: PD RX物理层软复位，高电平有效；
                                                               0b：不复位PD RX PHY；
                                                               1b：复位PD RX PHY；
                                                               默认0 */
        unsigned char  pd_snk_disc_by_cc    : 1;  /* bit[4-4]: 0b:作为Sink时，不支持依据CC状态判断Source的移除；
                                                               1b:作为Sink时，支持依据CC状态判断Source的移除；
                                                               默认0 */
        unsigned char  reserved             : 3;  /* bit[5-7]: reserved */
    } reg;
} SOC_SCHARGER_PD_VDM_CFG_1_UNION;
#endif
#define SOC_SCHARGER_PD_VDM_CFG_1_tc_fsm_reset_START          (0)
#define SOC_SCHARGER_PD_VDM_CFG_1_tc_fsm_reset_END            (0)
#define SOC_SCHARGER_PD_VDM_CFG_1_pd_fsm_reset_START          (1)
#define SOC_SCHARGER_PD_VDM_CFG_1_pd_fsm_reset_END            (1)
#define SOC_SCHARGER_PD_VDM_CFG_1_pd_tx_phy_soft_reset_START  (2)
#define SOC_SCHARGER_PD_VDM_CFG_1_pd_tx_phy_soft_reset_END    (2)
#define SOC_SCHARGER_PD_VDM_CFG_1_pd_rx_phy_soft_reset_START  (3)
#define SOC_SCHARGER_PD_VDM_CFG_1_pd_rx_phy_soft_reset_END    (3)
#define SOC_SCHARGER_PD_VDM_CFG_1_pd_snk_disc_by_cc_START     (4)
#define SOC_SCHARGER_PD_VDM_CFG_1_pd_snk_disc_by_cc_END       (4)


/*****************************************************************************
 结构名    : SOC_SCHARGER_PD_DBG_RDATA_CFG_UNION
 结构说明  : PD_DBG_RDATA_CFG 寄存器结构定义。地址偏移量:0x7D，初值:0x00，宽度:8
 寄存器说明: 芯片测试用，不对产品开放。测试数据配置寄存器
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  pd_dbg_rdata_sel  : 4;  /* bit[0-3]: 芯片测试用，不对产品开放。
                                                            测试回读数据选择
                                                            4'b0000:选择第0路数据
                                                            4'b0001:选择第1路数据
                                                            …
                                                            4'b1111:选择第15路数据
                                                            默认0 */
        unsigned char  pd_dbg_module_sel : 3;  /* bit[4-6]: 芯片测试用，不对产品开放。
                                                            测试模块选择。
                                                            3'b000:TCPC FSM
                                                            3'b001:PD TX/RX FSM
                                                            3'b010:PD TX PHY
                                                            3'b011:PD RX PHY
                                                            3'b100:VDM
                                                            3'b101:PD Timer/Counter
                                                            3'b110:PD Interrupt
                                                            3'b111:reserved
                                                            默认：3'b000 */
        unsigned char  pd_dbg_rdata_en   : 1;  /* bit[7-7]: 芯片测试用，不对产品开放。
                                                            测试回读数据使能，高电平有效。
                                                            1'b1:使能测试数据回读，数据从PD_DBG_RDATA中回读出来；
                                                            1'b0:不使能测试数据回读。
                                                            默认0 */
    } reg;
} SOC_SCHARGER_PD_DBG_RDATA_CFG_UNION;
#endif
#define SOC_SCHARGER_PD_DBG_RDATA_CFG_pd_dbg_rdata_sel_START   (0)
#define SOC_SCHARGER_PD_DBG_RDATA_CFG_pd_dbg_rdata_sel_END     (3)
#define SOC_SCHARGER_PD_DBG_RDATA_CFG_pd_dbg_module_sel_START  (4)
#define SOC_SCHARGER_PD_DBG_RDATA_CFG_pd_dbg_module_sel_END    (6)
#define SOC_SCHARGER_PD_DBG_RDATA_CFG_pd_dbg_rdata_en_START    (7)
#define SOC_SCHARGER_PD_DBG_RDATA_CFG_pd_dbg_rdata_en_END      (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_PD_DBG_RDATA_UNION
 结构说明  : PD_DBG_RDATA 寄存器结构定义。地址偏移量:0x7E，初值:0x00，宽度:8
 寄存器说明: 芯片测试用，不对产品开放。测试回读数据。
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  pd_dbg_rdata : 8;  /* bit[0-7]: 芯片测试用，不对产品开放。
                                                       测试回读数据。 */
    } reg;
} SOC_SCHARGER_PD_DBG_RDATA_UNION;
#endif
#define SOC_SCHARGER_PD_DBG_RDATA_pd_dbg_rdata_START  (0)
#define SOC_SCHARGER_PD_DBG_RDATA_pd_dbg_rdata_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_VDM_PAGE_SELECT_UNION
 结构说明  : VDM_PAGE_SELECT 寄存器结构定义。地址偏移量:0x7F，初值:0x00，宽度:8
 寄存器说明: Vendor Define Register, Page Select
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  page_select : 2;  /* bit[0-1]: 芯片内部寄存器分页配置
                                                      2'b00: REG_PAGE0;
                                                      2'b01: REG_PAGE1;
                                                      2'b10: REG_PAGE2;
                                                      2'b11: No Use, RFU
                                                      Default: 2'b00 */
        unsigned char  reserved    : 6;  /* bit[2-7]:  */
    } reg;
} SOC_SCHARGER_VDM_PAGE_SELECT_UNION;
#endif
#define SOC_SCHARGER_VDM_PAGE_SELECT_page_select_START  (0)
#define SOC_SCHARGER_VDM_PAGE_SELECT_page_select_END    (1)




/****************************************************************************
                     (2/4) REG_PAGE0
 ****************************************************************************/
/*****************************************************************************
 结构名    : SOC_SCHARGER_STATUIS_UNION
 结构说明  : STATUIS 寄存器结构定义。地址偏移量:0x00，初值:0x00，宽度:8
 寄存器说明: fcp slave 连接状态寄存器。
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  attach   : 1;  /* bit[0]  : 0：检测到装置未插入；
                                                   1：检测到装置插入 */
        unsigned char  reserved : 5;  /* bit[1-5]: 保留。 */
        unsigned char  dvc      : 2;  /* bit[6-7]: 00：检测没有开始或者正在检测状态；
                                                   01：检测到无效FCP slave；
                                                   10：保留；
                                                   11：检测到FCP slave。 */
    } reg;
} SOC_SCHARGER_STATUIS_UNION;
#endif
#define SOC_SCHARGER_STATUIS_attach_START    (0)
#define SOC_SCHARGER_STATUIS_attach_END      (0)
#define SOC_SCHARGER_STATUIS_dvc_START       (6)
#define SOC_SCHARGER_STATUIS_dvc_END         (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_CNTL_UNION
 结构说明  : CNTL 寄存器结构定义。地址偏移量:0x01，初值:0x00，宽度:8
 寄存器说明: FCP CNTL配置寄存器。
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  sndcmd   : 1;  /* bit[0]  : fcp master transaction 开始控制寄存器。 */
        unsigned char  reserved_0: 1;  /* bit[1]  : 保留。 */
        unsigned char  mstr_rst : 1;  /* bit[2]  : fcp master 控制slave复位寄存器。 */
        unsigned char  enable   : 1;  /* bit[3]  : fcp 使能控制寄存器。 */
        unsigned char  reserved_1: 4;  /* bit[4-7]: 保留。 */
    } reg;
} SOC_SCHARGER_CNTL_UNION;
#endif
#define SOC_SCHARGER_CNTL_sndcmd_START    (0)
#define SOC_SCHARGER_CNTL_sndcmd_END      (0)
#define SOC_SCHARGER_CNTL_mstr_rst_START  (2)
#define SOC_SCHARGER_CNTL_mstr_rst_END    (2)
#define SOC_SCHARGER_CNTL_enable_START    (3)
#define SOC_SCHARGER_CNTL_enable_END      (3)


/*****************************************************************************
 结构名    : SOC_SCHARGER_CMD_UNION
 结构说明  : CMD 寄存器结构定义。地址偏移量:0x04，初值:0x00，宽度:8
 寄存器说明: fcp 读写命令配置寄存器。
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  cmd : 8;  /* bit[0-7]: fcp 读写命令配置寄存器。 */
    } reg;
} SOC_SCHARGER_CMD_UNION;
#endif
#define SOC_SCHARGER_CMD_cmd_START  (0)
#define SOC_SCHARGER_CMD_cmd_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_FCP_LENGTH_UNION
 结构说明  : FCP_LENGTH 寄存器结构定义。地址偏移量:0x05，初值:0x01，宽度:8
 寄存器说明: fcp burst读写长度配置寄存器。
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  fcp_length : 4;  /* bit[0-3]: burst读写长度配置寄存器
                                                     4'd1：连读或连写1byte；
                                                     4'd2：连读或连写2byte；
                                                     4'd3：连读或连写3byte；
                                                     4'd4：连读或连写4byte；
                                                     4'd5：连读或连写5byte；
                                                     4'd6：连读或连写6byte；
                                                     4'd7：连读或连写7byte；
                                                     4'd8：连读或连写8byte；
                                                     0或大于8的值为非法值，数字内部报中断。
                                                     默认1  */
        unsigned char  reserved   : 4;  /* bit[4-7]: reserved */
    } reg;
} SOC_SCHARGER_FCP_LENGTH_UNION;
#endif
#define SOC_SCHARGER_FCP_LENGTH_fcp_length_START  (0)
#define SOC_SCHARGER_FCP_LENGTH_fcp_length_END    (3)


/*****************************************************************************
 结构名    : SOC_SCHARGER_ADDR_UNION
 结构说明  : ADDR 寄存器结构定义。地址偏移量:0x07，初值:0x00，宽度:8
 寄存器说明: fcp 读写地址配置寄存器。
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  addr : 8;  /* bit[0-7]: fcp 读写地址配置寄存器。 */
    } reg;
} SOC_SCHARGER_ADDR_UNION;
#endif
#define SOC_SCHARGER_ADDR_addr_START  (0)
#define SOC_SCHARGER_ADDR_addr_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_DATA0_UNION
 结构说明  : DATA0 寄存器结构定义。地址偏移量:0x08，初值:0x00，宽度:8
 寄存器说明: fcp 写数据寄存器。
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  data0 : 8;  /* bit[0-7]: fcp 写数据寄存器。 */
    } reg;
} SOC_SCHARGER_DATA0_UNION;
#endif
#define SOC_SCHARGER_DATA0_data0_START  (0)
#define SOC_SCHARGER_DATA0_data0_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_DATA1_UNION
 结构说明  : DATA1 寄存器结构定义。地址偏移量:0x09，初值:0x00，宽度:8
 寄存器说明: fcp 写数据寄存器。
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  data1 : 8;  /* bit[0-7]: fcp 写数据寄存器。 */
    } reg;
} SOC_SCHARGER_DATA1_UNION;
#endif
#define SOC_SCHARGER_DATA1_data1_START  (0)
#define SOC_SCHARGER_DATA1_data1_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_DATA2_UNION
 结构说明  : DATA2 寄存器结构定义。地址偏移量:0x0A，初值:0x00，宽度:8
 寄存器说明: fcp 写数据寄存器。
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  data2 : 8;  /* bit[0-7]: fcp 写数据寄存器。 */
    } reg;
} SOC_SCHARGER_DATA2_UNION;
#endif
#define SOC_SCHARGER_DATA2_data2_START  (0)
#define SOC_SCHARGER_DATA2_data2_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_DATA3_UNION
 结构说明  : DATA3 寄存器结构定义。地址偏移量:0x0B，初值:0x00，宽度:8
 寄存器说明: fcp 写数据寄存器。
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  data3 : 8;  /* bit[0-7]: fcp 写数据寄存器。 */
    } reg;
} SOC_SCHARGER_DATA3_UNION;
#endif
#define SOC_SCHARGER_DATA3_data3_START  (0)
#define SOC_SCHARGER_DATA3_data3_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_DATA4_UNION
 结构说明  : DATA4 寄存器结构定义。地址偏移量:0x0C，初值:0x00，宽度:8
 寄存器说明: fcp 写数据寄存器。
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  data4 : 8;  /* bit[0-7]: fcp 写数据寄存器。 */
    } reg;
} SOC_SCHARGER_DATA4_UNION;
#endif
#define SOC_SCHARGER_DATA4_data4_START  (0)
#define SOC_SCHARGER_DATA4_data4_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_DATA5_UNION
 结构说明  : DATA5 寄存器结构定义。地址偏移量:0x0D，初值:0x00，宽度:8
 寄存器说明: fcp 写数据寄存器。
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  data5 : 8;  /* bit[0-7]: fcp 写数据寄存器。 */
    } reg;
} SOC_SCHARGER_DATA5_UNION;
#endif
#define SOC_SCHARGER_DATA5_data5_START  (0)
#define SOC_SCHARGER_DATA5_data5_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_DATA6_UNION
 结构说明  : DATA6 寄存器结构定义。地址偏移量:0x0E，初值:0x00，宽度:8
 寄存器说明: fcp 写数据寄存器。
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  data6 : 8;  /* bit[0-7]: fcp 写数据寄存器。 */
    } reg;
} SOC_SCHARGER_DATA6_UNION;
#endif
#define SOC_SCHARGER_DATA6_data6_START  (0)
#define SOC_SCHARGER_DATA6_data6_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_DATA7_UNION
 结构说明  : DATA7 寄存器结构定义。地址偏移量:0x0F，初值:0x00，宽度:8
 寄存器说明: fcp 写数据寄存器。
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  data7 : 8;  /* bit[0-7]: fcp 写数据寄存器。 */
    } reg;
} SOC_SCHARGER_DATA7_UNION;
#endif
#define SOC_SCHARGER_DATA7_data7_START  (0)
#define SOC_SCHARGER_DATA7_data7_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_FCP_RDATA0_UNION
 结构说明  : FCP_RDATA0 寄存器结构定义。地址偏移量:0x10，初值:0x00，宽度:8
 寄存器说明: slave返回的数据。
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  fcp_rdata0 : 8;  /* bit[0-7]: I2C通过master读回的slaver的寄存器值。 */
    } reg;
} SOC_SCHARGER_FCP_RDATA0_UNION;
#endif
#define SOC_SCHARGER_FCP_RDATA0_fcp_rdata0_START  (0)
#define SOC_SCHARGER_FCP_RDATA0_fcp_rdata0_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_FCP_RDATA1_UNION
 结构说明  : FCP_RDATA1 寄存器结构定义。地址偏移量:0x11，初值:0x00，宽度:8
 寄存器说明: slave返回的数据。
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  fcp_rdata1 : 8;  /* bit[0-7]: I2C通过master读回的slaver的寄存器值。 */
    } reg;
} SOC_SCHARGER_FCP_RDATA1_UNION;
#endif
#define SOC_SCHARGER_FCP_RDATA1_fcp_rdata1_START  (0)
#define SOC_SCHARGER_FCP_RDATA1_fcp_rdata1_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_FCP_RDATA2_UNION
 结构说明  : FCP_RDATA2 寄存器结构定义。地址偏移量:0x12，初值:0x00，宽度:8
 寄存器说明: slave返回的数据。
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  fcp_rdata2 : 8;  /* bit[0-7]: I2C通过master读回的slaver的寄存器值。 */
    } reg;
} SOC_SCHARGER_FCP_RDATA2_UNION;
#endif
#define SOC_SCHARGER_FCP_RDATA2_fcp_rdata2_START  (0)
#define SOC_SCHARGER_FCP_RDATA2_fcp_rdata2_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_FCP_RDATA3_UNION
 结构说明  : FCP_RDATA3 寄存器结构定义。地址偏移量:0x13，初值:0x00，宽度:8
 寄存器说明: slave返回的数据。
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  fcp_rdata3 : 8;  /* bit[0-7]: I2C通过master读回的slaver的寄存器值。 */
    } reg;
} SOC_SCHARGER_FCP_RDATA3_UNION;
#endif
#define SOC_SCHARGER_FCP_RDATA3_fcp_rdata3_START  (0)
#define SOC_SCHARGER_FCP_RDATA3_fcp_rdata3_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_FCP_RDATA4_UNION
 结构说明  : FCP_RDATA4 寄存器结构定义。地址偏移量:0x14，初值:0x00，宽度:8
 寄存器说明: slave返回的数据。
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  fcp_rdata4 : 8;  /* bit[0-7]: I2C通过master读回的slaver的寄存器值。 */
    } reg;
} SOC_SCHARGER_FCP_RDATA4_UNION;
#endif
#define SOC_SCHARGER_FCP_RDATA4_fcp_rdata4_START  (0)
#define SOC_SCHARGER_FCP_RDATA4_fcp_rdata4_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_FCP_RDATA5_UNION
 结构说明  : FCP_RDATA5 寄存器结构定义。地址偏移量:0x15，初值:0x00，宽度:8
 寄存器说明: slave返回的数据。
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  fcp_rdata5 : 8;  /* bit[0-7]: I2C通过master读回的slaver的寄存器值。 */
    } reg;
} SOC_SCHARGER_FCP_RDATA5_UNION;
#endif
#define SOC_SCHARGER_FCP_RDATA5_fcp_rdata5_START  (0)
#define SOC_SCHARGER_FCP_RDATA5_fcp_rdata5_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_FCP_RDATA6_UNION
 结构说明  : FCP_RDATA6 寄存器结构定义。地址偏移量:0x16，初值:0x00，宽度:8
 寄存器说明: slave返回的数据。
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  fcp_rdata6 : 8;  /* bit[0-7]: I2C通过master读回的slaver的寄存器值。 */
    } reg;
} SOC_SCHARGER_FCP_RDATA6_UNION;
#endif
#define SOC_SCHARGER_FCP_RDATA6_fcp_rdata6_START  (0)
#define SOC_SCHARGER_FCP_RDATA6_fcp_rdata6_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_FCP_RDATA7_UNION
 结构说明  : FCP_RDATA7 寄存器结构定义。地址偏移量:0x17，初值:0x00，宽度:8
 寄存器说明: slave返回的数据。
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  fcp_rdata7 : 8;  /* bit[0-7]: I2C通过master读回的slaver的寄存器值。 */
    } reg;
} SOC_SCHARGER_FCP_RDATA7_UNION;
#endif
#define SOC_SCHARGER_FCP_RDATA7_fcp_rdata7_START  (0)
#define SOC_SCHARGER_FCP_RDATA7_fcp_rdata7_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_ISR1_UNION
 结构说明  : ISR1 寄存器结构定义。地址偏移量:0x19，初值:0x00，宽度:8
 寄存器说明: FCP 中断1寄存器。
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  reserved_0 : 1;  /* bit[0-0]: 保留。 */
        unsigned char  nack_alarm : 1;  /* bit[1]  : Slaver 返回NACK_ALARM中断：
                                                     0：无此中断；
                                                     1：有中断。 */
        unsigned char  ack_alarm  : 1;  /* bit[2]  : Slaver 返回ACK_ALARM中断：
                                                     0：无此中断；
                                                     1：有中断。 */
        unsigned char  crcpar     : 1;  /* bit[3]  : Slaver没有数据包返回中断；
                                                     0：无此中断；
                                                     1：有中断。 */
        unsigned char  nack       : 1;  /* bit[4]  : Slaver 返回NACK中断：
                                                     0：无此中断；
                                                     1：有中断。 */
        unsigned char  reserved_1 : 1;  /* bit[5]  : 保留。 */
        unsigned char  ack        : 1;  /* bit[6]  : Slaver 返回ACK中断：
                                                     0：无此中断；
                                                     1：有中断。 */
        unsigned char  cmdcpl     : 1;  /* bit[7]  : FCP命令成功完成中断：
                                                     0：无此中断；
                                                     1：有中断。 */
    } reg;
} SOC_SCHARGER_ISR1_UNION;
#endif
#define SOC_SCHARGER_ISR1_nack_alarm_START  (1)
#define SOC_SCHARGER_ISR1_nack_alarm_END    (1)
#define SOC_SCHARGER_ISR1_ack_alarm_START   (2)
#define SOC_SCHARGER_ISR1_ack_alarm_END     (2)
#define SOC_SCHARGER_ISR1_crcpar_START      (3)
#define SOC_SCHARGER_ISR1_crcpar_END        (3)
#define SOC_SCHARGER_ISR1_nack_START        (4)
#define SOC_SCHARGER_ISR1_nack_END          (4)
#define SOC_SCHARGER_ISR1_ack_START         (6)
#define SOC_SCHARGER_ISR1_ack_END           (6)
#define SOC_SCHARGER_ISR1_cmdcpl_START      (7)
#define SOC_SCHARGER_ISR1_cmdcpl_END        (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_ISR2_UNION
 结构说明  : ISR2 寄存器结构定义。地址偏移量:0x1A，初值:0x00，宽度:8
 寄存器说明: FCP 中断2寄存器。
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  reserved_0: 1;  /* bit[0]  : 保留。 */
        unsigned char  protstat : 1;  /* bit[1]  : Slaver检测状态变化中断：
                                                   0：无此中断；
                                                   1：有中断。 */
        unsigned char  reserved_1: 1;  /* bit[2]  : 保留。 */
        unsigned char  parrx    : 1;  /* bit[3]  : Slaver返回数据parity校验错误中断：
                                                   0：无此中断；
                                                   1：有中断。 */
        unsigned char  crcrx    : 1;  /* bit[4]  : Slaver返回数据crc校验错误中断；
                                                   0：无此中断；
                                                   1：有中断。 */
        unsigned char  reserved_2: 3;  /* bit[5-7]: 保留。 */
    } reg;
} SOC_SCHARGER_ISR2_UNION;
#endif
#define SOC_SCHARGER_ISR2_protstat_START  (1)
#define SOC_SCHARGER_ISR2_protstat_END    (1)
#define SOC_SCHARGER_ISR2_parrx_START     (3)
#define SOC_SCHARGER_ISR2_parrx_END       (3)
#define SOC_SCHARGER_ISR2_crcrx_START     (4)
#define SOC_SCHARGER_ISR2_crcrx_END       (4)


/*****************************************************************************
 结构名    : SOC_SCHARGER_IMR1_UNION
 结构说明  : IMR1 寄存器结构定义。地址偏移量:0x1B，初值:0x00，宽度:8
 寄存器说明: FCP 中断屏蔽1寄存器。
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  reserved_0    : 1;  /* bit[0]: 保留。 */
        unsigned char  nack_alarm_mk : 1;  /* bit[1]: nack_alarm中断屏蔽寄存器：
                                                      0：不屏蔽；
                                                      1：屏蔽。 */
        unsigned char  ack_alarm_mk  : 1;  /* bit[2]: ack_alarm中断屏蔽寄存器：
                                                      0：不屏蔽；
                                                      1：屏蔽。 */
        unsigned char  crcpar_mk     : 1;  /* bit[3]: crcpar中断屏蔽寄存器：
                                                      0：不屏蔽；
                                                      1：屏蔽。 */
        unsigned char  nack_mk       : 1;  /* bit[4]: nack中断屏蔽寄存器：
                                                      0：不屏蔽；
                                                      1：屏蔽。 */
        unsigned char  reserved_1    : 1;  /* bit[5]: 保留。 */
        unsigned char  ack_mk        : 1;  /* bit[6]: ack中断屏蔽寄存器：
                                                      0：不屏蔽；
                                                      1：屏蔽。 */
        unsigned char  cmdcpl_mk     : 1;  /* bit[7]: cmdcpl中断屏蔽寄存器：
                                                      0：不屏蔽；
                                                      1：屏蔽。 */
    } reg;
} SOC_SCHARGER_IMR1_UNION;
#endif
#define SOC_SCHARGER_IMR1_nack_alarm_mk_START  (1)
#define SOC_SCHARGER_IMR1_nack_alarm_mk_END    (1)
#define SOC_SCHARGER_IMR1_ack_alarm_mk_START   (2)
#define SOC_SCHARGER_IMR1_ack_alarm_mk_END     (2)
#define SOC_SCHARGER_IMR1_crcpar_mk_START      (3)
#define SOC_SCHARGER_IMR1_crcpar_mk_END        (3)
#define SOC_SCHARGER_IMR1_nack_mk_START        (4)
#define SOC_SCHARGER_IMR1_nack_mk_END          (4)
#define SOC_SCHARGER_IMR1_ack_mk_START         (6)
#define SOC_SCHARGER_IMR1_ack_mk_END           (6)
#define SOC_SCHARGER_IMR1_cmdcpl_mk_START      (7)
#define SOC_SCHARGER_IMR1_cmdcpl_mk_END        (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_IMR2_UNION
 结构说明  : IMR2 寄存器结构定义。地址偏移量:0x1C，初值:0x00，宽度:8
 寄存器说明: FCP 中断屏蔽2寄存器。
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  reserved_0  : 1;  /* bit[0]  : 保留。 */
        unsigned char  protstat_mk : 1;  /* bit[1]  : protstat中断屏蔽寄存器：
                                                      0：不屏蔽；
                                                      1：屏蔽。 */
        unsigned char  reserved_1  : 1;  /* bit[2]  : 保留。 */
        unsigned char  parrx_mk    : 1;  /* bit[3]  : parrx中断屏蔽寄存器：
                                                      0：不屏蔽；
                                                      1：屏蔽。 */
        unsigned char  crcrx_mk    : 1;  /* bit[4]  : crcrx中断屏蔽寄存器：
                                                      0：不屏蔽；
                                                      1：屏蔽。 */
        unsigned char  reserved_2  : 3;  /* bit[5-7]: 保留。 */
    } reg;
} SOC_SCHARGER_IMR2_UNION;
#endif
#define SOC_SCHARGER_IMR2_protstat_mk_START  (1)
#define SOC_SCHARGER_IMR2_protstat_mk_END    (1)
#define SOC_SCHARGER_IMR2_parrx_mk_START     (3)
#define SOC_SCHARGER_IMR2_parrx_mk_END       (3)
#define SOC_SCHARGER_IMR2_crcrx_mk_START     (4)
#define SOC_SCHARGER_IMR2_crcrx_mk_END       (4)


/*****************************************************************************
 结构名    : SOC_SCHARGER_FCP_IRQ5_UNION
 结构说明  : FCP_IRQ5 寄存器结构定义。地址偏移量:0x1D，初值:0x00，宽度:8
 寄存器说明: FCP中断5寄存器。
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  fcp_set_d60m_int : 1;  /* bit[0]  : 支持高压快充的Adapter中断（fcp_set滤波60ms，检测上升沿，上报中断）：
                                                           0：无此中断；
                                                           1：有中断。
                                                           注：在fcp_cmp_en=0时，模拟送出的fcp_set信号是0的。 */
        unsigned char  fcp_en_det_int   : 1;  /* bit[1]  : fcp_det_en&amp;fcp_cmp_en信号为高电平后2s内，没有收到fcp_set上升沿中断：
                                                           0：无此中断；
                                                           1：有中断。 */
        unsigned char  reserved         : 6;  /* bit[2-7]: 保留。 */
    } reg;
} SOC_SCHARGER_FCP_IRQ5_UNION;
#endif
#define SOC_SCHARGER_FCP_IRQ5_fcp_set_d60m_int_START  (0)
#define SOC_SCHARGER_FCP_IRQ5_fcp_set_d60m_int_END    (0)
#define SOC_SCHARGER_FCP_IRQ5_fcp_en_det_int_START    (1)
#define SOC_SCHARGER_FCP_IRQ5_fcp_en_det_int_END      (1)


/*****************************************************************************
 结构名    : SOC_SCHARGER_FCP_IRQ5_MASK_UNION
 结构说明  : FCP_IRQ5_MASK 寄存器结构定义。地址偏移量:0x1E，初值:0x00，宽度:8
 寄存器说明: FCP中断屏蔽5寄存器。
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  fcp_set_d60m_r_mk : 1;  /* bit[0]  : 0：使能该中断；
                                                            1：屏蔽该中断。 */
        unsigned char  fcp_en_det_int_mk : 1;  /* bit[1]  : 0：使能该中断；
                                                            1：屏蔽该中断。 */
        unsigned char  reserved          : 6;  /* bit[2-7]: 保留。 */
    } reg;
} SOC_SCHARGER_FCP_IRQ5_MASK_UNION;
#endif
#define SOC_SCHARGER_FCP_IRQ5_MASK_fcp_set_d60m_r_mk_START  (0)
#define SOC_SCHARGER_FCP_IRQ5_MASK_fcp_set_d60m_r_mk_END    (0)
#define SOC_SCHARGER_FCP_IRQ5_MASK_fcp_en_det_int_mk_START  (1)
#define SOC_SCHARGER_FCP_IRQ5_MASK_fcp_en_det_int_mk_END    (1)


/*****************************************************************************
 结构名    : SOC_SCHARGER_FCP_CTRL_UNION
 结构说明  : FCP_CTRL 寄存器结构定义。地址偏移量:0x1F，初值:0x00，宽度:8
 寄存器说明: 高压块充控制寄存器。
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  fcp_clk_test : 1;  /* bit[0]  : 高压快充的测试模式选择：
                                                       0：正常工作模式；
                                                       1：测试模式。 */
        unsigned char  fcp_mode     : 1;  /* bit[1]  : 高压快充模式选择：
                                                       0：模式1；
                                                       1：模式2； */
        unsigned char  fcp_cmp_en   : 1;  /* bit[2]  : 高压快充协议检测比较器使能：
                                                       0：关闭检测比较器；
                                                       1：检测比较器。
                                                       （注：看门狗发生异常后该寄存器会被置0。） */
        unsigned char  fcp_det_en   : 1;  /* bit[3]  : 高压快充协议检测使能：
                                                       0：关闭快充协议检测功能；
                                                       1：开启快充协议检测功能。
                                                       （注：看门狗发生异常后该寄存器会被置0。） */
        unsigned char  reserved     : 4;  /* bit[4-7]: 保留。 */
    } reg;
} SOC_SCHARGER_FCP_CTRL_UNION;
#endif
#define SOC_SCHARGER_FCP_CTRL_fcp_clk_test_START  (0)
#define SOC_SCHARGER_FCP_CTRL_fcp_clk_test_END    (0)
#define SOC_SCHARGER_FCP_CTRL_fcp_mode_START      (1)
#define SOC_SCHARGER_FCP_CTRL_fcp_mode_END        (1)
#define SOC_SCHARGER_FCP_CTRL_fcp_cmp_en_START    (2)
#define SOC_SCHARGER_FCP_CTRL_fcp_cmp_en_END      (2)
#define SOC_SCHARGER_FCP_CTRL_fcp_det_en_START    (3)
#define SOC_SCHARGER_FCP_CTRL_fcp_det_en_END      (3)


/*****************************************************************************
 结构名    : SOC_SCHARGER_FCP_MODE2_SET_UNION
 结构说明  : FCP_MODE2_SET 寄存器结构定义。地址偏移量:0x20，初值:0x00，宽度:8
 寄存器说明: 高压快充协议模式2档位控制寄存器。
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  fcp_mod2_set : 2;  /* bit[0-1]: 高压快充协议模式2档位控制：
                                                       00：（0.6V,0V)
                                                       01: (3.3V,0.6V)
                                                       10: (0.6V,0.6V)
                                                       11: (3.3V,3.3V) */
        unsigned char  reserved     : 6;  /* bit[2-7]: 保留。 */
    } reg;
} SOC_SCHARGER_FCP_MODE2_SET_UNION;
#endif
#define SOC_SCHARGER_FCP_MODE2_SET_fcp_mod2_set_START  (0)
#define SOC_SCHARGER_FCP_MODE2_SET_fcp_mod2_set_END    (1)


/*****************************************************************************
 结构名    : SOC_SCHARGER_FCP_ADAP_CTRL_UNION
 结构说明  : FCP_ADAP_CTRL 寄存器结构定义。地址偏移量:0x21，初值:0x00，宽度:8
 寄存器说明: 高压块充Adapter控制寄存器。
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  fcp_set_d60m_r : 1;  /* bit[0]  : 高压快充的Adapter判断（fcp_set_d60m_r中断状态寄存器）：
                                                         0：不支持高压快充的Adapter；
                                                         1：支持高压快充的Adapter。 */
        unsigned char  reserved       : 7;  /* bit[1-7]: 保留。 */
    } reg;
} SOC_SCHARGER_FCP_ADAP_CTRL_UNION;
#endif
#define SOC_SCHARGER_FCP_ADAP_CTRL_fcp_set_d60m_r_START  (0)
#define SOC_SCHARGER_FCP_ADAP_CTRL_fcp_set_d60m_r_END    (0)


/*****************************************************************************
 结构名    : SOC_SCHARGER_RDATA_READY_UNION
 结构说明  : RDATA_READY 寄存器结构定义。地址偏移量:0x22，初值:0x00，宽度:8
 寄存器说明: slave返回数据采集好指示信号。
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  fcp_rdata_ready : 1;  /* bit[0]  : I2C通过master读回的slaver的寄存器值准备好指示信号。 */
        unsigned char  reserved        : 7;  /* bit[1-7]: 保留。 */
    } reg;
} SOC_SCHARGER_RDATA_READY_UNION;
#endif
#define SOC_SCHARGER_RDATA_READY_fcp_rdata_ready_START  (0)
#define SOC_SCHARGER_RDATA_READY_fcp_rdata_ready_END    (0)


/*****************************************************************************
 结构名    : SOC_SCHARGER_FCP_SOFT_RST_CTRL_UNION
 结构说明  : FCP_SOFT_RST_CTRL 寄存器结构定义。地址偏移量:0x23，初值:0x5A，宽度:8
 寄存器说明: FCP软复位控制寄存器。
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  fcp_soft_rst_cfg : 8;  /* bit[0-7]: FCP软复位配置寄存器。复位FCP数字逻辑，低电平有效。
                                                           当写入0x5A时候，软复位为高电平，不复位；
                                                           当写入0xAC时候，软复位为低电平，复位FCP数字逻辑。
                                                           当写入其他值，软复位输出值不跳变，保持； */
    } reg;
} SOC_SCHARGER_FCP_SOFT_RST_CTRL_UNION;
#endif
#define SOC_SCHARGER_FCP_SOFT_RST_CTRL_fcp_soft_rst_cfg_START  (0)
#define SOC_SCHARGER_FCP_SOFT_RST_CTRL_fcp_soft_rst_cfg_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_FCP_RDATA_PARITY_ERR_UNION
 结构说明  : FCP_RDATA_PARITY_ERR 寄存器结构定义。地址偏移量:0x25，初值:0xff，宽度:8
 寄存器说明: FCP读数据Parity有误回读寄存器
            说明：不对产品开放
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  rdata_parity_err0 : 1;  /* bit[0]: burst连读模式，Slaver返回读数据的第0byte parity校验。 */
        unsigned char  rdata_parity_err1 : 1;  /* bit[1]: burst连读模式，Slaver返回读数据的第1byte parity校验。 */
        unsigned char  rdata_parity_err2 : 1;  /* bit[2]: burst连读模式，Slaver返回读数据的第2byte parity校验。 */
        unsigned char  rdata_parity_err3 : 1;  /* bit[3]: burst连读模式，Slaver返回读数据的第3byte parity校验。 */
        unsigned char  rdata_parity_err4 : 1;  /* bit[4]: burst连读模式，Slaver返回读数据的第4byte parity校验。 */
        unsigned char  rdata_parity_err5 : 1;  /* bit[5]: burst连读模式，Slaver返回读数据的第5byte parity校验。 */
        unsigned char  rdata_parity_err6 : 1;  /* bit[6]: burst连读模式，Slaver返回读数据的第6byte parity校验。 */
        unsigned char  rdata_parity_err7 : 1;  /* bit[7]: burst连读模式，Slaver返回读数据的第7byte parity校验。 */
    } reg;
} SOC_SCHARGER_FCP_RDATA_PARITY_ERR_UNION;
#endif
#define SOC_SCHARGER_FCP_RDATA_PARITY_ERR_rdata_parity_err0_START  (0)
#define SOC_SCHARGER_FCP_RDATA_PARITY_ERR_rdata_parity_err0_END    (0)
#define SOC_SCHARGER_FCP_RDATA_PARITY_ERR_rdata_parity_err1_START  (1)
#define SOC_SCHARGER_FCP_RDATA_PARITY_ERR_rdata_parity_err1_END    (1)
#define SOC_SCHARGER_FCP_RDATA_PARITY_ERR_rdata_parity_err2_START  (2)
#define SOC_SCHARGER_FCP_RDATA_PARITY_ERR_rdata_parity_err2_END    (2)
#define SOC_SCHARGER_FCP_RDATA_PARITY_ERR_rdata_parity_err3_START  (3)
#define SOC_SCHARGER_FCP_RDATA_PARITY_ERR_rdata_parity_err3_END    (3)
#define SOC_SCHARGER_FCP_RDATA_PARITY_ERR_rdata_parity_err4_START  (4)
#define SOC_SCHARGER_FCP_RDATA_PARITY_ERR_rdata_parity_err4_END    (4)
#define SOC_SCHARGER_FCP_RDATA_PARITY_ERR_rdata_parity_err5_START  (5)
#define SOC_SCHARGER_FCP_RDATA_PARITY_ERR_rdata_parity_err5_END    (5)
#define SOC_SCHARGER_FCP_RDATA_PARITY_ERR_rdata_parity_err6_START  (6)
#define SOC_SCHARGER_FCP_RDATA_PARITY_ERR_rdata_parity_err6_END    (6)
#define SOC_SCHARGER_FCP_RDATA_PARITY_ERR_rdata_parity_err7_START  (7)
#define SOC_SCHARGER_FCP_RDATA_PARITY_ERR_rdata_parity_err7_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_FCP_INIT_RETRY_CFG_UNION
 结构说明  : FCP_INIT_RETRY_CFG 寄存器结构定义。地址偏移量:0x26，初值:0x0F，宽度:8
 寄存器说明: FCP初始化Retry配置寄存器
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  fcp_init_retry_cfg : 5;  /* bit[0-4]: FCP初始化Retry配置寄存器 */
        unsigned char  reserved           : 3;  /* bit[5-7]: reserved */
    } reg;
} SOC_SCHARGER_FCP_INIT_RETRY_CFG_UNION;
#endif
#define SOC_SCHARGER_FCP_INIT_RETRY_CFG_fcp_init_retry_cfg_START  (0)
#define SOC_SCHARGER_FCP_INIT_RETRY_CFG_fcp_init_retry_cfg_END    (4)


/*****************************************************************************
 结构名    : SOC_SCHARGER_HKADC_SOFT_RST_CTRL_UNION
 结构说明  : HKADC_SOFT_RST_CTRL 寄存器结构定义。地址偏移量:0x28，初值:0x5A，宽度:8
 寄存器说明: HKADC软复位配置寄存器
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  sc_hkadc_soft_rst_n : 8;  /* bit[0-7]: HKADC软复位配置寄存器。复位HKADC数字逻辑，低电平有效。
                                                              当写入0x5A时候，软复位为高电平，不复位；
                                                              当写入0xAC时候，软复位为低电平，复位HKADC数字逻辑。
                                                              当写入其他值，软复位输出值不跳变，保持； */
    } reg;
} SOC_SCHARGER_HKADC_SOFT_RST_CTRL_UNION;
#endif
#define SOC_SCHARGER_HKADC_SOFT_RST_CTRL_sc_hkadc_soft_rst_n_START  (0)
#define SOC_SCHARGER_HKADC_SOFT_RST_CTRL_sc_hkadc_soft_rst_n_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_HKADC_CTRL0_UNION
 结构说明  : HKADC_CTRL0 寄存器结构定义。地址偏移量:0x29，初值:0x10，宽度:8
 寄存器说明: HKADC配置寄存器
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  sc_hkadc_chanel_sel : 4;  /* bit[0-3]: 低精度HKADC通道选择信号：
                                                              4'b0000：VUSB输入电压（默认）；
                                                              4'b0001：VBUS输入电流；
                                                              4'b0010：VBUS输入电压；
                                                              4'b0011:电池输出电压(VOUT)；
                                                              4'b0100：电池输出电压（VBAT)；
                                                              4'b0101：电池电流；
                                                              4'b0110：ACR电压；
                                                              4'b0111：ACR电流；
                                                              4'b1000：VBUS输入参考电流；
                                                              4'b1001：备用；
                                                              4'b1010：DPLUS、DMINUS电压检测；
                                                              4'b1011：外部输入温度检测（TSBAT)；
                                                              4'b1100：外部输入温度检测（TSBUS)；
                                                              4'b1101:外部输入温度检测（TSCHIP)；
                                                              4'b1110:REF分压0.1V用于gain error及offset校正；
                                                              4'b1111：REF分压2.45V用于gain error及offset校正； */
        unsigned char  sc_hkadc_seq_conv   : 1;  /* bit[4-4]: HKADC转换模式选择：
                                                              1'b0：低精度单次转换；
                                                              1'b1：高精度单次多通道转换(默认) */
        unsigned char  sc_hkadc_seq_loop   : 1;  /* bit[5-5]: 多通道循环轮询使能信号：
                                                              1'b0：不使能多通道循环轮询（默认）
                                                              1'b1：使能多通道循环轮询 */
        unsigned char  sc_hkadc_cali_ref   : 1;  /* bit[6-6]: 是否进行实时校正选择信号：
                                                              1'b0：高/低精度HKADC转换（默认）
                                                              1'b1：实时校正 */
        unsigned char  sc_hkadc_en         : 1;  /* bit[7-7]: HKADC使能信号：
                                                              1'b0：不使能（默认）；
                                                              1'b1：使能 */
    } reg;
} SOC_SCHARGER_HKADC_CTRL0_UNION;
#endif
#define SOC_SCHARGER_HKADC_CTRL0_sc_hkadc_chanel_sel_START  (0)
#define SOC_SCHARGER_HKADC_CTRL0_sc_hkadc_chanel_sel_END    (3)
#define SOC_SCHARGER_HKADC_CTRL0_sc_hkadc_seq_conv_START    (4)
#define SOC_SCHARGER_HKADC_CTRL0_sc_hkadc_seq_conv_END      (4)
#define SOC_SCHARGER_HKADC_CTRL0_sc_hkadc_seq_loop_START    (5)
#define SOC_SCHARGER_HKADC_CTRL0_sc_hkadc_seq_loop_END      (5)
#define SOC_SCHARGER_HKADC_CTRL0_sc_hkadc_cali_ref_START    (6)
#define SOC_SCHARGER_HKADC_CTRL0_sc_hkadc_cali_ref_END      (6)
#define SOC_SCHARGER_HKADC_CTRL0_sc_hkadc_en_START          (7)
#define SOC_SCHARGER_HKADC_CTRL0_sc_hkadc_en_END            (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_HKADC_START_UNION
 结构说明  : HKADC_START 寄存器结构定义。地址偏移量:0x2A，初值:0x00，宽度:8
 寄存器说明: HKADC START信号配置
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  sc_hkadc_start_cfg : 1;  /* bit[0-0]: HKADC启动设置寄存器；默认0，写一次地址0xB1（无论0或1），该寄存器发生一次翻转，触发一次数据转换
                                                             （注：禁止在HKADC发起hkadc_start后，转换过程中(hkadc_valid=0)重复发起hkadc_start信号，否则hkadc内部逻辑会发生混乱，需要重新开关hkadc或进行hkadc_reset置‘1’强制逻辑复位）
                                                             注：需要至少持续3个hkadc工作时钟clk周期（不降频为3个125k时钟，降频为3个62.5k时钟）后再清零 */
        unsigned char  reserved           : 7;  /* bit[1-7]: reserved */
    } reg;
} SOC_SCHARGER_HKADC_START_UNION;
#endif
#define SOC_SCHARGER_HKADC_START_sc_hkadc_start_cfg_START  (0)
#define SOC_SCHARGER_HKADC_START_sc_hkadc_start_cfg_END    (0)


/*****************************************************************************
 结构名    : SOC_SCHARGER_HKADC_CTRL1_UNION
 结构说明  : HKADC_CTRL1 寄存器结构定义。地址偏移量:0x2B，初值:0x27，宽度:8
 寄存器说明: HKADC配置寄存器
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  sc_hkadc_avg_times     : 2;  /* bit[0-1]: HKADC采样的平均次数选择：
                                                                 2'b00：2次；
                                                                 2'b01：4次；
                                                                 2'b10：8次；
                                                                 2'b11：16次（默认）；
                                                                 说明：ACR模式下只能是配置为2'b00和2'b01 */
        unsigned char  sc_hkadc_cali_set      : 1;  /* bit[2-2]: HKADC转换出来的数据是否要进行校正计算：
                                                                 1'b0：不需要进行计算；
                                                                 1'b1：需要进行计算（默认） */
        unsigned char  sc_hkadc_cali_realtime : 1;  /* bit[3-3]: HKADC采样误差校正系数的来源选择：
                                                                 1'b0：选择efuse数据（默认）；
                                                                 1'b1：选择寄存器数据 */
        unsigned char  sc_hkadc_chopper       : 1;  /* bit[4-4]: HKADC chopper指示信号：
                                                                 1'b0：正向连接模式（默认）
                                                                 1'b1：反向连接模式 */
        unsigned char  sc_hkadc_chopper_time  : 1;  /* bit[5-5]: Chopper模式置高da_hkadc_chopper到下一次转换的等待时间：
                                                                 1'b0：6us；
                                                                 1'b1：12.5us（默认） */
        unsigned char  sc_hkadc_chopper_en    : 1;  /* bit[6-6]: hkadc chopper功能使能信号：
                                                                 1'b0：不使能chopper功能（默认）
                                                                 1'b1：使能chopper功能 */
        unsigned char  sc_hkadc_cul_time      : 1;  /* bit[7-7]: HKADC采样计算时间配置：
                                                                 1'b0：采样计算时间为25us（默认）
                                                                 1'b1：采样计算时间为28us */
    } reg;
} SOC_SCHARGER_HKADC_CTRL1_UNION;
#endif
#define SOC_SCHARGER_HKADC_CTRL1_sc_hkadc_avg_times_START      (0)
#define SOC_SCHARGER_HKADC_CTRL1_sc_hkadc_avg_times_END        (1)
#define SOC_SCHARGER_HKADC_CTRL1_sc_hkadc_cali_set_START       (2)
#define SOC_SCHARGER_HKADC_CTRL1_sc_hkadc_cali_set_END         (2)
#define SOC_SCHARGER_HKADC_CTRL1_sc_hkadc_cali_realtime_START  (3)
#define SOC_SCHARGER_HKADC_CTRL1_sc_hkadc_cali_realtime_END    (3)
#define SOC_SCHARGER_HKADC_CTRL1_sc_hkadc_chopper_START        (4)
#define SOC_SCHARGER_HKADC_CTRL1_sc_hkadc_chopper_END          (4)
#define SOC_SCHARGER_HKADC_CTRL1_sc_hkadc_chopper_time_START   (5)
#define SOC_SCHARGER_HKADC_CTRL1_sc_hkadc_chopper_time_END     (5)
#define SOC_SCHARGER_HKADC_CTRL1_sc_hkadc_chopper_en_START     (6)
#define SOC_SCHARGER_HKADC_CTRL1_sc_hkadc_chopper_en_END       (6)
#define SOC_SCHARGER_HKADC_CTRL1_sc_hkadc_cul_time_START       (7)
#define SOC_SCHARGER_HKADC_CTRL1_sc_hkadc_cul_time_END         (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_HKADC_SEQ_CH_H_UNION
 结构说明  : HKADC_SEQ_CH_H 寄存器结构定义。地址偏移量:0x2C，初值:0x00，宽度:8
 寄存器说明: HKADC轮询标志位高8bit（高精度）
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  sc_hkadc_seq_chanel_h : 8;  /* bit[0-7]: 高精度轮询标志位，跟通道对应，设置为1的通道才参与轮询
                                                                bit[0]：VBUS输入参考电流；
                                                                bit[1]：reserved；
                                                                bit[2]：DPLUS、DMINUS电压检测；
                                                                bit[3]：外部输入温度检测（TSBAT)；
                                                                bit[4]：外部输入温度检测（TSBUS)；
                                                                bit[5]:外部输入温度检测（TSCHIP)；
                                                                bit[6]:REF分压0.1V用于gain error及offset校正；
                                                                bit[7]：REF分压2.45V用于gain error及offset校正； */
    } reg;
} SOC_SCHARGER_HKADC_SEQ_CH_H_UNION;
#endif
#define SOC_SCHARGER_HKADC_SEQ_CH_H_sc_hkadc_seq_chanel_h_START  (0)
#define SOC_SCHARGER_HKADC_SEQ_CH_H_sc_hkadc_seq_chanel_h_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_HKADC_SEQ_CH_L_UNION
 结构说明  : HKADC_SEQ_CH_L 寄存器结构定义。地址偏移量:0x2D，初值:0x00，宽度:8
 寄存器说明: HKADC轮询标志位低8bit（高精度）
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  sc_hkadc_seq_chanel_l : 8;  /* bit[0-7]: 轮询标志位，跟通道对应，设置为1的通道才参与轮询
                                                                bit[0]：VUSB输入电压（默认）；
                                                                bit[1]：VBUS输入电流；
                                                                bit[2]：VBUS输入电压；
                                                                bit[3]:电池输出电压(VOUT)；
                                                                bit[4]：电池输出电压（VBAT)；
                                                                bit[5]：电池电流；
                                                                bit[6]：ACR电压；
                                                                bit[7]：reserved */
    } reg;
} SOC_SCHARGER_HKADC_SEQ_CH_L_UNION;
#endif
#define SOC_SCHARGER_HKADC_SEQ_CH_L_sc_hkadc_seq_chanel_l_START  (0)
#define SOC_SCHARGER_HKADC_SEQ_CH_L_sc_hkadc_seq_chanel_l_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_HKADC_CTRL2_UNION
 结构说明  : HKADC_CTRL2 寄存器结构定义。地址偏移量:0x2E，初值:0x10，宽度:8
 寄存器说明: HKADC配置寄存器
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  sc_soh_test_mode : 1;  /* bit[0-0]: 电池过压保护时间档位测试模式（测试使用，不对产品开放）：
                                                           1'b0：正常模式，时间档位为10min，20min，30min，40min； （默认）
                                                           1'b1：测试模式，时间档位为30s */
        unsigned char  reserved_0       : 3;  /* bit[1-3]: reserved */
        unsigned char  sc_sohov_timer   : 2;  /* bit[4-5]: 电池过压保护定时器档位配置信号：
                                                           2'b00：10min；
                                                           2'b01：20min（默认）；
                                                           2'b10：30min；
                                                           2'bb11：40min。 */
        unsigned char  reserved_1       : 1;  /* bit[6-6]: reserved */
        unsigned char  sc_sohov_en      : 1;  /* bit[7-7]: 电池过压保护使能信号：
                                                           1'b0：不使能（默认）
                                                           1'b1：使能 */
    } reg;
} SOC_SCHARGER_HKADC_CTRL2_UNION;
#endif
#define SOC_SCHARGER_HKADC_CTRL2_sc_soh_test_mode_START  (0)
#define SOC_SCHARGER_HKADC_CTRL2_sc_soh_test_mode_END    (0)
#define SOC_SCHARGER_HKADC_CTRL2_sc_sohov_timer_START    (4)
#define SOC_SCHARGER_HKADC_CTRL2_sc_sohov_timer_END      (5)
#define SOC_SCHARGER_HKADC_CTRL2_sc_sohov_en_START       (7)
#define SOC_SCHARGER_HKADC_CTRL2_sc_sohov_en_END         (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_SOH_DISCHG_EN_UNION
 结构说明  : SOH_DISCHG_EN 寄存器结构定义。地址偏移量:0x2F，初值:0xAC，宽度:8
 寄存器说明: 放电控制寄存器
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  sc_soh_dischg_en : 8;  /* bit[0-7]: 放电控制信号配置寄存器。控制放电，高电平表示放电。
                                                           当写入0x5A时候，放电控制信号为高电平，进行放电；
                                                           当写入0xAC时候，放电控制信号为低电平，不进行放电。
                                                           当写入其他值，放电控制信号输出值不跳变，保持； */
    } reg;
} SOC_SCHARGER_SOH_DISCHG_EN_UNION;
#endif
#define SOC_SCHARGER_SOH_DISCHG_EN_sc_soh_dischg_en_START  (0)
#define SOC_SCHARGER_SOH_DISCHG_EN_sc_soh_dischg_en_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_ACR_CTRL_UNION
 结构说明  : ACR_CTRL 寄存器结构定义。地址偏移量:0x30，初值:0x01，宽度:8
 寄存器说明: ACR 配置寄存器
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  sc_acr_pulse_num : 2;  /* bit[0-1]: acr_pulse在一个en_pulse高电平期间内的周期数n_acr：
                                                           2’b00：8；
                                                           2’b01：16（默认）；
                                                           2’b10：24；
                                                           2’b11：32。 */
        unsigned char  reserved_0       : 2;  /* bit[2-3]: reserved */
        unsigned char  sc_acr_en        : 1;  /* bit[4-4]: ACR使能信号：
                                                           1'b0：不使能（默认）；
                                                           1'b1：使能。 */
        unsigned char  reserved_1       : 3;  /* bit[5-7]: reserved */
    } reg;
} SOC_SCHARGER_ACR_CTRL_UNION;
#endif
#define SOC_SCHARGER_ACR_CTRL_sc_acr_pulse_num_START  (0)
#define SOC_SCHARGER_ACR_CTRL_sc_acr_pulse_num_END    (1)
#define SOC_SCHARGER_ACR_CTRL_sc_acr_en_START         (4)
#define SOC_SCHARGER_ACR_CTRL_sc_acr_en_END           (4)


/*****************************************************************************
 结构名    : SOC_SCHARGER_HKADC_RD_SEQ_UNION
 结构说明  : HKADC_RD_SEQ 寄存器结构定义。地址偏移量:0x31，初值:0x00，宽度:8
 寄存器说明: HKADC循环轮询模式数据读请求信号
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  sc_hkadc_rd_req : 1;  /* bit[0-0]: 多通道循环读请求信号：SOC对该信号对应地址进行一次写操作，数字内部信号hkadc_rd_req进行一次翻转，使用hkadc_rd_req信号的双沿对数据进行拍照处理 */
        unsigned char  reserved        : 7;  /* bit[1-7]: reserved */
    } reg;
} SOC_SCHARGER_HKADC_RD_SEQ_UNION;
#endif
#define SOC_SCHARGER_HKADC_RD_SEQ_sc_hkadc_rd_req_START  (0)
#define SOC_SCHARGER_HKADC_RD_SEQ_sc_hkadc_rd_req_END    (0)


/*****************************************************************************
 结构名    : SOC_SCHARGER_PULSE_NON_CHG_FLAG_UNION
 结构说明  : PULSE_NON_CHG_FLAG 寄存器结构定义。地址偏移量:0x32，初值:0x00，宽度:8
 寄存器说明: 当前ADC采数是处于充电状态还是非充电状态
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  pulse_non_chg_flag : 1;  /* bit[0-0]: 当前ADC采数是处于充电状态还是非充电状态：
                                                             1'b0：当前不处于脉冲充电模式非充电状态
                                                             1'b1：当前处于脉冲充电模式非充电状态。 */
        unsigned char  reserved_0         : 3;  /* bit[1-3]: reserved */
        unsigned char  hkadc_data_valid   : 1;  /* bit[4-4]: HKADC数据采样完成指示信号：
                                                             1’b0：未开始转换或正在进行数据转换；
                                                             1’b1：数据转换完成 */
        unsigned char  reserved_1         : 3;  /* bit[5-7]: reserved */
    } reg;
} SOC_SCHARGER_PULSE_NON_CHG_FLAG_UNION;
#endif
#define SOC_SCHARGER_PULSE_NON_CHG_FLAG_pulse_non_chg_flag_START  (0)
#define SOC_SCHARGER_PULSE_NON_CHG_FLAG_pulse_non_chg_flag_END    (0)
#define SOC_SCHARGER_PULSE_NON_CHG_FLAG_hkadc_data_valid_START    (4)
#define SOC_SCHARGER_PULSE_NON_CHG_FLAG_hkadc_data_valid_END      (4)


/*****************************************************************************
 结构名    : SOC_SCHARGER_VUSB_ADC_L_UNION
 结构说明  : VUSB_ADC_L 寄存器结构定义。地址偏移量:0x33，初值:0x00，宽度:8
 寄存器说明: VUSB采样结果低8bit
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  vusb_adc_l : 8;  /* bit[0-7]: VUSB采样结果低8bit */
    } reg;
} SOC_SCHARGER_VUSB_ADC_L_UNION;
#endif
#define SOC_SCHARGER_VUSB_ADC_L_vusb_adc_l_START  (0)
#define SOC_SCHARGER_VUSB_ADC_L_vusb_adc_l_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_VUSB_ADC_H_UNION
 结构说明  : VUSB_ADC_H 寄存器结构定义。地址偏移量:0x34，初值:0x00，宽度:8
 寄存器说明: VUSB采样结果高6bit
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  vusb_adc_h : 6;  /* bit[0-5]: VUSB采样结果高6bit */
        unsigned char  reserved   : 2;  /* bit[6-7]: reserved */
    } reg;
} SOC_SCHARGER_VUSB_ADC_H_UNION;
#endif
#define SOC_SCHARGER_VUSB_ADC_H_vusb_adc_h_START  (0)
#define SOC_SCHARGER_VUSB_ADC_H_vusb_adc_h_END    (5)


/*****************************************************************************
 结构名    : SOC_SCHARGER_IBUS_ADC_L_UNION
 结构说明  : IBUS_ADC_L 寄存器结构定义。地址偏移量:0x35，初值:0x00，宽度:8
 寄存器说明: IBUS采样结果低8bit
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  ibus_adc_l : 8;  /* bit[0-7]: IBUS采样结果低8bit */
    } reg;
} SOC_SCHARGER_IBUS_ADC_L_UNION;
#endif
#define SOC_SCHARGER_IBUS_ADC_L_ibus_adc_l_START  (0)
#define SOC_SCHARGER_IBUS_ADC_L_ibus_adc_l_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_IBUS_ADC_H_UNION
 结构说明  : IBUS_ADC_H 寄存器结构定义。地址偏移量:0x36，初值:0x00，宽度:8
 寄存器说明: IBUS采样结果高5bit
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  ibus_adc_h : 5;  /* bit[0-4]: IBUS采样结果高5bit */
        unsigned char  reserved   : 3;  /* bit[5-7]: reserved */
    } reg;
} SOC_SCHARGER_IBUS_ADC_H_UNION;
#endif
#define SOC_SCHARGER_IBUS_ADC_H_ibus_adc_h_START  (0)
#define SOC_SCHARGER_IBUS_ADC_H_ibus_adc_h_END    (4)


/*****************************************************************************
 结构名    : SOC_SCHARGER_VBUS_ADC_L_UNION
 结构说明  : VBUS_ADC_L 寄存器结构定义。地址偏移量:0x37，初值:0x00，宽度:8
 寄存器说明: VBUS采样结果低8bit
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  vbus_adc_l : 8;  /* bit[0-7]: VBUS采样结果低8bit */
    } reg;
} SOC_SCHARGER_VBUS_ADC_L_UNION;
#endif
#define SOC_SCHARGER_VBUS_ADC_L_vbus_adc_l_START  (0)
#define SOC_SCHARGER_VBUS_ADC_L_vbus_adc_l_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_VBUS_ADC_H_UNION
 结构说明  : VBUS_ADC_H 寄存器结构定义。地址偏移量:0x38，初值:0x00，宽度:8
 寄存器说明: VBUS采样结果高6bit
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  vbus_adc_h : 6;  /* bit[0-5]: VBUS采样结果高6bit */
        unsigned char  reserved   : 2;  /* bit[6-7]: reserved */
    } reg;
} SOC_SCHARGER_VBUS_ADC_H_UNION;
#endif
#define SOC_SCHARGER_VBUS_ADC_H_vbus_adc_h_START  (0)
#define SOC_SCHARGER_VBUS_ADC_H_vbus_adc_h_END    (5)


/*****************************************************************************
 结构名    : SOC_SCHARGER_VOUT_ADC_L_UNION
 结构说明  : VOUT_ADC_L 寄存器结构定义。地址偏移量:0x39，初值:0x00，宽度:8
 寄存器说明: VOUT采样结果低8bit
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  vout_adc_l : 8;  /* bit[0-7]: VOUT采样结果低8bit */
    } reg;
} SOC_SCHARGER_VOUT_ADC_L_UNION;
#endif
#define SOC_SCHARGER_VOUT_ADC_L_vout_adc_l_START  (0)
#define SOC_SCHARGER_VOUT_ADC_L_vout_adc_l_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_VOUT_ADC_H_UNION
 结构说明  : VOUT_ADC_H 寄存器结构定义。地址偏移量:0x3A，初值:0x00，宽度:8
 寄存器说明: VOUT采样结果高5bit
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  vout_adc_h : 5;  /* bit[0-4]: VOUT采样结果高5bit */
        unsigned char  reserved   : 3;  /* bit[5-7]: reserved */
    } reg;
} SOC_SCHARGER_VOUT_ADC_H_UNION;
#endif
#define SOC_SCHARGER_VOUT_ADC_H_vout_adc_h_START  (0)
#define SOC_SCHARGER_VOUT_ADC_H_vout_adc_h_END    (4)


/*****************************************************************************
 结构名    : SOC_SCHARGER_VBAT_ADC_L_UNION
 结构说明  : VBAT_ADC_L 寄存器结构定义。地址偏移量:0x3B，初值:0x00，宽度:8
 寄存器说明: VBAT采样结果低8bit
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  vbat_adc_l : 8;  /* bit[0-7]: VBAT采样结果低8bit */
    } reg;
} SOC_SCHARGER_VBAT_ADC_L_UNION;
#endif
#define SOC_SCHARGER_VBAT_ADC_L_vbat_adc_l_START  (0)
#define SOC_SCHARGER_VBAT_ADC_L_vbat_adc_l_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_VBAT_ADC_H_UNION
 结构说明  : VBAT_ADC_H 寄存器结构定义。地址偏移量:0x3C，初值:0x00，宽度:8
 寄存器说明: VBAT采样结果高5bit
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  vbat_adc_h : 5;  /* bit[0-4]: VBAT采样结果高5bit */
        unsigned char  reserved   : 3;  /* bit[5-7]: reserved */
    } reg;
} SOC_SCHARGER_VBAT_ADC_H_UNION;
#endif
#define SOC_SCHARGER_VBAT_ADC_H_vbat_adc_h_START  (0)
#define SOC_SCHARGER_VBAT_ADC_H_vbat_adc_h_END    (4)


/*****************************************************************************
 结构名    : SOC_SCHARGER_IBAT_ADC_L_UNION
 结构说明  : IBAT_ADC_L 寄存器结构定义。地址偏移量:0x3D，初值:0x00，宽度:8
 寄存器说明: IBAT采样结果低8bit
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  ibat_adc_l : 8;  /* bit[0-7]: IBAT采样结果低8bit */
    } reg;
} SOC_SCHARGER_IBAT_ADC_L_UNION;
#endif
#define SOC_SCHARGER_IBAT_ADC_L_ibat_adc_l_START  (0)
#define SOC_SCHARGER_IBAT_ADC_L_ibat_adc_l_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_IBAT_ADC_H_UNION
 结构说明  : IBAT_ADC_H 寄存器结构定义。地址偏移量:0x3E，初值:0x00，宽度:8
 寄存器说明: IBAT采样结果高6bit
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  ibat_adc_h : 6;  /* bit[0-5]: IBAT采样结果高6bit */
        unsigned char  reserved   : 2;  /* bit[6-7]: reserved */
    } reg;
} SOC_SCHARGER_IBAT_ADC_H_UNION;
#endif
#define SOC_SCHARGER_IBAT_ADC_H_ibat_adc_h_START  (0)
#define SOC_SCHARGER_IBAT_ADC_H_ibat_adc_h_END    (5)


/*****************************************************************************
 结构名    : SOC_SCHARGER_HKADC_DATA_6_L_UNION
 结构说明  : HKADC_DATA_6_L 寄存器结构定义。地址偏移量:0x3F，初值:0x00，宽度:8
 寄存器说明: 通道6 ADC转换结果低8bit
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  hkadc_data_6_l : 8;  /* bit[0-7]: 通道6 ADC转换结果低8bit */
    } reg;
} SOC_SCHARGER_HKADC_DATA_6_L_UNION;
#endif
#define SOC_SCHARGER_HKADC_DATA_6_L_hkadc_data_6_l_START  (0)
#define SOC_SCHARGER_HKADC_DATA_6_L_hkadc_data_6_l_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_HKADC_DATA_6_H_UNION
 结构说明  : HKADC_DATA_6_H 寄存器结构定义。地址偏移量:0x40，初值:0x00，宽度:8
 寄存器说明: 通道6 ADC转换结果高4bit
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  hkadc_data_6_h : 4;  /* bit[0-3]: 通道6 ADC转换结果高4bit */
        unsigned char  reserved       : 4;  /* bit[4-7]: reserved */
    } reg;
} SOC_SCHARGER_HKADC_DATA_6_H_UNION;
#endif
#define SOC_SCHARGER_HKADC_DATA_6_H_hkadc_data_6_h_START  (0)
#define SOC_SCHARGER_HKADC_DATA_6_H_hkadc_data_6_h_END    (3)


/*****************************************************************************
 结构名    : SOC_SCHARGER_HKADC_DATA_7_L_UNION
 结构说明  : HKADC_DATA_7_L 寄存器结构定义。地址偏移量:0x41，初值:0x00，宽度:8
 寄存器说明: 通道7 ADC转换结果低8bit
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  hkadc_data_7_l : 8;  /* bit[0-7]: 通道7 ADC转换结果低8bit */
    } reg;
} SOC_SCHARGER_HKADC_DATA_7_L_UNION;
#endif
#define SOC_SCHARGER_HKADC_DATA_7_L_hkadc_data_7_l_START  (0)
#define SOC_SCHARGER_HKADC_DATA_7_L_hkadc_data_7_l_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_HKADC_DATA_7_H_UNION
 结构说明  : HKADC_DATA_7_H 寄存器结构定义。地址偏移量:0x42，初值:0x00，宽度:8
 寄存器说明: 通道7 ADC转换结果高4bit
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  hkadc_data_7_h : 4;  /* bit[0-3]: 通道7 ADC转换结果高4bit */
        unsigned char  reserved       : 4;  /* bit[4-7]: reserved */
    } reg;
} SOC_SCHARGER_HKADC_DATA_7_H_UNION;
#endif
#define SOC_SCHARGER_HKADC_DATA_7_H_hkadc_data_7_h_START  (0)
#define SOC_SCHARGER_HKADC_DATA_7_H_hkadc_data_7_h_END    (3)


/*****************************************************************************
 结构名    : SOC_SCHARGER_HKADC_DATA_8_L_UNION
 结构说明  : HKADC_DATA_8_L 寄存器结构定义。地址偏移量:0x43，初值:0x00，宽度:8
 寄存器说明: 通道8 ADC转换结果低8bit
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  hkadc_data_8_l : 8;  /* bit[0-7]: 通道8 ADC转换结果低8bit */
    } reg;
} SOC_SCHARGER_HKADC_DATA_8_L_UNION;
#endif
#define SOC_SCHARGER_HKADC_DATA_8_L_hkadc_data_8_l_START  (0)
#define SOC_SCHARGER_HKADC_DATA_8_L_hkadc_data_8_l_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_HKADC_DATA_8_H_UNION
 结构说明  : HKADC_DATA_8_H 寄存器结构定义。地址偏移量:0x44，初值:0x00，宽度:8
 寄存器说明: 通道8 ADC转换结果高4bit
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  hkadc_data_8_h : 4;  /* bit[0-3]: 通道8 ADC转换结果高4bit */
        unsigned char  reserved       : 4;  /* bit[4-7]: reserved */
    } reg;
} SOC_SCHARGER_HKADC_DATA_8_H_UNION;
#endif
#define SOC_SCHARGER_HKADC_DATA_8_H_hkadc_data_8_h_START  (0)
#define SOC_SCHARGER_HKADC_DATA_8_H_hkadc_data_8_h_END    (3)


/*****************************************************************************
 结构名    : SOC_SCHARGER_HKADC_DATA_9_L_UNION
 结构说明  : HKADC_DATA_9_L 寄存器结构定义。地址偏移量:0x45，初值:0x00，宽度:8
 寄存器说明: 通道9 ADC转换结果低8bit
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  hkadc_data_9_l : 8;  /* bit[0-7]: 通道9 ADC转换结果低8bit */
    } reg;
} SOC_SCHARGER_HKADC_DATA_9_L_UNION;
#endif
#define SOC_SCHARGER_HKADC_DATA_9_L_hkadc_data_9_l_START  (0)
#define SOC_SCHARGER_HKADC_DATA_9_L_hkadc_data_9_l_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_HKADC_DATA_9_H_UNION
 结构说明  : HKADC_DATA_9_H 寄存器结构定义。地址偏移量:0x46，初值:0x00，宽度:8
 寄存器说明: 通道9 ADC转换结果高4bit
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  hkadc_data_9_h : 4;  /* bit[0-3]: 通道9 ADC转换结果高4bit */
        unsigned char  reserved       : 4;  /* bit[4-7]: reserved */
    } reg;
} SOC_SCHARGER_HKADC_DATA_9_H_UNION;
#endif
#define SOC_SCHARGER_HKADC_DATA_9_H_hkadc_data_9_h_START  (0)
#define SOC_SCHARGER_HKADC_DATA_9_H_hkadc_data_9_h_END    (3)


/*****************************************************************************
 结构名    : SOC_SCHARGER_HKADC_DATA_10_L_UNION
 结构说明  : HKADC_DATA_10_L 寄存器结构定义。地址偏移量:0x47，初值:0x00，宽度:8
 寄存器说明: 通道10 ADC转换结果低8bit
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  hkadc_data_10_l : 8;  /* bit[0-7]: 通道10 ADC转换结果低8bit */
    } reg;
} SOC_SCHARGER_HKADC_DATA_10_L_UNION;
#endif
#define SOC_SCHARGER_HKADC_DATA_10_L_hkadc_data_10_l_START  (0)
#define SOC_SCHARGER_HKADC_DATA_10_L_hkadc_data_10_l_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_HKADC_DATA_10_H_UNION
 结构说明  : HKADC_DATA_10_H 寄存器结构定义。地址偏移量:0x48，初值:0x00，宽度:8
 寄存器说明: 通道10 ADC转换结果高4bit
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  hkadc_data_10_h : 4;  /* bit[0-3]: 通道10 ADC转换结果高4bit */
        unsigned char  reserved        : 4;  /* bit[4-7]: reserved */
    } reg;
} SOC_SCHARGER_HKADC_DATA_10_H_UNION;
#endif
#define SOC_SCHARGER_HKADC_DATA_10_H_hkadc_data_10_h_START  (0)
#define SOC_SCHARGER_HKADC_DATA_10_H_hkadc_data_10_h_END    (3)


/*****************************************************************************
 结构名    : SOC_SCHARGER_HKADC_DATA_11_L_UNION
 结构说明  : HKADC_DATA_11_L 寄存器结构定义。地址偏移量:0x49，初值:0x00，宽度:8
 寄存器说明: 通道11 ADC转换结果低8bit
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  hkadc_data_11_l : 8;  /* bit[0-7]: 通道11 ADC转换结果低8bit */
    } reg;
} SOC_SCHARGER_HKADC_DATA_11_L_UNION;
#endif
#define SOC_SCHARGER_HKADC_DATA_11_L_hkadc_data_11_l_START  (0)
#define SOC_SCHARGER_HKADC_DATA_11_L_hkadc_data_11_l_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_HKADC_DATA_11_H_UNION
 结构说明  : HKADC_DATA_11_H 寄存器结构定义。地址偏移量:0x4A，初值:0x00，宽度:8
 寄存器说明: 通道11 ADC转换结果高4bit
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  hkadc_data_11_h : 4;  /* bit[0-3]: 通道11 ADC转换结果高4bit */
        unsigned char  reserved        : 4;  /* bit[4-7]: reserved */
    } reg;
} SOC_SCHARGER_HKADC_DATA_11_H_UNION;
#endif
#define SOC_SCHARGER_HKADC_DATA_11_H_hkadc_data_11_h_START  (0)
#define SOC_SCHARGER_HKADC_DATA_11_H_hkadc_data_11_h_END    (3)


/*****************************************************************************
 结构名    : SOC_SCHARGER_HKADC_DATA_12_L_UNION
 结构说明  : HKADC_DATA_12_L 寄存器结构定义。地址偏移量:0x4B，初值:0x00，宽度:8
 寄存器说明: 通道12 ADC转换结果低8bit
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  hkadc_data_12_l : 8;  /* bit[0-7]: 通道12 ADC转换结果低8bit */
    } reg;
} SOC_SCHARGER_HKADC_DATA_12_L_UNION;
#endif
#define SOC_SCHARGER_HKADC_DATA_12_L_hkadc_data_12_l_START  (0)
#define SOC_SCHARGER_HKADC_DATA_12_L_hkadc_data_12_l_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_HKADC_DATA_12_H_UNION
 结构说明  : HKADC_DATA_12_H 寄存器结构定义。地址偏移量:0x4C，初值:0x00，宽度:8
 寄存器说明: 通道12 ADC转换结果高4bit
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  hkadc_data_12_h : 4;  /* bit[0-3]: 通道12 ADC转换结果高4bit */
        unsigned char  reserved        : 4;  /* bit[4-7]: reserved */
    } reg;
} SOC_SCHARGER_HKADC_DATA_12_H_UNION;
#endif
#define SOC_SCHARGER_HKADC_DATA_12_H_hkadc_data_12_h_START  (0)
#define SOC_SCHARGER_HKADC_DATA_12_H_hkadc_data_12_h_END    (3)


/*****************************************************************************
 结构名    : SOC_SCHARGER_HKADC_DATA_13_L_UNION
 结构说明  : HKADC_DATA_13_L 寄存器结构定义。地址偏移量:0x4D，初值:0x00，宽度:8
 寄存器说明: 通道13 ADC转换结果低8bit
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  hkadc_data_13_l : 8;  /* bit[0-7]: 通道13 ADC转换结果低8bit */
    } reg;
} SOC_SCHARGER_HKADC_DATA_13_L_UNION;
#endif
#define SOC_SCHARGER_HKADC_DATA_13_L_hkadc_data_13_l_START  (0)
#define SOC_SCHARGER_HKADC_DATA_13_L_hkadc_data_13_l_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_HKADC_DATA_13_H_UNION
 结构说明  : HKADC_DATA_13_H 寄存器结构定义。地址偏移量:0x4E，初值:0x00，宽度:8
 寄存器说明: 通道13 ADC转换结果高4bit
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  hkadc_data_13_h : 4;  /* bit[0-3]: 通道13 ADC转换结果高4bit */
        unsigned char  reserved        : 4;  /* bit[4-7]: reserved */
    } reg;
} SOC_SCHARGER_HKADC_DATA_13_H_UNION;
#endif
#define SOC_SCHARGER_HKADC_DATA_13_H_hkadc_data_13_h_START  (0)
#define SOC_SCHARGER_HKADC_DATA_13_H_hkadc_data_13_h_END    (3)


/*****************************************************************************
 结构名    : SOC_SCHARGER_HKADC_DATA_14_L_UNION
 结构说明  : HKADC_DATA_14_L 寄存器结构定义。地址偏移量:0x4F，初值:0x00，宽度:8
 寄存器说明: 通道14 ADC转换结果低8bit
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  hkadc_data_14_l : 8;  /* bit[0-7]: 通道14 ADC转换结果低8bit */
    } reg;
} SOC_SCHARGER_HKADC_DATA_14_L_UNION;
#endif
#define SOC_SCHARGER_HKADC_DATA_14_L_hkadc_data_14_l_START  (0)
#define SOC_SCHARGER_HKADC_DATA_14_L_hkadc_data_14_l_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_HKADC_DATA_14_H_UNION
 结构说明  : HKADC_DATA_14_H 寄存器结构定义。地址偏移量:0x50，初值:0x00，宽度:8
 寄存器说明: 通道14 ADC转换结果高4bit
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  hkadc_data_14_h : 4;  /* bit[0-3]: 通道14 ADC转换结果高4bit */
        unsigned char  reserved        : 4;  /* bit[4-7]: reserved */
    } reg;
} SOC_SCHARGER_HKADC_DATA_14_H_UNION;
#endif
#define SOC_SCHARGER_HKADC_DATA_14_H_hkadc_data_14_h_START  (0)
#define SOC_SCHARGER_HKADC_DATA_14_H_hkadc_data_14_h_END    (3)


/*****************************************************************************
 结构名    : SOC_SCHARGER_HKADC_DATA_15_L_UNION
 结构说明  : HKADC_DATA_15_L 寄存器结构定义。地址偏移量:0x51，初值:0x00，宽度:8
 寄存器说明: 通道15 ADC转换结果低8bit
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  hkadc_data_15_l : 8;  /* bit[0-7]: 通道15 ADC转换结果低8bit */
    } reg;
} SOC_SCHARGER_HKADC_DATA_15_L_UNION;
#endif
#define SOC_SCHARGER_HKADC_DATA_15_L_hkadc_data_15_l_START  (0)
#define SOC_SCHARGER_HKADC_DATA_15_L_hkadc_data_15_l_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_HKADC_DATA_15_H_UNION
 结构说明  : HKADC_DATA_15_H 寄存器结构定义。地址偏移量:0x52，初值:0x00，宽度:8
 寄存器说明: 通道15 ADC转换结果高4bit
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  hkadc_data_15_h : 4;  /* bit[0-3]: 通道15 ADC转换结果高4bit */
        unsigned char  reserved        : 4;  /* bit[4-7]: reserved */
    } reg;
} SOC_SCHARGER_HKADC_DATA_15_H_UNION;
#endif
#define SOC_SCHARGER_HKADC_DATA_15_H_hkadc_data_15_h_START  (0)
#define SOC_SCHARGER_HKADC_DATA_15_H_hkadc_data_15_h_END    (3)


/*****************************************************************************
 结构名    : SOC_SCHARGER_PD_CDR_CFG_0_UNION
 结构说明  : PD_CDR_CFG_0 寄存器结构定义。地址偏移量:0x58，初值:0x16，宽度:8
 寄存器说明: PD模块BMC时钟恢复电路配置寄存器
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  pd_cdr_cfg_0 : 8;  /* bit[0-7]: PD模块BMC时钟恢复电路配置寄存器
                                                       bit[0]: pd_bmc_cdr_edge_sel
                                                       bit[2:1]:pd_rx_cdr_edge_cfg
                                                       0X:双沿检测
                                                       10:上升沿检测
                                                       11:下降沿检测
                                                       默认：11
                                                       bit[4:3]:pd_rx_preamble_cfg
                                                       00：连续32比特符合判定为Preamble
                                                       01：连续24比特符合判定为Preamble
                                                       10: 连续16比特符合判定为Preamble
                                                       11: 连续8比特符合判定为Preamble
                                                       默认：10
                                                       bit[5]:pd_bmc_dds_init_sel
                                                       1：
                                                       0：
                                                       默认：0 */
    } reg;
} SOC_SCHARGER_PD_CDR_CFG_0_UNION;
#endif
#define SOC_SCHARGER_PD_CDR_CFG_0_pd_cdr_cfg_0_START  (0)
#define SOC_SCHARGER_PD_CDR_CFG_0_pd_cdr_cfg_0_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_PD_CDR_CFG_1_UNION
 结构说明  : PD_CDR_CFG_1 寄存器结构定义。地址偏移量:0x59，初值:0x21，宽度:8
 寄存器说明: PD模块BMC时钟恢复电路配置寄存器
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  pd_cdr_cfg_1 : 8;  /* bit[0-7]: PD模块BMC时钟恢复电路配置寄存器
                                                       bit[2:0]：CDR Kp档位设置
                                                       bit[7:4]：CDR Ki档位设置 */
    } reg;
} SOC_SCHARGER_PD_CDR_CFG_1_UNION;
#endif
#define SOC_SCHARGER_PD_CDR_CFG_1_pd_cdr_cfg_1_START  (0)
#define SOC_SCHARGER_PD_CDR_CFG_1_pd_cdr_cfg_1_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_PD_DBG_CFG_0_UNION
 结构说明  : PD_DBG_CFG_0 寄存器结构定义。地址偏移量:0x5A，初值:0x9A，宽度:8
 寄存器说明: PD模块Debug用配置寄存器
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  pd_dbg_cfg_0 : 8;  /* bit[0-7]: PD模块Debug用配置寄存器
                                                       CDR FCW预留配置值低8比特 */
    } reg;
} SOC_SCHARGER_PD_DBG_CFG_0_UNION;
#endif
#define SOC_SCHARGER_PD_DBG_CFG_0_pd_dbg_cfg_0_START  (0)
#define SOC_SCHARGER_PD_DBG_CFG_0_pd_dbg_cfg_0_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_PD_DBG_CFG_1_UNION
 结构说明  : PD_DBG_CFG_1 寄存器结构定义。地址偏移量:0x5B，初值:0x19，宽度:8
 寄存器说明: PD模块Debug用配置寄存器
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  pd_dbg_cfg_1 : 8;  /* bit[0-7]: PD模块Debug用配置寄存器
                                                       bit[4:0]：CDR FCW预留配置值高5比特
                                                       bit[5]：pd_disch_thres_sel，自动放电至vSafe0V的时间阈值tSave0V设置位
                                                       1b:250ms
                                                       0b:400ms
                                                       默认0
                                                       bit[6]：wd_chg_reset_disable
                                                       1b：看门狗狗叫不复位TypeC协议与CC1/CC2上下拉电阻；
                                                       0b：看门狗狗叫会复位TypeC协议，并且断开CC1/CC2为Open（持续260ms）后设置为双Rd下拉；
                                                       默认0
                                                       bit[7]:pd_rx_sop_window_disable
                                                       0b:使能RX SOP检测窗；
                                                       1b:不使能RX SOP检测窗；
                                                       默认0 */
    } reg;
} SOC_SCHARGER_PD_DBG_CFG_1_UNION;
#endif
#define SOC_SCHARGER_PD_DBG_CFG_1_pd_dbg_cfg_1_START  (0)
#define SOC_SCHARGER_PD_DBG_CFG_1_pd_dbg_cfg_1_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_PD_DBG_RO_0_UNION
 结构说明  : PD_DBG_RO_0 寄存器结构定义。地址偏移量:0x5C，初值:0x00，宽度:8
 寄存器说明: PD模块Debug用回读寄存器
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  pd_dbg_status_0 : 8;  /* bit[0-7]: PD模块Debug用回读寄存器 */
    } reg;
} SOC_SCHARGER_PD_DBG_RO_0_UNION;
#endif
#define SOC_SCHARGER_PD_DBG_RO_0_pd_dbg_status_0_START  (0)
#define SOC_SCHARGER_PD_DBG_RO_0_pd_dbg_status_0_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_PD_DBG_RO_1_UNION
 结构说明  : PD_DBG_RO_1 寄存器结构定义。地址偏移量:0x5D，初值:0x00，宽度:8
 寄存器说明: PD模块Debug用回读寄存器
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  pd_dbg_status_1 : 8;  /* bit[0-7]: PD模块Debug用回读寄存器 */
    } reg;
} SOC_SCHARGER_PD_DBG_RO_1_UNION;
#endif
#define SOC_SCHARGER_PD_DBG_RO_1_pd_dbg_status_1_START  (0)
#define SOC_SCHARGER_PD_DBG_RO_1_pd_dbg_status_1_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_PD_DBG_RO_2_UNION
 结构说明  : PD_DBG_RO_2 寄存器结构定义。地址偏移量:0x5E，初值:0x00，宽度:8
 寄存器说明: PD模块Debug用回读寄存器
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  pd_dbg_status_2 : 8;  /* bit[0-7]: PD模块Debug用回读寄存器 */
    } reg;
} SOC_SCHARGER_PD_DBG_RO_2_UNION;
#endif
#define SOC_SCHARGER_PD_DBG_RO_2_pd_dbg_status_2_START  (0)
#define SOC_SCHARGER_PD_DBG_RO_2_pd_dbg_status_2_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_PD_DBG_RO_3_UNION
 结构说明  : PD_DBG_RO_3 寄存器结构定义。地址偏移量:0x5F，初值:0xFF，宽度:8
 寄存器说明: PD模块Debug用回读寄存器
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  pd_dbg_status_3 : 8;  /* bit[0-7]: PD模块Debug用回读寄存器 */
    } reg;
} SOC_SCHARGER_PD_DBG_RO_3_UNION;
#endif
#define SOC_SCHARGER_PD_DBG_RO_3_pd_dbg_status_3_START  (0)
#define SOC_SCHARGER_PD_DBG_RO_3_pd_dbg_status_3_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_IRQ_FAKE_SEL_UNION
 结构说明  : IRQ_FAKE_SEL 寄存器结构定义。地址偏移量:0x60，初值:0xAC，宽度:8
 寄存器说明: 实际中断和伪中断选择信号
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  sc_irq_sel : 8;  /* bit[0-7]: 实际中断与伪中断选择信号：
                                                     0xAC：中断源为实际中断；
                                                     0x5A：中断源为伪中断；
                                                     默认为0xAC */
    } reg;
} SOC_SCHARGER_IRQ_FAKE_SEL_UNION;
#endif
#define SOC_SCHARGER_IRQ_FAKE_SEL_sc_irq_sel_START  (0)
#define SOC_SCHARGER_IRQ_FAKE_SEL_sc_irq_sel_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_IRQ_FAKE_UNION
 结构说明  : IRQ_FAKE 寄存器结构定义。地址偏移量:0x61，初值:0xFF，宽度:8
 寄存器说明: 伪中断源
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  sc_fake_irq : 8;  /* bit[0-7]: 伪中断源：
                                                      8’d0 irq_vbus_ovp
                                                      8’d1 irq_vbus_uvp
                                                      8’d2 irq_vbat_ovp
                                                      8’d3 irq_otg_ovp
                                                      8’d4 irq_otg_uvp
                                                      8’d5 irq_regn_ocp
                                                      8’d6 irq_buck_scp
                                                      8’d7 irq_buck_ocp
                                                      8’d8 irq_otg_scp
                                                      8’d9 irq_otg_ocp
                                                      8’d10 irq_chg_batfet_ocp
                                                      8’d11 irq_trickle_chg_fault
                                                      8’d12 irq_pre_chg_fault
                                                      8’d13 irq_fast_chg_fault
                                                      8’d14 irq_otmp_jreg
                                                      8’d15 irq_otmp_140
                                                      8’d16 irq_chg_chgstate[1:0]
                                                      8’d17 irq_chg_rechg_state
                                                      8’d18 irq_sleep_mod
                                                      8’d19 irq_reversbst
                                                      8’d20 irq_vusb_buck_ovp
                                                      8’d21 irq_vusb_lvc_ovp
                                                      8’d22 irq_vusb_otg_ovp
                                                      8’d23 irq_vusb_scauto_ovp
                                                      8’d24 irq_vusb_ovp_alm
                                                      8’d25 irq_vusb_uv
                                                      8’d26 irq_bat_ov_clamp
                                                      8’d27 irq_vout_ov_clamp
                                                      8’d28 irq_ibus_clamp
                                                      8’d29 irq_ibat_clamp
                                                      8’d30 irq_vbus_sc_ov
                                                      8’d31 irq_vbus_lvc_ov
                                                      8’d32 irq_vbus_sc_uv
                                                      8’d33 irq_vbus_lvc_uv
                                                      8’d34 irq_ibus_dc_ocp
                                                      8’d35 irq_ibus_dc_ucp
                                                      8’d36 irq_ibus_dc_rcp
                                                      8’d37 irq_vin2vout_sc
                                                      8’d38 irq_vdrop_lvc_ov
                                                      8’d39 irq_vbat_dc_ovp
                                                      8’d40 irq_vbat_sc_uvp
                                                      8’d41 irq_ibat_dc_ocp
                                                      8’d42 irq_ibat_dcucp_alm
                                                      8’d43 irq_ilim_sc_ocp
                                                      8’d44 irq_ilim_bus_sc_ocp
                                                      8’d45 irq_tdie_ot_alm
                                                      8’d46 irq_vbat_lv
                                                      8’d47 irq_cc_ov
                                                      8’d48 irq_cc_ocp
                                                      8’d49 irq_cc_uv
                                                      8’d50 irq_fcp_set
                                                      8’d51 soh_ovh
                                                      8’d52 soh_ovl
                                                      8’d53 acr_flag
                                                      8’d54 irq_wl_otg_usbok
                                                      8’d55 irq_acr_scp
                                                      8'd56~8'd255：输出伪中断为0 */
    } reg;
} SOC_SCHARGER_IRQ_FAKE_UNION;
#endif
#define SOC_SCHARGER_IRQ_FAKE_sc_fake_irq_START  (0)
#define SOC_SCHARGER_IRQ_FAKE_sc_fake_irq_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_IRQ_FLAG_UNION
 结构说明  : IRQ_FLAG 寄存器结构定义。地址偏移量:0x62，初值:0x00，宽度:8
 寄存器说明:
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  sc_irq_flag : 8;  /* bit[0-7]: 中断标记寄存器，写1清零
                                                      1：中断发生
                                                      0：中断不发生
                                                      bit[7:5]：reserved
                                                      bit[4]：BUCK总中断
                                                      bit[3]：LVC_SC总中断
                                                      bit[2]：PD总中断
                                                      bit[1]：Others总中断
                                                      bit[0]：FCP总中断 */
    } reg;
} SOC_SCHARGER_IRQ_FLAG_UNION;
#endif
#define SOC_SCHARGER_IRQ_FLAG_sc_irq_flag_START  (0)
#define SOC_SCHARGER_IRQ_FLAG_sc_irq_flag_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_IRQ_FLAG_0_UNION
 结构说明  : IRQ_FLAG_0 寄存器结构定义。地址偏移量:0x63，初值:0x00，宽度:8
 寄存器说明:
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  sc_irq_flag_0 : 8;  /* bit[0-7]: 中断标记寄存器，写1清零
                                                        1：中断发生
                                                        0：中断不发生
                                                        bit[7]:irq_chg_chgstate
                                                        bit[6]:irq_vbus_ovp
                                                        bit[5]:irq_vbus_uvp
                                                        bit[4]:irq_vbat_ovp
                                                        bit[3]:irq_otg_ovp
                                                        bit[2]:irq_otg_uvp
                                                        bit[1]:irq_buck_ocp
                                                        bit[0]:irq_otg_scp */
    } reg;
} SOC_SCHARGER_IRQ_FLAG_0_UNION;
#endif
#define SOC_SCHARGER_IRQ_FLAG_0_sc_irq_flag_0_START  (0)
#define SOC_SCHARGER_IRQ_FLAG_0_sc_irq_flag_0_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_IRQ_FLAG_1_UNION
 结构说明  : IRQ_FLAG_1 寄存器结构定义。地址偏移量:0x64，初值:0x00，宽度:8
 寄存器说明:
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  sc_irq_flag_1 : 8;  /* bit[0-7]: 中断标记寄存器，写1清零
                                                        1：中断发生
                                                        0：中断不发生
                                                        bit[7]:irq_otg_ocp
                                                        bit[6]:irq_buck_scp
                                                        bit[5]:irq_trickle_chg_fault
                                                        bit[4]:irq_pre_chg_fault
                                                        bit[3]:irq_fast_chg_fault
                                                        bit[2]:irq_otmp_jreg
                                                        bit[1]:irq_chg_rechg_state
                                                        bit[0]:irq_sleep_mod */
    } reg;
} SOC_SCHARGER_IRQ_FLAG_1_UNION;
#endif
#define SOC_SCHARGER_IRQ_FLAG_1_sc_irq_flag_1_START  (0)
#define SOC_SCHARGER_IRQ_FLAG_1_sc_irq_flag_1_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_IRQ_FLAG_2_UNION
 结构说明  : IRQ_FLAG_2 寄存器结构定义。地址偏移量:0x65，初值:0x00，宽度:8
 寄存器说明:
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  sc_irq_flag_2 : 8;  /* bit[0-7]: 中断标记寄存器，写1清零
                                                        1：中断发生
                                                        0：中断不发生
                                                        bit[7]:irq_reversbst
                                                        bit[6]:irq_vusb_buck_ovp
                                                        bit[5]:irq_vusb_otg_ovp

                                                        bit[4]:irq_vusb_lvc_ovp
                                                        bit[3]:irq_vbus_lvc_ov
                                                        bit[2]:irq_vbus_lvc_uv
                                                        bit[1]:irq_ibus_dc_ocp
                                                        bit[0]:irq_ibus_dc_ucp */
    } reg;
} SOC_SCHARGER_IRQ_FLAG_2_UNION;
#endif
#define SOC_SCHARGER_IRQ_FLAG_2_sc_irq_flag_2_START  (0)
#define SOC_SCHARGER_IRQ_FLAG_2_sc_irq_flag_2_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_IRQ_FLAG_3_UNION
 结构说明  : IRQ_FLAG_3 寄存器结构定义。地址偏移量:0x66，初值:0x00，宽度:8
 寄存器说明:
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  sc_irq_flag_3 : 8;  /* bit[0-7]: 中断标记寄存器，写1清零
                                                        1：中断发生
                                                        0：中断不发生
                                                        bit[7]:irq_ibus_dc_rcp
                                                        bit[6]:irq_vdrop_lvc_ov
                                                        bit[5]:irq_vbat_dc_ovp
                                                        bit[4]:irq_vusb_scauto_ovp
                                                        bit[3]:irq_bat_ov_clamp
                                                        bit[2]:irq_vout_ov_clamp
                                                        bit[1]:irq_ibus_clamp
                                                        bit[0]:irq_ibat_clamp */
    } reg;
} SOC_SCHARGER_IRQ_FLAG_3_UNION;
#endif
#define SOC_SCHARGER_IRQ_FLAG_3_sc_irq_flag_3_START  (0)
#define SOC_SCHARGER_IRQ_FLAG_3_sc_irq_flag_3_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_IRQ_FLAG_4_UNION
 结构说明  : IRQ_FLAG_4 寄存器结构定义。地址偏移量:0x67，初值:0x00，宽度:8
 寄存器说明:
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  sc_irq_flag_4 : 8;  /* bit[0-7]: 中断标记寄存器，写1清零
                                                        1：中断发生
                                                        0：中断不发生
                                                        bit[7]:irq_vbus_sc_ov
                                                        bit[6]:irq_vbus_sc_uv
                                                        bit[5]:irq_vin2vout_sc
                                                        bit[4]:irq_vbat_sc_uvp
                                                        bit[3]:irq_ibat_dc_ocp
                                                        bit[2]:irq_ibat_dcucp_alm
                                                        bit[1]:irq_ilim_sc_ocp
                                                        bit[0]:irq_ilim_bus_sc_ocp */
    } reg;
} SOC_SCHARGER_IRQ_FLAG_4_UNION;
#endif
#define SOC_SCHARGER_IRQ_FLAG_4_sc_irq_flag_4_START  (0)
#define SOC_SCHARGER_IRQ_FLAG_4_sc_irq_flag_4_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_IRQ_FLAG_5_UNION
 结构说明  : IRQ_FLAG_5 寄存器结构定义。地址偏移量:0x68，初值:0x00，宽度:8
 寄存器说明:
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  sc_irq_flag_5 : 8;  /* bit[0-7]: 中断标记寄存器，写1清零
                                                        1：中断发生
                                                        0：中断不发生
                                                        bit[7]:irq_cc_ov
                                                        bit[6]:irq_cc_ocp
                                                        bit[5]:irq_cc_uv

                                                        bit[4]:irq_acr_scp
                                                        bit[3]:soh_ovh
                                                        bit[2]:soh_ovl
                                                        bit[1]:acr_flag
                                                        bit[0]:irq_wdt */
    } reg;
} SOC_SCHARGER_IRQ_FLAG_5_UNION;
#endif
#define SOC_SCHARGER_IRQ_FLAG_5_sc_irq_flag_5_START  (0)
#define SOC_SCHARGER_IRQ_FLAG_5_sc_irq_flag_5_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_IRQ_FLAG_6_UNION
 结构说明  : IRQ_FLAG_6 寄存器结构定义。地址偏移量:0x69，初值:0x00，宽度:8
 寄存器说明:
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  sc_irq_flag_6 : 8;  /* bit[0-7]: 中断标记寄存器，写1清零
                                                        1：中断发生
                                                        0：中断不发生
                                                        bit[7]:irq_regn_ocp
                                                        bit[6]:irq_chg_batfet_ocp
                                                        bit[5]:irq_otmp_140
                                                        bit[4]:irq_wl_otg_usbok
                                                        bit[3]:irq_vusb_ovp_alm
                                                        bit[2]:irq_vusb_uv
                                                        bit[1]:irq_tdie_ot_alm
                                                        bit[0]:irq_vbat_lv */
    } reg;
} SOC_SCHARGER_IRQ_FLAG_6_UNION;
#endif
#define SOC_SCHARGER_IRQ_FLAG_6_sc_irq_flag_6_START  (0)
#define SOC_SCHARGER_IRQ_FLAG_6_sc_irq_flag_6_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_IRQ_FLAG_7_UNION
 结构说明  : IRQ_FLAG_7 寄存器结构定义。地址偏移量:0x6A，初值:0x00，宽度:8
 寄存器说明:
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  sc_irq_flag_7 : 8;  /* bit[0-7]: 中断标记寄存器，写1清零
                                                        1：中断发生
                                                        0：中断不发生
                                                        bit[7]:irq_reserved[7]
                                                        bit[6]:irq_reserved[6]
                                                        bit[5]:irq_reserved[5]
                                                        bit[4]:irq_reserved[4]
                                                        bit[3]:irq_reserved[3]，irq_vout_dc_ovp vout过压保护，dc模式下使能
                                                        bit[2]:irq_reserved[2]，预留中断寄存器
                                                        bit[1]:irq_reserved[1]，irq_vdrop_usb_ovp ovp管 drop保护中断；dc模式才使能；
                                                        bit[0]:irq_reserved[0]，irq_cfly_scp,fly电容短路中断；dc模式才使能 */
    } reg;
} SOC_SCHARGER_IRQ_FLAG_7_UNION;
#endif
#define SOC_SCHARGER_IRQ_FLAG_7_sc_irq_flag_7_START  (0)
#define SOC_SCHARGER_IRQ_FLAG_7_sc_irq_flag_7_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_WDT_SOFT_RST_UNION
 结构说明  : WDT_SOFT_RST 寄存器结构定义。地址偏移量:0x6B，初值:0x00，宽度:8
 寄存器说明: 看门狗软复位控制寄存器。
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  wd_rst_n : 1;  /* bit[0-0]: 写1该寄存器，系统watchdog timer计数重新计时(写1后，自动清零)
                                                   如果SoC在设置时间内不对该寄存器写‘1’操作，则复位chg_en和watchdog_timer寄存器。 */
        unsigned char  reserved : 7;  /* bit[1-7]: 保留。 */
    } reg;
} SOC_SCHARGER_WDT_SOFT_RST_UNION;
#endif
#define SOC_SCHARGER_WDT_SOFT_RST_wd_rst_n_START  (0)
#define SOC_SCHARGER_WDT_SOFT_RST_wd_rst_n_END    (0)


/*****************************************************************************
 结构名    : SOC_SCHARGER_WDT_CTRL_UNION
 结构说明  : WDT_CTRL 寄存器结构定义。地址偏移量:0x6C，初值:0x00，宽度:8
 寄存器说明: 喂狗时间控制寄存器。
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  sc_watchdog_timer : 3;  /* bit[0-2]: SOC每隔一定时间对wd_rst_n进行一次寄存器写操作；如果没有写操作事件发生，则进入Default模式，复位sc_chg_en/sc_sc_en/sc_lvc_en/fcp_det_en/fcp_cmp_en和sc_watchdog_timer寄存器；否则是正常的Host模式。
                                                            系统watchdog_timer时间设置：
                                                            3'b000：系统watchdog_timer功能屏蔽；
                                                            3'b001：0.5s；
                                                            3'b010：1s；
                                                            3'b011：2s；
                                                            3'b100：10s；
                                                            3'b101：20s；
                                                            3'b110：40s；
                                                            3'b111：80s。 */
        unsigned char  reserved_0        : 1;  /* bit[3-3]: 保留 */
        unsigned char  sc_wdt_test_sel   : 1;  /* bit[4-4]: WDT时间档位可根据测试模式和正常模式选择信号：
                                                            1’b0：看门狗时间档位为正常档位；
                                                            1’b1：看门狗时间档位为测试档位（sc_watchdog_timer=3'b000：系统watchdog_timer功能屏蔽；
                                                            sc_watchdog_timer=3'b001~3'b111：1ms） */
        unsigned char  reserved_1        : 3;  /* bit[5-7]: 保留 */
    } reg;
} SOC_SCHARGER_WDT_CTRL_UNION;
#endif
#define SOC_SCHARGER_WDT_CTRL_sc_watchdog_timer_START  (0)
#define SOC_SCHARGER_WDT_CTRL_sc_watchdog_timer_END    (2)
#define SOC_SCHARGER_WDT_CTRL_sc_wdt_test_sel_START    (4)
#define SOC_SCHARGER_WDT_CTRL_sc_wdt_test_sel_END      (4)


/*****************************************************************************
 结构名    : SOC_SCHARGER_DC_IBAT_REGULATOR_UNION
 结构说明  : DC_IBAT_REGULATOR 寄存器结构定义。地址偏移量:0x6D，初值:0x14，宽度:8
 寄存器说明: 直充IBAT校准档位调节寄存器
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  sc_dc_ibat_regulator : 8;  /* bit[0-7]: 直充IBAT 校准档位调节寄存器：
                                                               电流范围3A~13A，step=50mA
                                                               8'h00:3A;
                                                               8'h01:3.05A;
                                                               ...
                                                               8'h14:4A;
                                                               ...
                                                               8'hC7:12.95A;
                                                               8'hC8:13A;
                                                               8'hC9~8'hFF:13A（寄存器超出13A的配置数字不做特殊处理，与EFUSE的值相加之后做饱和处理)

                                                               默认0x14 */
    } reg;
} SOC_SCHARGER_DC_IBAT_REGULATOR_UNION;
#endif
#define SOC_SCHARGER_DC_IBAT_REGULATOR_sc_dc_ibat_regulator_START  (0)
#define SOC_SCHARGER_DC_IBAT_REGULATOR_sc_dc_ibat_regulator_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_DC_VBAT_REGULATOR_UNION
 结构说明  : DC_VBAT_REGULATOR 寄存器结构定义。地址偏移量:0x6E，初值:0x0A，宽度:8
 寄存器说明: 直充VBAT校准档位调节寄存器
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  sc_dc_vbat_regulator : 7;  /* bit[0-6]: 直充VBAT校准档位调节寄存器:
                                                               电压范围4.2V~5V, step=10mV，
                                                               7'h00:4.2V;
                                                               7'h01:4.21V;
                                                               ...
                                                               7'h0A:4.3V;
                                                               ...
                                                               7'h50:5V;
                                                               7'h51~7'h7F:5V，
                                                               （寄存器超出5V的配置数字不做特殊处理，与EFUSE的值相加之后做饱和处理)step=10mV，默认4.3V */
        unsigned char  reserved             : 1;  /* bit[7-7]: reserved */
    } reg;
} SOC_SCHARGER_DC_VBAT_REGULATOR_UNION;
#endif
#define SOC_SCHARGER_DC_VBAT_REGULATOR_sc_dc_vbat_regulator_START  (0)
#define SOC_SCHARGER_DC_VBAT_REGULATOR_sc_dc_vbat_regulator_END    (6)


/*****************************************************************************
 结构名    : SOC_SCHARGER_DC_VOUT_REGULATOR_UNION
 结构说明  : DC_VOUT_REGULATOR 寄存器结构定义。地址偏移量:0x6F，初值:0x0A，宽度:8
 寄存器说明: 直充VOUT校准档位调节寄存器
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  sc_dc_vout_regulator : 7;  /* bit[0-6]: 直充VOUT校准档位调节寄存器:
                                                               电压范围4.2V~5V, step=10mV，
                                                               7'h00:4.2V;
                                                               7'h01:4.21V;
                                                               ...
                                                               7'h0A:4.3V;
                                                               ...
                                                               7'h50:5V;
                                                               7'h51~7'h7F:5V，
                                                               （寄存器超出5V的配置数字不做特殊处理，与EFUSE的值相加之后做饱和处理)step=10mV，默认4.3V */
        unsigned char  reserved             : 1;  /* bit[7-7]: reserved */
    } reg;
} SOC_SCHARGER_DC_VOUT_REGULATOR_UNION;
#endif
#define SOC_SCHARGER_DC_VOUT_REGULATOR_sc_dc_vout_regulator_START  (0)
#define SOC_SCHARGER_DC_VOUT_REGULATOR_sc_dc_vout_regulator_END    (6)


/*****************************************************************************
 结构名    : SOC_SCHARGER_OTG_CFG_UNION
 结构说明  : OTG_CFG 寄存器结构定义。地址偏移量:0x70，初值:0x09，宽度:8
 寄存器说明: OTG模块配置寄存器
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  sc_otg_dmd_ofs  : 4;  /* bit[0-3]: boost DMD失调电压调节
                                                          0000: 20mV (300mA)
                                                          0001: 16mV (240mA)
                                                          0010: 14mV (210mA)
                                                          0011: 12mV (180mA)
                                                          0100: 10mV (150mA)
                                                          0101: 8mV (120mA)
                                                          0110: 6mV (90mA)
                                                          0111: 4mV (60mA)
                                                          1000: 2mV (30mA)
                                                          1001:0
                                                          1010: -16mV (-180mA).
                                                          1011: -32mV (-360mA).
                                                          1100: -48mV (-540mA).
                                                          1101: -64mV (-720mA)..
                                                          1110: -128mV (-900mA).
                                                          1111: -256mV (-1080mA) */
        unsigned char  sc_otg_dmd_ramp : 3;  /* bit[4-6]: 控制数字DMD算法，优化寄存器，待测试后固定（实际会影响DMD档位） */
        unsigned char  sc_otg_en       : 1;  /* bit[7-7]: OTG使能信号：
                                                          1'b0：不使能；
                                                          1'b1：使能 */
    } reg;
} SOC_SCHARGER_OTG_CFG_UNION;
#endif
#define SOC_SCHARGER_OTG_CFG_sc_otg_dmd_ofs_START   (0)
#define SOC_SCHARGER_OTG_CFG_sc_otg_dmd_ofs_END     (3)
#define SOC_SCHARGER_OTG_CFG_sc_otg_dmd_ramp_START  (4)
#define SOC_SCHARGER_OTG_CFG_sc_otg_dmd_ramp_END    (6)
#define SOC_SCHARGER_OTG_CFG_sc_otg_en_START        (7)
#define SOC_SCHARGER_OTG_CFG_sc_otg_en_END          (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_PULSE_CHG_CFG0_UNION
 结构说明  : PULSE_CHG_CFG0 寄存器结构定义。地址偏移量:0x71，初值:0x00，宽度:8
 寄存器说明: 脉冲充电配置寄存器0
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  sc_lvc_en        : 1;  /* bit[0-0]: 低压直充全局充电使能：
                                                           1’b0: 关闭（默认）；
                                                           1’b1: 使能。
                                                           （注：看门狗发生异常后该寄存器会被置为默认值） */
        unsigned char  sc_pulse_mode_en : 1;  /* bit[1-1]: 脉冲充电和直流充电模式选择：
                                                           1’b0：不使能脉冲冲充电模式（默认）；
                                                           1’b1：使能脉冲充电模式充电。 */
        unsigned char  reserved_0       : 2;  /* bit[2-3]: reserved */
        unsigned char  sc_chg_en        : 1;  /* bit[4-4]: 高压快充全局充电使能：
                                                           1'b0：不使能充电（默认）
                                                           1'b1：使能充电
                                                           （注：看门狗发生异常后该寄存器会被置为默认值） */
        unsigned char  reserved_1       : 3;  /* bit[5-7]: reserved */
    } reg;
} SOC_SCHARGER_PULSE_CHG_CFG0_UNION;
#endif
#define SOC_SCHARGER_PULSE_CHG_CFG0_sc_lvc_en_START         (0)
#define SOC_SCHARGER_PULSE_CHG_CFG0_sc_lvc_en_END           (0)
#define SOC_SCHARGER_PULSE_CHG_CFG0_sc_pulse_mode_en_START  (1)
#define SOC_SCHARGER_PULSE_CHG_CFG0_sc_pulse_mode_en_END    (1)
#define SOC_SCHARGER_PULSE_CHG_CFG0_sc_chg_en_START         (4)
#define SOC_SCHARGER_PULSE_CHG_CFG0_sc_chg_en_END           (4)


/*****************************************************************************
 结构名    : SOC_SCHARGER_PULSE_CHG_CFG1_UNION
 结构说明  : PULSE_CHG_CFG1 寄存器结构定义。地址偏移量:0x72，初值:0x13，宽度:8
 寄存器说明: 脉冲充电配置寄存器1
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  sc_chg_time_set : 5;  /* bit[0-4]: 脉冲模式充电时间设置：
                                                          5’b00000:100ms
                                                          5’b 00001:200ms
                                                          5’b 00010:300ms
                                                          5’b 11110:3100ms
                                                          5’b 11111:3200ms
                                                          默认2000ms */
        unsigned char  sc_sc_en        : 1;  /* bit[5-5]: 高压直充使能信号：
                                                          1’b0: 关闭（默认）；
                                                          1’b1: 使能。
                                                          （注：看门狗发生异常后该寄存器会被置为默认值） */
        unsigned char  reserved        : 2;  /* bit[6-7]: reserved */
    } reg;
} SOC_SCHARGER_PULSE_CHG_CFG1_UNION;
#endif
#define SOC_SCHARGER_PULSE_CHG_CFG1_sc_chg_time_set_START  (0)
#define SOC_SCHARGER_PULSE_CHG_CFG1_sc_chg_time_set_END    (4)
#define SOC_SCHARGER_PULSE_CHG_CFG1_sc_sc_en_START         (5)
#define SOC_SCHARGER_PULSE_CHG_CFG1_sc_sc_en_END           (5)


/*****************************************************************************
 结构名    : SOC_SCHARGER_DISCHG_TIME_UNION
 结构说明  : DISCHG_TIME 寄存器结构定义。地址偏移量:0x73，初值:0x63，宽度:8
 寄存器说明: 脉冲模式放电时间配置寄存器
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  sc_dischg_time_set : 8;  
    } reg;
} SOC_SCHARGER_DISCHG_TIME_UNION;
#endif
#define SOC_SCHARGER_DISCHG_TIME_sc_dischg_time_set_START  (0)
#define SOC_SCHARGER_DISCHG_TIME_sc_dischg_time_set_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_DIG_STATUS0_UNION
 结构说明  : DIG_STATUS0 寄存器结构定义。地址偏移量:0x74，初值:0x01，宽度:8
 寄存器说明: 数字内部信号状态寄存器
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  wdt_time_out_n : 1;  /* bit[0-0]: 看门狗状态信号：
                                                         0: watchdog timer 溢出，SOC在规定时间内没有清watchdog timer寄存器，即在规定时间内没有对wd_rst_n（0xE8）寄存器写1；
                                                         1: watchdog timer 功能没有触发 或watchdog timer正常；
                                                         说明：由于看门狗发生timeout之后会将看门狗时间档位sc_watchdog_timer寄存器复位为0，即disable看门狗功能，导致wdt_time_out_n为0的时间约为24us，导致该寄存器回读到状态的0的概率非常小，查询看门狗溢出请使用看门狗timeout中断寄存器sc_irq_flag_5[0] */
        unsigned char  reserved       : 7;  /* bit[1-7]: reserved */
    } reg;
} SOC_SCHARGER_DIG_STATUS0_UNION;
#endif
#define SOC_SCHARGER_DIG_STATUS0_wdt_time_out_n_START  (0)
#define SOC_SCHARGER_DIG_STATUS0_wdt_time_out_n_END    (0)


/*****************************************************************************
 结构名    : SOC_SCHARGER_SC_TESTBUS_CFG_UNION
 结构说明  : SC_TESTBUS_CFG 寄存器结构定义。地址偏移量:0x76，初值:0x00，宽度:8
 寄存器说明: 测试总线配置寄存器
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  sc_cg_testbus_sel : 4;  /* bit[0-3]: 配置数字CRG内部观察信号的选择比特
                                                            见Programming Guide
                                                            默认4'b0000 */
        unsigned char  sc_cg_testsig_sel : 3;  /* bit[4-6]: 数字送给模拟单比特信号选择
                                                            000:选择clk_fcp时钟输出；(125KHz)
                                                            001:选择clk_2m时钟输出；(2MHz)
                                                            010：选择clk_pd时钟输出（6MHz）
                                                            Others: reserved
                                                            默认：3'b000 */
        unsigned char  sc_cg_testbus_en  : 1;  /* bit[7-7]: 数字送给模拟测试信号使能
                                                            0:不使能测试信号
                                                            1：使能测试信号输出
                                                            默认0 */
    } reg;
} SOC_SCHARGER_SC_TESTBUS_CFG_UNION;
#endif
#define SOC_SCHARGER_SC_TESTBUS_CFG_sc_cg_testbus_sel_START  (0)
#define SOC_SCHARGER_SC_TESTBUS_CFG_sc_cg_testbus_sel_END    (3)
#define SOC_SCHARGER_SC_TESTBUS_CFG_sc_cg_testsig_sel_START  (4)
#define SOC_SCHARGER_SC_TESTBUS_CFG_sc_cg_testsig_sel_END    (6)
#define SOC_SCHARGER_SC_TESTBUS_CFG_sc_cg_testbus_en_START   (7)
#define SOC_SCHARGER_SC_TESTBUS_CFG_sc_cg_testbus_en_END     (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_SC_TESTBUS_RDATA_UNION
 结构说明  : SC_TESTBUS_RDATA 寄存器结构定义。地址偏移量:0x77，初值:0x00，宽度:8
 寄存器说明: 测试总线回读寄存器
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  cg_testbus_rdata : 8;  /* bit[0-7]: CRG内部观察信号的回读数据 */
    } reg;
} SOC_SCHARGER_SC_TESTBUS_RDATA_UNION;
#endif
#define SOC_SCHARGER_SC_TESTBUS_RDATA_cg_testbus_rdata_START  (0)
#define SOC_SCHARGER_SC_TESTBUS_RDATA_cg_testbus_rdata_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_GLB_SOFT_RST_CTRL_UNION
 结构说明  : GLB_SOFT_RST_CTRL 寄存器结构定义。地址偏移量:0x78，初值:0x5A，宽度:8
 寄存器说明: 全局软复位控制寄存器。
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  sc_cg_glb_soft_rst_n : 8;  /* bit[0-7]: 芯片全局软复位配置寄存器，写保护。
                                                               当写入0x5A时候，软复位为高电平，不复位；
                                                               当写入0xAC时候，软复位为低电平，复位除batfet_ctrl，vbat_lv以及EFUSE外的所有数字逻辑；
                                                               当写入其他值，软复位不跳变，保持； */
    } reg;
} SOC_SCHARGER_GLB_SOFT_RST_CTRL_UNION;
#endif
#define SOC_SCHARGER_GLB_SOFT_RST_CTRL_sc_cg_glb_soft_rst_n_START  (0)
#define SOC_SCHARGER_GLB_SOFT_RST_CTRL_sc_cg_glb_soft_rst_n_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_EFUSE_SOFT_RST_CTRL_UNION
 结构说明  : EFUSE_SOFT_RST_CTRL 寄存器结构定义。地址偏移量:0x79，初值:0x5A，宽度:8
 寄存器说明: EFUSE软复位控制寄存器。
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  sc_cg_efuse_soft_rst_n : 8;  /* bit[0-7]: EFUSE软复位配置寄存器，写保护
                                                                 当写入0x5A时候，软复位为高电平，不复位EFUSE数字逻辑；
                                                                 当写入0xAC时候，软复位为低电平，复位EFUSE数字逻辑，EFUSE控制逻辑会从EFUSE重新回读一次数据。
                                                                 当写入其他值，软复位不跳变，保持； */
    } reg;
} SOC_SCHARGER_EFUSE_SOFT_RST_CTRL_UNION;
#endif
#define SOC_SCHARGER_EFUSE_SOFT_RST_CTRL_sc_cg_efuse_soft_rst_n_START  (0)
#define SOC_SCHARGER_EFUSE_SOFT_RST_CTRL_sc_cg_efuse_soft_rst_n_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_SC_CRG_CLK_EN_CTRL_UNION
 结构说明  : SC_CRG_CLK_EN_CTRL 寄存器结构定义。地址偏移量:0x7A，初值:0x0F，宽度:8
 寄存器说明: CORE CRG时钟使能配置寄存器
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  sc_cg_com_clk_en    : 1;  /* bit[0-0]: 1MHz时钟域时钟使能配置寄存器。
                                                              0: 不使能1MHz时钟域；
                                                              1：使能1MHz时钟域；
                                                              默认1 */
        unsigned char  sc_cg_red_clk_en    : 1;  /* bit[1-1]: 数字测试用。Redundancy时钟域时钟使能配置寄存器。
                                                              0: 不使能
                                                              1：使能
                                                              默认1 */
        unsigned char  sc_cg_pd_clk_en     : 1;  /* bit[2-2]: 数字测试用。clk_pd时钟域时钟使能配置寄存器。
                                                              0: 不使能
                                                              1：使能
                                                              默认1 */
        unsigned char  sc_cg_2m_clk_en     : 1;  /* bit[3-3]: 数字测试用。clk_2m时钟域时钟使能配置寄存器。
                                                              0: 不使能
                                                              1：使能
                                                              默认1 */
        unsigned char  sc_cg_pd_clk_en_sel : 1;  /* bit[4-4]: 数字测试用。clk_pd时钟域时钟使能源头选择寄存器。
                                                              0: 使用模拟给的ad_clk2m_en作为门控信号
                                                              1：使用软件配置sc_cg_pd_clk_en作为门控信号
                                                              默认0 */
        unsigned char  sc_cg_2m_clk_en_sel : 1;  /* bit[5-5]: 数字测试用。clk_2m时钟域时钟使能源头选择寄存器。
                                                              0: 使用模拟给的ad_clk2m_en作为门控信号
                                                              1：使用软件配置sc_cg_2m_clk_en作为门控信号
                                                              默认0 */
        unsigned char  reserved            : 2;  /* bit[6-7]: reserved */
    } reg;
} SOC_SCHARGER_SC_CRG_CLK_EN_CTRL_UNION;
#endif
#define SOC_SCHARGER_SC_CRG_CLK_EN_CTRL_sc_cg_com_clk_en_START     (0)
#define SOC_SCHARGER_SC_CRG_CLK_EN_CTRL_sc_cg_com_clk_en_END       (0)
#define SOC_SCHARGER_SC_CRG_CLK_EN_CTRL_sc_cg_red_clk_en_START     (1)
#define SOC_SCHARGER_SC_CRG_CLK_EN_CTRL_sc_cg_red_clk_en_END       (1)
#define SOC_SCHARGER_SC_CRG_CLK_EN_CTRL_sc_cg_pd_clk_en_START      (2)
#define SOC_SCHARGER_SC_CRG_CLK_EN_CTRL_sc_cg_pd_clk_en_END        (2)
#define SOC_SCHARGER_SC_CRG_CLK_EN_CTRL_sc_cg_2m_clk_en_START      (3)
#define SOC_SCHARGER_SC_CRG_CLK_EN_CTRL_sc_cg_2m_clk_en_END        (3)
#define SOC_SCHARGER_SC_CRG_CLK_EN_CTRL_sc_cg_pd_clk_en_sel_START  (4)
#define SOC_SCHARGER_SC_CRG_CLK_EN_CTRL_sc_cg_pd_clk_en_sel_END    (4)
#define SOC_SCHARGER_SC_CRG_CLK_EN_CTRL_sc_cg_2m_clk_en_sel_START  (5)
#define SOC_SCHARGER_SC_CRG_CLK_EN_CTRL_sc_cg_2m_clk_en_sel_END    (5)


/*****************************************************************************
 结构名    : SOC_SCHARGER_BUCK_MODE_CFG_UNION
 结构说明  : BUCK_MODE_CFG 寄存器结构定义。地址偏移量:0x7B，初值:0x01，宽度:8
 寄存器说明: BUCK模式配置寄存器
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  buck_mode_cfg : 1;  /* bit[0-0]: buck模式指示
                                                        0:不是buck模式；
                                                        1：buck模式；
                                                        （在irq_vusb_uv为高的时候会复位为初始值）
                                                        默认1 */
        unsigned char  reserved      : 7;  /* bit[1-7]: reserved */
    } reg;
} SOC_SCHARGER_BUCK_MODE_CFG_UNION;
#endif
#define SOC_SCHARGER_BUCK_MODE_CFG_buck_mode_cfg_START  (0)
#define SOC_SCHARGER_BUCK_MODE_CFG_buck_mode_cfg_END    (0)


/*****************************************************************************
 结构名    : SOC_SCHARGER_SC_MODE_CFG_UNION
 结构说明  : SC_MODE_CFG 寄存器结构定义。地址偏移量:0x7C，初值:0x00，宽度:8
 寄存器说明: SCC充电模式配置寄存器
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  sc_mode_cfg : 1;  /* bit[0-0]: 充电模式选择：
                                                      0：非SCC模式；
                                                      1：SCC模式；
                                                      （在irq_vusb_uv为高的时候会复位为初始值）
                                                      默认0 */
        unsigned char  reserved    : 7;  /* bit[1-7]: reserved */
    } reg;
} SOC_SCHARGER_SC_MODE_CFG_UNION;
#endif
#define SOC_SCHARGER_SC_MODE_CFG_sc_mode_cfg_START  (0)
#define SOC_SCHARGER_SC_MODE_CFG_sc_mode_cfg_END    (0)


/*****************************************************************************
 结构名    : SOC_SCHARGER_LVC_MODE_CFG_UNION
 结构说明  : LVC_MODE_CFG 寄存器结构定义。地址偏移量:0x7D，初值:0x00，宽度:8
 寄存器说明: LVC充电模式配置寄存器
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  lvc_mode_cfg : 1;  /* bit[0-0]: 充电模式选择：
                                                       0：非LVC模式；
                                                       1：LVC模式；
                                                       （在irq_vusb_uv为高的时候会复位为初始值）
                                                       默认0 */
        unsigned char  reserved     : 7;  /* bit[1-7]: reserved */
    } reg;
} SOC_SCHARGER_LVC_MODE_CFG_UNION;
#endif
#define SOC_SCHARGER_LVC_MODE_CFG_lvc_mode_cfg_START  (0)
#define SOC_SCHARGER_LVC_MODE_CFG_lvc_mode_cfg_END    (0)


/*****************************************************************************
 结构名    : SOC_SCHARGER_SC_BUCK_ENB_UNION
 结构说明  : SC_BUCK_ENB 寄存器结构定义。地址偏移量:0x7E，初值:0x00，宽度:8
 寄存器说明: BUCK使能配置信号
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  sc_buck_enb : 1;  /* bit[0-0]: buck强制关闭控制寄存器
                                                      0b：da_buck_enb信号为0，buck使能由模拟自动控制；
                                                      1b：da_buck_enb信号为1，buck强制关闭；
                                                      默认0 */
        unsigned char  reserved    : 7;  /* bit[1-7]: reserved */
    } reg;
} SOC_SCHARGER_SC_BUCK_ENB_UNION;
#endif
#define SOC_SCHARGER_SC_BUCK_ENB_sc_buck_enb_START  (0)
#define SOC_SCHARGER_SC_BUCK_ENB_sc_buck_enb_END    (0)


/*****************************************************************************
 结构名    : SOC_SCHARGER_SC_OVP_MOS_EN_UNION
 结构说明  : SC_OVP_MOS_EN 寄存器结构定义。地址偏移量:0x7F，初值:0x01，宽度:8
 寄存器说明: OVP模块使能控制
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  sc_ovp_mos_en : 1;  /* bit[0-0]: OVP模块使能控制寄存器
                                                        0b：da_ovp_mos_en信号为0，OVP模块不使能；
                                                        1b：da_ovp_mos_en信号为1，OVP模块使能
                                                        默认1 */
        unsigned char  reserved      : 7;  /* bit[1-7]: reserved */
    } reg;
} SOC_SCHARGER_SC_OVP_MOS_EN_UNION;
#endif
#define SOC_SCHARGER_SC_OVP_MOS_EN_sc_ovp_mos_en_START  (0)
#define SOC_SCHARGER_SC_OVP_MOS_EN_sc_ovp_mos_en_END    (0)




/****************************************************************************
                     (3/4) REG_PAGE1
 ****************************************************************************/
/*****************************************************************************
 结构名    : SOC_SCHARGER_FCP_IRQ3_UNION
 结构说明  : FCP_IRQ3 寄存器结构定义。地址偏移量:0x00，初值:0x00，宽度:8
 寄存器说明: FCP中断3寄存器。
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  last_hand_fail_irq  : 1;  /* bit[0]: 数据包发送完后握手失败中断：
                                                            0：无此中断；
                                                            1：有中断。 */
        unsigned char  tail_hand_fail_irq  : 1;  /* bit[1]: Slaver 返回数据后第一次返回ping握手失败中断：
                                                            0：无此中断；
                                                            1：有中断。 */
        unsigned char  trans_hand_fail_irq : 1;  /* bit[2]: Master发送数据后握手失败中断：
                                                            0：无此中断；
                                                            1：有中断。 */
        unsigned char  init_hand_fail_irq  : 1;  /* bit[3]: 初始化握手失败中断：
                                                            0：无此中断；
                                                            1：有中断。 */
        unsigned char  rx_data_det_irq     : 1;  /* bit[4]: 等不到slaver返回的数据中断：
                                                            0：无此中断；
                                                            1：有中断。 */
        unsigned char  rx_head_det_irq     : 1;  /* bit[5]: 等不到slaver返回的数据包头中断：
                                                            0：无此中断；
                                                            1：有中断。 */
        unsigned char  cmd_err_irq         : 1;  /* bit[6]: SNDCMD不为读或者写中断：
                                                            0：无此中断；
                                                            1：有中断。 */
        unsigned char  length_err_irq      : 1;  /* bit[7]: LENGTH出错
                                                            0：无此中断；
                                                            1：有中断。 */
    } reg;
} SOC_SCHARGER_FCP_IRQ3_UNION;
#endif
#define SOC_SCHARGER_FCP_IRQ3_last_hand_fail_irq_START   (0)
#define SOC_SCHARGER_FCP_IRQ3_last_hand_fail_irq_END     (0)
#define SOC_SCHARGER_FCP_IRQ3_tail_hand_fail_irq_START   (1)
#define SOC_SCHARGER_FCP_IRQ3_tail_hand_fail_irq_END     (1)
#define SOC_SCHARGER_FCP_IRQ3_trans_hand_fail_irq_START  (2)
#define SOC_SCHARGER_FCP_IRQ3_trans_hand_fail_irq_END    (2)
#define SOC_SCHARGER_FCP_IRQ3_init_hand_fail_irq_START   (3)
#define SOC_SCHARGER_FCP_IRQ3_init_hand_fail_irq_END     (3)
#define SOC_SCHARGER_FCP_IRQ3_rx_data_det_irq_START      (4)
#define SOC_SCHARGER_FCP_IRQ3_rx_data_det_irq_END        (4)
#define SOC_SCHARGER_FCP_IRQ3_rx_head_det_irq_START      (5)
#define SOC_SCHARGER_FCP_IRQ3_rx_head_det_irq_END        (5)
#define SOC_SCHARGER_FCP_IRQ3_cmd_err_irq_START          (6)
#define SOC_SCHARGER_FCP_IRQ3_cmd_err_irq_END            (6)
#define SOC_SCHARGER_FCP_IRQ3_length_err_irq_START       (7)
#define SOC_SCHARGER_FCP_IRQ3_length_err_irq_END         (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_FCP_IRQ4_UNION
 结构说明  : FCP_IRQ4 寄存器结构定义。地址偏移量:0x01，初值:0x00，宽度:8
 寄存器说明: FCP中断4寄存器。
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  last_hand_no_respond_irq   : 1;  /* bit[0]  : 数据包发送完后等不到slaver ping中断：
                                                                     0：无此中断；
                                                                     1：有中断。 */
        unsigned char  tail_hand_no_respond_irq   : 1;  /* bit[1]  : Slaver 返回数据后第一次返回ping握手等不到slaver ping中断：
                                                                     0：无此中断；
                                                                     1：有中断。 */
        unsigned char  trans_hand_no_respond_irq  : 1;  /* bit[2]  : Master发送数据后握手等不到slaver ping中断：
                                                                     0：无此中断；
                                                                     1：有中断。 */
        unsigned char  init_hand_no_respond_irq   : 1;  /* bit[3]  : 初始化slaver没有ping返回中断：
                                                                     0：无此中断；
                                                                     1：有中断。 */
        unsigned char  enable_hand_fail_irq       : 1;  /* bit[4]  : 通过enable发送master ping后，握手失败（握手失败会自动重复握手15次）：
                                                                     0：无此中断；
                                                                     1：有中断。 */
        unsigned char  enable_hand_no_respond_irq : 1;  /* bit[5]  : 通过enable发送master ping后，slaver没有响应中断（握手失败会自动重复握手15次）：
                                                                     0：无此中断；
                                                                     1：有中断。 */
        unsigned char  reserved                   : 2;  /* bit[6-7]: 保留。 */
    } reg;
} SOC_SCHARGER_FCP_IRQ4_UNION;
#endif
#define SOC_SCHARGER_FCP_IRQ4_last_hand_no_respond_irq_START    (0)
#define SOC_SCHARGER_FCP_IRQ4_last_hand_no_respond_irq_END      (0)
#define SOC_SCHARGER_FCP_IRQ4_tail_hand_no_respond_irq_START    (1)
#define SOC_SCHARGER_FCP_IRQ4_tail_hand_no_respond_irq_END      (1)
#define SOC_SCHARGER_FCP_IRQ4_trans_hand_no_respond_irq_START   (2)
#define SOC_SCHARGER_FCP_IRQ4_trans_hand_no_respond_irq_END     (2)
#define SOC_SCHARGER_FCP_IRQ4_init_hand_no_respond_irq_START    (3)
#define SOC_SCHARGER_FCP_IRQ4_init_hand_no_respond_irq_END      (3)
#define SOC_SCHARGER_FCP_IRQ4_enable_hand_fail_irq_START        (4)
#define SOC_SCHARGER_FCP_IRQ4_enable_hand_fail_irq_END          (4)
#define SOC_SCHARGER_FCP_IRQ4_enable_hand_no_respond_irq_START  (5)
#define SOC_SCHARGER_FCP_IRQ4_enable_hand_no_respond_irq_END    (5)


/*****************************************************************************
 结构名    : SOC_SCHARGER_FCP_IRQ3_MASK_UNION
 结构说明  : FCP_IRQ3_MASK 寄存器结构定义。地址偏移量:0x02，初值:0xFF，宽度:8
 寄存器说明: FCP中断屏蔽3寄存器。
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  last_hand_fail_irq_mk  : 1;  /* bit[0]: 0：使能该中断；
                                                               1：屏蔽该中断。 */
        unsigned char  tail_hand_fail_irq_mk  : 1;  /* bit[1]: 0：使能该中断；
                                                               1：屏蔽该中断。 */
        unsigned char  trans_hand_fail_irq_mk : 1;  /* bit[2]: 0：使能该中断；
                                                               1：屏蔽该中断。 */
        unsigned char  init_hand_fail_irq_mk  : 1;  /* bit[3]: 0：使能该中断；
                                                               1：屏蔽该中断。 */
        unsigned char  rx_data_det_irq_mk     : 1;  /* bit[4]: 0：使能该中断；
                                                               1：屏蔽该中断。 */
        unsigned char  rx_head_det_irq_mk     : 1;  /* bit[5]: 0：使能该中断；
                                                               1：屏蔽该中断。 */
        unsigned char  cmd_err_irq_mk         : 1;  /* bit[6]: 0：使能该中断；
                                                               1：屏蔽该中断。 */
        unsigned char  length_err_irq_mk      : 1;  /* bit[7]: 0：使能该中断；
                                                               1：屏蔽该中断。 */
    } reg;
} SOC_SCHARGER_FCP_IRQ3_MASK_UNION;
#endif
#define SOC_SCHARGER_FCP_IRQ3_MASK_last_hand_fail_irq_mk_START   (0)
#define SOC_SCHARGER_FCP_IRQ3_MASK_last_hand_fail_irq_mk_END     (0)
#define SOC_SCHARGER_FCP_IRQ3_MASK_tail_hand_fail_irq_mk_START   (1)
#define SOC_SCHARGER_FCP_IRQ3_MASK_tail_hand_fail_irq_mk_END     (1)
#define SOC_SCHARGER_FCP_IRQ3_MASK_trans_hand_fail_irq_mk_START  (2)
#define SOC_SCHARGER_FCP_IRQ3_MASK_trans_hand_fail_irq_mk_END    (2)
#define SOC_SCHARGER_FCP_IRQ3_MASK_init_hand_fail_irq_mk_START   (3)
#define SOC_SCHARGER_FCP_IRQ3_MASK_init_hand_fail_irq_mk_END     (3)
#define SOC_SCHARGER_FCP_IRQ3_MASK_rx_data_det_irq_mk_START      (4)
#define SOC_SCHARGER_FCP_IRQ3_MASK_rx_data_det_irq_mk_END        (4)
#define SOC_SCHARGER_FCP_IRQ3_MASK_rx_head_det_irq_mk_START      (5)
#define SOC_SCHARGER_FCP_IRQ3_MASK_rx_head_det_irq_mk_END        (5)
#define SOC_SCHARGER_FCP_IRQ3_MASK_cmd_err_irq_mk_START          (6)
#define SOC_SCHARGER_FCP_IRQ3_MASK_cmd_err_irq_mk_END            (6)
#define SOC_SCHARGER_FCP_IRQ3_MASK_length_err_irq_mk_START       (7)
#define SOC_SCHARGER_FCP_IRQ3_MASK_length_err_irq_mk_END         (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_FCP_IRQ4_MASK_UNION
 结构说明  : FCP_IRQ4_MASK 寄存器结构定义。地址偏移量:0x03，初值:0x3F，宽度:8
 寄存器说明: FCP中断屏蔽4寄存器。
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  last_hand_no_respond_irq_mk   : 1;  /* bit[0]  : 0：使能该中断；
                                                                        1：屏蔽该中断。 */
        unsigned char  tail_hand_no_respond_irq_mk   : 1;  /* bit[1]  : 0：使能该中断；
                                                                        1：屏蔽该中断。 */
        unsigned char  trans_hand_no_respond_irq_mk  : 1;  /* bit[2]  : 0：使能该中断；
                                                                        1：屏蔽该中断。 */
        unsigned char  init_hand_no_respond_irq_mk   : 1;  /* bit[3]  : 0：使能该中断；
                                                                        1：屏蔽该中断。 */
        unsigned char  enable_hand_fail_irq_mk       : 1;  /* bit[4]  : 0：使能该中断；
                                                                        1：屏蔽该中断。 */
        unsigned char  enable_hand_no_respond_irq_mk : 1;  /* bit[5]  : 0：使能该中断；
                                                                        1：屏蔽该中断。 */
        unsigned char  reserved                      : 2;  /* bit[6-7]: 保留。 */
    } reg;
} SOC_SCHARGER_FCP_IRQ4_MASK_UNION;
#endif
#define SOC_SCHARGER_FCP_IRQ4_MASK_last_hand_no_respond_irq_mk_START    (0)
#define SOC_SCHARGER_FCP_IRQ4_MASK_last_hand_no_respond_irq_mk_END      (0)
#define SOC_SCHARGER_FCP_IRQ4_MASK_tail_hand_no_respond_irq_mk_START    (1)
#define SOC_SCHARGER_FCP_IRQ4_MASK_tail_hand_no_respond_irq_mk_END      (1)
#define SOC_SCHARGER_FCP_IRQ4_MASK_trans_hand_no_respond_irq_mk_START   (2)
#define SOC_SCHARGER_FCP_IRQ4_MASK_trans_hand_no_respond_irq_mk_END     (2)
#define SOC_SCHARGER_FCP_IRQ4_MASK_init_hand_no_respond_irq_mk_START    (3)
#define SOC_SCHARGER_FCP_IRQ4_MASK_init_hand_no_respond_irq_mk_END      (3)
#define SOC_SCHARGER_FCP_IRQ4_MASK_enable_hand_fail_irq_mk_START        (4)
#define SOC_SCHARGER_FCP_IRQ4_MASK_enable_hand_fail_irq_mk_END          (4)
#define SOC_SCHARGER_FCP_IRQ4_MASK_enable_hand_no_respond_irq_mk_START  (5)
#define SOC_SCHARGER_FCP_IRQ4_MASK_enable_hand_no_respond_irq_mk_END    (5)


/*****************************************************************************
 结构名    : SOC_SCHARGER_MSTATE_UNION
 结构说明  : MSTATE 寄存器结构定义。地址偏移量:0x04，初值:0x00，宽度:8
 寄存器说明: 状态机状态寄存器。
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  mstate   : 4;  /* bit[0-3]: FCP状态机状态寄存器。 */
        unsigned char  reserved : 4;  /* bit[4-7]: 保留。 */
    } reg;
} SOC_SCHARGER_MSTATE_UNION;
#endif
#define SOC_SCHARGER_MSTATE_mstate_START    (0)
#define SOC_SCHARGER_MSTATE_mstate_END      (3)


/*****************************************************************************
 结构名    : SOC_SCHARGER_CRC_ENABLE_UNION
 结构说明  : CRC_ENABLE 寄存器结构定义。地址偏移量:0x05，初值:0x01，宽度:8
 寄存器说明: crc 使能控制寄存器。
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  crc_en   : 1;  /* bit[0]  : crc 使能控制寄存器：
                                                   0：不使能；
                                                   1：使能。 */
        unsigned char  reserved : 7;  /* bit[1-7]: 保留。 */
    } reg;
} SOC_SCHARGER_CRC_ENABLE_UNION;
#endif
#define SOC_SCHARGER_CRC_ENABLE_crc_en_START    (0)
#define SOC_SCHARGER_CRC_ENABLE_crc_en_END      (0)


/*****************************************************************************
 结构名    : SOC_SCHARGER_CRC_START_VALUE_UNION
 结构说明  : CRC_START_VALUE 寄存器结构定义。地址偏移量:0x06，初值:0x00，宽度:8
 寄存器说明: crc 初始值。
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  crc_start_val : 8;  /* bit[0-7]: crc 初始值。 */
    } reg;
} SOC_SCHARGER_CRC_START_VALUE_UNION;
#endif
#define SOC_SCHARGER_CRC_START_VALUE_crc_start_val_START  (0)
#define SOC_SCHARGER_CRC_START_VALUE_crc_start_val_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_SAMPLE_CNT_ADJ_UNION
 结构说明  : SAMPLE_CNT_ADJ 寄存器结构定义。地址偏移量:0x07，初值:0x00，宽度:8
 寄存器说明: 采样点调整寄存器。
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  sample_cnt_adjust : 5;  /* bit[0-4]: 采样slaver数据周期手动调节寄存器，为0时采样周期为计算slaver ping长度得出，当配置其他值时，采样周期就为配置值。
                                                            注：可配置的最大值为28 */
        unsigned char  reserved          : 3;  /* bit[5-7]: 保留。 */
    } reg;
} SOC_SCHARGER_SAMPLE_CNT_ADJ_UNION;
#endif
#define SOC_SCHARGER_SAMPLE_CNT_ADJ_sample_cnt_adjust_START  (0)
#define SOC_SCHARGER_SAMPLE_CNT_ADJ_sample_cnt_adjust_END    (4)


/*****************************************************************************
 结构名    : SOC_SCHARGER_RX_PING_WIDTH_MIN_H_UNION
 结构说明  : RX_PING_WIDTH_MIN_H 寄存器结构定义。地址偏移量:0x08，初值:0x01，宽度:8
 寄存器说明: RX ping 最小长度高位。
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  rx_ping_width_min_h : 1;  /* bit[0]  : Slaver ping最短有效长度高位。 */
        unsigned char  reserved            : 7;  /* bit[1-7]: 保留。 */
    } reg;
} SOC_SCHARGER_RX_PING_WIDTH_MIN_H_UNION;
#endif
#define SOC_SCHARGER_RX_PING_WIDTH_MIN_H_rx_ping_width_min_h_START  (0)
#define SOC_SCHARGER_RX_PING_WIDTH_MIN_H_rx_ping_width_min_h_END    (0)


/*****************************************************************************
 结构名    : SOC_SCHARGER_RX_PING_WIDTH_MIN_L_UNION
 结构说明  : RX_PING_WIDTH_MIN_L 寄存器结构定义。地址偏移量:0x09，初值:0x00，宽度:8
 寄存器说明: RX ping 最小长度低8位
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  rx_ping_width_min_l : 8;  /* bit[0-7]: Slaver ping最短有效长度低8位。 */
    } reg;
} SOC_SCHARGER_RX_PING_WIDTH_MIN_L_UNION;
#endif
#define SOC_SCHARGER_RX_PING_WIDTH_MIN_L_rx_ping_width_min_l_START  (0)
#define SOC_SCHARGER_RX_PING_WIDTH_MIN_L_rx_ping_width_min_l_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_RX_PING_WIDTH_MAX_H_UNION
 结构说明  : RX_PING_WIDTH_MAX_H 寄存器结构定义。地址偏移量:0x0A，初值:0x01，宽度:8
 寄存器说明: RX ping 最大长度高位
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  rx_ping_width_max_h : 1;  /* bit[0]  : Slaver ping最长有效长度高位。 */
        unsigned char  reserved            : 7;  /* bit[1-7]: 保留。 */
    } reg;
} SOC_SCHARGER_RX_PING_WIDTH_MAX_H_UNION;
#endif
#define SOC_SCHARGER_RX_PING_WIDTH_MAX_H_rx_ping_width_max_h_START  (0)
#define SOC_SCHARGER_RX_PING_WIDTH_MAX_H_rx_ping_width_max_h_END    (0)


/*****************************************************************************
 结构名    : SOC_SCHARGER_RX_PING_WIDTH_MAX_L_UNION
 结构说明  : RX_PING_WIDTH_MAX_L 寄存器结构定义。地址偏移量:0x0B，初值:0x8B，宽度:8
 寄存器说明: RX ping 最大长度低8位。
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  rx_ping_width_max_l : 8;  /* bit[0-7]: Slaver ping最长有效长度低8位。 */
    } reg;
} SOC_SCHARGER_RX_PING_WIDTH_MAX_L_UNION;
#endif
#define SOC_SCHARGER_RX_PING_WIDTH_MAX_L_rx_ping_width_max_l_START  (0)
#define SOC_SCHARGER_RX_PING_WIDTH_MAX_L_rx_ping_width_max_l_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_DATA_WAIT_TIME_UNION
 结构说明  : DATA_WAIT_TIME 寄存器结构定义。地址偏移量:0x0C，初值:0x64，宽度:8
 寄存器说明: 数据等待时间寄存器。
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  data_wait_time : 7;  /* bit[0-6]: 数据包等待配置时间寄存器，实际周期为配置值*4 cycle。 */
        unsigned char  reserved       : 1;  /* bit[7]  : 保留。 */
    } reg;
} SOC_SCHARGER_DATA_WAIT_TIME_UNION;
#endif
#define SOC_SCHARGER_DATA_WAIT_TIME_data_wait_time_START  (0)
#define SOC_SCHARGER_DATA_WAIT_TIME_data_wait_time_END    (6)


/*****************************************************************************
 结构名    : SOC_SCHARGER_RETRY_CNT_UNION
 结构说明  : RETRY_CNT 寄存器结构定义。地址偏移量:0x0D，初值:0x03，宽度:8
 寄存器说明: 数据重新发送次数。
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  cmd_retry_config : 4;  /* bit[0-3]: 数据包出错，master retry次数。 */
        unsigned char  reserved         : 4;  /* bit[4-7]: 保留。 */
    } reg;
} SOC_SCHARGER_RETRY_CNT_UNION;
#endif
#define SOC_SCHARGER_RETRY_CNT_cmd_retry_config_START  (0)
#define SOC_SCHARGER_RETRY_CNT_cmd_retry_config_END    (3)


/*****************************************************************************
 结构名    : SOC_SCHARGER_FCP_RO_RESERVE_UNION
 结构说明  : FCP_RO_RESERVE 寄存器结构定义。地址偏移量:0x0E，初值:0x00，宽度:8
 寄存器说明: fcp只读预留寄存器。
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  fcp_ro_reserve : 8;  /* bit[0-7]: fcp只读预留寄存器。 */
    } reg;
} SOC_SCHARGER_FCP_RO_RESERVE_UNION;
#endif
#define SOC_SCHARGER_FCP_RO_RESERVE_fcp_ro_reserve_START  (0)
#define SOC_SCHARGER_FCP_RO_RESERVE_fcp_ro_reserve_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_FCP_DEBUG_REG0_UNION
 结构说明  : FCP_DEBUG_REG0 寄存器结构定义。地址偏移量:0x0F，初值:0x0a，宽度:8
 寄存器说明: FCP debug寄存器0。
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  slv_crc_err         : 1;  /* bit[0]  : Slaver数据crc校验状态。 */
        unsigned char  resp_ack_parity_err : 1;  /* bit[1]  : Slaver返回的ACK/NACK数据的partity校验。 */
        unsigned char  fcp_head_early_err  : 1;  /* bit[2]  : Slaver返回的同步头过早上报的中断 */
        unsigned char  slv_crc_parity_err  : 1;  /* bit[3]  : Slaver返回的CRC数据的parity校验。 */
        unsigned char  rdata_range_err     : 1;  /* bit[4]  : Slave返回数据频偏超出master可接受误差范围出错。 */
        unsigned char  reserved            : 3;  /* bit[5-7]: 保留。 */
    } reg;
} SOC_SCHARGER_FCP_DEBUG_REG0_UNION;
#endif
#define SOC_SCHARGER_FCP_DEBUG_REG0_slv_crc_err_START          (0)
#define SOC_SCHARGER_FCP_DEBUG_REG0_slv_crc_err_END            (0)
#define SOC_SCHARGER_FCP_DEBUG_REG0_resp_ack_parity_err_START  (1)
#define SOC_SCHARGER_FCP_DEBUG_REG0_resp_ack_parity_err_END    (1)
#define SOC_SCHARGER_FCP_DEBUG_REG0_fcp_head_early_err_START   (2)
#define SOC_SCHARGER_FCP_DEBUG_REG0_fcp_head_early_err_END     (2)
#define SOC_SCHARGER_FCP_DEBUG_REG0_slv_crc_parity_err_START   (3)
#define SOC_SCHARGER_FCP_DEBUG_REG0_slv_crc_parity_err_END     (3)
#define SOC_SCHARGER_FCP_DEBUG_REG0_rdata_range_err_START      (4)
#define SOC_SCHARGER_FCP_DEBUG_REG0_rdata_range_err_END        (4)


/*****************************************************************************
 结构名    : SOC_SCHARGER_FCP_DEBUG_REG1_UNION
 结构说明  : FCP_DEBUG_REG1 寄存器结构定义。地址偏移量:0x10，初值:0x00，宽度:8
 寄存器说明: FCP debug寄存器1。
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  resp_ack : 8;  /* bit[0-7]: FCP debug寄存器1，Slaver返回的ACK/NAC数据。 */
    } reg;
} SOC_SCHARGER_FCP_DEBUG_REG1_UNION;
#endif
#define SOC_SCHARGER_FCP_DEBUG_REG1_resp_ack_START  (0)
#define SOC_SCHARGER_FCP_DEBUG_REG1_resp_ack_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_FCP_DEBUG_REG2_UNION
 结构说明  : FCP_DEBUG_REG2 寄存器结构定义。地址偏移量:0x11，初值:0x00，宽度:8
 寄存器说明: FCP debug寄存器2。
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  slv_crc : 8;  /* bit[0-7]: FCP debug寄存器2，Slaver返回的CRC数据。 */
    } reg;
} SOC_SCHARGER_FCP_DEBUG_REG2_UNION;
#endif
#define SOC_SCHARGER_FCP_DEBUG_REG2_slv_crc_START  (0)
#define SOC_SCHARGER_FCP_DEBUG_REG2_slv_crc_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_FCP_DEBUG_REG3_UNION
 结构说明  : FCP_DEBUG_REG3 寄存器结构定义。地址偏移量:0x12，初值:0x07，宽度:8
 寄存器说明: FCP debug寄存器3。
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  rx_ping_cnt_l : 8;  /* bit[0-7]: FCP debug寄存器3，检测到slave ping长度，低8bit。 */
    } reg;
} SOC_SCHARGER_FCP_DEBUG_REG3_UNION;
#endif
#define SOC_SCHARGER_FCP_DEBUG_REG3_rx_ping_cnt_l_START  (0)
#define SOC_SCHARGER_FCP_DEBUG_REG3_rx_ping_cnt_l_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_FCP_DEBUG_REG4_UNION
 结构说明  : FCP_DEBUG_REG4 寄存器结构定义。地址偏移量:0x13，初值:0x00，宽度:8
 寄存器说明: FCP debug寄存器4。
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  reserved        : 2;  /* bit[0-1]: 保留。 */
        unsigned char  rx_ping_low_cnt : 5;  /* bit[2-6]: FCP debug寄存器5，检测到slave ping后UI长度。 */
        unsigned char  rx_ping_cnt_h   : 1;  /* bit[7]  : FCP debug寄存器4，检测到slave ping长度，最高bit。 */
    } reg;
} SOC_SCHARGER_FCP_DEBUG_REG4_UNION;
#endif
#define SOC_SCHARGER_FCP_DEBUG_REG4_rx_ping_low_cnt_START  (2)
#define SOC_SCHARGER_FCP_DEBUG_REG4_rx_ping_low_cnt_END    (6)
#define SOC_SCHARGER_FCP_DEBUG_REG4_rx_ping_cnt_h_START    (7)
#define SOC_SCHARGER_FCP_DEBUG_REG4_rx_ping_cnt_h_END      (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_RX_PACKET_WAIT_ADJUST_UNION
 结构说明  : RX_PACKET_WAIT_ADJUST 寄存器结构定义。地址偏移量:0x14，初值:0x14，宽度:8
 寄存器说明: ACK前同步头等待微调节寄存器。
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  rx_packet_wait_adjust : 7;  /* bit[0-6]: SLAVE PING与ACK之间默认等待5UI，该寄存器为在原时间基础上得微调寄存器，等待时间为5UI+设置值。 */
        unsigned char  reserved              : 1;  /* bit[7]  : 保留。 */
    } reg;
} SOC_SCHARGER_RX_PACKET_WAIT_ADJUST_UNION;
#endif
#define SOC_SCHARGER_RX_PACKET_WAIT_ADJUST_rx_packet_wait_adjust_START  (0)
#define SOC_SCHARGER_RX_PACKET_WAIT_ADJUST_rx_packet_wait_adjust_END    (6)


/*****************************************************************************
 结构名    : SOC_SCHARGER_SAMPLE_CNT_TINYJUST_UNION
 结构说明  : SAMPLE_CNT_TINYJUST 寄存器结构定义。地址偏移量:0x15，初值:0x00，宽度:8
 寄存器说明: FCP采样微调寄存器
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  sample_cnt_tinyjust : 5;  /* bit[0-4]: 默认采样点为slave ping/32处，该寄存器可微调采样点。 sample_cnt_tinyjust[4]为1： 原采样点 + sample_cnt_tinyjust[3:0]. （注：sample_cnt_tinyjust[3:0]仅可配置0或1） sample_cnt_tinyjust[4]为0： 原采样点 - sample_cnt_tinyjust[3:0]。 （注：sample_cnt_tinyjust[3:0]仅可配置小于8）  */
        unsigned char  reserved            : 3;  /* bit[5-7]: 保留。 */
    } reg;
} SOC_SCHARGER_SAMPLE_CNT_TINYJUST_UNION;
#endif
#define SOC_SCHARGER_SAMPLE_CNT_TINYJUST_sample_cnt_tinyjust_START  (0)
#define SOC_SCHARGER_SAMPLE_CNT_TINYJUST_sample_cnt_tinyjust_END    (4)


/*****************************************************************************
 结构名    : SOC_SCHARGER_RX_PING_CNT_TINYJUST_UNION
 结构说明  : RX_PING_CNT_TINYJUST 寄存器结构定义。地址偏移量:0x16，初值:0x00，宽度:8
 寄存器说明: FCP检测RX PING CNT微调寄存器
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  rx_ping_cnt_tinyjust : 5;  /* bit[0-4]: 通过slave ping长度得到rx_ping_cnt，同时该cnt作为内部采集数据基准。 rx_ping_cnt_tinyjust[4]为1： rx_ping_cnt + rx_ping_cnt_tinyjust[3:0]. rx_ping_cnt_tinyjust[4]为0： rx_ping_cnt - rx_ping_cnt_tinyjust[3:0]。 （注：rx_ping_cnt_tinyjust仅可配置为小于7）  */
        unsigned char  reserved             : 3;  /* bit[5-7]: 保留。 */
    } reg;
} SOC_SCHARGER_RX_PING_CNT_TINYJUST_UNION;
#endif
#define SOC_SCHARGER_RX_PING_CNT_TINYJUST_rx_ping_cnt_tinyjust_START  (0)
#define SOC_SCHARGER_RX_PING_CNT_TINYJUST_rx_ping_cnt_tinyjust_END    (4)


/*****************************************************************************
 结构名    : SOC_SCHARGER_SHIFT_CNT_CFG_MAX_UNION
 结构说明  : SHIFT_CNT_CFG_MAX 寄存器结构定义。地址偏移量:0x17，初值:0x0C，宽度:8
 寄存器说明: SHIFT_CNT最大值配置寄存器
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  shift_cnt_cfg_max : 4;  /* bit[0-3]: 采样计数器最大值设置寄存器，超过该值master状态机跳转下一状态。 */
        unsigned char  reserved          : 4;  /* bit[4-7]: 保留。 */
    } reg;
} SOC_SCHARGER_SHIFT_CNT_CFG_MAX_UNION;
#endif
#define SOC_SCHARGER_SHIFT_CNT_CFG_MAX_shift_cnt_cfg_max_START  (0)
#define SOC_SCHARGER_SHIFT_CNT_CFG_MAX_shift_cnt_cfg_max_END    (3)


/*****************************************************************************
 结构名    : SOC_SCHARGER_HKADC_CFG_REG_0_UNION
 结构说明  : HKADC_CFG_REG_0 寄存器结构定义。地址偏移量:0x18，初值:0x00，宽度:8
 寄存器说明: HKADC_配置寄存器_0
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  da_hkadc_fdiv : 1;  /* bit[0-0]: HKADC频率选择：
                                                        0：默认频率；
                                                        1：频率减半； */
        unsigned char  reserved      : 7;  /* bit[1-7]: reserved */
    } reg;
} SOC_SCHARGER_HKADC_CFG_REG_0_UNION;
#endif
#define SOC_SCHARGER_HKADC_CFG_REG_0_da_hkadc_fdiv_START  (0)
#define SOC_SCHARGER_HKADC_CFG_REG_0_da_hkadc_fdiv_END    (0)


/*****************************************************************************
 结构名    : SOC_SCHARGER_HKADC_CFG_REG_1_UNION
 结构说明  : HKADC_CFG_REG_1 寄存器结构定义。地址偏移量:0x19，初值:0x00，宽度:8
 寄存器说明: HKADC_配置寄存器_1
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  da_hkadc_ibias_sel : 8;  /* bit[0-7]: HKADC工作电流配置，默认0 */
    } reg;
} SOC_SCHARGER_HKADC_CFG_REG_1_UNION;
#endif
#define SOC_SCHARGER_HKADC_CFG_REG_1_da_hkadc_ibias_sel_START  (0)
#define SOC_SCHARGER_HKADC_CFG_REG_1_da_hkadc_ibias_sel_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_HKADC_CFG_REG_2_UNION
 结构说明  : HKADC_CFG_REG_2 寄存器结构定义。地址偏移量:0x1A，初值:0x00，宽度:8
 寄存器说明: HKADC_配置寄存器_2
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  da_hkadc_reset    : 1;  /* bit[0-0]: HKADC内部逻辑复位控制，0不复位，1强制复位 */
        unsigned char  da_hkadc_reserved : 2;  /* bit[1-2]: HKADC备用寄存器 */
        unsigned char  da_hkadc_ibsel    : 3;  /* bit[3-5]: HKADC工作电流配置，默认0 */
        unsigned char  reserved          : 2;  /* bit[6-7]: reserved */
    } reg;
} SOC_SCHARGER_HKADC_CFG_REG_2_UNION;
#endif
#define SOC_SCHARGER_HKADC_CFG_REG_2_da_hkadc_reset_START     (0)
#define SOC_SCHARGER_HKADC_CFG_REG_2_da_hkadc_reset_END       (0)
#define SOC_SCHARGER_HKADC_CFG_REG_2_da_hkadc_reserved_START  (1)
#define SOC_SCHARGER_HKADC_CFG_REG_2_da_hkadc_reserved_END    (2)
#define SOC_SCHARGER_HKADC_CFG_REG_2_da_hkadc_ibsel_START     (3)
#define SOC_SCHARGER_HKADC_CFG_REG_2_da_hkadc_ibsel_END       (5)


/*****************************************************************************
 结构名    : SOC_SCHARGER_HKADC_OFFSET_0P1_UNION
 结构说明  : HKADC_OFFSET_0P1 寄存器结构定义。地址偏移量:0x1B，初值:0x00，宽度:8
 寄存器说明: HKADC 采样0p1校正值
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  sc_offset_0p1_r : 7;  /* bit[0-6]: HKADC采样误差校正adc_code_0p1寄存器配置值 */
        unsigned char  reserved        : 1;  /* bit[7-7]: reserved */
    } reg;
} SOC_SCHARGER_HKADC_OFFSET_0P1_UNION;
#endif
#define SOC_SCHARGER_HKADC_OFFSET_0P1_sc_offset_0p1_r_START  (0)
#define SOC_SCHARGER_HKADC_OFFSET_0P1_sc_offset_0p1_r_END    (6)


/*****************************************************************************
 结构名    : SOC_SCHARGER_HKADC_OFFSET_2P45_UNION
 结构说明  : HKADC_OFFSET_2P45 寄存器结构定义。地址偏移量:0x1C，初值:0x00，宽度:8
 寄存器说明: HKADC 采样2p45校正值
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  sc_offset_2p45_r : 7;  /* bit[0-6]: HKADC采样误差校正adc_code_2p45寄存器配置值 */
        unsigned char  reserved         : 1;  /* bit[7-7]: reserved */
    } reg;
} SOC_SCHARGER_HKADC_OFFSET_2P45_UNION;
#endif
#define SOC_SCHARGER_HKADC_OFFSET_2P45_sc_offset_2p45_r_START  (0)
#define SOC_SCHARGER_HKADC_OFFSET_2P45_sc_offset_2p45_r_END    (6)


/*****************************************************************************
 结构名    : SOC_SCHARGER_SOH_OVH_TH0_L_UNION
 结构说明  : SOH_OVH_TH0_L 寄存器结构定义。地址偏移量:0x1D，初值:0xFF，宽度:8
 寄存器说明: 电池过压保护电压检测高阈值0的低8bit
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  sc_soh_ovh_th0_l : 8;  /* bit[0-7]: 电池过压保护电压检测高阈值0的低8bit */
    } reg;
} SOC_SCHARGER_SOH_OVH_TH0_L_UNION;
#endif
#define SOC_SCHARGER_SOH_OVH_TH0_L_sc_soh_ovh_th0_l_START  (0)
#define SOC_SCHARGER_SOH_OVH_TH0_L_sc_soh_ovh_th0_l_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_SOH_OVH_TH0_H_UNION
 结构说明  : SOH_OVH_TH0_H 寄存器结构定义。地址偏移量:0x1E，初值:0x0F，宽度:8
 寄存器说明: 电池过压保护电压检测高阈值0的高4bit
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  sc_soh_ovh_th0_h : 4;  /* bit[0-3]: 电池过压保护电压检测高阈值0的高4bit */
        unsigned char  reserved         : 4;  /* bit[4-7]: reserved */
    } reg;
} SOC_SCHARGER_SOH_OVH_TH0_H_UNION;
#endif
#define SOC_SCHARGER_SOH_OVH_TH0_H_sc_soh_ovh_th0_h_START  (0)
#define SOC_SCHARGER_SOH_OVH_TH0_H_sc_soh_ovh_th0_h_END    (3)


/*****************************************************************************
 结构名    : SOC_SCHARGER_TSBAT_OVH_TH0_L_UNION
 结构说明  : TSBAT_OVH_TH0_L 寄存器结构定义。地址偏移量:0x1F，初值:0x00，宽度:8
 寄存器说明: 电池过压保护温度检测高阈值0的低8bit
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  sc_tsbat_ot0_l : 8;  /* bit[0-7]: 电池过压保护温度检测高阈值0的低8bit */
    } reg;
} SOC_SCHARGER_TSBAT_OVH_TH0_L_UNION;
#endif
#define SOC_SCHARGER_TSBAT_OVH_TH0_L_sc_tsbat_ot0_l_START  (0)
#define SOC_SCHARGER_TSBAT_OVH_TH0_L_sc_tsbat_ot0_l_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_TSBAT_OVH_TH0_H_UNION
 结构说明  : TSBAT_OVH_TH0_H 寄存器结构定义。地址偏移量:0x20，初值:0x00，宽度:8
 寄存器说明: 电池过压保护温度检测高阈值0的高4bit
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  sc_tsbat_ot0_h : 4;  /* bit[0-3]: 电池过压保护温度检测高阈值0的高4bit */
        unsigned char  reserved       : 4;  /* bit[4-7]: reserved */
    } reg;
} SOC_SCHARGER_TSBAT_OVH_TH0_H_UNION;
#endif
#define SOC_SCHARGER_TSBAT_OVH_TH0_H_sc_tsbat_ot0_h_START  (0)
#define SOC_SCHARGER_TSBAT_OVH_TH0_H_sc_tsbat_ot0_h_END    (3)


/*****************************************************************************
 结构名    : SOC_SCHARGER_SOH_OVH_TH1_L_UNION
 结构说明  : SOH_OVH_TH1_L 寄存器结构定义。地址偏移量:0x21，初值:0xFF，宽度:8
 寄存器说明: 电池过压保护电压检测高阈值1的低8bit
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  sc_soh_ovh_th1_l : 8;  /* bit[0-7]: 电池过压保护电压检测高阈值1的低8bit */
    } reg;
} SOC_SCHARGER_SOH_OVH_TH1_L_UNION;
#endif
#define SOC_SCHARGER_SOH_OVH_TH1_L_sc_soh_ovh_th1_l_START  (0)
#define SOC_SCHARGER_SOH_OVH_TH1_L_sc_soh_ovh_th1_l_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_SOH_OVH_TH1_H_UNION
 结构说明  : SOH_OVH_TH1_H 寄存器结构定义。地址偏移量:0x22，初值:0x0F，宽度:8
 寄存器说明: 电池过压保护电压检测高阈值1的高4bit
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  sc_soh_ovh_th1_h : 4;  /* bit[0-3]: 电池过压保护电压检测高阈值1的高4bit */
        unsigned char  reserved         : 4;  /* bit[4-7]: reserved */
    } reg;
} SOC_SCHARGER_SOH_OVH_TH1_H_UNION;
#endif
#define SOC_SCHARGER_SOH_OVH_TH1_H_sc_soh_ovh_th1_h_START  (0)
#define SOC_SCHARGER_SOH_OVH_TH1_H_sc_soh_ovh_th1_h_END    (3)


/*****************************************************************************
 结构名    : SOC_SCHARGER_TSBAT_OVH_TH1_L_UNION
 结构说明  : TSBAT_OVH_TH1_L 寄存器结构定义。地址偏移量:0x23，初值:0x00，宽度:8
 寄存器说明: 电池过压保护温度检测高阈值1的低8bit
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  sc_tsbat_ot1_l : 8;  /* bit[0-7]: 电池过压保护温度检测高阈值1的低8bit */
    } reg;
} SOC_SCHARGER_TSBAT_OVH_TH1_L_UNION;
#endif
#define SOC_SCHARGER_TSBAT_OVH_TH1_L_sc_tsbat_ot1_l_START  (0)
#define SOC_SCHARGER_TSBAT_OVH_TH1_L_sc_tsbat_ot1_l_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_TSBAT_OVH_TH1_H_UNION
 结构说明  : TSBAT_OVH_TH1_H 寄存器结构定义。地址偏移量:0x24，初值:0x00，宽度:8
 寄存器说明: 电池过压保护温度检测高阈值1的高4bit
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  sc_tsbat_ot1_h : 4;  /* bit[0-3]: 电池过压保护温度检测高阈值1的高4bit */
        unsigned char  reserved       : 4;  /* bit[4-7]: reserved */
    } reg;
} SOC_SCHARGER_TSBAT_OVH_TH1_H_UNION;
#endif
#define SOC_SCHARGER_TSBAT_OVH_TH1_H_sc_tsbat_ot1_h_START  (0)
#define SOC_SCHARGER_TSBAT_OVH_TH1_H_sc_tsbat_ot1_h_END    (3)


/*****************************************************************************
 结构名    : SOC_SCHARGER_SOH_OVH_TH2_L_UNION
 结构说明  : SOH_OVH_TH2_L 寄存器结构定义。地址偏移量:0x25，初值:0xFF，宽度:8
 寄存器说明: 电池过压保护电压检测高阈值2的低8bit
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  sc_soh_ovh_th2_l : 8;  /* bit[0-7]: 电池过压保护电压检测高阈值2的低8bit */
    } reg;
} SOC_SCHARGER_SOH_OVH_TH2_L_UNION;
#endif
#define SOC_SCHARGER_SOH_OVH_TH2_L_sc_soh_ovh_th2_l_START  (0)
#define SOC_SCHARGER_SOH_OVH_TH2_L_sc_soh_ovh_th2_l_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_SOH_OVH_TH2_H_UNION
 结构说明  : SOH_OVH_TH2_H 寄存器结构定义。地址偏移量:0x26，初值:0x0F，宽度:8
 寄存器说明: 电池过压保护电压检测高阈值2的高4bit
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  sc_soh_ovh_th2_h : 4;  /* bit[0-3]: 电池过压保护电压检测高阈值2的高4bit */
        unsigned char  reserved         : 4;  /* bit[4-7]: reserved */
    } reg;
} SOC_SCHARGER_SOH_OVH_TH2_H_UNION;
#endif
#define SOC_SCHARGER_SOH_OVH_TH2_H_sc_soh_ovh_th2_h_START  (0)
#define SOC_SCHARGER_SOH_OVH_TH2_H_sc_soh_ovh_th2_h_END    (3)


/*****************************************************************************
 结构名    : SOC_SCHARGER_TSBAT_OVH_TH2_L_UNION
 结构说明  : TSBAT_OVH_TH2_L 寄存器结构定义。地址偏移量:0x27，初值:0x00，宽度:8
 寄存器说明: 电池过压保护温度检测高阈值2的低8bit
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  sc_tsbat_ot2_l : 8;  /* bit[0-7]: 电池过压保护温度检测高阈值2的低8bit */
    } reg;
} SOC_SCHARGER_TSBAT_OVH_TH2_L_UNION;
#endif
#define SOC_SCHARGER_TSBAT_OVH_TH2_L_sc_tsbat_ot2_l_START  (0)
#define SOC_SCHARGER_TSBAT_OVH_TH2_L_sc_tsbat_ot2_l_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_TSBAT_OVH_TH2_H_UNION
 结构说明  : TSBAT_OVH_TH2_H 寄存器结构定义。地址偏移量:0x28，初值:0x00，宽度:8
 寄存器说明: 电池过压保护温度检测高阈值2的高4bit
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  sc_tsbat_ot2_h : 4;  /* bit[0-3]: 电池过压保护温度检测高阈值2的高4bit */
        unsigned char  reserved       : 4;  /* bit[4-7]: reserved */
    } reg;
} SOC_SCHARGER_TSBAT_OVH_TH2_H_UNION;
#endif
#define SOC_SCHARGER_TSBAT_OVH_TH2_H_sc_tsbat_ot2_h_START  (0)
#define SOC_SCHARGER_TSBAT_OVH_TH2_H_sc_tsbat_ot2_h_END    (3)


/*****************************************************************************
 结构名    : SOC_SCHARGER_SOH_OVL_TH_L_UNION
 结构说明  : SOH_OVL_TH_L 寄存器结构定义。地址偏移量:0x29，初值:0x52，宽度:8
 寄存器说明: 电池过压保护电压检测低阈值的低8bit
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  sc_soh_ovl_th_l : 8;  /* bit[0-7]: 电池过压保护电压检测低阈值的低8bit */
    } reg;
} SOC_SCHARGER_SOH_OVL_TH_L_UNION;
#endif
#define SOC_SCHARGER_SOH_OVL_TH_L_sc_soh_ovl_th_l_START  (0)
#define SOC_SCHARGER_SOH_OVL_TH_L_sc_soh_ovl_th_l_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_SOH_OVL_TH_H_UNION
 结构说明  : SOH_OVL_TH_H 寄存器结构定义。地址偏移量:0x2A，初值:0x08，宽度:8
 寄存器说明: 电池过压保护电压检测低阈值的高4bit
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  sc_soh_ovl_th_h : 4;  /* bit[0-3]: 电池过压保护电压检测低阈值的高4bit */
        unsigned char  reserved        : 4;  /* bit[4-7]: reserved */
    } reg;
} SOC_SCHARGER_SOH_OVL_TH_H_UNION;
#endif
#define SOC_SCHARGER_SOH_OVL_TH_H_sc_soh_ovl_th_h_START  (0)
#define SOC_SCHARGER_SOH_OVL_TH_H_sc_soh_ovl_th_h_END    (3)


/*****************************************************************************
 结构名    : SOC_SCHARGER_SOH_OVP_REAL_UNION
 结构说明  : SOH_OVP_REAL 寄存器结构定义。地址偏移量:0x2B，初值:0x00，宽度:8
 寄存器说明: SOH OVP 高/底阈值检测实时记录
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  tb_soh_ovh   : 1;  /* bit[0-0]: 高阈值检测结果（持续时间比较短，查看对应的中断寄存器） */
        unsigned char  tb_soh_ovl   : 1;  /* bit[1-1]: 电池过压保护低阈值检测结果 */
        unsigned char  tb_tmp_ovh_2 : 1;  /* bit[2-2]: TBAT温度保护高阈值2检测结果 */
        unsigned char  tb_soh_ovh_2 : 1;  /* bit[3-3]: 电池过压保护高压阈值2检测结果 */
        unsigned char  tb_tmp_ovh_1 : 1;  /* bit[4-4]: TBAT温度保护高阈值1检测结果 */
        unsigned char  tb_soh_ovh_1 : 1;  /* bit[5-5]: 电池过压保护高压阈值1检测结果 */
        unsigned char  tb_tmp_ovh_0 : 1;  /* bit[6-6]: TBAT温度保护高阈值0检测结果 */
        unsigned char  tb_soh_ovh_0 : 1;  /* bit[7-7]: 电池过压保护高压阈值0检测结果 */
    } reg;
} SOC_SCHARGER_SOH_OVP_REAL_UNION;
#endif
#define SOC_SCHARGER_SOH_OVP_REAL_tb_soh_ovh_START    (0)
#define SOC_SCHARGER_SOH_OVP_REAL_tb_soh_ovh_END      (0)
#define SOC_SCHARGER_SOH_OVP_REAL_tb_soh_ovl_START    (1)
#define SOC_SCHARGER_SOH_OVP_REAL_tb_soh_ovl_END      (1)
#define SOC_SCHARGER_SOH_OVP_REAL_tb_tmp_ovh_2_START  (2)
#define SOC_SCHARGER_SOH_OVP_REAL_tb_tmp_ovh_2_END    (2)
#define SOC_SCHARGER_SOH_OVP_REAL_tb_soh_ovh_2_START  (3)
#define SOC_SCHARGER_SOH_OVP_REAL_tb_soh_ovh_2_END    (3)
#define SOC_SCHARGER_SOH_OVP_REAL_tb_tmp_ovh_1_START  (4)
#define SOC_SCHARGER_SOH_OVP_REAL_tb_tmp_ovh_1_END    (4)
#define SOC_SCHARGER_SOH_OVP_REAL_tb_soh_ovh_1_START  (5)
#define SOC_SCHARGER_SOH_OVP_REAL_tb_soh_ovh_1_END    (5)
#define SOC_SCHARGER_SOH_OVP_REAL_tb_tmp_ovh_0_START  (6)
#define SOC_SCHARGER_SOH_OVP_REAL_tb_tmp_ovh_0_END    (6)
#define SOC_SCHARGER_SOH_OVP_REAL_tb_soh_ovh_0_START  (7)
#define SOC_SCHARGER_SOH_OVP_REAL_tb_soh_ovh_0_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_HKADC_TB_EN_SEL_UNION
 结构说明  : HKADC_TB_EN_SEL 寄存器结构定义。地址偏移量:0x2C，初值:0x00，宽度:8
 寄存器说明: HKADC Testbus使能及选择信号
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  sc_hkadc_tb_sel : 7;  /* bit[0-6]: Testbus选择信号（debug使用，不对产品开放） */
        unsigned char  sc_hkadc_tb_en  : 1;  /* bit[7-7]: Testbus使能信号，高有效（debug使用，不对产品开放） */
    } reg;
} SOC_SCHARGER_HKADC_TB_EN_SEL_UNION;
#endif
#define SOC_SCHARGER_HKADC_TB_EN_SEL_sc_hkadc_tb_sel_START  (0)
#define SOC_SCHARGER_HKADC_TB_EN_SEL_sc_hkadc_tb_sel_END    (6)
#define SOC_SCHARGER_HKADC_TB_EN_SEL_sc_hkadc_tb_en_START   (7)
#define SOC_SCHARGER_HKADC_TB_EN_SEL_sc_hkadc_tb_en_END     (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_HKADC_TB_DATA0_UNION
 结构说明  : HKADC_TB_DATA0 寄存器结构定义。地址偏移量:0x2D，初值:0x00，宽度:8
 寄存器说明: HKADC testbus回读数据
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  hkadc_tb_bus_0 : 8;  /* bit[0-7]: HKADC testbus测试回读数据（debug使用，不对产品开放） */
    } reg;
} SOC_SCHARGER_HKADC_TB_DATA0_UNION;
#endif
#define SOC_SCHARGER_HKADC_TB_DATA0_hkadc_tb_bus_0_START  (0)
#define SOC_SCHARGER_HKADC_TB_DATA0_hkadc_tb_bus_0_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_HKADC_TB_DATA1_UNION
 结构说明  : HKADC_TB_DATA1 寄存器结构定义。地址偏移量:0x2E，初值:0x00，宽度:8
 寄存器说明: HKADC testbus回读数据
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  hkadc_tb_bus_1 : 8;  /* bit[0-7]: HKADC testbus测试回读数据（debug使用，不对产品开放） */
    } reg;
} SOC_SCHARGER_HKADC_TB_DATA1_UNION;
#endif
#define SOC_SCHARGER_HKADC_TB_DATA1_hkadc_tb_bus_1_START  (0)
#define SOC_SCHARGER_HKADC_TB_DATA1_hkadc_tb_bus_1_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_ACR_TOP_CFG_REG_0_UNION
 结构说明  : ACR_TOP_CFG_REG_0 寄存器结构定义。地址偏移量:0x30，初值:0x09，宽度:8
 寄存器说明: ACR_TOP_配置寄存器_0
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  da_acr_mul_sel    : 2;  /* bit[0-1]: acr电压检测放大倍数：
                                                            00：50倍
                                                            01：100倍
                                                            10：200倍
                                                            11：300倍 */
        unsigned char  da_acr_iref_sel   : 1;  /* bit[2-2]: 现在仅有内部放电电流，该寄存器无效 */
        unsigned char  da_acr_cap_sel    : 2;  /* bit[3-4]: acr电压检测补偿电容选择：
                                                            00：8C
                                                            01：4C
                                                            10：2C
                                                            11：1C */
        unsigned char  da_acr_vdetop_cap : 1;  /* bit[5-5]: vdet_op内部补偿电容：
                                                            0：不增大vdet_op内部补偿电容，
                                                            1：增大vdet_op内部补偿电容； */
        unsigned char  da_acr_testmode   : 1;  /* bit[6-6]: acr测试模式：
                                                            0：不使能测试模式
                                                            1：使能测试模式 */
        unsigned char  reserved          : 1;  /* bit[7-7]: reserved */
    } reg;
} SOC_SCHARGER_ACR_TOP_CFG_REG_0_UNION;
#endif
#define SOC_SCHARGER_ACR_TOP_CFG_REG_0_da_acr_mul_sel_START     (0)
#define SOC_SCHARGER_ACR_TOP_CFG_REG_0_da_acr_mul_sel_END       (1)
#define SOC_SCHARGER_ACR_TOP_CFG_REG_0_da_acr_iref_sel_START    (2)
#define SOC_SCHARGER_ACR_TOP_CFG_REG_0_da_acr_iref_sel_END      (2)
#define SOC_SCHARGER_ACR_TOP_CFG_REG_0_da_acr_cap_sel_START     (3)
#define SOC_SCHARGER_ACR_TOP_CFG_REG_0_da_acr_cap_sel_END       (4)
#define SOC_SCHARGER_ACR_TOP_CFG_REG_0_da_acr_vdetop_cap_START  (5)
#define SOC_SCHARGER_ACR_TOP_CFG_REG_0_da_acr_vdetop_cap_END    (5)
#define SOC_SCHARGER_ACR_TOP_CFG_REG_0_da_acr_testmode_START    (6)
#define SOC_SCHARGER_ACR_TOP_CFG_REG_0_da_acr_testmode_END      (6)


/*****************************************************************************
 结构名    : SOC_SCHARGER_ACR_TOP_CFG_REG_1_UNION
 结构说明  : ACR_TOP_CFG_REG_1 寄存器结构定义。地址偏移量:0x31，初值:0x00，宽度:8
 寄存器说明: ACR_TOP_配置寄存器_1
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  da_acr_reserve : 8;  /* bit[0-7]: 预留寄存器:
                                                         <2:0>:直充保护内部信号输出选择：
                                                         000-DGND
                                                         001-IBUS_UCP_INIT
                                                         010-VDROP_OVP_INIT
                                                         011-VDROP_NEG_OVP_INIT
                                                         100-VDROP_OVP_NFLT_INIT
                                                         101-VDROP_NEG_OVP_NFLT_INIT
                                                         110-IBAT_DCUCP_ALM_INIT
                                                         111-S_CFLYP_SCP
                                                         <3>:直充保护内部信号输出使能控制：1-输出；0-不输出
                                                         <7:4>:预留 */
    } reg;
} SOC_SCHARGER_ACR_TOP_CFG_REG_1_UNION;
#endif
#define SOC_SCHARGER_ACR_TOP_CFG_REG_1_da_acr_reserve_START  (0)
#define SOC_SCHARGER_ACR_TOP_CFG_REG_1_da_acr_reserve_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_ACR_TOP_CFG_REG_2_UNION
 结构说明  : ACR_TOP_CFG_REG_2 寄存器结构定义。地址偏移量:0x32，初值:0xFF，宽度:8
 寄存器说明: ACR_TOP_配置寄存器_2
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  da_ovp_idis_sel : 4;  /* bit[0-3]: ovp放电电流选择:
                                                          0000：50mA
                                                          0001：60mA
                                                          0010：70mA
                                                          0011：80mA
                                                          0100：90mA
                                                          0101：100mA
                                                          0110：110mA
                                                          0111：120mA
                                                          1000：130mA
                                                          1001：140mA
                                                          1010：150mA
                                                          1011：160mA
                                                          1100：170mA
                                                          1101：180mA
                                                          1110：190mA
                                                          1111：200mA */
        unsigned char  da_pc_idis_sel  : 4;  /* bit[4-7]: 脉冲充电模式放电电流选择：
                                                          0000：50mA
                                                          0001：60mA
                                                          0010：70mA
                                                          0011：80mA
                                                          0100：90mA
                                                          0101：100mA
                                                          0110：110mA
                                                          0111：120mA
                                                          1000：130mA
                                                          1001：140mA
                                                          1010：150mA
                                                          1011：160mA
                                                          1100：170mA
                                                          1101：180mA
                                                          1110：190mA
                                                          1111：200mA */
    } reg;
} SOC_SCHARGER_ACR_TOP_CFG_REG_2_UNION;
#endif
#define SOC_SCHARGER_ACR_TOP_CFG_REG_2_da_ovp_idis_sel_START  (0)
#define SOC_SCHARGER_ACR_TOP_CFG_REG_2_da_ovp_idis_sel_END    (3)
#define SOC_SCHARGER_ACR_TOP_CFG_REG_2_da_pc_idis_sel_START   (4)
#define SOC_SCHARGER_ACR_TOP_CFG_REG_2_da_pc_idis_sel_END     (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_ACR_SAMPLE_TIME_H_UNION
 结构说明  : ACR_SAMPLE_TIME_H 寄存器结构定义。地址偏移量:0x33，初值:0x33，宽度:8
 寄存器说明: 第n+1个acr_pulse高电平期采样时间档位
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  sc_acr_sample_t2 : 3;  /* bit[0-2]: ACR模块被采样信号高电平期间采样间隔T2档位：
                                                           3’b000：113us；
                                                           3’b001：109us；
                                                           3’b010：105us；
                                                           3’b011：101us（默认）；
                                                           3’b100：97us；
                                                           3’b101：93us；
                                                           3’b110：89us；
                                                           3’b111：85us；
                                                           软件保证：1）配置的T1+4*T2不超出acr_pulse的半个周期（500us)；
                                                           2）T3+4*T4不能超出半个个acr_pulse周期（500us） */
        unsigned char  reserved_0       : 1;  /* bit[3-3]: reserved */
        unsigned char  sc_acr_sample_t1 : 3;  /* bit[4-6]: ACR模块被采样信号上升沿稳定时间T1档位：
                                                           3’b000：48us；
                                                           3’b001：64us；
                                                           3’b010：80us；
                                                           3’b011：96us（默认）；
                                                           3’b100：112us；
                                                           3’b101：128us；
                                                           3’b110：144us；
                                                           3’b111：160us；
                                                           软件保证：1）配置的T1+4*T2不超出acr_pulse的半个周期（500us)；
                                                           2）T3+4*T4不能超出半个个acr_pulse周期（500us） */
        unsigned char  reserved_1       : 1;  /* bit[7-7]: reserved */
    } reg;
} SOC_SCHARGER_ACR_SAMPLE_TIME_H_UNION;
#endif
#define SOC_SCHARGER_ACR_SAMPLE_TIME_H_sc_acr_sample_t2_START  (0)
#define SOC_SCHARGER_ACR_SAMPLE_TIME_H_sc_acr_sample_t2_END    (2)
#define SOC_SCHARGER_ACR_SAMPLE_TIME_H_sc_acr_sample_t1_START  (4)
#define SOC_SCHARGER_ACR_SAMPLE_TIME_H_sc_acr_sample_t1_END    (6)


/*****************************************************************************
 结构名    : SOC_SCHARGER_ACR_SAMPLE_TIME_L_UNION
 结构说明  : ACR_SAMPLE_TIME_L 寄存器结构定义。地址偏移量:0x34，初值:0x33，宽度:8
 寄存器说明: 第n+1个acr_pulse低电平期采样时间档位
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  sc_acr_sample_t4 : 3;  /* bit[0-2]: ACR模块被采样信号低电平期间采样间隔T4档位：
                                                           3’b000：113us；
                                                           3’b001：109us；
                                                           3’b010：105us；
                                                           3’b011：101us（默认）；
                                                           3’b100：97us；
                                                           3’b101：93us；
                                                           3’b110：89us；
                                                           3’b111：85us；
                                                           软件保证：1）配置的T1+4*T2不超出acr_pulse的半个周期（500us)；
                                                           2）T3+4*T4不能超出半个个acr_pulse周期（500us） */
        unsigned char  reserved_0       : 1;  /* bit[3-3]: reserved */
        unsigned char  sc_acr_sample_t3 : 3;  /* bit[4-6]: ACR模块被采样信号下降沿稳定时间T3档位：
                                                           3’b000：48us；
                                                           3’b001：64us；
                                                           3’b010：80us；
                                                           3’b011：96us（默认）；
                                                           3’b100：112us；
                                                           3’b101：128us；
                                                           3’b110：144us；
                                                           3’b111：160us；
                                                           软件保证：1）配置的T1+4*T2不超出acr_pulse的半个周期（500us)；
                                                           2）T3+4*T4不能超出半个个acr_pulse周期（500us） */
        unsigned char  reserved_1       : 1;  /* bit[7-7]: reserved */
    } reg;
} SOC_SCHARGER_ACR_SAMPLE_TIME_L_UNION;
#endif
#define SOC_SCHARGER_ACR_SAMPLE_TIME_L_sc_acr_sample_t4_START  (0)
#define SOC_SCHARGER_ACR_SAMPLE_TIME_L_sc_acr_sample_t4_END    (2)
#define SOC_SCHARGER_ACR_SAMPLE_TIME_L_sc_acr_sample_t3_START  (4)
#define SOC_SCHARGER_ACR_SAMPLE_TIME_L_sc_acr_sample_t3_END    (6)


/*****************************************************************************
 结构名    : SOC_SCHARGER_ACR_DATA0_L_UNION
 结构说明  : ACR_DATA0_L 寄存器结构定义。地址偏移量:0x35，初值:0x00，宽度:8
 寄存器说明: 第n+1个acr_pulse高电平期间第一个采样数据低8bit。
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  acr_data0_l : 8;  /* bit[0-7]: 第n+1个acr_pulse高电平期间第一个采样数据低8bit。 */
    } reg;
} SOC_SCHARGER_ACR_DATA0_L_UNION;
#endif
#define SOC_SCHARGER_ACR_DATA0_L_acr_data0_l_START  (0)
#define SOC_SCHARGER_ACR_DATA0_L_acr_data0_l_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_ACR_DATA0_H_UNION
 结构说明  : ACR_DATA0_H 寄存器结构定义。地址偏移量:0x36，初值:0x00，宽度:8
 寄存器说明: 第n+1个acr_pulse高电平期间第一个采样数据高4bit。
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  acr_data0_h : 4;  /* bit[0-3]: 第n+1个acr_pulse高电平期间第一个采样数据高4bit。 */
        unsigned char  reserved    : 4;  /* bit[4-7]: reserved */
    } reg;
} SOC_SCHARGER_ACR_DATA0_H_UNION;
#endif
#define SOC_SCHARGER_ACR_DATA0_H_acr_data0_h_START  (0)
#define SOC_SCHARGER_ACR_DATA0_H_acr_data0_h_END    (3)


/*****************************************************************************
 结构名    : SOC_SCHARGER_ACR_DATA1_L_UNION
 结构说明  : ACR_DATA1_L 寄存器结构定义。地址偏移量:0x37，初值:0x00，宽度:8
 寄存器说明: 第n+1个acr_pulse高电平期间第二个采样数据低8bit。
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  acr_data1_l : 8;  /* bit[0-7]: 第n+1个acr_pulse高电平期间第二个采样数据低8bit。 */
    } reg;
} SOC_SCHARGER_ACR_DATA1_L_UNION;
#endif
#define SOC_SCHARGER_ACR_DATA1_L_acr_data1_l_START  (0)
#define SOC_SCHARGER_ACR_DATA1_L_acr_data1_l_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_ACR_DATA1_H_UNION
 结构说明  : ACR_DATA1_H 寄存器结构定义。地址偏移量:0x38，初值:0x00，宽度:8
 寄存器说明: 第n+1个acr_pulse高电平期间第二个采样数据高4bit。
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  acr_data1_h : 4;  /* bit[0-3]: 第n+1个acr_pulse高电平期间第二个采样数据高4bit。 */
        unsigned char  reserved    : 4;  /* bit[4-7]: reserved */
    } reg;
} SOC_SCHARGER_ACR_DATA1_H_UNION;
#endif
#define SOC_SCHARGER_ACR_DATA1_H_acr_data1_h_START  (0)
#define SOC_SCHARGER_ACR_DATA1_H_acr_data1_h_END    (3)


/*****************************************************************************
 结构名    : SOC_SCHARGER_ACR_DATA2_L_UNION
 结构说明  : ACR_DATA2_L 寄存器结构定义。地址偏移量:0x39，初值:0x00，宽度:8
 寄存器说明: 第n+1个acr_pulse高电平期间第三个采样数据低8bit。
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  acr_data2_l : 8;  /* bit[0-7]: 第n+1个acr_pulse高电平期间第三个采样数据低8bit。 */
    } reg;
} SOC_SCHARGER_ACR_DATA2_L_UNION;
#endif
#define SOC_SCHARGER_ACR_DATA2_L_acr_data2_l_START  (0)
#define SOC_SCHARGER_ACR_DATA2_L_acr_data2_l_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_ACR_DATA2_H_UNION
 结构说明  : ACR_DATA2_H 寄存器结构定义。地址偏移量:0x3A，初值:0x00，宽度:8
 寄存器说明: 第n+1个acr_pulse高电平期间第三个采样数据高4bit。
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  acr_data2_h : 4;  /* bit[0-3]: 第n+1个acr_pulse高电平期间第三个采样数据高4bit。 */
        unsigned char  reserved    : 4;  /* bit[4-7]: reserved */
    } reg;
} SOC_SCHARGER_ACR_DATA2_H_UNION;
#endif
#define SOC_SCHARGER_ACR_DATA2_H_acr_data2_h_START  (0)
#define SOC_SCHARGER_ACR_DATA2_H_acr_data2_h_END    (3)


/*****************************************************************************
 结构名    : SOC_SCHARGER_ACR_DATA3_L_UNION
 结构说明  : ACR_DATA3_L 寄存器结构定义。地址偏移量:0x3B，初值:0x00，宽度:8
 寄存器说明: 第n+1个acr_pulse高电平期间第四个采样数据低8bit。
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  acr_data3_l : 8;  /* bit[0-7]: 第n+1个acr_pulse高电平期间第四个采样数据低8bit。 */
    } reg;
} SOC_SCHARGER_ACR_DATA3_L_UNION;
#endif
#define SOC_SCHARGER_ACR_DATA3_L_acr_data3_l_START  (0)
#define SOC_SCHARGER_ACR_DATA3_L_acr_data3_l_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_ACR_DATA3_H_UNION
 结构说明  : ACR_DATA3_H 寄存器结构定义。地址偏移量:0x3C，初值:0x00，宽度:8
 寄存器说明: 第n+1个acr_pulse高电平期间第四个采样数据高4bit。
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  acr_data3_h : 4;  /* bit[0-3]: 第n+1个acr_pulse高电平期间第四个采样数据高4bit。 */
        unsigned char  reserved    : 4;  /* bit[4-7]: reserved */
    } reg;
} SOC_SCHARGER_ACR_DATA3_H_UNION;
#endif
#define SOC_SCHARGER_ACR_DATA3_H_acr_data3_h_START  (0)
#define SOC_SCHARGER_ACR_DATA3_H_acr_data3_h_END    (3)


/*****************************************************************************
 结构名    : SOC_SCHARGER_ACR_DATA4_L_UNION
 结构说明  : ACR_DATA4_L 寄存器结构定义。地址偏移量:0x3D，初值:0x00，宽度:8
 寄存器说明: 第n+1个acr_pulse高电平期间第五个采样数据低8bit。
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  acr_data4_l : 8;  /* bit[0-7]: 第n+1个acr_pulse高电平期间第五个采样数据低8bit。 */
    } reg;
} SOC_SCHARGER_ACR_DATA4_L_UNION;
#endif
#define SOC_SCHARGER_ACR_DATA4_L_acr_data4_l_START  (0)
#define SOC_SCHARGER_ACR_DATA4_L_acr_data4_l_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_ACR_DATA4_H_UNION
 结构说明  : ACR_DATA4_H 寄存器结构定义。地址偏移量:0x3E，初值:0x00，宽度:8
 寄存器说明: 第n+1个acr_pulse高电平期间第五个采样数据高4bit。
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  acr_data4_h : 4;  /* bit[0-3]: 第n+1个acr_pulse高电平期间第五个采样数据高4bit。 */
        unsigned char  reserved    : 4;  /* bit[4-7]: reserved */
    } reg;
} SOC_SCHARGER_ACR_DATA4_H_UNION;
#endif
#define SOC_SCHARGER_ACR_DATA4_H_acr_data4_h_START  (0)
#define SOC_SCHARGER_ACR_DATA4_H_acr_data4_h_END    (3)


/*****************************************************************************
 结构名    : SOC_SCHARGER_ACR_DATA5_L_UNION
 结构说明  : ACR_DATA5_L 寄存器结构定义。地址偏移量:0x3F，初值:0x00，宽度:8
 寄存器说明: 第n+1个acr_pulse高电平期间第六个采样数据低8bit。
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  acr_data5_l : 8;  /* bit[0-7]: 第n+1个acr_pulse高电平期间第六个采样数据低8bit。 */
    } reg;
} SOC_SCHARGER_ACR_DATA5_L_UNION;
#endif
#define SOC_SCHARGER_ACR_DATA5_L_acr_data5_l_START  (0)
#define SOC_SCHARGER_ACR_DATA5_L_acr_data5_l_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_ACR_DATA5_H_UNION
 结构说明  : ACR_DATA5_H 寄存器结构定义。地址偏移量:0x40，初值:0x00，宽度:8
 寄存器说明: 第n+1个acr_pulse高电平期间第六个采样数据高4bit。
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  acr_data5_h : 4;  /* bit[0-3]: 第n+1个acr_pulse高电平期间第六个采样数据高4bit。 */
        unsigned char  reserved    : 4;  /* bit[4-7]: reserved */
    } reg;
} SOC_SCHARGER_ACR_DATA5_H_UNION;
#endif
#define SOC_SCHARGER_ACR_DATA5_H_acr_data5_h_START  (0)
#define SOC_SCHARGER_ACR_DATA5_H_acr_data5_h_END    (3)


/*****************************************************************************
 结构名    : SOC_SCHARGER_ACR_DATA6_L_UNION
 结构说明  : ACR_DATA6_L 寄存器结构定义。地址偏移量:0x41，初值:0x00，宽度:8
 寄存器说明: 第n+1个acr_pulse高电平期间第七个采样数据低8bit。
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  acr_data6_l : 8;  /* bit[0-7]: 第n+1个acr_pulse高电平期间第七个采样数据低8bit。 */
    } reg;
} SOC_SCHARGER_ACR_DATA6_L_UNION;
#endif
#define SOC_SCHARGER_ACR_DATA6_L_acr_data6_l_START  (0)
#define SOC_SCHARGER_ACR_DATA6_L_acr_data6_l_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_ACR_DATA6_H_UNION
 结构说明  : ACR_DATA6_H 寄存器结构定义。地址偏移量:0x42，初值:0x00，宽度:8
 寄存器说明: 第n+1个acr_pulse高电平期间第七个采样数据高4bit。
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  acr_data6_h : 4;  /* bit[0-3]: 第n+1个acr_pulse高电平期间第七个采样数据高4bit。 */
        unsigned char  reserved    : 4;  /* bit[4-7]: reserved */
    } reg;
} SOC_SCHARGER_ACR_DATA6_H_UNION;
#endif
#define SOC_SCHARGER_ACR_DATA6_H_acr_data6_h_START  (0)
#define SOC_SCHARGER_ACR_DATA6_H_acr_data6_h_END    (3)


/*****************************************************************************
 结构名    : SOC_SCHARGER_ACR_DATA7_L_UNION
 结构说明  : ACR_DATA7_L 寄存器结构定义。地址偏移量:0x43，初值:0x00，宽度:8
 寄存器说明: 第n+1个acr_pulse高电平期间第八个采样数据低8bit。
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  acr_data7_l : 8;  /* bit[0-7]: 第n+1个acr_pulse高电平期间第八个采样数据低8bit。 */
    } reg;
} SOC_SCHARGER_ACR_DATA7_L_UNION;
#endif
#define SOC_SCHARGER_ACR_DATA7_L_acr_data7_l_START  (0)
#define SOC_SCHARGER_ACR_DATA7_L_acr_data7_l_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_ACR_DATA7_H_UNION
 结构说明  : ACR_DATA7_H 寄存器结构定义。地址偏移量:0x44，初值:0x00，宽度:8
 寄存器说明: 第n+1个acr_pulse高电平期间第八个采样数据高4bit。
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  acr_data7_h : 4;  /* bit[0-3]: 第n+1个acr_pulse高电平期间第八个采样数据高4bit。 */
        unsigned char  reserved    : 4;  /* bit[4-7]: reserved */
    } reg;
} SOC_SCHARGER_ACR_DATA7_H_UNION;
#endif
#define SOC_SCHARGER_ACR_DATA7_H_acr_data7_h_START  (0)
#define SOC_SCHARGER_ACR_DATA7_H_acr_data7_h_END    (3)


/*****************************************************************************
 结构名    : SOC_SCHARGER_IRQ_MASK_UNION
 结构说明  : IRQ_MASK 寄存器结构定义。地址偏移量:0x48，初值:0x00，宽度:8
 寄存器说明:
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  sc_irq_mask : 8;  /* bit[0-7]: 中断屏蔽信号
                                                      bit[7]：全局屏蔽寄存器
                                                      1: 屏蔽所有中断上报
                                                      0：不屏蔽所有中断上报
                                                      bit[6]：中断屏蔽源头选择
                                                      1：中断屏蔽位屏蔽中断源头
                                                      0：中断屏蔽位不屏蔽中断源头，仅屏蔽输出
                                                      bit[5]：reserved
                                                      bit[4]：BUCK总中断屏蔽，
                                                      1：屏蔽上报中断；
                                                      0：屏蔽上报中断；
                                                      bit[3]：LVC_SC总中断屏蔽，
                                                      1：屏蔽上报中断；
                                                      0：屏蔽上报中断；
                                                      bit[2]：PD总中断屏蔽
                                                      1: 屏蔽中断上报
                                                      0：不屏蔽中断上报
                                                      bit[1]：Others总中断屏蔽信号
                                                      1: 屏蔽上报中断
                                                      0：不屏蔽上报中断
                                                      bit[0]：FCP总中断屏蔽信号
                                                      1: 屏蔽上报中断
                                                      0：不屏蔽上报中断 */
    } reg;
} SOC_SCHARGER_IRQ_MASK_UNION;
#endif
#define SOC_SCHARGER_IRQ_MASK_sc_irq_mask_START  (0)
#define SOC_SCHARGER_IRQ_MASK_sc_irq_mask_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_IRQ_MASK_0_UNION
 结构说明  : IRQ_MASK_0 寄存器结构定义。地址偏移量:0x49，初值:0x02，宽度:8
 寄存器说明:
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  sc_irq_mask_0 : 8;  /* bit[0-7]: 中断屏蔽寄存器：
                                                        1：屏蔽对应中断，屏蔽中断管脚上报，不屏蔽中断标志寄存器
                                                        0：不屏蔽对应中断，不屏蔽中断管脚和中断标志寄存器
                                                        bit[7]:irq_chg_chgstate（对应数模接口两bit信号）
                                                        bit[6]:irq_vbus_ovp
                                                        bit[5]:irq_vbus_uvp
                                                        bit[4]:irq_vbat_ovp
                                                        bit[3]:irq_otg_ovp
                                                        bit[2]:irq_otg_uvp
                                                        bit[1]:irq_buck_ocp（默认屏蔽上报中断）
                                                        bit[0]:irq_otg_scp */
    } reg;
} SOC_SCHARGER_IRQ_MASK_0_UNION;
#endif
#define SOC_SCHARGER_IRQ_MASK_0_sc_irq_mask_0_START  (0)
#define SOC_SCHARGER_IRQ_MASK_0_sc_irq_mask_0_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_IRQ_MASK_1_UNION
 结构说明  : IRQ_MASK_1 寄存器结构定义。地址偏移量:0x4A，初值:0x80，宽度:8
 寄存器说明:
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  sc_irq_mask_1 : 8;  /* bit[0-7]: 中断屏蔽寄存器：
                                                        1：屏蔽对应中断，屏蔽中断管脚上报，不屏蔽中断标志寄存器
                                                        0：不屏蔽对应中断，不屏蔽中断管脚和中断标志寄存器
                                                        bit[7]:irq_otg_ocp（默认屏蔽上报中断）
                                                        bit[6]:irq_buck_scp
                                                        bit[5]:irq_trickle_chg_fault
                                                        bit[4]:irq_pre_chg_fault
                                                        bit[3]:irq_fast_chg_fault
                                                        bit[2]:irq_otmp_jreg
                                                        bit[1]:irq_chg_rechg_state
                                                        bit[0]:irq_sleep_mod */
    } reg;
} SOC_SCHARGER_IRQ_MASK_1_UNION;
#endif
#define SOC_SCHARGER_IRQ_MASK_1_sc_irq_mask_1_START  (0)
#define SOC_SCHARGER_IRQ_MASK_1_sc_irq_mask_1_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_IRQ_MASK_2_UNION
 结构说明  : IRQ_MASK_2 寄存器结构定义。地址偏移量:0x4B，初值:0x00，宽度:8
 寄存器说明:
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  sc_irq_mask_2 : 8;  /* bit[0-7]: 中断屏蔽寄存器：
                                                        1：屏蔽对应中断，屏蔽中断管脚上报，不屏蔽中断标志寄存器
                                                        0：不屏蔽对应中断，不屏蔽中断管脚和中断标志寄存器
                                                        bit[7]:irq_reversbst
                                                        bit[6]:irq_vusb_buck_ovp
                                                        bit[5]:irq_vusb_otg_ovp

                                                        bit[4]:irq_vusb_lvc_ovp
                                                        bit[3]:irq_vbus_lvc_ov
                                                        bit[2]:irq_vbus_lvc_uv
                                                        bit[1]:irq_ibus_dc_ocp
                                                        bit[0]:irq_ibus_dc_ucp */
    } reg;
} SOC_SCHARGER_IRQ_MASK_2_UNION;
#endif
#define SOC_SCHARGER_IRQ_MASK_2_sc_irq_mask_2_START  (0)
#define SOC_SCHARGER_IRQ_MASK_2_sc_irq_mask_2_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_IRQ_MASK_3_UNION
 结构说明  : IRQ_MASK_3 寄存器结构定义。地址偏移量:0x4C，初值:0x00，宽度:8
 寄存器说明:
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  sc_irq_mask_3 : 8;  /* bit[0-7]: 中断屏蔽寄存器：
                                                        1：屏蔽对应中断，屏蔽中断管脚上报，不屏蔽中断标志寄存器
                                                        0：不屏蔽对应中断，不屏蔽中断管脚和中断标志寄存器
                                                        bit[7]:irq_ibus_dc_rcp
                                                        bit[6]:irq_vdrop_lvc_ov
                                                        bit[5]:irq_vbat_dc_ovp
                                                        bit[4]:irq_vusb_scauto_ovp
                                                        bit[3]:irq_bat_ov_clamp
                                                        bit[2]:irq_vout_ov_clamp
                                                        bit[1]:irq_ibus_clamp
                                                        bit[0]:irq_ibat_clamp */
    } reg;
} SOC_SCHARGER_IRQ_MASK_3_UNION;
#endif
#define SOC_SCHARGER_IRQ_MASK_3_sc_irq_mask_3_START  (0)
#define SOC_SCHARGER_IRQ_MASK_3_sc_irq_mask_3_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_IRQ_MASK_4_UNION
 结构说明  : IRQ_MASK_4 寄存器结构定义。地址偏移量:0x4D，初值:0x00，宽度:8
 寄存器说明:
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  sc_irq_mask_4 : 8;  /* bit[0-7]: 中断屏蔽寄存器：
                                                        1：屏蔽对应中断，屏蔽中断管脚上报，不屏蔽中断标志寄存器
                                                        0：不屏蔽对应中断，不屏蔽中断管脚和中断标志寄存器
                                                        bit[7]:irq_vbus_sc_ov
                                                        bit[6]:irq_vbus_sc_uv
                                                        bit[5]:irq_vin2vout_sc
                                                        bit[4]:irq_vbat_sc_uvp
                                                        bit[3]:irq_ibat_dc_ocp
                                                        bit[2]:irq_ibat_dcucp_alm
                                                        bit[1]:irq_ilim_sc_ocp
                                                        bit[0]:irq_ilim_bus_sc_ocp */
    } reg;
} SOC_SCHARGER_IRQ_MASK_4_UNION;
#endif
#define SOC_SCHARGER_IRQ_MASK_4_sc_irq_mask_4_START  (0)
#define SOC_SCHARGER_IRQ_MASK_4_sc_irq_mask_4_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_IRQ_MASK_5_UNION
 结构说明  : IRQ_MASK_5 寄存器结构定义。地址偏移量:0x4E，初值:0x00，宽度:8
 寄存器说明:
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  sc_irq_mask_5 : 8;  /* bit[0-7]: 中断屏蔽寄存器：
                                                        1：屏蔽对应中断，屏蔽中断管脚上报，不屏蔽中断标志寄存器
                                                        0：不屏蔽对应中断，不屏蔽中断管脚和中断标志寄存器
                                                        bit[7]:irq_cc_ov
                                                        bit[6]:irq_cc_ocp
                                                        bit[5]:irq_cc_uv
                                                        bit[4]:irq_acr_scp（由于无外置放电通路，不需此中断，模拟内部接为地，该中断实际无效）
                                                        bit[3]:soh_ovh
                                                        bit[2]:soh_ovl
                                                        bit[1]:acr_flag
                                                        bit[0]:irq_wdt */
    } reg;
} SOC_SCHARGER_IRQ_MASK_5_UNION;
#endif
#define SOC_SCHARGER_IRQ_MASK_5_sc_irq_mask_5_START  (0)
#define SOC_SCHARGER_IRQ_MASK_5_sc_irq_mask_5_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_IRQ_MASK_6_UNION
 结构说明  : IRQ_MASK_6 寄存器结构定义。地址偏移量:0x4F，初值:0x04，宽度:8
 寄存器说明:
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  sc_irq_mask_6 : 8;  /* bit[0-7]: 中断屏蔽寄存器：
                                                        1：屏蔽对应中断，屏蔽中断管脚上报，不屏蔽中断标志寄存器
                                                        0：不屏蔽对应中断，不屏蔽中断管脚和中断标志寄存器
                                                        bit[7]:irq_regn_ocp
                                                        bit[6]:irq_chg_batfet_ocp
                                                        bit[5]:irq_otmp_140
                                                        bit[4]:irq_wl_otg_usbok
                                                        bit[3]:irq_vusb_ovp_alm
                                                        bit[2]:irq_vusb_uv（默认屏蔽上报中断）
                                                        bit[1]:irq_tdie_ot_alm
                                                        bit[0]:irq_vbat_lv */
    } reg;
} SOC_SCHARGER_IRQ_MASK_6_UNION;
#endif
#define SOC_SCHARGER_IRQ_MASK_6_sc_irq_mask_6_START  (0)
#define SOC_SCHARGER_IRQ_MASK_6_sc_irq_mask_6_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_IRQ_MASK_7_UNION
 结构说明  : IRQ_MASK_7 寄存器结构定义。地址偏移量:0x50，初值:0x00，宽度:8
 寄存器说明:
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  sc_irq_mask_7 : 8;  /* bit[0-7]: 中断屏蔽寄存器：
                                                        1：屏蔽对应中断，屏蔽中断管脚上报，不屏蔽中断标志寄存器
                                                        0：不屏蔽对应中断，不屏蔽中断管脚和中断标志寄存器
                                                        bit[7]:irq_reserved[7]
                                                        bit[6]:irq_reserved[6]
                                                        bit[5]:irq_reserved[5]
                                                        bit[4]:irq_reserved[4]
                                                        bit[3]:irq_reserved[3]，irq_vout_dc_ovp vout过压保护，dc模式下使能
                                                        bit[2]:irq_reserved[2]
                                                        bit[1]:irq_reserved[1]，irq_vdrop_usb_ovp ovp管 drop保护中断；dc模式才使能；
                                                        bit[0]:irq_reserved[0]，irq_cfly_scp,fly电容短路中断；dc模式才使能 */
    } reg;
} SOC_SCHARGER_IRQ_MASK_7_UNION;
#endif
#define SOC_SCHARGER_IRQ_MASK_7_sc_irq_mask_7_START  (0)
#define SOC_SCHARGER_IRQ_MASK_7_sc_irq_mask_7_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_IRQ_STATUS_0_UNION
 结构说明  : IRQ_STATUS_0 寄存器结构定义。地址偏移量:0x51，初值:0x00，宽度:8
 寄存器说明:
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  tb_irq_status_0 : 8;  /* bit[0-7]: 中断源实时状态寄存器
                                                          bit[7]:reserved
                                                          bit[6]:irq_vbus_ovp
                                                          bit[5]:irq_vbus_uvp
                                                          bit[4]:irq_vbat_ovp
                                                          bit[3]:irq_otg_ovp
                                                          bit[2]:irq_otg_uvp
                                                          bit[1]:irq_buck_ocp
                                                          bit[0]:irq_otg_scp */
    } reg;
} SOC_SCHARGER_IRQ_STATUS_0_UNION;
#endif
#define SOC_SCHARGER_IRQ_STATUS_0_tb_irq_status_0_START  (0)
#define SOC_SCHARGER_IRQ_STATUS_0_tb_irq_status_0_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_IRQ_STATUS_1_UNION
 结构说明  : IRQ_STATUS_1 寄存器结构定义。地址偏移量:0x52，初值:0x01，宽度:8
 寄存器说明:
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  tb_irq_status_1 : 8;  /* bit[0-7]: 中断源实时状态寄存器
                                                          bit[7]:irq_otg_ocp
                                                          bit[6]:irq_buck_scp
                                                          bit[5]:irq_trickle_chg_fault
                                                          bit[4]:irq_pre_chg_fault
                                                          bit[3]:irq_fast_chg_fault
                                                          bit[2]:irq_otmp_jreg
                                                          bit[1]:irq_chg_rechg_state
                                                          bit[0]:irq_sleep_mod */
    } reg;
} SOC_SCHARGER_IRQ_STATUS_1_UNION;
#endif
#define SOC_SCHARGER_IRQ_STATUS_1_tb_irq_status_1_START  (0)
#define SOC_SCHARGER_IRQ_STATUS_1_tb_irq_status_1_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_IRQ_STATUS_2_UNION
 结构说明  : IRQ_STATUS_2 寄存器结构定义。地址偏移量:0x53，初值:0x00，宽度:8
 寄存器说明:
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  tb_irq_status_2 : 8;  /* bit[0-7]: 中断源实时状态寄存器
                                                          bit[7]:irq_reversbst
                                                          bit[6]:irq_vusb_buck_ovp
                                                          bit[5]:irq_vusb_otg_ovp

                                                          bit[4]:irq_vusb_lvc_ovp
                                                          bit[3]:irq_vbus_lvc_ov
                                                          bit[2]:irq_vbus_lvc_uv
                                                          bit[1]:irq_ibus_dc_ocp
                                                          bit[0]:irq_ibus_dc_ucp */
    } reg;
} SOC_SCHARGER_IRQ_STATUS_2_UNION;
#endif
#define SOC_SCHARGER_IRQ_STATUS_2_tb_irq_status_2_START  (0)
#define SOC_SCHARGER_IRQ_STATUS_2_tb_irq_status_2_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_IRQ_STATUS_3_UNION
 结构说明  : IRQ_STATUS_3 寄存器结构定义。地址偏移量:0x54，初值:0x00，宽度:8
 寄存器说明:
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  tb_irq_status_3 : 8;  /* bit[0-7]: 中断源实时状态寄存器
                                                          bit[7]:irq_ibus_dc_rcp
                                                          bit[6]:irq_vdrop_lvc_ov
                                                          bit[5]:irq_vbat_dc_ovp
                                                          bit[4]:irq_vusb_scauto_ovp
                                                          bit[3]:irq_bat_ov_clamp
                                                          bit[2]:irq_vout_ov_clamp
                                                          bit[1]:irq_ibus_clamp
                                                          bit[0]:irq_ibat_clamp */
    } reg;
} SOC_SCHARGER_IRQ_STATUS_3_UNION;
#endif
#define SOC_SCHARGER_IRQ_STATUS_3_tb_irq_status_3_START  (0)
#define SOC_SCHARGER_IRQ_STATUS_3_tb_irq_status_3_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_IRQ_STATUS_4_UNION
 结构说明  : IRQ_STATUS_4 寄存器结构定义。地址偏移量:0x55，初值:0x00，宽度:8
 寄存器说明:
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  tb_irq_status_4 : 8;  /* bit[0-7]: 中断源实时状态寄存器
                                                          bit[7]:irq_vbus_sc_ov
                                                          bit[6]:irq_vbus_sc_uv
                                                          bit[5]:irq_vin2vout_sc
                                                          bit[4]:irq_vbat_sc_uvp
                                                          bit[3]:irq_ibat_dc_ocp
                                                          bit[2]:irq_ibat_dcucp_alm
                                                          bit[1]:irq_ilim_sc_ocp
                                                          bit[0]:irq_ilim_bus_sc_ocp */
    } reg;
} SOC_SCHARGER_IRQ_STATUS_4_UNION;
#endif
#define SOC_SCHARGER_IRQ_STATUS_4_tb_irq_status_4_START  (0)
#define SOC_SCHARGER_IRQ_STATUS_4_tb_irq_status_4_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_IRQ_STATUS_5_UNION
 结构说明  : IRQ_STATUS_5 寄存器结构定义。地址偏移量:0x56，初值:0x00，宽度:8
 寄存器说明:
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  tb_irq_status_5 : 8;  /* bit[0-7]: 中断源实时状态寄存器
                                                          bit[7]:irq_cc_ov
                                                          bit[6]:irq_cc_ocp
                                                          bit[5]:irq_cc_uv
                                                          bit[4]:irq_acr_scp（由于无外置放电通路，不需此中断，模拟内部接为地，该中断实际无效）
                                                          bit[3]:soh_ovh
                                                          bit[2]:soh_ovl
                                                          bit[1]:acr_flag
                                                          bit[0]:irq_wdt
                                                          说明：由于看门狗发生timeout之后会将看门狗时间档位sc_watchdog_timer寄存器复位为0，即disable看门狗功能，导致看门狗上报中断信号为1的时间约为24us，导致sc_irq_flag_5[0]回读到状态1的概率非常小，查询看门狗溢出请使用看门狗timeout中断寄存器sc_irq_flag_5[0] */
    } reg;
} SOC_SCHARGER_IRQ_STATUS_5_UNION;
#endif
#define SOC_SCHARGER_IRQ_STATUS_5_tb_irq_status_5_START  (0)
#define SOC_SCHARGER_IRQ_STATUS_5_tb_irq_status_5_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_IRQ_STATUS_6_UNION
 结构说明  : IRQ_STATUS_6 寄存器结构定义。地址偏移量:0x57，初值:0x00，宽度:8
 寄存器说明:
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  tb_irq_status_6 : 8;  /* bit[0-7]: 中断源实时状态寄存器
                                                          bit[7]:irq_regn_ocp
                                                          bit[6]:irq_chg_batfet_ocp
                                                          bit[5]:irq_otmp_140
                                                          bit[4]:irq_wl_otg_usbok
                                                          bit[3]:irq_vusb_ovp_alm
                                                          bit[2]:irq_vusb_uv
                                                          bit[1]:irq_tdie_ot_alm
                                                          bit[0]:irq_vbat_lv */
    } reg;
} SOC_SCHARGER_IRQ_STATUS_6_UNION;
#endif
#define SOC_SCHARGER_IRQ_STATUS_6_tb_irq_status_6_START  (0)
#define SOC_SCHARGER_IRQ_STATUS_6_tb_irq_status_6_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_IRQ_STATUS_7_UNION
 结构说明  : IRQ_STATUS_7 寄存器结构定义。地址偏移量:0x58，初值:0x00，宽度:8
 寄存器说明:
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  tb_irq_status_7 : 8;  /* bit[0-7]: 中断源实时状态寄存器
                                                          bit[7]:irq_reserved[7]
                                                          bit[6]:irq_reserved[6]
                                                          bit[5]:irq_reserved[5]
                                                          bit[4]:irq_reserved[4]
                                                          bit[3]:irq_reserved[3]，irq_vout_dc_ovp vout过压保护，dc模式下使能
                                                          bit[2]:irq_reserved[2]
                                                          bit[1]:irq_reserved[1]，irq_vdrop_usb_ovp ovp管 drop保护中断；dc模式才使能；
                                                          bit[0]:irq_reserved[0]，irq_cfly_scp,fly电容短路中断；dc模式才使能 */
    } reg;
} SOC_SCHARGER_IRQ_STATUS_7_UNION;
#endif
#define SOC_SCHARGER_IRQ_STATUS_7_tb_irq_status_7_START  (0)
#define SOC_SCHARGER_IRQ_STATUS_7_tb_irq_status_7_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_IRQ_STATUS_8_UNION
 结构说明  : IRQ_STATUS_8 寄存器结构定义。地址偏移量:0x59，初值:0x00，宽度:8
 寄存器说明:
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  tb_irq_status_8 : 8;  /* bit[0-7]: 中断源实时状态寄存器：
                                                          bit[7:2]：reserved
                                                          bit[1:0]：irq_chg_chgstate[1:0] */
    } reg;
} SOC_SCHARGER_IRQ_STATUS_8_UNION;
#endif
#define SOC_SCHARGER_IRQ_STATUS_8_tb_irq_status_8_START  (0)
#define SOC_SCHARGER_IRQ_STATUS_8_tb_irq_status_8_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_EFUSE_SEL_UNION
 结构说明  : EFUSE_SEL 寄存器结构定义。地址偏移量:0x60，初值:0x00，宽度:8
 寄存器说明: EFUSE选择信号
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  sc_efuse_sel      : 2;  /* bit[0-1]: efuse选择信号：
                                                            2'b00：对EFUSE1进行控制（默认）；
                                                            2'b01：对EFUSE2进行控制；
                                                            2'b10：对EFUSE3进行控制；
                                                            2'b11：不选择任何EFUSE */
        unsigned char  reserved_0        : 2;  /* bit[2-3]: reserved */
        unsigned char  sc_efuse_pdob_sel : 1;  /* bit[4-4]: EFUSE数据寄存器和相应的预修调寄存器选择信号：
                                                            1’b0：输出修调信号（默认）；
                                                            1’b1：输出寄存器预修调信号 */
        unsigned char  reserved_1        : 3;  /* bit[5-7]: reserved */
    } reg;
} SOC_SCHARGER_EFUSE_SEL_UNION;
#endif
#define SOC_SCHARGER_EFUSE_SEL_sc_efuse_sel_START       (0)
#define SOC_SCHARGER_EFUSE_SEL_sc_efuse_sel_END         (1)
#define SOC_SCHARGER_EFUSE_SEL_sc_efuse_pdob_sel_START  (4)
#define SOC_SCHARGER_EFUSE_SEL_sc_efuse_pdob_sel_END    (4)


/*****************************************************************************
 结构名    : SOC_SCHARGER_EFUSE_CFG_0_UNION
 结构说明  : EFUSE_CFG_0 寄存器结构定义。地址偏移量:0x61，初值:0x00，宽度:8
 寄存器说明:
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  sc_efuse_nr_cfg      : 1;  /* bit[0-0]: EFUSE模块软件配置NR信号（对应EFUSE由sc_efuse_sel选择） */
        unsigned char  sc_efuse_pgenb_cfg   : 1;  /* bit[1-1]: EFUSE模块软件配置PGENB信号（对应EFUSE由sc_efuse_sel选择） */
        unsigned char  sc_efuse_strobe_cfg  : 1;  /* bit[2-2]: EFUSE模块软件配置STROBE信号（对应EFUSE由sc_efuse_sel选择） */
        unsigned char  sc_efuse_rd_ctrl     : 1;  /* bit[3-3]: EFUSE模块软件读控制信号（对应EFUSE由sc_efuse_sel选择） */
        unsigned char  sc_efuse_prog_int    : 1;  /* bit[4-4]: EFUSE模块软件编程和读模式选择信号（对应EFUSE由sc_efuse_sel选择） */
        unsigned char  sc_efuse_prog_sel    : 1;  /* bit[5-5]: EFUSE模块编程选择信号（对应EFUSE由sc_efuse_sel选择） */
        unsigned char  sc_efuse_inctrl_sel  : 1;  /* bit[6-6]: EFUSE模块EFUSE调试数字自动模式和数字手动模式选择信号（对应EFUSE由sc_efuse_sel选择） */
        unsigned char  sc_efuse_rd_mode_sel : 1;  /* bit[7-7]: EFUSE模块读模式选择信号（对应EFUSE由sc_efuse_sel选择）：
                                                               1：统一64比特刷新；
                                                               0：每一比特刷新；
                                                               默认：0 */
    } reg;
} SOC_SCHARGER_EFUSE_CFG_0_UNION;
#endif
#define SOC_SCHARGER_EFUSE_CFG_0_sc_efuse_nr_cfg_START       (0)
#define SOC_SCHARGER_EFUSE_CFG_0_sc_efuse_nr_cfg_END         (0)
#define SOC_SCHARGER_EFUSE_CFG_0_sc_efuse_pgenb_cfg_START    (1)
#define SOC_SCHARGER_EFUSE_CFG_0_sc_efuse_pgenb_cfg_END      (1)
#define SOC_SCHARGER_EFUSE_CFG_0_sc_efuse_strobe_cfg_START   (2)
#define SOC_SCHARGER_EFUSE_CFG_0_sc_efuse_strobe_cfg_END     (2)
#define SOC_SCHARGER_EFUSE_CFG_0_sc_efuse_rd_ctrl_START      (3)
#define SOC_SCHARGER_EFUSE_CFG_0_sc_efuse_rd_ctrl_END        (3)
#define SOC_SCHARGER_EFUSE_CFG_0_sc_efuse_prog_int_START     (4)
#define SOC_SCHARGER_EFUSE_CFG_0_sc_efuse_prog_int_END       (4)
#define SOC_SCHARGER_EFUSE_CFG_0_sc_efuse_prog_sel_START     (5)
#define SOC_SCHARGER_EFUSE_CFG_0_sc_efuse_prog_sel_END       (5)
#define SOC_SCHARGER_EFUSE_CFG_0_sc_efuse_inctrl_sel_START   (6)
#define SOC_SCHARGER_EFUSE_CFG_0_sc_efuse_inctrl_sel_END     (6)
#define SOC_SCHARGER_EFUSE_CFG_0_sc_efuse_rd_mode_sel_START  (7)
#define SOC_SCHARGER_EFUSE_CFG_0_sc_efuse_rd_mode_sel_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_EFUSE_WE_0_UNION
 结构说明  : EFUSE_WE_0 寄存器结构定义。地址偏移量:0x62，初值:0x00，宽度:8
 寄存器说明:
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  sc_efuse_we0 : 8;  /* bit[0-7]: EFUSE模块配置信号WE0（对应EFUSE由sc_efuse_sel选择） */
    } reg;
} SOC_SCHARGER_EFUSE_WE_0_UNION;
#endif
#define SOC_SCHARGER_EFUSE_WE_0_sc_efuse_we0_START  (0)
#define SOC_SCHARGER_EFUSE_WE_0_sc_efuse_we0_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_EFUSE_WE_1_UNION
 结构说明  : EFUSE_WE_1 寄存器结构定义。地址偏移量:0x63，初值:0x00，宽度:8
 寄存器说明:
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  sc_efuse_we1 : 8;  /* bit[0-7]: EFUSE模块配置信号WE1（对应EFUSE由sc_efuse_sel选择） */
    } reg;
} SOC_SCHARGER_EFUSE_WE_1_UNION;
#endif
#define SOC_SCHARGER_EFUSE_WE_1_sc_efuse_we1_START  (0)
#define SOC_SCHARGER_EFUSE_WE_1_sc_efuse_we1_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_EFUSE_WE_2_UNION
 结构说明  : EFUSE_WE_2 寄存器结构定义。地址偏移量:0x64，初值:0x00，宽度:8
 寄存器说明:
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  sc_efuse_we2 : 8;  /* bit[0-7]: EFUSE模块配置信号WE2（对应EFUSE由sc_efuse_sel选择） */
    } reg;
} SOC_SCHARGER_EFUSE_WE_2_UNION;
#endif
#define SOC_SCHARGER_EFUSE_WE_2_sc_efuse_we2_START  (0)
#define SOC_SCHARGER_EFUSE_WE_2_sc_efuse_we2_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_EFUSE_WE_3_UNION
 结构说明  : EFUSE_WE_3 寄存器结构定义。地址偏移量:0x65，初值:0x00，宽度:8
 寄存器说明:
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  sc_efuse_we3 : 8;  /* bit[0-7]: EFUSE模块配置信号WE3（对应EFUSE由sc_efuse_sel选择） */
    } reg;
} SOC_SCHARGER_EFUSE_WE_3_UNION;
#endif
#define SOC_SCHARGER_EFUSE_WE_3_sc_efuse_we3_START  (0)
#define SOC_SCHARGER_EFUSE_WE_3_sc_efuse_we3_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_EFUSE_WE_4_UNION
 结构说明  : EFUSE_WE_4 寄存器结构定义。地址偏移量:0x66，初值:0x00，宽度:8
 寄存器说明:
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  sc_efuse_we4 : 8;  /* bit[0-7]: EFUSE模块配置信号WE4（对应EFUSE由sc_efuse_sel选择） */
    } reg;
} SOC_SCHARGER_EFUSE_WE_4_UNION;
#endif
#define SOC_SCHARGER_EFUSE_WE_4_sc_efuse_we4_START  (0)
#define SOC_SCHARGER_EFUSE_WE_4_sc_efuse_we4_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_EFUSE_WE_5_UNION
 结构说明  : EFUSE_WE_5 寄存器结构定义。地址偏移量:0x67，初值:0x00，宽度:8
 寄存器说明:
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  sc_efuse_we5 : 8;  /* bit[0-7]: EFUSE模块配置信号WE5（对应EFUSE由sc_efuse_sel选择） */
    } reg;
} SOC_SCHARGER_EFUSE_WE_5_UNION;
#endif
#define SOC_SCHARGER_EFUSE_WE_5_sc_efuse_we5_START  (0)
#define SOC_SCHARGER_EFUSE_WE_5_sc_efuse_we5_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_EFUSE_WE_6_UNION
 结构说明  : EFUSE_WE_6 寄存器结构定义。地址偏移量:0x68，初值:0x00，宽度:8
 寄存器说明:
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  sc_efuse_we6 : 8;  /* bit[0-7]: EFUSE模块配置信号WE6（对应EFUSE由sc_efuse_sel选择） */
    } reg;
} SOC_SCHARGER_EFUSE_WE_6_UNION;
#endif
#define SOC_SCHARGER_EFUSE_WE_6_sc_efuse_we6_START  (0)
#define SOC_SCHARGER_EFUSE_WE_6_sc_efuse_we6_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_EFUSE_WE_7_UNION
 结构说明  : EFUSE_WE_7 寄存器结构定义。地址偏移量:0x69，初值:0x00，宽度:8
 寄存器说明:
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  sc_efuse_we7 : 8;  /* bit[0-7]: EFUSE模块配置信号WE7（对应EFUSE由sc_efuse_sel选择） */
    } reg;
} SOC_SCHARGER_EFUSE_WE_7_UNION;
#endif
#define SOC_SCHARGER_EFUSE_WE_7_sc_efuse_we7_START  (0)
#define SOC_SCHARGER_EFUSE_WE_7_sc_efuse_we7_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_EFUSE_PDOB_PRE_ADDR_WE_UNION
 结构说明  : EFUSE_PDOB_PRE_ADDR_WE 寄存器结构定义。地址偏移量:0x6A，初值:0x00，宽度:8
 寄存器说明:
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  sc_efuse_pdob_pre_addr : 3;  /* bit[0-2]: EFUSE模块预调寄存器地址选择:
                                                                 3'b000：选择预调寄存器0
                                                                 3'b001：选择预调寄存器1
                                                                 …
                                                                 3'b111：选择预调寄存器7
                                                                 默认：0 */
        unsigned char  reserved_0             : 1;  /* bit[3-3]: reserved */
        unsigned char  sc_efuse_pdob_pre_we   : 1;  /* bit[4-4]: EFUSE模块预调寄存器写使能，高电平有效，再次写入需先配置为0
                                                                 默认：0 */
        unsigned char  reserved_1             : 3;  /* bit[5-7]: reserved */
    } reg;
} SOC_SCHARGER_EFUSE_PDOB_PRE_ADDR_WE_UNION;
#endif
#define SOC_SCHARGER_EFUSE_PDOB_PRE_ADDR_WE_sc_efuse_pdob_pre_addr_START  (0)
#define SOC_SCHARGER_EFUSE_PDOB_PRE_ADDR_WE_sc_efuse_pdob_pre_addr_END    (2)
#define SOC_SCHARGER_EFUSE_PDOB_PRE_ADDR_WE_sc_efuse_pdob_pre_we_START    (4)
#define SOC_SCHARGER_EFUSE_PDOB_PRE_ADDR_WE_sc_efuse_pdob_pre_we_END      (4)


/*****************************************************************************
 结构名    : SOC_SCHARGER_EFUSE_PDOB_PRE_WDATA_UNION
 结构说明  : EFUSE_PDOB_PRE_WDATA 寄存器结构定义。地址偏移量:0x6B，初值:0x00，宽度:8
 寄存器说明:
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  sc_efuse_pdob_pre_wdata : 8;  /* bit[0-7]: EFUSE模块预调寄存器写入值
                                                                  默认：0 */
    } reg;
} SOC_SCHARGER_EFUSE_PDOB_PRE_WDATA_UNION;
#endif
#define SOC_SCHARGER_EFUSE_PDOB_PRE_WDATA_sc_efuse_pdob_pre_wdata_START  (0)
#define SOC_SCHARGER_EFUSE_PDOB_PRE_WDATA_sc_efuse_pdob_pre_wdata_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_EFUSE1_TESTBUS_0_UNION
 结构说明  : EFUSE1_TESTBUS_0 寄存器结构定义。地址偏移量:0x6C，初值:0x10，宽度:8
 寄存器说明:
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  tb_efuse1_state      : 4;  /* bit[0-3]: EFUSE1模块内部状态机回读信号（debug使用，不对产品开放）：
                                                               4'h0:STANDBY
                                                               4'h1:RD_READY
                                                               4'h2:RD_START
                                                               4'h3:RD_BIT
                                                               4'h4:RD_BIT_DONE
                                                               4'h5:RD_INT
                                                               4'h6:WR_READY
                                                               4'h7:WR_START
                                                               4'h8:WR_BIT
                                                               4'h9:WR_BIT_DONE
                                                               4'ha:WR_INT
                                                               默认：4'h0 */
        unsigned char  tb_efuse1_por_int_ro : 1;  /* bit[4-4]: 数模接口信号efuse_por_int回读（debug使用，不对产品开放） */
        unsigned char  reserved             : 3;  /* bit[5-7]: reserved */
    } reg;
} SOC_SCHARGER_EFUSE1_TESTBUS_0_UNION;
#endif
#define SOC_SCHARGER_EFUSE1_TESTBUS_0_tb_efuse1_state_START       (0)
#define SOC_SCHARGER_EFUSE1_TESTBUS_0_tb_efuse1_state_END         (3)
#define SOC_SCHARGER_EFUSE1_TESTBUS_0_tb_efuse1_por_int_ro_START  (4)
#define SOC_SCHARGER_EFUSE1_TESTBUS_0_tb_efuse1_por_int_ro_END    (4)


/*****************************************************************************
 结构名    : SOC_SCHARGER_EFUSE2_TESTBUS_0_UNION
 结构说明  : EFUSE2_TESTBUS_0 寄存器结构定义。地址偏移量:0x6D，初值:0x10，宽度:8
 寄存器说明:
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  tb_efuse2_state      : 4;  /* bit[0-3]: EFUSE2模块内部状态机回读信号（debug使用，不对产品开放）：
                                                               4'h0:STANDBY
                                                               4'h1:RD_READY
                                                               4'h2:RD_START
                                                               4'h3:RD_BIT
                                                               4'h4:RD_BIT_DONE
                                                               4'h5:RD_INT
                                                               4'h6:WR_READY
                                                               4'h7:WR_START
                                                               4'h8:WR_BIT
                                                               4'h9:WR_BIT_DONE
                                                               4'ha:WR_INT
                                                               默认：4'h0 */
        unsigned char  tb_efuse2_por_int_ro : 1;  /* bit[4-4]: 数模接口信号efuse_por_int回读（debug使用，不对产品开放） */
        unsigned char  reserved             : 3;  /* bit[5-7]: reserved */
    } reg;
} SOC_SCHARGER_EFUSE2_TESTBUS_0_UNION;
#endif
#define SOC_SCHARGER_EFUSE2_TESTBUS_0_tb_efuse2_state_START       (0)
#define SOC_SCHARGER_EFUSE2_TESTBUS_0_tb_efuse2_state_END         (3)
#define SOC_SCHARGER_EFUSE2_TESTBUS_0_tb_efuse2_por_int_ro_START  (4)
#define SOC_SCHARGER_EFUSE2_TESTBUS_0_tb_efuse2_por_int_ro_END    (4)


/*****************************************************************************
 结构名    : SOC_SCHARGER_EFUSE3_TESTBUS_0_UNION
 结构说明  : EFUSE3_TESTBUS_0 寄存器结构定义。地址偏移量:0x6E，初值:0x10，宽度:8
 寄存器说明:
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  tb_efuse3_state      : 4;  /* bit[0-3]: EFUSE3模块内部状态机回读信号（debug使用，不对产品开放）：
                                                               4'h0:STANDBY
                                                               4'h1:RD_READY
                                                               4'h2:RD_START
                                                               4'h3:RD_BIT
                                                               4'h4:RD_BIT_DONE
                                                               4'h5:RD_INT
                                                               4'h6:WR_READY
                                                               4'h7:WR_START
                                                               4'h8:WR_BIT
                                                               4'h9:WR_BIT_DONE
                                                               4'ha:WR_INT
                                                               默认：4'h0 */
        unsigned char  tb_efuse3_por_int_ro : 1;  /* bit[4-4]: 数模接口信号efuse_por_int回读（debug使用，不对产品开放） */
        unsigned char  reserved             : 3;  /* bit[5-7]: reserved */
    } reg;
} SOC_SCHARGER_EFUSE3_TESTBUS_0_UNION;
#endif
#define SOC_SCHARGER_EFUSE3_TESTBUS_0_tb_efuse3_state_START       (0)
#define SOC_SCHARGER_EFUSE3_TESTBUS_0_tb_efuse3_state_END         (3)
#define SOC_SCHARGER_EFUSE3_TESTBUS_0_tb_efuse3_por_int_ro_START  (4)
#define SOC_SCHARGER_EFUSE3_TESTBUS_0_tb_efuse3_por_int_ro_END    (4)


/*****************************************************************************
 结构名    : SOC_SCHARGER_EFUSE_TESTBUS_SEL_UNION
 结构说明  : EFUSE_TESTBUS_SEL 寄存器结构定义。地址偏移量:0x6F，初值:0x00，宽度:8
 寄存器说明:
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  sc_efuse2_testbus_sel : 4;  /* bit[0-3]: EFUSE2模块测试信号选择（debug使用，不对产品开放）
                                                                默认：0 */
        unsigned char  sc_efuse3_testbus_sel : 4;  /* bit[4-7]: EFUSE3模块测试信号选择（debug使用，不对产品开放）
                                                                默认：0 */
    } reg;
} SOC_SCHARGER_EFUSE_TESTBUS_SEL_UNION;
#endif
#define SOC_SCHARGER_EFUSE_TESTBUS_SEL_sc_efuse2_testbus_sel_START  (0)
#define SOC_SCHARGER_EFUSE_TESTBUS_SEL_sc_efuse2_testbus_sel_END    (3)
#define SOC_SCHARGER_EFUSE_TESTBUS_SEL_sc_efuse3_testbus_sel_START  (4)
#define SOC_SCHARGER_EFUSE_TESTBUS_SEL_sc_efuse3_testbus_sel_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_EFUSE_TESTBUS_CFG_UNION
 结构说明  : EFUSE_TESTBUS_CFG 寄存器结构定义。地址偏移量:0x70，初值:0x00，宽度:8
 寄存器说明:
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  sc_efuse1_state_reset : 1;  /* bit[0-0]: EFUSE1模块状态机复位信号，电平有效（debug使用，不对产品开放）
                                                                1：复位状态机；
                                                                0：不复位状态机；
                                                                默认：0 */
        unsigned char  sc_efuse2_state_reset : 1;  /* bit[1-1]: EFUSE2模块状态机复位信号，电平有效（debug使用，不对产品开放）
                                                                1：复位状态机；
                                                                0：不复位状态机；
                                                                默认：0 */
        unsigned char  sc_efuse3_state_reset : 1;  /* bit[2-2]: EFUSE3模块状态机复位信号，电平有效（debug使用，不对产品开放）
                                                                1：复位状态机；
                                                                0：不复位状态机；
                                                                默认：0 */
        unsigned char  sc_efuse_testbus_en   : 1;  /* bit[3-3]: EFUSE1/2/3模块测试信号输出使能（debug使用，不对产品开放）：
                                                                1：输出测试信号；
                                                                0：不输出测试信号；
                                                                默认：0 */
        unsigned char  sc_efuse1_testbus_sel : 4;  /* bit[4-7]: EFUSE1模块测试信号选择（debug使用，不对产品开放）
                                                                默认：0 */
    } reg;
} SOC_SCHARGER_EFUSE_TESTBUS_CFG_UNION;
#endif
#define SOC_SCHARGER_EFUSE_TESTBUS_CFG_sc_efuse1_state_reset_START  (0)
#define SOC_SCHARGER_EFUSE_TESTBUS_CFG_sc_efuse1_state_reset_END    (0)
#define SOC_SCHARGER_EFUSE_TESTBUS_CFG_sc_efuse2_state_reset_START  (1)
#define SOC_SCHARGER_EFUSE_TESTBUS_CFG_sc_efuse2_state_reset_END    (1)
#define SOC_SCHARGER_EFUSE_TESTBUS_CFG_sc_efuse3_state_reset_START  (2)
#define SOC_SCHARGER_EFUSE_TESTBUS_CFG_sc_efuse3_state_reset_END    (2)
#define SOC_SCHARGER_EFUSE_TESTBUS_CFG_sc_efuse_testbus_en_START    (3)
#define SOC_SCHARGER_EFUSE_TESTBUS_CFG_sc_efuse_testbus_en_END      (3)
#define SOC_SCHARGER_EFUSE_TESTBUS_CFG_sc_efuse1_testbus_sel_START  (4)
#define SOC_SCHARGER_EFUSE_TESTBUS_CFG_sc_efuse1_testbus_sel_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_EFUSE1_TESTBUS_RDATA_UNION
 结构说明  : EFUSE1_TESTBUS_RDATA 寄存器结构定义。地址偏移量:0x71，初值:0x00，宽度:8
 寄存器说明:
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  efuse1_testbus_rdata : 8;  /* bit[0-7]: EFUSE1模块测试回读数据（debug使用，不对产品开放） */
    } reg;
} SOC_SCHARGER_EFUSE1_TESTBUS_RDATA_UNION;
#endif
#define SOC_SCHARGER_EFUSE1_TESTBUS_RDATA_efuse1_testbus_rdata_START  (0)
#define SOC_SCHARGER_EFUSE1_TESTBUS_RDATA_efuse1_testbus_rdata_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_EFUSE2_TESTBUS_RDATA_UNION
 结构说明  : EFUSE2_TESTBUS_RDATA 寄存器结构定义。地址偏移量:0x72，初值:0x00，宽度:8
 寄存器说明:
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  efuse2_testbus_rdata : 8;  /* bit[0-7]: EFUSE2模块测试回读数据（debug使用，不对产品开放） */
    } reg;
} SOC_SCHARGER_EFUSE2_TESTBUS_RDATA_UNION;
#endif
#define SOC_SCHARGER_EFUSE2_TESTBUS_RDATA_efuse2_testbus_rdata_START  (0)
#define SOC_SCHARGER_EFUSE2_TESTBUS_RDATA_efuse2_testbus_rdata_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_EFUSE3_TESTBUS_RDATA_UNION
 结构说明  : EFUSE3_TESTBUS_RDATA 寄存器结构定义。地址偏移量:0x73，初值:0x00，宽度:8
 寄存器说明:
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  efuse3_testbus_rdata : 8;  /* bit[0-7]: EFUSE3模块测试回读数据（debug使用，不对产品开放） */
    } reg;
} SOC_SCHARGER_EFUSE3_TESTBUS_RDATA_UNION;
#endif
#define SOC_SCHARGER_EFUSE3_TESTBUS_RDATA_efuse3_testbus_rdata_START  (0)
#define SOC_SCHARGER_EFUSE3_TESTBUS_RDATA_efuse3_testbus_rdata_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_GLB_TESTBUS_CFG_UNION
 结构说明  : GLB_TESTBUS_CFG 寄存器结构定义。地址偏移量:0x74，初值:0x00，宽度:8
 寄存器说明: 公共模块测试信号配置寄存器
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  sc_glb_tb_sel : 4;  /* bit[0-3]: 公共模块测试信号选择信号（debug使用，不对产品开放） */
        unsigned char  reserved_0    : 1;  /* bit[4-4]: reserved */
        unsigned char  sc_glb_tb_en  : 1;  /* bit[5-5]: 规格模块测试信号使能信号（debug使用，不对产品开放） */
        unsigned char  reserved_1    : 2;  /* bit[6-7]: reserved */
    } reg;
} SOC_SCHARGER_GLB_TESTBUS_CFG_UNION;
#endif
#define SOC_SCHARGER_GLB_TESTBUS_CFG_sc_glb_tb_sel_START  (0)
#define SOC_SCHARGER_GLB_TESTBUS_CFG_sc_glb_tb_sel_END    (3)
#define SOC_SCHARGER_GLB_TESTBUS_CFG_sc_glb_tb_en_START   (5)
#define SOC_SCHARGER_GLB_TESTBUS_CFG_sc_glb_tb_en_END     (5)


/*****************************************************************************
 结构名    : SOC_SCHARGER_GLB_TEST_DATA_UNION
 结构说明  : GLB_TEST_DATA 寄存器结构定义。地址偏移量:0x75，初值:0x00，宽度:8
 寄存器说明: 公共模块测试信号
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  tb_glb_data : 8;  /* bit[0-7]: 公共模块测试信号（debug使用，不对产品开放） */
    } reg;
} SOC_SCHARGER_GLB_TEST_DATA_UNION;
#endif
#define SOC_SCHARGER_GLB_TEST_DATA_tb_glb_data_START  (0)
#define SOC_SCHARGER_GLB_TEST_DATA_tb_glb_data_END    (7)




/****************************************************************************
                     (4/4) REG_PAGE2
 ****************************************************************************/
/*****************************************************************************
 结构名    : SOC_SCHARGER_CHARGER_CFG_REG_0_UNION
 结构说明  : CHARGER_CFG_REG_0 寄存器结构定义。地址偏移量:0x00，初值:0x09，宽度:8
 寄存器说明: CHARGER_配置寄存器_0
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  da_chg_cap2_sel    : 2;  /* bit[0-1]: charger环路补偿电容调节
                                                             00:2.2pF
                                                             01:4.4pF
                                                             10:6.6pF
                                                             11:8.8pF */
        unsigned char  da_chg_cap1_sel    : 2;  /* bit[2-3]: charger环路补偿电容调节
                                                             00:1.1pF
                                                             01:2.2pF
                                                             10:3.3pF
                                                             11:4.4pF */
        unsigned char  da_chg_bat_open    : 1;  /* bit[4-4]: 电池不在位指示
                                                             0：电池在位
                                                             1：电池不在位 */
        unsigned char  da_chg_ate_mode    : 1;  /* bit[5-5]: ATE测试模式设置位：
                                                             0：不在ATE测试模式
                                                             1：设置为ATE测试模式 */
        unsigned char  da_chg_acl_rpt_en  : 1;  /* bit[6-6]: ACL状态上报使能位：
                                                             0：不使能ACL上报
                                                             1：使能ACL上报 */
        unsigned char  da_chg_2x_ibias_en : 1;  /* bit[7-7]: rev_ok比较器偏置电流调整位：
                                                             0：default偏置电流
                                                             1：偏置电流x2 */
    } reg;
} SOC_SCHARGER_CHARGER_CFG_REG_0_UNION;
#endif
#define SOC_SCHARGER_CHARGER_CFG_REG_0_da_chg_cap2_sel_START     (0)
#define SOC_SCHARGER_CHARGER_CFG_REG_0_da_chg_cap2_sel_END       (1)
#define SOC_SCHARGER_CHARGER_CFG_REG_0_da_chg_cap1_sel_START     (2)
#define SOC_SCHARGER_CHARGER_CFG_REG_0_da_chg_cap1_sel_END       (3)
#define SOC_SCHARGER_CHARGER_CFG_REG_0_da_chg_bat_open_START     (4)
#define SOC_SCHARGER_CHARGER_CFG_REG_0_da_chg_bat_open_END       (4)
#define SOC_SCHARGER_CHARGER_CFG_REG_0_da_chg_ate_mode_START     (5)
#define SOC_SCHARGER_CHARGER_CFG_REG_0_da_chg_ate_mode_END       (5)
#define SOC_SCHARGER_CHARGER_CFG_REG_0_da_chg_acl_rpt_en_START   (6)
#define SOC_SCHARGER_CHARGER_CFG_REG_0_da_chg_acl_rpt_en_END     (6)
#define SOC_SCHARGER_CHARGER_CFG_REG_0_da_chg_2x_ibias_en_START  (7)
#define SOC_SCHARGER_CHARGER_CFG_REG_0_da_chg_2x_ibias_en_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_CHARGER_CFG_REG_1_UNION
 结构说明  : CHARGER_CFG_REG_1 寄存器结构定义。地址偏移量:0x01，初值:0x96，宽度:8
 寄存器说明: CHARGER_配置寄存器_1
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  da_chg_eoc_del2m_en : 1;  /* bit[0-0]: EOC 2ms去抖档位使能位：
                                                              0：不使能2ms去抖
                                                              1：使能2ms去抖 */
        unsigned char  da_chg_en_term      : 1;  /* bit[1-1]: 终止充电控制位
                                                              0: Disabled
                                                              1: Enabled */
        unsigned char  da_chg_cp_src_sel   : 1;  /* bit[2-2]: cp时钟电源选择位：
                                                              0：cp时钟电源强制选择VBAT
                                                              1：cp时钟电源不强制选择VBAT */
        unsigned char  da_chg_clk_div2_shd : 1;  /* bit[3-3]: 快充安全计时器在热调整、输入限流和DPM过程中放缓2倍计数功能屏蔽位
                                                              0: 2X功能不屏蔽
                                                              1: 2X功能屏蔽 */
        unsigned char  da_chg_cap4_sel     : 2;  /* bit[4-5]: charger环路补偿电容调节
                                                              00:3.6pF
                                                              01:7.2pF
                                                              10:10.8pF
                                                              11:14.4pF */
        unsigned char  da_chg_cap3_sel     : 2;  /* bit[6-7]: charger环路补偿电容调节
                                                              00:2.3pF
                                                              01:4.6pF
                                                              10:6.9pF
                                                              11:9.2pF */
    } reg;
} SOC_SCHARGER_CHARGER_CFG_REG_1_UNION;
#endif
#define SOC_SCHARGER_CHARGER_CFG_REG_1_da_chg_eoc_del2m_en_START  (0)
#define SOC_SCHARGER_CHARGER_CFG_REG_1_da_chg_eoc_del2m_en_END    (0)
#define SOC_SCHARGER_CHARGER_CFG_REG_1_da_chg_en_term_START       (1)
#define SOC_SCHARGER_CHARGER_CFG_REG_1_da_chg_en_term_END         (1)
#define SOC_SCHARGER_CHARGER_CFG_REG_1_da_chg_cp_src_sel_START    (2)
#define SOC_SCHARGER_CHARGER_CFG_REG_1_da_chg_cp_src_sel_END      (2)
#define SOC_SCHARGER_CHARGER_CFG_REG_1_da_chg_clk_div2_shd_START  (3)
#define SOC_SCHARGER_CHARGER_CFG_REG_1_da_chg_clk_div2_shd_END    (3)
#define SOC_SCHARGER_CHARGER_CFG_REG_1_da_chg_cap4_sel_START      (4)
#define SOC_SCHARGER_CHARGER_CFG_REG_1_da_chg_cap4_sel_END        (5)
#define SOC_SCHARGER_CHARGER_CFG_REG_1_da_chg_cap3_sel_START      (6)
#define SOC_SCHARGER_CHARGER_CFG_REG_1_da_chg_cap3_sel_END        (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_CHARGER_CFG_REG_2_UNION
 结构说明  : CHARGER_CFG_REG_2 寄存器结构定义。地址偏移量:0x02，初值:0x13，宽度:8
 寄存器说明: CHARGER_配置寄存器_2
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  da_chg_fast_ichg : 5;  /* bit[0-4]: 快充电电流大小调节位
                                                           00000: 100mA
                                                           00001: 200mA
                                                           00010: 300mA
                                                           00011: 400mA
                                                           00100: 500mA
                                                           00101: 600mA
                                                           00110: 700mA
                                                           00111: 800mA
                                                           01000: 900mA
                                                           01001: 1000mA
                                                           01010: 1100mA
                                                           01011: 1200mA
                                                           01100: 1300mA
                                                           01101: 1400mA
                                                           01110: 1500mA
                                                           01111: 1600mA
                                                           10000: 1700mA
                                                           10001: 1800mA
                                                           10010: 1900mA
                                                           10011: 2000mA
                                                           10100: 2100mA
                                                           10101: 2200mA
                                                           10110: 2300mA
                                                           10111: 2400mA
                                                           11000: 2500mA
                                                           11001: 2600mA
                                                           11010: 2700mA
                                                           11011: 2800mA
                                                           11100: 2900mA
                                                           11101: 3000mA
                                                           11110: 3100mA
                                                           11111: 3200mA */
        unsigned char  reserved         : 3;  /* bit[5-7]: reserved */
    } reg;
} SOC_SCHARGER_CHARGER_CFG_REG_2_UNION;
#endif
#define SOC_SCHARGER_CHARGER_CFG_REG_2_da_chg_fast_ichg_START  (0)
#define SOC_SCHARGER_CHARGER_CFG_REG_2_da_chg_fast_ichg_END    (4)


/*****************************************************************************
 结构名    : SOC_SCHARGER_CHARGER_CFG_REG_3_UNION
 结构说明  : CHARGER_CFG_REG_3 寄存器结构定义。地址偏移量:0x03，初值:0x60，宽度:8
 寄存器说明: CHARGER_配置寄存器_3
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  da_chg_fastchg_timer : 2;  /* bit[0-1]: 快充安全计时器（小时）
                                                               00: 5 h
                                                               01: 8 h
                                                               10: 12 h
                                                               11: 20 h */
        unsigned char  da_chg_fast_vchg     : 5;  /* bit[2-6]: 快充电完成电压阈值大小调节位
                                                               00000: 4V
                                                                。
                                                                。
                                                                。
                                                               11000: 4.4V
                                                                。
                                                                。
                                                                。
                                                               11111: 4.516V */
        unsigned char  reserved             : 1;  /* bit[7-7]: reserved */
    } reg;
} SOC_SCHARGER_CHARGER_CFG_REG_3_UNION;
#endif
#define SOC_SCHARGER_CHARGER_CFG_REG_3_da_chg_fastchg_timer_START  (0)
#define SOC_SCHARGER_CHARGER_CFG_REG_3_da_chg_fastchg_timer_END    (1)
#define SOC_SCHARGER_CHARGER_CFG_REG_3_da_chg_fast_vchg_START      (2)
#define SOC_SCHARGER_CHARGER_CFG_REG_3_da_chg_fast_vchg_END        (6)


/*****************************************************************************
 结构名    : SOC_SCHARGER_CHARGER_CFG_REG_4_UNION
 结构说明  : CHARGER_CFG_REG_4 寄存器结构定义。地址偏移量:0x04，初值:0x0C，宽度:8
 寄存器说明: CHARGER_配置寄存器_4
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  da_chg_ocp_adj    : 2;  /* bit[0-1]: batfet ocp档位设置：
                                                            00：1x
                                                            01：-20%
                                                            10：+20%
                                                            11：+30% */
        unsigned char  da_chg_md_sel     : 1;  /* bit[2-2]: VBAT>3.65V，充电档位小于400mA，IR/GAP模式设置位：
                                                            0：设置为GAP模式
                                                            1：设置为IR模式 */
        unsigned char  da_chg_iref_clamp : 2;  /* bit[3-4]: 充电电流钳位
                                                            00：100mV
                                                            01: 150mV
                                                            10：200mV
                                                            11：250mV */
        unsigned char  da_chg_ir_set     : 3;  /* bit[5-7]: 电池通道电阻补偿
                                                            000: 0mohm
                                                            001: 15mohm
                                                            010: 30mohm
                                                            011: 45mohm
                                                            100: 60mohm
                                                            101: 75mohm
                                                            110: 90mohm
                                                            111: 105mohm */
    } reg;
} SOC_SCHARGER_CHARGER_CFG_REG_4_UNION;
#endif
#define SOC_SCHARGER_CHARGER_CFG_REG_4_da_chg_ocp_adj_START     (0)
#define SOC_SCHARGER_CHARGER_CFG_REG_4_da_chg_ocp_adj_END       (1)
#define SOC_SCHARGER_CHARGER_CFG_REG_4_da_chg_md_sel_START      (2)
#define SOC_SCHARGER_CHARGER_CFG_REG_4_da_chg_md_sel_END        (2)
#define SOC_SCHARGER_CHARGER_CFG_REG_4_da_chg_iref_clamp_START  (3)
#define SOC_SCHARGER_CHARGER_CFG_REG_4_da_chg_iref_clamp_END    (4)
#define SOC_SCHARGER_CHARGER_CFG_REG_4_da_chg_ir_set_START      (5)
#define SOC_SCHARGER_CHARGER_CFG_REG_4_da_chg_ir_set_END        (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_CHARGER_CFG_REG_5_UNION
 结构说明  : CHARGER_CFG_REG_5 寄存器结构定义。地址偏移量:0x05，初值:0x04，宽度:8
 寄存器说明: CHARGER_配置寄存器_5
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  da_chg_pre_vchg      : 2;  /* bit[0-1]: 预充电电压阈值大小调节位
                                                               00: 2.8V
                                                               01: 3.0V
                                                               10: 3.1V
                                                               11: 3.2V */
        unsigned char  da_chg_pre_ichg      : 2;  /* bit[2-3]: 预充电电流大小调节位
                                                               00: 100mA
                                                               01: 200mA
                                                               10: 300mA
                                                               11: 400mA */
        unsigned char  da_chg_ocp_test      : 1;  /* bit[4-4]: batfet ocp test mode设置位：
                                                               0：batfet ocp不使能test mode
                                                               1:batfet ocp使能test mode */
        unsigned char  da_chg_ocp_shd       : 1;  /* bit[5-5]: 充电电流ocp保护屏蔽位
                                                               0：不屏蔽
                                                               1：屏蔽 */
        unsigned char  da_chg_ocp_delay_shd : 1;  /* bit[6-6]: chg_ocp 128us去抖时间屏蔽位：
                                                               0：不屏蔽128us去抖
                                                               1：屏蔽128us去抖 */
        unsigned char  reserved             : 1;  /* bit[7-7]: reserved */
    } reg;
} SOC_SCHARGER_CHARGER_CFG_REG_5_UNION;
#endif
#define SOC_SCHARGER_CHARGER_CFG_REG_5_da_chg_pre_vchg_START       (0)
#define SOC_SCHARGER_CHARGER_CFG_REG_5_da_chg_pre_vchg_END         (1)
#define SOC_SCHARGER_CHARGER_CFG_REG_5_da_chg_pre_ichg_START       (2)
#define SOC_SCHARGER_CHARGER_CFG_REG_5_da_chg_pre_ichg_END         (3)
#define SOC_SCHARGER_CHARGER_CFG_REG_5_da_chg_ocp_test_START       (4)
#define SOC_SCHARGER_CHARGER_CFG_REG_5_da_chg_ocp_test_END         (4)
#define SOC_SCHARGER_CHARGER_CFG_REG_5_da_chg_ocp_shd_START        (5)
#define SOC_SCHARGER_CHARGER_CFG_REG_5_da_chg_ocp_shd_END          (5)
#define SOC_SCHARGER_CHARGER_CFG_REG_5_da_chg_ocp_delay_shd_START  (6)
#define SOC_SCHARGER_CHARGER_CFG_REG_5_da_chg_ocp_delay_shd_END    (6)


/*****************************************************************************
 结构名    : SOC_SCHARGER_CHARGER_CFG_REG_6_UNION
 结构说明  : CHARGER_CFG_REG_6 寄存器结构定义。地址偏移量:0x06，初值:0x02，宽度:8
 寄存器说明: CHARGER_配置寄存器_6
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  da_chg_rechg_time   : 2;  /* bit[0-1]: 重新充电模式去抖时间设置位
                                                              00: 0.1s
                                                              01: 1s
                                                              10: 2s
                                                              11: 4s */
        unsigned char  da_chg_q4_3x_shd    : 1;  /* bit[2-2]: batfet 电阻调整屏蔽位
                                                              0：允许batfet电阻增大到典型值4倍
                                                              1：不允许batfet电阻增大到典型值4倍 */
        unsigned char  da_chg_prechg_timer : 2;  /* bit[3-4]: 预充电计时器（分钟）
                                                              00: 30min
                                                              01: 45min
                                                              10: 60min
                                                              11: 75min */
        unsigned char  reserved            : 3;  /* bit[5-7]: reserved */
    } reg;
} SOC_SCHARGER_CHARGER_CFG_REG_6_UNION;
#endif
#define SOC_SCHARGER_CHARGER_CFG_REG_6_da_chg_rechg_time_START    (0)
#define SOC_SCHARGER_CHARGER_CFG_REG_6_da_chg_rechg_time_END      (1)
#define SOC_SCHARGER_CHARGER_CFG_REG_6_da_chg_q4_3x_shd_START     (2)
#define SOC_SCHARGER_CHARGER_CFG_REG_6_da_chg_q4_3x_shd_END       (2)
#define SOC_SCHARGER_CHARGER_CFG_REG_6_da_chg_prechg_timer_START  (3)
#define SOC_SCHARGER_CHARGER_CFG_REG_6_da_chg_prechg_timer_END    (4)


/*****************************************************************************
 结构名    : SOC_SCHARGER_CHARGER_CFG_REG_7_UNION
 结构说明  : CHARGER_CFG_REG_7 寄存器结构定义。地址偏移量:0x07，初值:0x01，宽度:8
 寄存器说明: CHARGER_配置寄存器_7
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  da_chg_scp_en   : 1;  /* bit[0-0]: batfet_scp功能使能位
                                                          0：不使能batfet_scp
                                                          1：使能batfet_scp */
        unsigned char  da_chg_rpt_en   : 1;  /* bit[1-1]: charger内部信号上报使能位：
                                                          0：不使能charger内部信号上报
                                                          1：使能charger内部信号上报 */
        unsigned char  da_chg_rev_mode : 2;  /* bit[2-3]: supplement环路工作模式选择：
                                                          00：纯线性模式
                                                          01：纯迟滞模式
                                                          10：线性（小带宽）+迟滞模式
                                                          11：线性（正常带宽）+迟滞模式 */
        unsigned char  da_chg_resvo    : 4;  /* bit[4-7]: bit<0>:gap模式sense栅极电压补偿屏蔽位
                                                          0：使能
                                                          1：不使能
                                                          bit<1>:IR模式栅极电压强制开启到最大电压使能位
                                                          0：不使能
                                                          1：使能
                                                          bit<3:2>:rev_ok比较器翻转阈值调整位
                                                          00:43.75mV
                                                          01:31.25mV
                                                          10:56.25mV
                                                          11:68.75mV */
    } reg;
} SOC_SCHARGER_CHARGER_CFG_REG_7_UNION;
#endif
#define SOC_SCHARGER_CHARGER_CFG_REG_7_da_chg_scp_en_START    (0)
#define SOC_SCHARGER_CHARGER_CFG_REG_7_da_chg_scp_en_END      (0)
#define SOC_SCHARGER_CHARGER_CFG_REG_7_da_chg_rpt_en_START    (1)
#define SOC_SCHARGER_CHARGER_CFG_REG_7_da_chg_rpt_en_END      (1)
#define SOC_SCHARGER_CHARGER_CFG_REG_7_da_chg_rev_mode_START  (2)
#define SOC_SCHARGER_CHARGER_CFG_REG_7_da_chg_rev_mode_END    (3)
#define SOC_SCHARGER_CHARGER_CFG_REG_7_da_chg_resvo_START     (4)
#define SOC_SCHARGER_CHARGER_CFG_REG_7_da_chg_resvo_END       (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_CHARGER_CFG_REG_8_UNION
 结构说明  : CHARGER_CFG_REG_8 寄存器结构定义。地址偏移量:0x08，初值:0x10，宽度:8
 寄存器说明: CHARGER_配置寄存器_8
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  da_chg_vclamp_set   : 3;  /* bit[0-2]: 电池通道电阻补偿钳位电压
                                                              000： 0mV
                                                              001: 50mV
                                                              010: 100mV
                                                              011: 150mV
                                                              000: 200mV
                                                              101: 250mV
                                                              110: 300mV
                                                              111: 350mV */
        unsigned char  da_chg_vbat_plus6mv : 1;  /* bit[3-3]: rev_ok比较器上翻转阈值选择位：
                                                              0：比较器上翻转阈值设置为vbat-6mV
                                                              1：比较器上翻转阈值设置为vbat+6mV */
        unsigned char  da_chg_timer_en     : 1;  /* bit[4-4]: 充电安全计时器使能位：
                                                              0：不使能timer
                                                              1：使能timer */
        unsigned char  da_chg_test_md      : 1;  /* bit[5-5]: 测试模式设置位：
                                                              0：非测试模式
                                                              1：设置为测试模式 */
        unsigned char  da_chg_term_ichg    : 2;  /* bit[6-7]: 终止充电电流阈值调节位

                                                              00: 150mA
                                                              01: 200mA
                                                              10: 300mA
                                                              11: 400mA */
    } reg;
} SOC_SCHARGER_CHARGER_CFG_REG_8_UNION;
#endif
#define SOC_SCHARGER_CHARGER_CFG_REG_8_da_chg_vclamp_set_START    (0)
#define SOC_SCHARGER_CHARGER_CFG_REG_8_da_chg_vclamp_set_END      (2)
#define SOC_SCHARGER_CHARGER_CFG_REG_8_da_chg_vbat_plus6mv_START  (3)
#define SOC_SCHARGER_CHARGER_CFG_REG_8_da_chg_vbat_plus6mv_END    (3)
#define SOC_SCHARGER_CHARGER_CFG_REG_8_da_chg_timer_en_START      (4)
#define SOC_SCHARGER_CHARGER_CFG_REG_8_da_chg_timer_en_END        (4)
#define SOC_SCHARGER_CHARGER_CFG_REG_8_da_chg_test_md_START       (5)
#define SOC_SCHARGER_CHARGER_CFG_REG_8_da_chg_test_md_END         (5)
#define SOC_SCHARGER_CHARGER_CFG_REG_8_da_chg_term_ichg_START     (6)
#define SOC_SCHARGER_CHARGER_CFG_REG_8_da_chg_term_ichg_END       (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_CHARGER_CFG_REG_9_UNION
 结构说明  : CHARGER_CFG_REG_9 寄存器结构定义。地址偏移量:0x09，初值:0x15，宽度:8
 寄存器说明: CHARGER_配置寄存器_9
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  da_chg_vres2_sel  : 2;  /* bit[0-1]: charger环路补偿mos电阻调节
                                                            00:1X RES
                                                            01:2X RES
                                                            10:3X RES
                                                            11:4X RES */
        unsigned char  da_chg_vres1_sel  : 2;  /* bit[2-3]: charger环路补偿mos电阻调节
                                                            00:1X RES
                                                            01:2X RES
                                                            10:3X RES
                                                            11:4X RES */
        unsigned char  da_chg_vrechg_hys : 2;  /* bit[4-5]: 重新充电回滞电压调节位
                                                            00: 100mV
                                                            01: 200mV
                                                            10: 300mV
                                                            11: 400mV */
        unsigned char  reserved          : 2;  /* bit[6-7]: reserved */
    } reg;
} SOC_SCHARGER_CHARGER_CFG_REG_9_UNION;
#endif
#define SOC_SCHARGER_CHARGER_CFG_REG_9_da_chg_vres2_sel_START   (0)
#define SOC_SCHARGER_CHARGER_CFG_REG_9_da_chg_vres2_sel_END     (1)
#define SOC_SCHARGER_CHARGER_CFG_REG_9_da_chg_vres1_sel_START   (2)
#define SOC_SCHARGER_CHARGER_CFG_REG_9_da_chg_vres1_sel_END     (3)
#define SOC_SCHARGER_CHARGER_CFG_REG_9_da_chg_vrechg_hys_START  (4)
#define SOC_SCHARGER_CHARGER_CFG_REG_9_da_chg_vrechg_hys_END    (5)


/*****************************************************************************
 结构名    : SOC_SCHARGER_CHARGER_RO_REG_10_UNION
 结构说明  : CHARGER_RO_REG_10 寄存器结构定义。地址偏移量:0x0A，初值:0x00，宽度:8
 寄存器说明: CHARGER_只读寄存器_10
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  ad_chg_sft_pd    : 1;  /* bit[0-0]: batfet 弱关闭信号上报：
                                                           0：batfet没有弱关闭
                                                           1：batfet弱关闭 */
        unsigned char  ad_chg_rev_ok    : 1;  /* bit[1-1]: rev_ok比较器输出状态上报
                                                           0：rev_ok=0
                                                           1：rev_ok=1 */
        unsigned char  ad_chg_rev_en    : 1;  /* bit[2-2]: supplement模式使能信号上报：
                                                           0：supplement模式不使能
                                                           1：supplement模式使能 */
        unsigned char  ad_chg_pu_btft   : 1;  /* bit[3-3]: batfet gate pull up信号上报：
                                                           0：batfet没有pull up
                                                           1：batfet pull up */
        unsigned char  ad_chg_pre_state : 1;  /* bit[4-4]: 预充电状态指示位：
                                                           0：不在预充状态
                                                           1：在预充状态 */
        unsigned char  ad_chg_fwd_en    : 1;  /* bit[5-5]: 充电使能信号上报：
                                                           0：充电不使能
                                                           1：充电使能 */
        unsigned char  ad_chg_dpm_state : 1;  /* bit[6-6]: 系统dpm状态记录寄存器
                                                           0：Normal
                                                           1：In dpm regl */
        unsigned char  ad_chg_acl_state : 1;  /* bit[7-7]: 系统acl状态记录寄存器
                                                           0：Normal
                                                           1：In acl regl */
    } reg;
} SOC_SCHARGER_CHARGER_RO_REG_10_UNION;
#endif

#define SOC_SCHARGER_CHARGER_RO_REG_10_ad_chg_sft_pd_START     (0)
#define SOC_SCHARGER_CHARGER_RO_REG_10_ad_chg_sft_pd_END       (0)
#define SOC_SCHARGER_CHARGER_RO_REG_10_ad_chg_rev_ok_START     (1)
#define SOC_SCHARGER_CHARGER_RO_REG_10_ad_chg_rev_ok_END       (1)
#define SOC_SCHARGER_CHARGER_RO_REG_10_ad_chg_rev_en_START     (2)
#define SOC_SCHARGER_CHARGER_RO_REG_10_ad_chg_rev_en_END       (2)
#define SOC_SCHARGER_CHARGER_RO_REG_10_ad_chg_pu_btft_START    (3)
#define SOC_SCHARGER_CHARGER_RO_REG_10_ad_chg_pu_btft_END      (3)
#define SOC_SCHARGER_CHARGER_RO_REG_10_ad_chg_pre_state_START  (4)
#define SOC_SCHARGER_CHARGER_RO_REG_10_ad_chg_pre_state_END    (4)
#define SOC_SCHARGER_CHARGER_RO_REG_10_ad_chg_fwd_en_START     (5)
#define SOC_SCHARGER_CHARGER_RO_REG_10_ad_chg_fwd_en_END       (5)
#define SOC_SCHARGER_CHARGER_RO_REG_10_ad_chg_dpm_state_START  (6)
#define SOC_SCHARGER_CHARGER_RO_REG_10_ad_chg_dpm_state_END    (6)
#define SOC_SCHARGER_CHARGER_RO_REG_10_ad_chg_acl_state_START  (7)
#define SOC_SCHARGER_CHARGER_RO_REG_10_ad_chg_acl_state_END    (7)

/*****************************************************************************
 结构名    : SOC_SCHARGER_CHARGER_RO_REG_11_UNION
 结构说明  : CHARGER_RO_REG_11 寄存器结构定义。地址偏移量:0x0B，初值:0x00，宽度:8
 寄存器说明: CHARGER_只读寄存器_11
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  ad_chg_tri_state   : 1;  /* bit[0-0]: 涓流充电状态指示位：
                                                             0：不在涓流充电状态
                                                             1：在涓流充电状态 */
        unsigned char  ad_chg_therm_state : 1;  /* bit[1-1]: 系统热调整状态记录寄存器
                                                             0：Normal
                                                             1：In Thermal regl */
        unsigned char  reserved           : 6;  /* bit[2-7]: reserved */
    } reg;
} SOC_SCHARGER_CHARGER_RO_REG_11_UNION;
#endif
#define SOC_SCHARGER_CHARGER_RO_REG_11_ad_chg_tri_state_START    (0)
#define SOC_SCHARGER_CHARGER_RO_REG_11_ad_chg_tri_state_END      (0)
#define SOC_SCHARGER_CHARGER_RO_REG_11_ad_chg_therm_state_START  (1)
#define SOC_SCHARGER_CHARGER_RO_REG_11_ad_chg_therm_state_END    (1)


/*****************************************************************************
 结构名    : SOC_SCHARGER_USB_OVP_CFG_REG_0_UNION
 结构说明  : USB_OVP_CFG_REG_0 寄存器结构定义。地址偏移量:0x0C，初值:0x01，宽度:8
 寄存器说明: USB_OVP_配置寄存器_0
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  da_vusb_ovp_resv    : 4;  /* bit[0-3]: <1:0>:sc_auto_ovp阈值条件寄存器，分别对应
                                                              00 1.2V
                                                              01 0.9V
                                                              10 1.8V
                                                              11 1.5V
                                                              <3:2>预留 */
        unsigned char  da_vusb_ovp_lpf_sel : 1;  /* bit[4-4]: vusb滤波时间；
                                                              0：不滤波 默认；
                                                              1：滤波100nS */
        unsigned char  da_vusb_force_ovpok : 1;  /* bit[5-5]: 强制ovpok
                                                              0：不强制 默认；
                                                              1：强制ovpok */
        unsigned char  reserved            : 2;  /* bit[6-7]: reserved */
    } reg;
} SOC_SCHARGER_USB_OVP_CFG_REG_0_UNION;
#endif
#define SOC_SCHARGER_USB_OVP_CFG_REG_0_da_vusb_ovp_resv_START     (0)
#define SOC_SCHARGER_USB_OVP_CFG_REG_0_da_vusb_ovp_resv_END       (3)
#define SOC_SCHARGER_USB_OVP_CFG_REG_0_da_vusb_ovp_lpf_sel_START  (4)
#define SOC_SCHARGER_USB_OVP_CFG_REG_0_da_vusb_ovp_lpf_sel_END    (4)
#define SOC_SCHARGER_USB_OVP_CFG_REG_0_da_vusb_force_ovpok_START  (5)
#define SOC_SCHARGER_USB_OVP_CFG_REG_0_da_vusb_force_ovpok_END    (5)


/*****************************************************************************
 结构名    : SOC_SCHARGER_USB_OVP_CFG_REG_1_UNION
 结构说明  : USB_OVP_CFG_REG_1 寄存器结构定义。地址偏移量:0x0D，初值:0x1C，宽度:8
 寄存器说明: USB_OVP_配置寄存器_1
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  da_vusb_uvlo_sel        : 1;  /* bit[0-0]: vusb uvlo阈值设置
                                                                  0:3.2V 默认
                                                                  1:3.4V  */
        unsigned char  da_vusb_ovp_shield      : 1;  /* bit[1-1]: 屏蔽vusb ovp功能
                                                                  调试、定位专用 */
        unsigned char  da_vusb_ovp_set         : 4;  /* bit[2-5]: 0000:OVP电压为7.5V
                                                                  1100及以上:OVP电压为14V
                                                                  默认：0111，11V, step=0.5V，迟滞1V */
        unsigned char  da_vusb_sc_auto_ovp_enb : 1;  /* bit[6-6]: sc模式下auto vusb ovp功能
                                                                  0:该功能生效；
                                                                  1：该功能不生效； */
        unsigned char  reserved                : 1;  /* bit[7-7]: reserved */
    } reg;
} SOC_SCHARGER_USB_OVP_CFG_REG_1_UNION;
#endif
#define SOC_SCHARGER_USB_OVP_CFG_REG_1_da_vusb_uvlo_sel_START         (0)
#define SOC_SCHARGER_USB_OVP_CFG_REG_1_da_vusb_uvlo_sel_END           (0)
#define SOC_SCHARGER_USB_OVP_CFG_REG_1_da_vusb_ovp_shield_START       (1)
#define SOC_SCHARGER_USB_OVP_CFG_REG_1_da_vusb_ovp_shield_END         (1)
#define SOC_SCHARGER_USB_OVP_CFG_REG_1_da_vusb_ovp_set_START          (2)
#define SOC_SCHARGER_USB_OVP_CFG_REG_1_da_vusb_ovp_set_END            (5)
#define SOC_SCHARGER_USB_OVP_CFG_REG_1_da_vusb_sc_auto_ovp_enb_START  (6)
#define SOC_SCHARGER_USB_OVP_CFG_REG_1_da_vusb_sc_auto_ovp_enb_END    (6)


/*****************************************************************************
 结构名    : SOC_SCHARGER_TCPC_CFG_REG_1_UNION
 结构说明  : TCPC_CFG_REG_1 寄存器结构定义。地址偏移量:0x0E，初值:0x00，宽度:8
 寄存器说明: TCPC_配置寄存器_1
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  da_typ_cfg0 : 8;  /* bit[0-7]: type_C配置寄存器
                                                      bit<0>，RX工作模式选择：0，相对阈值检测方式；1，绝对阈值检测方式
                                                      bit<1>，CC OVP上报屏蔽信号：0，不屏蔽；1，屏蔽
                                                      bit<2>，TX的ATE修调使能信号：0，不使能；1，使能
                                                      bit<3>，RX的ATE修调使能信号：0，不使能；1，使能
                                                      bit<5:4>，CC的OCP档位调节位：
                                                      00，1x
                                                      01，0.8x
                                                      10，1.4x
                                                      11,1.2x
                                                      bit<6>，fast role swap检测使能位：0，不使能；1，使能
                                                      bit<7>，CC UVP上报屏蔽信号：0，不屏蔽；1，屏蔽 */
    } reg;
} SOC_SCHARGER_TCPC_CFG_REG_1_UNION;
#endif
#define SOC_SCHARGER_TCPC_CFG_REG_1_da_typ_cfg0_START  (0)
#define SOC_SCHARGER_TCPC_CFG_REG_1_da_typ_cfg0_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_TCPC_CFG_REG_2_UNION
 结构说明  : TCPC_CFG_REG_2 寄存器结构定义。地址偏移量:0x0F，初值:0x00，宽度:8
 寄存器说明: TCPC_配置寄存器_2
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  da_typ_cfg1 : 8;  /* bit[0-7]: type_C配置寄存器
                                                      bit<2:0>，RX相对阈值检测模式阈值档位选择：
                                                      000，105mV
                                                      001，113mV
                                                      010，120mV
                                                      011，127mV
                                                      100，97mV
                                                      101，89mV
                                                      110，81mV
                                                      111，73mV
                                                      bit<5:3>，TX环路补偿电阻档位选择：
                                                      000，31Kohm
                                                      001，41Kohm
                                                      010，51Kohm
                                                      011，61Kohm
                                                      100，21Kohm
                                                      101，11Kohm
                                                      110，6Kohm
                                                      111，1Kohm
                                                      bit<6>，CC OCP上报屏蔽信号：0，不屏蔽；1，屏蔽
                                                      bit<7>，reserved */
    } reg;
} SOC_SCHARGER_TCPC_CFG_REG_2_UNION;
#endif
#define SOC_SCHARGER_TCPC_CFG_REG_2_da_typ_cfg1_START  (0)
#define SOC_SCHARGER_TCPC_CFG_REG_2_da_typ_cfg1_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_TCPC_CFG_REG_3_UNION
 结构说明  : TCPC_CFG_REG_3 寄存器结构定义。地址偏移量:0x10，初值:0x00，宽度:8
 寄存器说明: TCPC_配置寄存器_3
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  da_typ_cfg2 : 8;  /* bit[0-7]: type_C配置寄存器
                                                      bit<2:0>，Tx环路补偿电容档位选择：
                                                      000，1pF
                                                      001，1.25pF
                                                      010，1.5pF
                                                      011，1.75pF
                                                      100，1pF
                                                      101，0.75pF
                                                      110，0.5pF
                                                      111，0.25pF
                                                      bit<7:3>，reserved */
    } reg;
} SOC_SCHARGER_TCPC_CFG_REG_3_UNION;
#endif
#define SOC_SCHARGER_TCPC_CFG_REG_3_da_typ_cfg2_START  (0)
#define SOC_SCHARGER_TCPC_CFG_REG_3_da_typ_cfg2_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_TCPC_RO_REG_5_UNION
 结构说明  : TCPC_RO_REG_5 寄存器结构定义。地址偏移量:0x11，初值:0x00，宽度:8
 寄存器说明: TCPC_只读寄存器_5
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  ad_cc_resvi : 4;  /* bit[0-3]: reserve寄存器 */
        unsigned char  reserved    : 4;  /* bit[4-7]: reserved */
    } reg;
} SOC_SCHARGER_TCPC_RO_REG_5_UNION;
#endif
#define SOC_SCHARGER_TCPC_RO_REG_5_ad_cc_resvi_START  (0)
#define SOC_SCHARGER_TCPC_RO_REG_5_ad_cc_resvi_END    (3)


/*****************************************************************************
 结构名    : SOC_SCHARGER_SCHG_LOGIC_CFG_REG_0_UNION
 结构说明  : SCHG_LOGIC_CFG_REG_0 寄存器结构定义。地址偏移量:0x12，初值:0x08，宽度:8
 寄存器说明: SCHG_LOGIC_配置寄存器_0
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  da_regn_ocp_shield : 1;  /* bit[0-0]: regn ocp信号的屏蔽信号：
                                                             0：不屏蔽regn_ocp；
                                                             1：屏蔽regn_ocp。 */
        unsigned char  da_ovp_wl_en       : 1;  /* bit[1-1]: ovp无线充电模式：
                                                             0：非无线充电状态；
                                                             1：无线充电状态； */
        unsigned char  da_otg_wl_en       : 1;  /* bit[2-2]: otg无线充电模式：
                                                             0：非无线充电状态；
                                                             1：无线充电状态； */
        unsigned char  da_otg_delay_sel   : 2;  /* bit[3-4]: OTG启动延时时间的档位选择
                                                             00：延时32ms；
                                                             01：延时64ms；
                                                             10：延时128ms；
                                                             11：延时256ms； */
        unsigned char  da_ldo33_en        : 1;  /* bit[5-5]: LDO33强制使能：
                                                             0：不强制使能；
                                                             1：强制使能； */
        unsigned char  reserved           : 2;  /* bit[6-7]: reserved */
    } reg;
} SOC_SCHARGER_SCHG_LOGIC_CFG_REG_0_UNION;
#endif
#define SOC_SCHARGER_SCHG_LOGIC_CFG_REG_0_da_regn_ocp_shield_START  (0)
#define SOC_SCHARGER_SCHG_LOGIC_CFG_REG_0_da_regn_ocp_shield_END    (0)
#define SOC_SCHARGER_SCHG_LOGIC_CFG_REG_0_da_ovp_wl_en_START        (1)
#define SOC_SCHARGER_SCHG_LOGIC_CFG_REG_0_da_ovp_wl_en_END          (1)
#define SOC_SCHARGER_SCHG_LOGIC_CFG_REG_0_da_otg_wl_en_START        (2)
#define SOC_SCHARGER_SCHG_LOGIC_CFG_REG_0_da_otg_wl_en_END          (2)
#define SOC_SCHARGER_SCHG_LOGIC_CFG_REG_0_da_otg_delay_sel_START    (3)
#define SOC_SCHARGER_SCHG_LOGIC_CFG_REG_0_da_otg_delay_sel_END      (4)
#define SOC_SCHARGER_SCHG_LOGIC_CFG_REG_0_da_ldo33_en_START         (5)
#define SOC_SCHARGER_SCHG_LOGIC_CFG_REG_0_da_ldo33_en_END           (5)


/*****************************************************************************
 结构名    : SOC_SCHARGER_SCHG_LOGIC_CFG_REG_1_UNION
 结构说明  : SCHG_LOGIC_CFG_REG_1 寄存器结构定义。地址偏移量:0x13，初值:0x00，宽度:8
 寄存器说明: SCHG_LOGIC_配置寄存器_1
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  da_sys_resvo1 : 8;  /* bit[0-7]: <0>:预留寄存器
                                                        <5>:irq_vbus_uvp控制buck使能时的滤抖时间选择：
                                                        0：无滤抖；
                                                        1：滤抖16us； */
    } reg;
} SOC_SCHARGER_SCHG_LOGIC_CFG_REG_1_UNION;
#endif
#define SOC_SCHARGER_SCHG_LOGIC_CFG_REG_1_da_sys_resvo1_START  (0)
#define SOC_SCHARGER_SCHG_LOGIC_CFG_REG_1_da_sys_resvo1_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_SCHG_LOGIC_CFG_REG_2_UNION
 结构说明  : SCHG_LOGIC_CFG_REG_2 寄存器结构定义。地址偏移量:0x14，初值:0x00，宽度:8
 寄存器说明: SCHG_LOGIC_配置寄存器_2
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  da_sys_resvo2 : 8;  /* bit[0-7]: da_sys_resvo2<4>：ADC输入信号测试使能控制；
                                                        da_sys_resvo2<3:0>:测试通道选择
                                                        0000：NA;
                                                        0001:VUSB电压；
                                                        0010：IBUS电流；
                                                        0011：VBUS电压；
                                                        0100：VOUT电压；
                                                        0101：电池电压；
                                                        0110：电池电流；
                                                        0111：ACR电压；
                                                        1000:NA;
                                                        1001：IBUS参考电流；
                                                        1010：NA;
                                                        1011:apple充电器输入电压；
                                                        1100：芯片结温； */
    } reg;
} SOC_SCHARGER_SCHG_LOGIC_CFG_REG_2_UNION;
#endif
#define SOC_SCHARGER_SCHG_LOGIC_CFG_REG_2_da_sys_resvo2_START  (0)
#define SOC_SCHARGER_SCHG_LOGIC_CFG_REG_2_da_sys_resvo2_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_SCHG_LOGIC_CFG_REG_3_UNION
 结构说明  : SCHG_LOGIC_CFG_REG_3 寄存器结构定义。地址偏移量:0x15，初值:0x00，宽度:8
 寄存器说明: SCHG_LOGIC_配置寄存器_3
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  da_wl_en      : 1;  /* bit[0-0]: 无线充电模式：
                                                        0：非无线充电状态；
                                                        1：无线充电状态； */
        unsigned char  da_timer_test : 1;  /* bit[1-1]: 计时器测试模式：
                                                        0：芯片正常工作模式；
                                                        1：测试模式，通过该信号选择计时时间短的计时，方便测试。 */
        unsigned char  reserved      : 6;  /* bit[2-7]: reserved */
    } reg;
} SOC_SCHARGER_SCHG_LOGIC_CFG_REG_3_UNION;
#endif
#define SOC_SCHARGER_SCHG_LOGIC_CFG_REG_3_da_wl_en_START       (0)
#define SOC_SCHARGER_SCHG_LOGIC_CFG_REG_3_da_wl_en_END         (0)
#define SOC_SCHARGER_SCHG_LOGIC_CFG_REG_3_da_timer_test_START  (1)
#define SOC_SCHARGER_SCHG_LOGIC_CFG_REG_3_da_timer_test_END    (1)


/*****************************************************************************
 结构名    : SOC_SCHARGER_SCHG_LOGIC_RO_REG_4_UNION
 结构说明  : SCHG_LOGIC_RO_REG_4 寄存器结构定义。地址偏移量:0x16，初值:0x00，宽度:8
 寄存器说明: SCHG_LOGIC_只读寄存器_4
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  ad_sys_resvi1 : 8;  /* bit[0-7]: 寄存器预留 */
    } reg;
} SOC_SCHARGER_SCHG_LOGIC_RO_REG_4_UNION;
#endif
#define SOC_SCHARGER_SCHG_LOGIC_RO_REG_4_ad_sys_resvi1_START  (0)
#define SOC_SCHARGER_SCHG_LOGIC_RO_REG_4_ad_sys_resvi1_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_CHARGER_BATFET_CTRL_UNION
 结构说明  : CHARGER_BATFET_CTRL 寄存器结构定义。地址偏移量:0x17，初值:0x01，宽度:8
 寄存器说明: BATFET配置寄存器
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  sc_batfet_ctrl : 1;  /* bit[0-0]: 充电管控制寄存器。系统上电复位默认为1，可由软件配置为0，另外dc_plug_n为低电平的时候对复位该寄存器。
                                                         0：关闭batfet;
                                                         1：打开batfet; */
        unsigned char  reserved       : 7;  /* bit[1-7]: reserved */
    } reg;
} SOC_SCHARGER_CHARGER_BATFET_CTRL_UNION;
#endif
#define SOC_SCHARGER_CHARGER_BATFET_CTRL_sc_batfet_ctrl_START  (0)
#define SOC_SCHARGER_CHARGER_BATFET_CTRL_sc_batfet_ctrl_END    (0)


/*****************************************************************************
 结构名    : SOC_SCHARGER_VBAT_LV_REG_UNION
 结构说明  : VBAT_LV_REG 寄存器结构定义。地址偏移量:0x18，初值:0x00，宽度:8
 寄存器说明: VBAT LV寄存器
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  vbat_lv_cfg : 1;  /* bit[0-0]: 电池电压欠压标记寄存器。上电复位值为0，上电后需由软件配置为1，在检测到电池不在位（即irq_vbat_lv为1）时，该寄存器被清零。另外该寄存器仅受硬复位pwr_rst_n影响。 */
        unsigned char  reserved    : 7;  /* bit[1-7]: reserved */
    } reg;
} SOC_SCHARGER_VBAT_LV_REG_UNION;
#endif
#define SOC_SCHARGER_VBAT_LV_REG_vbat_lv_cfg_START  (0)
#define SOC_SCHARGER_VBAT_LV_REG_vbat_lv_cfg_END    (0)


/*****************************************************************************
 结构名    : SOC_SCHARGER_VDM_CFG_REG_0_UNION
 结构说明  : VDM_CFG_REG_0 寄存器结构定义。地址偏移量:0x1A，初值:0x00，宽度:8
 寄存器说明: VDM测试模式下配置寄存器0
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  sc_pmos_en : 1;  /* bit[0-0]: VUSB到VBAT外置PMOS开关使能信号：
                                                     0：不使能PMOS开关
                                                     1：使能PMOS开关
                                                     默认0 */
        unsigned char  reserved_0 : 3;  /* bit[1-3]: reserved */
        unsigned char  sc_chg_lp  : 1;  /* bit[4-4]: 测试模式低功耗状态使能位：
                                                     0：不使能低功耗
                                                     1：使能低功耗
                                                     默认0 */
        unsigned char  reserved_1 : 3;  /* bit[5-7]: reserved */
    } reg;
} SOC_SCHARGER_VDM_CFG_REG_0_UNION;
#endif
#define SOC_SCHARGER_VDM_CFG_REG_0_sc_pmos_en_START  (0)
#define SOC_SCHARGER_VDM_CFG_REG_0_sc_pmos_en_END    (0)
#define SOC_SCHARGER_VDM_CFG_REG_0_sc_chg_lp_START   (4)
#define SOC_SCHARGER_VDM_CFG_REG_0_sc_chg_lp_END     (4)


/*****************************************************************************
 结构名    : SOC_SCHARGER_VDM_CFG_REG_1_UNION
 结构说明  : VDM_CFG_REG_1 寄存器结构定义。地址偏移量:0x1B，初值:0x00，宽度:8
 寄存器说明: VDM测试模式下配置寄存器1
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  sc_uart_en : 1;  /* bit[0-0]: UART通路使能信号：
                                                     0：不使能UART通路
                                                     1：使能UART通路
                                                     默认0 */
        unsigned char  reserved_0 : 3;  /* bit[1-3]: reserved */
        unsigned char  sc_boot_en : 1;  /* bit[4-4]: BOOT使能信号：
                                                     0：不使能BOOT下拉
                                                     1：使能BOOT下拉
                                                     默认0 */
        unsigned char  reserved_1 : 3;  /* bit[5-7]: reserved */
    } reg;
} SOC_SCHARGER_VDM_CFG_REG_1_UNION;
#endif
#define SOC_SCHARGER_VDM_CFG_REG_1_sc_uart_en_START  (0)
#define SOC_SCHARGER_VDM_CFG_REG_1_sc_uart_en_END    (0)
#define SOC_SCHARGER_VDM_CFG_REG_1_sc_boot_en_START  (4)
#define SOC_SCHARGER_VDM_CFG_REG_1_sc_boot_en_END    (4)


/*****************************************************************************
 结构名    : SOC_SCHARGER_VDM_CFG_REG_2_UNION
 结构说明  : VDM_CFG_REG_2 寄存器结构定义。地址偏移量:0x1C，初值:0x01，宽度:8
 寄存器说明: VDM测试模式下配置寄存器2
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  sc_regn_en       : 1;  /* bit[0-0]: ldo5使能控制：
                                                           0：不使能；
                                                           1：使能；
                                                           默认1 */
        unsigned char  reserved_0       : 3;  /* bit[1-3]: reserved */
        unsigned char  sc_vusb_off_mask : 1;  /* bit[4-4]: VBUS下电复位test mode屏蔽位：
                                                           0：不屏蔽
                                                           1：屏蔽
                                                           默认0 */
        unsigned char  reserved_1       : 3;  /* bit[5-7]: reserved */
    } reg;
} SOC_SCHARGER_VDM_CFG_REG_2_UNION;
#endif
#define SOC_SCHARGER_VDM_CFG_REG_2_sc_regn_en_START        (0)
#define SOC_SCHARGER_VDM_CFG_REG_2_sc_regn_en_END          (0)
#define SOC_SCHARGER_VDM_CFG_REG_2_sc_vusb_off_mask_START  (4)
#define SOC_SCHARGER_VDM_CFG_REG_2_sc_vusb_off_mask_END    (4)


/*****************************************************************************
 结构名    : SOC_SCHARGER_DET_TOP_CFG_REG_0_UNION
 结构说明  : DET_TOP_CFG_REG_0 寄存器结构定义。地址偏移量:0x20，初值:0x20，宽度:8
 寄存器说明: DET_TOP_配置寄存器_0
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  reserved            : 1;  /* bit[0-0]: reserved */
        unsigned char  da_dp_res_det_iqsel : 2;  /* bit[1-2]: DPLUS端口阻抗检测电流选择
                                                              00：1uA
                                                              01/10:10uA
                                                              11:100uA */
        unsigned char  da_dp_res_det_en    : 1;  /* bit[3-3]: DPLUS端口阻抗检测功能使能：
                                                              0：不使能；
                                                              1：使能； */
        unsigned char  da_bat_gd_shield    : 1;  /* bit[4-4]: 强制使bat_gd为高：
                                                              0：不强制，由模拟判断bat_gd是否为高电平
                                                              1：强制使bat_gd为高电平。 */
        unsigned char  da_bat_gd_sel       : 1;  /* bit[5-5]: OTG开启的电池电压判断档位选择：
                                                              0：3V
                                                              1：2.8V */
        unsigned char  da_app_det_en       : 1;  /* bit[6-6]: 苹果充电器检测功能使能：
                                                              0：不使能；
                                                              1：使能； */
        unsigned char  da_app_det_chsel    : 1;  /* bit[7-7]: 苹果充电器检测通道选择：
                                                              0：DMINUS通道；
                                                              1：DPLUS通道； */
    } reg;
} SOC_SCHARGER_DET_TOP_CFG_REG_0_UNION;
#endif
#define SOC_SCHARGER_DET_TOP_CFG_REG_0_da_dp_res_det_iqsel_START  (1)
#define SOC_SCHARGER_DET_TOP_CFG_REG_0_da_dp_res_det_iqsel_END    (2)
#define SOC_SCHARGER_DET_TOP_CFG_REG_0_da_dp_res_det_en_START     (3)
#define SOC_SCHARGER_DET_TOP_CFG_REG_0_da_dp_res_det_en_END       (3)
#define SOC_SCHARGER_DET_TOP_CFG_REG_0_da_bat_gd_shield_START     (4)
#define SOC_SCHARGER_DET_TOP_CFG_REG_0_da_bat_gd_shield_END       (4)
#define SOC_SCHARGER_DET_TOP_CFG_REG_0_da_bat_gd_sel_START        (5)
#define SOC_SCHARGER_DET_TOP_CFG_REG_0_da_bat_gd_sel_END          (5)
#define SOC_SCHARGER_DET_TOP_CFG_REG_0_da_app_det_en_START        (6)
#define SOC_SCHARGER_DET_TOP_CFG_REG_0_da_app_det_en_END          (6)
#define SOC_SCHARGER_DET_TOP_CFG_REG_0_da_app_det_chsel_START     (7)
#define SOC_SCHARGER_DET_TOP_CFG_REG_0_da_app_det_chsel_END       (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_DET_TOP_CFG_REG_1_UNION
 结构说明  : DET_TOP_CFG_REG_1 寄存器结构定义。地址偏移量:0x21，初值:0x0A，宽度:8
 寄存器说明: DET_TOP_配置寄存器_1
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  da_vbat_dc_ovp_t : 1;  /* bit[0-0]: SCC/LVC mode 下触发VBAT 过压保护，去抖时间
                                                           1:1ms
                                                           0:0.1ms */
        unsigned char  da_slp_vset      : 1;  /* bit[1-1]: sleep点退出的档位选择：
                                                           0: sleep点退出为160mV；
                                                           1：sleep点退出为120mV。 */
        unsigned char  da_sleep_block   : 1;  /* bit[2-2]: sleep点屏蔽信号：
                                                           1: 屏蔽sleep点判断结果，slp_mod输出为1
                                                           0：不屏蔽sleep点判断结果 */
        unsigned char  da_io_level      : 1;  /* bit[3-3]: D- 电源选择：
                                                           0：D-由VDDIO供电(1.8V)
                                                           1：D-由AVDD供电(3.3V) */
        unsigned char  reserved         : 4;  /* bit[4-7]: reserved */
    } reg;
} SOC_SCHARGER_DET_TOP_CFG_REG_1_UNION;
#endif
#define SOC_SCHARGER_DET_TOP_CFG_REG_1_da_vbat_dc_ovp_t_START  (0)
#define SOC_SCHARGER_DET_TOP_CFG_REG_1_da_vbat_dc_ovp_t_END    (0)
#define SOC_SCHARGER_DET_TOP_CFG_REG_1_da_slp_vset_START       (1)
#define SOC_SCHARGER_DET_TOP_CFG_REG_1_da_slp_vset_END         (1)
#define SOC_SCHARGER_DET_TOP_CFG_REG_1_da_sleep_block_START    (2)
#define SOC_SCHARGER_DET_TOP_CFG_REG_1_da_sleep_block_END      (2)
#define SOC_SCHARGER_DET_TOP_CFG_REG_1_da_io_level_START       (3)
#define SOC_SCHARGER_DET_TOP_CFG_REG_1_da_io_level_END         (3)


/*****************************************************************************
 结构名    : SOC_SCHARGER_DET_TOP_CFG_REG_2_UNION
 结构说明  : DET_TOP_CFG_REG_2 寄存器结构定义。地址偏移量:0x22，初值:0x01，宽度:8
 寄存器说明: DET_TOP_配置寄存器_2
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  da_vbat_lv_tset : 1;  /* bit[0-0]: 电池低压滤抖：
                                                          0：滤抖0.1ms；
                                                          1：滤抖1ms;  */
        unsigned char  da_vbat_lv_t    : 1;  /* bit[1-1]: VBAT LV 去抖时间设置
                                                          1：1ms
                                                          0：0.1ms */
        unsigned char  reserved        : 6;  /* bit[2-7]: reserved */
    } reg;
} SOC_SCHARGER_DET_TOP_CFG_REG_2_UNION;
#endif
#define SOC_SCHARGER_DET_TOP_CFG_REG_2_da_vbat_lv_tset_START  (0)
#define SOC_SCHARGER_DET_TOP_CFG_REG_2_da_vbat_lv_tset_END    (0)
#define SOC_SCHARGER_DET_TOP_CFG_REG_2_da_vbat_lv_t_START     (1)
#define SOC_SCHARGER_DET_TOP_CFG_REG_2_da_vbat_lv_t_END       (1)


/*****************************************************************************
 结构名    : SOC_SCHARGER_DET_TOP_CFG_REG_3_UNION
 结构说明  : DET_TOP_CFG_REG_3 寄存器结构定义。地址偏移量:0x23，初值:0x64，宽度:8
 寄存器说明: DET_TOP_配置寄存器_3
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  da_vbat_sc_uvp_t : 1;  /* bit[0-0]: SCC mode 下发生vbat欠压去抖时间
                                                           1：8ms
                                                           0：1ms */
        unsigned char  da_vbat_ov_dc    : 7;  /* bit[1-7]: SCC/LVC 过压保护档位设置
                                                           4 to 5.27 in 10mV
                                                           默认4.5V（110010） */
    } reg;
} SOC_SCHARGER_DET_TOP_CFG_REG_3_UNION;
#endif
#define SOC_SCHARGER_DET_TOP_CFG_REG_3_da_vbat_sc_uvp_t_START  (0)
#define SOC_SCHARGER_DET_TOP_CFG_REG_3_da_vbat_sc_uvp_t_END    (0)
#define SOC_SCHARGER_DET_TOP_CFG_REG_3_da_vbat_ov_dc_START     (1)
#define SOC_SCHARGER_DET_TOP_CFG_REG_3_da_vbat_ov_dc_END       (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_DET_TOP_CFG_REG_4_UNION
 结构说明  : DET_TOP_CFG_REG_4 寄存器结构定义。地址偏移量:0x24，初值:0x30，宽度:8
 寄存器说明: DET_TOP_配置寄存器_4
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  da_vbus_lvc_ov_t   : 1;  /* bit[0-0]: LVC vbus过压保护去抖时间设置
                                                             1：256us
                                                             0：4us */
        unsigned char  da_vbus_hi_ovp_sel : 1;  /* bit[1-1]: VBUS HI OVP 门限选择
                                                             1:OVP 门限为16V
                                                             0:OVP 门限为7V */
        unsigned char  da_vbus_bkvset     : 2;  /* bit[2-3]: BUCK mode 适配器输入电压档位配置：
                                                             00：5V
                                                             01：9V
                                                             10&amp;11:12V */
        unsigned char  da_vbat_uv_dc      : 3;  /* bit[4-6]: SCC/LVC 欠压保护档位设置
                                                             3.35 to 3.7 in 50mV
                                                             默认3.5V （011） */
        unsigned char  reserved           : 1;  /* bit[7-7]: reserved */
    } reg;
} SOC_SCHARGER_DET_TOP_CFG_REG_4_UNION;
#endif
#define SOC_SCHARGER_DET_TOP_CFG_REG_4_da_vbus_lvc_ov_t_START    (0)
#define SOC_SCHARGER_DET_TOP_CFG_REG_4_da_vbus_lvc_ov_t_END      (0)
#define SOC_SCHARGER_DET_TOP_CFG_REG_4_da_vbus_hi_ovp_sel_START  (1)
#define SOC_SCHARGER_DET_TOP_CFG_REG_4_da_vbus_hi_ovp_sel_END    (1)
#define SOC_SCHARGER_DET_TOP_CFG_REG_4_da_vbus_bkvset_START      (2)
#define SOC_SCHARGER_DET_TOP_CFG_REG_4_da_vbus_bkvset_END        (3)
#define SOC_SCHARGER_DET_TOP_CFG_REG_4_da_vbat_uv_dc_START       (4)
#define SOC_SCHARGER_DET_TOP_CFG_REG_4_da_vbat_uv_dc_END         (6)


/*****************************************************************************
 结构名    : SOC_SCHARGER_DET_TOP_CFG_REG_5_UNION
 结构说明  : DET_TOP_CFG_REG_5 寄存器结构定义。地址偏移量:0x25，初值:0x34，宽度:8
 寄存器说明: DET_TOP_配置寄存器_5
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  da_vbus_sc_ov_t : 1;  /* bit[0-0]: SC vbus过压保护去抖时间设置
                                                          1：256us
                                                          0：4us */
        unsigned char  da_vbus_ov_dc   : 5;  /* bit[1-5]: SCC/LVC 下 VBUS过压点的寄存器选择
                                                          SCC: 8.4 to 11.5 in 100mV
                                                           默认11V（11010）
                                                          LVC: 4.2 to 5.75 in 50mV
                                                           默认5.5V（11010） */
        unsigned char  da_vbus_ov_bk   : 2;  /* bit[6-7]: BUCK mode 下VBUS过压点的寄存器选择
                                                          00:过压点为6.5V；
                                                          01:过压点为10.5V；
                                                          10&amp;11:过压点13.5V； */
    } reg;
} SOC_SCHARGER_DET_TOP_CFG_REG_5_UNION;
#endif
#define SOC_SCHARGER_DET_TOP_CFG_REG_5_da_vbus_sc_ov_t_START  (0)
#define SOC_SCHARGER_DET_TOP_CFG_REG_5_da_vbus_sc_ov_t_END    (0)
#define SOC_SCHARGER_DET_TOP_CFG_REG_5_da_vbus_ov_dc_START    (1)
#define SOC_SCHARGER_DET_TOP_CFG_REG_5_da_vbus_ov_dc_END      (5)
#define SOC_SCHARGER_DET_TOP_CFG_REG_5_da_vbus_ov_bk_START    (6)
#define SOC_SCHARGER_DET_TOP_CFG_REG_5_da_vbus_ov_bk_END      (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_DET_TOP_CFG_REG_6_UNION
 结构说明  : DET_TOP_CFG_REG_6 寄存器结构定义。地址偏移量:0x26，初值:0x02，宽度:8
 寄存器说明: DET_TOP_配置寄存器_6
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  da_vbus_uv_sc : 3;  /* bit[0-2]: SCC/LVC欠压档位选择：
                                                        000: SCC/LVC 共用，SCC时为7.1V， LVC时为3V
                                                        以下档位只在SCC mode有效
                                                        001:欠压点为7.05V；
                                                        010:欠压点为7.0V；（default for SCC）
                                                        011:欠压点为6.95V；
                                                        100:欠压点为6.9V；
                                                        101:欠压点为6.85V；
                                                        110:欠压点为6.8V；
                                                        111:欠压点为6.75V； */
        unsigned char  da_vbus_uv_bk : 2;  /* bit[3-4]: BUCK模式欠压档位选择：
                                                        00:欠压点为3.8V；
                                                        01:欠压点为7.4V；
                                                        10&amp;11:欠压点9.7V； */
        unsigned char  reserved      : 3;  /* bit[5-7]: reserved */
    } reg;
} SOC_SCHARGER_DET_TOP_CFG_REG_6_UNION;
#endif
#define SOC_SCHARGER_DET_TOP_CFG_REG_6_da_vbus_uv_sc_START  (0)
#define SOC_SCHARGER_DET_TOP_CFG_REG_6_da_vbus_uv_sc_END    (2)
#define SOC_SCHARGER_DET_TOP_CFG_REG_6_da_vbus_uv_bk_START  (3)
#define SOC_SCHARGER_DET_TOP_CFG_REG_6_da_vbus_uv_bk_END    (4)


/*****************************************************************************
 结构名    : SOC_SCHARGER_PSEL_CFG_REG_0_UNION
 结构说明  : PSEL_CFG_REG_0 寄存器结构定义。地址偏移量:0x27，初值:0x00，宽度:8
 寄存器说明: PSEL_配置寄存器_0
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  da_ldo33_tsmod : 1;  /* bit[0-0]: AVDD OK TestMode 使能：
                                                         0：不开启AVDD_OK TEST MOD；
                                                         1：不开启AVDD_OK TEST MOD； */
        unsigned char  reserved       : 7;  /* bit[1-7]: reserved */
    } reg;
} SOC_SCHARGER_PSEL_CFG_REG_0_UNION;
#endif
#define SOC_SCHARGER_PSEL_CFG_REG_0_da_ldo33_tsmod_START  (0)
#define SOC_SCHARGER_PSEL_CFG_REG_0_da_ldo33_tsmod_END    (0)


/*****************************************************************************
 结构名    : SOC_SCHARGER_PSEL_RO_REG_1_UNION
 结构说明  : PSEL_RO_REG_1 寄存器结构定义。地址偏移量:0x28，初值:0x00，宽度:8
 寄存器说明: PSEL_只读寄存器_1
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  ad_ldo33_ok : 1;  /* bit[0-0]: psel输出的avdd_OK：
                                                      0：代表psel判断avdd_ok=0；
                                                      1：代表psel判断avdd_ok=1； */
        unsigned char  reserved    : 7;  /* bit[1-7]: reserved */
    } reg;
} SOC_SCHARGER_PSEL_RO_REG_1_UNION;
#endif
#define SOC_SCHARGER_PSEL_RO_REG_1_ad_ldo33_ok_START  (0)
#define SOC_SCHARGER_PSEL_RO_REG_1_ad_ldo33_ok_END    (0)


/*****************************************************************************
 结构名    : SOC_SCHARGER_REF_TOP_CFG_REG_0_UNION
 结构说明  : REF_TOP_CFG_REG_0 寄存器结构定义。地址偏移量:0x29，初值:0xB4，宽度:8
 寄存器说明: REF_TOP_配置寄存器_0
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  da_ref_ichg_sel     : 2;  /* bit[0-1]: IR充电电流档位调整：
                                                              00：+0；
                                                              01：+5%；
                                                              10：+10%；
                                                              11：+15%。
                                                              默认：00 */
        unsigned char  da_ref_clk_chop_sel : 2;  /* bit[2-3]: chopper时钟频率选择：
                                                              00：500KHz；
                                                              01：250KHz；
                                                              10：167KHz；
                                                              11：125KHz。
                                                              默认：01 */
        unsigned char  da_ref_chop_en      : 1;  /* bit[4-4]: 0:不使能chopper模式
                                                              1:使能chopper模式
                                                              默认：1 */
        unsigned char  da_otmp_adj         : 2;  /* bit[5-6]: 过温检测校正：
                                                              00：+10度；
                                                              01：+0；
                                                              10：-10度；
                                                              11：-20度。
                                                              默认：01 */
        unsigned char  da_ibias_switch_en  : 1;  /* bit[7-7]: 现已无外置IBIAS电阻，无需切换功能，该接口在模拟内部浮空 */
    } reg;
} SOC_SCHARGER_REF_TOP_CFG_REG_0_UNION;
#endif
#define SOC_SCHARGER_REF_TOP_CFG_REG_0_da_ref_ichg_sel_START      (0)
#define SOC_SCHARGER_REF_TOP_CFG_REG_0_da_ref_ichg_sel_END        (1)
#define SOC_SCHARGER_REF_TOP_CFG_REG_0_da_ref_clk_chop_sel_START  (2)
#define SOC_SCHARGER_REF_TOP_CFG_REG_0_da_ref_clk_chop_sel_END    (3)
#define SOC_SCHARGER_REF_TOP_CFG_REG_0_da_ref_chop_en_START       (4)
#define SOC_SCHARGER_REF_TOP_CFG_REG_0_da_ref_chop_en_END         (4)
#define SOC_SCHARGER_REF_TOP_CFG_REG_0_da_otmp_adj_START          (5)
#define SOC_SCHARGER_REF_TOP_CFG_REG_0_da_otmp_adj_END            (6)
#define SOC_SCHARGER_REF_TOP_CFG_REG_0_da_ibias_switch_en_START   (7)
#define SOC_SCHARGER_REF_TOP_CFG_REG_0_da_ibias_switch_en_END     (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_REF_TOP_CFG_REG_1_UNION
 结构说明  : REF_TOP_CFG_REG_1 寄存器结构定义。地址偏移量:0x2A，初值:0x00，宽度:8
 寄存器说明: REF_TOP_配置寄存器_1
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  da_ref_iref_sel : 1;  /* bit[0-0]: 现已无外置IBIAS电阻，该接口在模拟内部浮空 */
        unsigned char  reserved        : 7;  /* bit[1-7]: reserved */
    } reg;
} SOC_SCHARGER_REF_TOP_CFG_REG_1_UNION;
#endif
#define SOC_SCHARGER_REF_TOP_CFG_REG_1_da_ref_iref_sel_START  (0)
#define SOC_SCHARGER_REF_TOP_CFG_REG_1_da_ref_iref_sel_END    (0)


/*****************************************************************************
 结构名    : SOC_SCHARGER_REF_TOP_CFG_REG_2_UNION
 结构说明  : REF_TOP_CFG_REG_2 寄存器结构定义。地址偏移量:0x2B，初值:0x00，宽度:8
 寄存器说明: REF_TOP_配置寄存器_2
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  da_ref_reserve : 8;  /* bit[0-7]: 预留寄存器
                                                         <0>:0-不屏蔽Tsense自动切换功能,1-屏蔽Tsense自动切换功能；
                                                         <1>:0-不使能Vref伪负载,1-使能Vref伪负载；
                                                         <3:2>:GAP充电电流档位调整：
                                                         00：+0；
                                                         01：+5%；
                                                         10：+10%；
                                                         11：+15%。 */
    } reg;
} SOC_SCHARGER_REF_TOP_CFG_REG_2_UNION;
#endif
#define SOC_SCHARGER_REF_TOP_CFG_REG_2_da_ref_reserve_START  (0)
#define SOC_SCHARGER_REF_TOP_CFG_REG_2_da_ref_reserve_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_REF_TOP_CFG_REG_3_UNION
 结构说明  : REF_TOP_CFG_REG_3 寄存器结构定义。地址偏移量:0x2C，初值:0x1A，宽度:8
 寄存器说明: REF_TOP_配置寄存器_3
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  da_tdie_treg_set  : 2;  /* bit[0-1]: 热调整档位选择：
                                                            00：60度；
                                                            01：80度；
                                                            10：100度；
                                                            11：120度。
                                                            默认：10 */
        unsigned char  da_tdie_alm_set   : 2;  /* bit[2-3]: 过温预警档位选择：
                                                            00：60度；
                                                            01：80度；
                                                            10：100度；
                                                            11：120度。
                                                            默认：10 */
        unsigned char  da_ref_vrc_en     : 1;  /* bit[4-4]: 0:不使能不同温度系数电阻曲率校正模式
                                                            1:使能不同温度系数电阻曲率校正模式
                                                            默认：1 */
        unsigned char  da_ref_testib_en  : 1;  /* bit[5-5]: 0:不使能ibias测试模式
                                                            1:使能ibias测试模式
                                                            默认：0 */
        unsigned char  da_ref_testbg_en  : 1;  /* bit[6-6]: 0:不使能vbg测试模式
                                                            1:使能vbg测试模式
                                                            默认：0 */
        unsigned char  da_ref_selgate_en : 1;  /* bit[7-7]: 0:不使能vref buffer选择avddrc
                                                            1:使能vref buffer选择avddrc
                                                            默认：0 */
    } reg;
} SOC_SCHARGER_REF_TOP_CFG_REG_3_UNION;
#endif
#define SOC_SCHARGER_REF_TOP_CFG_REG_3_da_tdie_treg_set_START   (0)
#define SOC_SCHARGER_REF_TOP_CFG_REG_3_da_tdie_treg_set_END     (1)
#define SOC_SCHARGER_REF_TOP_CFG_REG_3_da_tdie_alm_set_START    (2)
#define SOC_SCHARGER_REF_TOP_CFG_REG_3_da_tdie_alm_set_END      (3)
#define SOC_SCHARGER_REF_TOP_CFG_REG_3_da_ref_vrc_en_START      (4)
#define SOC_SCHARGER_REF_TOP_CFG_REG_3_da_ref_vrc_en_END        (4)
#define SOC_SCHARGER_REF_TOP_CFG_REG_3_da_ref_testib_en_START   (5)
#define SOC_SCHARGER_REF_TOP_CFG_REG_3_da_ref_testib_en_END     (5)
#define SOC_SCHARGER_REF_TOP_CFG_REG_3_da_ref_testbg_en_START   (6)
#define SOC_SCHARGER_REF_TOP_CFG_REG_3_da_ref_testbg_en_END     (6)
#define SOC_SCHARGER_REF_TOP_CFG_REG_3_da_ref_selgate_en_START  (7)
#define SOC_SCHARGER_REF_TOP_CFG_REG_3_da_ref_selgate_en_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_DC_TOP_CFG_REG_0_UNION
 结构说明  : DC_TOP_CFG_REG_0 寄存器结构定义。地址偏移量:0x30，初值:0xE4，宽度:8
 寄存器说明: DC_TOP_配置寄存器_0
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  da_dc_ibat_regl_sns_cap : 1;  /* bit[0-0]: ibat regl sns电路补偿电容调节寄存器,
                                                                  默认0 */
        unsigned char  da_dc_ibat_regl_res     : 2;  /* bit[1-2]: ibat regl环路电阻调节寄存器
                                                                  默认10 */
        unsigned char  da_dc_ibat_regl_hys     : 1;  /* bit[3-3]: ibat regl迟滞调节预留寄存器
                                                                  默认0 */
        unsigned char  da_dc_ibat_regl_cap     : 2;  /* bit[4-5]: ibat regl环路电容调节寄存器
                                                                  默认10 */
        unsigned char  da_dc_det_psechop_en    : 1;  /* bit[6-6]: 直充检测伪chop使能信号：
                                                                  0：不使能
                                                                  1：使能 */
        unsigned char  da_cfly_scp_en          : 1;  /* bit[7-7]: 直充sc cfly scp保护使能信号
                                                                  0：不使能
                                                                  1：使能 */
    } reg;
} SOC_SCHARGER_DC_TOP_CFG_REG_0_UNION;
#endif
#define SOC_SCHARGER_DC_TOP_CFG_REG_0_da_dc_ibat_regl_sns_cap_START  (0)
#define SOC_SCHARGER_DC_TOP_CFG_REG_0_da_dc_ibat_regl_sns_cap_END    (0)
#define SOC_SCHARGER_DC_TOP_CFG_REG_0_da_dc_ibat_regl_res_START      (1)
#define SOC_SCHARGER_DC_TOP_CFG_REG_0_da_dc_ibat_regl_res_END        (2)
#define SOC_SCHARGER_DC_TOP_CFG_REG_0_da_dc_ibat_regl_hys_START      (3)
#define SOC_SCHARGER_DC_TOP_CFG_REG_0_da_dc_ibat_regl_hys_END        (3)
#define SOC_SCHARGER_DC_TOP_CFG_REG_0_da_dc_ibat_regl_cap_START      (4)
#define SOC_SCHARGER_DC_TOP_CFG_REG_0_da_dc_ibat_regl_cap_END        (5)
#define SOC_SCHARGER_DC_TOP_CFG_REG_0_da_dc_det_psechop_en_START     (6)
#define SOC_SCHARGER_DC_TOP_CFG_REG_0_da_dc_det_psechop_en_END       (6)
#define SOC_SCHARGER_DC_TOP_CFG_REG_0_da_cfly_scp_en_START           (7)
#define SOC_SCHARGER_DC_TOP_CFG_REG_0_da_cfly_scp_en_END             (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_DC_TOP_CFG_REG_1_UNION
 结构说明  : DC_TOP_CFG_REG_1 寄存器结构定义。地址偏移量:0x31，初值:0x12，宽度:8
 寄存器说明: DC_TOP_配置寄存器_1
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  da_dc_ibus_regl_res      : 2;  /* bit[0-1]: 直充ibus regl环路稳定性电阻调节寄存器 */
        unsigned char  da_dc_ibus_regl_hys      : 1;  /* bit[2-2]: ibus regl迟滞调节寄存器
                                                                   默认0 */
        unsigned char  da_dc_ibus_regl_cap      : 2;  /* bit[3-4]: ibus regl环路电容调节寄存器
                                                                   默认10 */
        unsigned char  da_dc_ibat_sns_ibias_sel : 1;  /* bit[5-5]: 直充ibat sns电路ibias调节寄存器
                                                                   0：2uA；（默认）
                                                                   1：4uA； */
        unsigned char  reserved                 : 2;  /* bit[6-7]: reserved */
    } reg;
} SOC_SCHARGER_DC_TOP_CFG_REG_1_UNION;
#endif
#define SOC_SCHARGER_DC_TOP_CFG_REG_1_da_dc_ibus_regl_res_START       (0)
#define SOC_SCHARGER_DC_TOP_CFG_REG_1_da_dc_ibus_regl_res_END         (1)
#define SOC_SCHARGER_DC_TOP_CFG_REG_1_da_dc_ibus_regl_hys_START       (2)
#define SOC_SCHARGER_DC_TOP_CFG_REG_1_da_dc_ibus_regl_hys_END         (2)
#define SOC_SCHARGER_DC_TOP_CFG_REG_1_da_dc_ibus_regl_cap_START       (3)
#define SOC_SCHARGER_DC_TOP_CFG_REG_1_da_dc_ibus_regl_cap_END         (4)
#define SOC_SCHARGER_DC_TOP_CFG_REG_1_da_dc_ibat_sns_ibias_sel_START  (5)
#define SOC_SCHARGER_DC_TOP_CFG_REG_1_da_dc_ibat_sns_ibias_sel_END    (5)


/*****************************************************************************
 结构名    : SOC_SCHARGER_DC_TOP_CFG_REG_2_UNION
 结构说明  : DC_TOP_CFG_REG_2 寄存器结构定义。地址偏移量:0x32，初值:0x29，宽度:8
 寄存器说明: DC_TOP_配置寄存器_2
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  da_dc_rcpen_delay_adj : 2;  /* bit[0-1]: 直充下电软启动使能rcp时间配置
                                                                00 0mS;
                                                                01 50mS 默认
                                                                10 100mS
                                                                11 200mS  */
        unsigned char  da_dc_ibus_regl_sel   : 6;  /* bit[2-7]: 直充ibus regl档位调节寄存器
                                                                001010对应1A，110010及以上对应5A，step=0.1A，默认1A */
    } reg;
} SOC_SCHARGER_DC_TOP_CFG_REG_2_UNION;
#endif
#define SOC_SCHARGER_DC_TOP_CFG_REG_2_da_dc_rcpen_delay_adj_START  (0)
#define SOC_SCHARGER_DC_TOP_CFG_REG_2_da_dc_rcpen_delay_adj_END    (1)
#define SOC_SCHARGER_DC_TOP_CFG_REG_2_da_dc_ibus_regl_sel_START    (2)
#define SOC_SCHARGER_DC_TOP_CFG_REG_2_da_dc_ibus_regl_sel_END      (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_DC_TOP_CFG_REG_3_UNION
 结构说明  : DC_TOP_CFG_REG_3 寄存器结构定义。地址偏移量:0x33，初值:0x1E，宽度:8
 寄存器说明: DC_TOP_配置寄存器_3
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  da_dc_regl_lpf_time : 1;  /* bit[0-0]: regl环路比较器滤波时间调节寄存器
                                                              默认0 */
        unsigned char  da_dc_regl_en       : 4;  /* bit[1-4]: DC regualtor的使能信号
                                                              0000 关闭；
                                                              1111 使能
                                                              默认1111，依次分别对应ibus/ibat/vout/vbat regl */
        unsigned char  reserved            : 3;  /* bit[5-7]: reserved */
    } reg;
} SOC_SCHARGER_DC_TOP_CFG_REG_3_UNION;
#endif
#define SOC_SCHARGER_DC_TOP_CFG_REG_3_da_dc_regl_lpf_time_START  (0)
#define SOC_SCHARGER_DC_TOP_CFG_REG_3_da_dc_regl_lpf_time_END    (0)
#define SOC_SCHARGER_DC_TOP_CFG_REG_3_da_dc_regl_en_START        (1)
#define SOC_SCHARGER_DC_TOP_CFG_REG_3_da_dc_regl_en_END          (4)


/*****************************************************************************
 结构名    : SOC_SCHARGER_DC_TOP_CFG_REG_4_UNION
 结构说明  : DC_TOP_CFG_REG_4 寄存器结构定义。地址偏移量:0x34，初值:0x00，宽度:8
 寄存器说明: DC_TOP_配置寄存器_4
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  da_dc_regl_resv : 4;  /* bit[0-3]: regultor预留寄存器 */
        unsigned char  reserved        : 4;  /* bit[4-7]: reserved */
    } reg;
} SOC_SCHARGER_DC_TOP_CFG_REG_4_UNION;
#endif
#define SOC_SCHARGER_DC_TOP_CFG_REG_4_da_dc_regl_resv_START  (0)
#define SOC_SCHARGER_DC_TOP_CFG_REG_4_da_dc_regl_resv_END    (3)


/*****************************************************************************
 结构名    : SOC_SCHARGER_DC_TOP_CFG_REG_5_UNION
 结构说明  : DC_TOP_CFG_REG_5 寄存器结构定义。地址偏移量:0x35，初值:0x00，宽度:8
 寄存器说明: DC_TOP_配置寄存器_5
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  da_dc_reserve0 : 8;  /* bit[0-7]: 寄存器预留
                                                         <0>：屏蔽直充不滤波保护：0：不屏蔽，1：屏蔽
                                                         <2:1>：sc ilim阈值校正寄存器：00：不变；01：降低20%，10：升高10%，11：升高20%
                                                         <3>：da_ibus_rcp_nflt_en：0：3A档位不使能不滤波RCP保护，1：3A档位使能不滤波RCP保护
                                                         <4>：加长vdrop scc dbt：0：不加长；1：加长（8u/1000us加长为8ms/32ms）
                                                         <5>:ovp_drop阈值设置，0:300mV,1:400mV
                                                         <6>:ovp_drop debunce time，默认0 1mS,1 20mS
                                                         <7>:s是否使能ovp_drop功能，0不使能，1表示使能，默认不使能 */
    } reg;
} SOC_SCHARGER_DC_TOP_CFG_REG_5_UNION;
#endif
#define SOC_SCHARGER_DC_TOP_CFG_REG_5_da_dc_reserve0_START  (0)
#define SOC_SCHARGER_DC_TOP_CFG_REG_5_da_dc_reserve0_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_DC_TOP_CFG_REG_6_UNION
 结构说明  : DC_TOP_CFG_REG_6 寄存器结构定义。地址偏移量:0x36，初值:0x00，宽度:8
 寄存器说明: DC_TOP_配置寄存器_6
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  da_dc_reserve1 : 8;  /* bit[0-7]: 寄存器预留：
                                                         <1:0>:2LSB控制SC的一级MUX：
                                                         00：SC1的mux1 信号被mux到芯片的URT_TX管脚，SC1的mux2 信号被mux到芯片的URT_RX管脚
                                                         01：SC1的mux1 信号被mux到芯片的URT_TX管脚，SC2的mux2 信号被mux到芯片的URT_RX管脚
                                                         10：SC2的mux1 信号被mux到芯片的URT_TX管脚，SC1的mux2 信号被mux到芯片的URT_RX管脚
                                                         11：SC2的mux1 信号被mux到芯片的URT_TX管脚，SC2的mux2 信号被mux到芯片的URT_RX管脚
                                                         <2>:regultor中断是否上报，0不上报，1上报
                                                         <3>:直充ILIM保护模式选择1：1-屏蔽128次ILIM周期关闭SC动作，计满128次后再关闭SC；0-不屏蔽128次ILIM周期关闭SC动作
                                                         <4>:直充ILIM保护模式选择2：1-触发ILIM第一次即关闭SC；0-不使能该模式
                                                         <7:5>:预留 */
    } reg;
} SOC_SCHARGER_DC_TOP_CFG_REG_6_UNION;
#endif
#define SOC_SCHARGER_DC_TOP_CFG_REG_6_da_dc_reserve1_START  (0)
#define SOC_SCHARGER_DC_TOP_CFG_REG_6_da_dc_reserve1_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_DC_TOP_CFG_REG_7_UNION
 结构说明  : DC_TOP_CFG_REG_7 寄存器结构定义。地址偏移量:0x37，初值:0x4A，宽度:8
 寄存器说明: DC_TOP_配置寄存器_7
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  da_dc_vbat_regl_res   : 2;  /* bit[0-1]: vbat regl环路条件寄存器
                                                                默认10 */
        unsigned char  da_dc_vbat_regl_cap   : 2;  /* bit[2-3]: vbat regl环路条件寄存器
                                                                默认10 */
        unsigned char  da_dc_ucpen_mode      : 1;  /* bit[4-4]: 直充ibus ucp使能方式选择
                                                                0：UCP上电屏蔽20ms（>SC缓起时间）；
                                                                1：缓起完成后100、200、400、800mS delay后使能 */
        unsigned char  da_dc_ucpen_delay_adj : 2;  /* bit[5-6]: 直充下电软启动结束后多少时间配置使能ucp时间配置
                                                                00 100mS;
                                                                01 200mS
                                                                10 400mS 默认
                                                                11 800mS  */
        unsigned char  da_dc_ss_time         : 1;  /* bit[7-7]: SC/LVC缓起时间档位：
                                                                0:6.8ms(600KHz CLK)
                                                                1:27.4ms */
    } reg;
} SOC_SCHARGER_DC_TOP_CFG_REG_7_UNION;
#endif
#define SOC_SCHARGER_DC_TOP_CFG_REG_7_da_dc_vbat_regl_res_START    (0)
#define SOC_SCHARGER_DC_TOP_CFG_REG_7_da_dc_vbat_regl_res_END      (1)
#define SOC_SCHARGER_DC_TOP_CFG_REG_7_da_dc_vbat_regl_cap_START    (2)
#define SOC_SCHARGER_DC_TOP_CFG_REG_7_da_dc_vbat_regl_cap_END      (3)
#define SOC_SCHARGER_DC_TOP_CFG_REG_7_da_dc_ucpen_mode_START       (4)
#define SOC_SCHARGER_DC_TOP_CFG_REG_7_da_dc_ucpen_mode_END         (4)
#define SOC_SCHARGER_DC_TOP_CFG_REG_7_da_dc_ucpen_delay_adj_START  (5)
#define SOC_SCHARGER_DC_TOP_CFG_REG_7_da_dc_ucpen_delay_adj_END    (6)
#define SOC_SCHARGER_DC_TOP_CFG_REG_7_da_dc_ss_time_START          (7)
#define SOC_SCHARGER_DC_TOP_CFG_REG_7_da_dc_ss_time_END            (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_DC_TOP_CFG_REG_8_UNION
 结构说明  : DC_TOP_CFG_REG_8 寄存器结构定义。地址偏移量:0x38，初值:0x4B，宽度:8
 寄存器说明: DC_TOP_配置寄存器_8
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  da_ibat_ocp_en      : 1;  /* bit[0-0]: 直充ibat过流保护使能信号
                                                              0：不使能
                                                              1：使能 */
        unsigned char  da_ibat_ocp_dbt     : 1;  /* bit[1-1]: 直充ibat过流保护debounce time：
                                                              0：0.1ms
                                                              1：1ms */
        unsigned char  da_dc_vout_regl_res : 2;  /* bit[2-3]: vout regl环路电阻调节寄存器
                                                              默认10 */
        unsigned char  da_dc_vout_regl_hys : 1;  /* bit[4-4]: vout regl环路迟滞调节寄存器
                                                              默认0 */
        unsigned char  da_dc_vout_regl_cap : 2;  /* bit[5-6]: vout regl环路电容调节寄存器
                                                              默认10 */
        unsigned char  reserved            : 1;  /* bit[7-7]: reserved */
    } reg;
} SOC_SCHARGER_DC_TOP_CFG_REG_8_UNION;
#endif
#define SOC_SCHARGER_DC_TOP_CFG_REG_8_da_ibat_ocp_en_START       (0)
#define SOC_SCHARGER_DC_TOP_CFG_REG_8_da_ibat_ocp_en_END         (0)
#define SOC_SCHARGER_DC_TOP_CFG_REG_8_da_ibat_ocp_dbt_START      (1)
#define SOC_SCHARGER_DC_TOP_CFG_REG_8_da_ibat_ocp_dbt_END        (1)
#define SOC_SCHARGER_DC_TOP_CFG_REG_8_da_dc_vout_regl_res_START  (2)
#define SOC_SCHARGER_DC_TOP_CFG_REG_8_da_dc_vout_regl_res_END    (3)
#define SOC_SCHARGER_DC_TOP_CFG_REG_8_da_dc_vout_regl_hys_START  (4)
#define SOC_SCHARGER_DC_TOP_CFG_REG_8_da_dc_vout_regl_hys_END    (4)
#define SOC_SCHARGER_DC_TOP_CFG_REG_8_da_dc_vout_regl_cap_START  (5)
#define SOC_SCHARGER_DC_TOP_CFG_REG_8_da_dc_vout_regl_cap_END    (6)


/*****************************************************************************
 结构名    : SOC_SCHARGER_DC_TOP_CFG_REG_9_UNION
 结构说明  : DC_TOP_CFG_REG_9 寄存器结构定义。地址偏移量:0x39，初值:0x8C，宽度:8
 寄存器说明: DC_TOP_配置寄存器_9
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  da_ibat_res_sel : 1;  /* bit[0-0]: 直充ibat采样电阻阻值：
                                                          0：2mohm
                                                          1：5mohm */
        unsigned char  da_ibat_ocp_sel : 7;  /* bit[1-7]: ibat峰值限流保护阈值点选择：
                                                          0000000：5A
                                                          0000001：5.1A
                                                          0000010：5.2A

                                                          …
                                                          1000110:12A
                                                          …

                                                          1010000：13A
                                                          以上的档位都为13A */
    } reg;
} SOC_SCHARGER_DC_TOP_CFG_REG_9_UNION;
#endif
#define SOC_SCHARGER_DC_TOP_CFG_REG_9_da_ibat_res_sel_START  (0)
#define SOC_SCHARGER_DC_TOP_CFG_REG_9_da_ibat_res_sel_END    (0)
#define SOC_SCHARGER_DC_TOP_CFG_REG_9_da_ibat_ocp_sel_START  (1)
#define SOC_SCHARGER_DC_TOP_CFG_REG_9_da_ibat_ocp_sel_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_DC_TOP_CFG_REG_10_UNION
 结构说明  : DC_TOP_CFG_REG_10 寄存器结构定义。地址偏移量:0x3A，初值:0xA0，宽度:8
 寄存器说明: DC_TOP_配置寄存器_10
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  da_ibus_ocp_dbt     : 1;  /* bit[0-0]: 直充ibus过流保护debounce time：
                                                              0：4us
                                                              1：64us */
        unsigned char  da_ibat_ucp_alm_sel : 5;  /* bit[1-5]: ibat峰值限流保护阈值点选择：
                                                              00000：0.5A
                                                              00001：0.6A
                                                              …
                                                              10000:2A
                                                              …
                                                              11110：3.5A
                                                              11111：3.6A */
        unsigned char  da_ibat_ucp_alm_en  : 1;  /* bit[6-6]: 直充ibat ucp保护使能信号
                                                              0：不使能
                                                              1：使能 */
        unsigned char  da_ibat_ucp_alm_dbt : 1;  /* bit[7-7]: 直充ibat过流保护debounce time：
                                                              0：1ms
                                                              1：96ms */
    } reg;
} SOC_SCHARGER_DC_TOP_CFG_REG_10_UNION;
#endif
#define SOC_SCHARGER_DC_TOP_CFG_REG_10_da_ibus_ocp_dbt_START      (0)
#define SOC_SCHARGER_DC_TOP_CFG_REG_10_da_ibus_ocp_dbt_END        (0)
#define SOC_SCHARGER_DC_TOP_CFG_REG_10_da_ibat_ucp_alm_sel_START  (1)
#define SOC_SCHARGER_DC_TOP_CFG_REG_10_da_ibat_ucp_alm_sel_END    (5)
#define SOC_SCHARGER_DC_TOP_CFG_REG_10_da_ibat_ucp_alm_en_START   (6)
#define SOC_SCHARGER_DC_TOP_CFG_REG_10_da_ibat_ucp_alm_en_END     (6)
#define SOC_SCHARGER_DC_TOP_CFG_REG_10_da_ibat_ucp_alm_dbt_START  (7)
#define SOC_SCHARGER_DC_TOP_CFG_REG_10_da_ibat_ucp_alm_dbt_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_DC_TOP_CFG_REG_11_UNION
 结构说明  : DC_TOP_CFG_REG_11 寄存器结构定义。地址偏移量:0x3B，初值:0xD0，宽度:8
 寄存器说明: DC_TOP_配置寄存器_11
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  da_ibus_rcp_sel : 2;  /* bit[0-1]: 直充ibus反向电流保护阈值点选择：
                                                          00：65mA
                                                          01：0.5A
                                                          10：1A
                                                          11：3A */
        unsigned char  da_ibus_rcp_en  : 1;  /* bit[2-2]: 直充ibus反向电流保护使能信号
                                                          0：不使能；
                                                          1：使能； */
        unsigned char  da_ibus_rcp_dbt : 1;  /* bit[3-3]: 直充ibus反向电流保护debounce time：
                                                          0：4us
                                                          1：64us */
        unsigned char  da_ibus_ocp_sel : 3;  /* bit[4-6]: 直充ibus过流保护阈值点选择：
                                                          000：2.5A
                                                          001：3A
                                                          010：3.5A
                                                          011：4A
                                                          100：4.5A
                                                          101：5A
                                                          110：5.5A
                                                          111：6A */
        unsigned char  da_ibus_ocp_en  : 1;  /* bit[7-7]: 直充ibus过流保护使能信号
                                                          0：不使能
                                                          1：使能 */
    } reg;
} SOC_SCHARGER_DC_TOP_CFG_REG_11_UNION;
#endif
#define SOC_SCHARGER_DC_TOP_CFG_REG_11_da_ibus_rcp_sel_START  (0)
#define SOC_SCHARGER_DC_TOP_CFG_REG_11_da_ibus_rcp_sel_END    (1)
#define SOC_SCHARGER_DC_TOP_CFG_REG_11_da_ibus_rcp_en_START   (2)
#define SOC_SCHARGER_DC_TOP_CFG_REG_11_da_ibus_rcp_en_END     (2)
#define SOC_SCHARGER_DC_TOP_CFG_REG_11_da_ibus_rcp_dbt_START  (3)
#define SOC_SCHARGER_DC_TOP_CFG_REG_11_da_ibus_rcp_dbt_END    (3)
#define SOC_SCHARGER_DC_TOP_CFG_REG_11_da_ibus_ocp_sel_START  (4)
#define SOC_SCHARGER_DC_TOP_CFG_REG_11_da_ibus_ocp_sel_END    (6)
#define SOC_SCHARGER_DC_TOP_CFG_REG_11_da_ibus_ocp_en_START   (7)
#define SOC_SCHARGER_DC_TOP_CFG_REG_11_da_ibus_ocp_en_END     (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_DC_TOP_CFG_REG_12_UNION
 结构说明  : DC_TOP_CFG_REG_12 寄存器结构定义。地址偏移量:0x3C，初值:0x57，宽度:8
 寄存器说明: DC_TOP_配置寄存器_12
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  da_ilim_sc_ocp_en      : 1;  /* bit[0-0]: SCC 下管峰值过流保护使能信号
                                                                 0：不使能
                                                                 1：使能 */
        unsigned char  da_ilim_bus_sc_ocp_sel : 5;  /* bit[1-5]: SCC ibus峰值限流保护阈值点选择：
                                                                 00000：4.8A
                                                                 00001：5A
                                                                 00010：5.2A
                                                                 00011：5.4A
                                                                 00100：5.6A
                                                                 …
                                                                 01011:7A
                                                                 …
                                                                 11110：10.8A
                                                                 11111：11A */
        unsigned char  da_ilim_bus_sc_ocp_en  : 1;  /* bit[6-6]: SCC ibus峰值过流保护使能信号
                                                                 0：不使能
                                                                 1：使能 */
        unsigned char  da_ibus_ucp_en         : 1;  /* bit[7-7]: 直充ibus欠流保护使能信号
                                                                 0：不使能
                                                                 1：使能 */
    } reg;
} SOC_SCHARGER_DC_TOP_CFG_REG_12_UNION;
#endif
#define SOC_SCHARGER_DC_TOP_CFG_REG_12_da_ilim_sc_ocp_en_START       (0)
#define SOC_SCHARGER_DC_TOP_CFG_REG_12_da_ilim_sc_ocp_en_END         (0)
#define SOC_SCHARGER_DC_TOP_CFG_REG_12_da_ilim_bus_sc_ocp_sel_START  (1)
#define SOC_SCHARGER_DC_TOP_CFG_REG_12_da_ilim_bus_sc_ocp_sel_END    (5)
#define SOC_SCHARGER_DC_TOP_CFG_REG_12_da_ilim_bus_sc_ocp_en_START   (6)
#define SOC_SCHARGER_DC_TOP_CFG_REG_12_da_ilim_bus_sc_ocp_en_END     (6)
#define SOC_SCHARGER_DC_TOP_CFG_REG_12_da_ibus_ucp_en_START          (7)
#define SOC_SCHARGER_DC_TOP_CFG_REG_12_da_ibus_ucp_en_END            (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_DC_TOP_CFG_REG_13_UNION
 结构说明  : DC_TOP_CFG_REG_13 寄存器结构定义。地址偏移量:0x3D，初值:0xD8，宽度:8
 寄存器说明: DC_TOP_配置寄存器_13
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  da_sc_drvq1f       : 3;  /* bit[0-2]: Q1 driver 下降沿调节档位
                                                             2MSB 是reserved
                                                             xx1：是xx0 驱动档位的7/3 */
        unsigned char  da_ilim_sc_ocp_sel : 5;  /* bit[3-7]: SCC 下管峰值限流保护阈值点选择：
                                                             00000：4.6A
                                                             00001：5A
                                                             00010：5.2A
                                                             00011：5.4A
                                                             00100：5.6A
                                                             …
                                                             11011:10A
                                                             …
                                                             11110：10.8A
                                                             11111：11A */
    } reg;
} SOC_SCHARGER_DC_TOP_CFG_REG_13_UNION;
#endif
#define SOC_SCHARGER_DC_TOP_CFG_REG_13_da_sc_drvq1f_START        (0)
#define SOC_SCHARGER_DC_TOP_CFG_REG_13_da_sc_drvq1f_END          (2)
#define SOC_SCHARGER_DC_TOP_CFG_REG_13_da_ilim_sc_ocp_sel_START  (3)
#define SOC_SCHARGER_DC_TOP_CFG_REG_13_da_ilim_sc_ocp_sel_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_DC_TOP_CFG_REG_14_UNION
 结构说明  : DC_TOP_CFG_REG_14 寄存器结构定义。地址偏移量:0x3E，初值:0x00，宽度:8
 寄存器说明: DC_TOP_配置寄存器_14
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  da_sc_drvq2f : 3;  /* bit[0-2]: Q2 driver 下降沿调节档位
                                                       2MSB 是reserved
                                                       xx1：是xx0 驱动档位的7/3 */
        unsigned char  da_sc_drvq1r : 3;  /* bit[3-5]: Q1 driver 上升沿调节档位
                                                       1MSB 是reserved
                                                       x01：驱动档位为x00的2倍
                                                       x10: 驱动档位为x00的3倍
                                                       x11：驱动档位为x00的4倍 */
        unsigned char  reserved     : 2;  /* bit[6-7]: reserved */
    } reg;
} SOC_SCHARGER_DC_TOP_CFG_REG_14_UNION;
#endif
#define SOC_SCHARGER_DC_TOP_CFG_REG_14_da_sc_drvq2f_START  (0)
#define SOC_SCHARGER_DC_TOP_CFG_REG_14_da_sc_drvq2f_END    (2)
#define SOC_SCHARGER_DC_TOP_CFG_REG_14_da_sc_drvq1r_START  (3)
#define SOC_SCHARGER_DC_TOP_CFG_REG_14_da_sc_drvq1r_END    (5)


/*****************************************************************************
 结构名    : SOC_SCHARGER_DC_TOP_CFG_REG_15_UNION
 结构说明  : DC_TOP_CFG_REG_15 寄存器结构定义。地址偏移量:0x3F，初值:0x00，宽度:8
 寄存器说明: DC_TOP_配置寄存器_15
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  da_sc_drvq3f : 3;  /* bit[0-2]: Q3 driver 下降沿调节档位
                                                       2MSB 是reserved
                                                       xx1：是xx0 驱动档位的7/3 */
        unsigned char  da_sc_drvq2r : 3;  /* bit[3-5]: Q2 driver 上升沿调节档位
                                                       1MSB 是reserved
                                                       x01：驱动档位为x00的2倍
                                                       x10: 驱动档位为x00的3倍
                                                       x11：驱动档位为x00的4倍 */
        unsigned char  reserved     : 2;  /* bit[6-7]: reserved */
    } reg;
} SOC_SCHARGER_DC_TOP_CFG_REG_15_UNION;
#endif
#define SOC_SCHARGER_DC_TOP_CFG_REG_15_da_sc_drvq3f_START  (0)
#define SOC_SCHARGER_DC_TOP_CFG_REG_15_da_sc_drvq3f_END    (2)
#define SOC_SCHARGER_DC_TOP_CFG_REG_15_da_sc_drvq2r_START  (3)
#define SOC_SCHARGER_DC_TOP_CFG_REG_15_da_sc_drvq2r_END    (5)


/*****************************************************************************
 结构名    : SOC_SCHARGER_DC_TOP_CFG_REG_16_UNION
 结构说明  : DC_TOP_CFG_REG_16 寄存器结构定义。地址偏移量:0x40，初值:0x00，宽度:8
 寄存器说明: DC_TOP_配置寄存器_16
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  da_sc_drvq4f : 3;  /* bit[0-2]: Q4 driver 下降沿调节档位
                                                       1MSB 是特殊寄存器
                                                       第2个bit reserved
                                                       0x1：是000 驱动档位的7/3
                                                       1xx: 关闭SC1（不对用户开放） */
        unsigned char  da_sc_drvq3r : 3;  /* bit[3-5]: Q3 driver 上升沿调节档位
                                                       1MSB 是reserved
                                                       x01：驱动档位为x00的2倍
                                                       x10: 驱动档位为x00的3倍
                                                       x11：驱动档位为x00的4倍 */
        unsigned char  reserved     : 2;  /* bit[6-7]: reserved */
    } reg;
} SOC_SCHARGER_DC_TOP_CFG_REG_16_UNION;
#endif
#define SOC_SCHARGER_DC_TOP_CFG_REG_16_da_sc_drvq4f_START  (0)
#define SOC_SCHARGER_DC_TOP_CFG_REG_16_da_sc_drvq4f_END    (2)
#define SOC_SCHARGER_DC_TOP_CFG_REG_16_da_sc_drvq3r_START  (3)
#define SOC_SCHARGER_DC_TOP_CFG_REG_16_da_sc_drvq3r_END    (5)


/*****************************************************************************
 结构名    : SOC_SCHARGER_DC_TOP_CFG_REG_17_UNION
 结构说明  : DC_TOP_CFG_REG_17 寄存器结构定义。地址偏移量:0x41，初值:0x00，宽度:8
 寄存器说明: DC_TOP_配置寄存器_17
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  da_sc_dtgbga_dly : 2;  /* bit[0-1]: SC 死区时间档位。SC 特殊寄存器，不对用户开放 */
        unsigned char  da_sc_dt_grev    : 2;  /* bit[2-3]: SC 死区时间保留寄存器。SC 特殊寄存器，不对用户开放 */
        unsigned char  da_sc_drvq4r     : 3;  /* bit[4-6]: Q4 driver 上升沿调节档位
                                                           1MSB 是特殊寄存器
                                                           x01：驱动档位为x00的2倍
                                                           x10: 驱动档位为x00的3倍
                                                           x11：驱动档位为x00的4倍
                                                           1xx：关闭SC2（不对用户开放） */
        unsigned char  reserved         : 1;  /* bit[7-7]: reserved */
    } reg;
} SOC_SCHARGER_DC_TOP_CFG_REG_17_UNION;
#endif
#define SOC_SCHARGER_DC_TOP_CFG_REG_17_da_sc_dtgbga_dly_START  (0)
#define SOC_SCHARGER_DC_TOP_CFG_REG_17_da_sc_dtgbga_dly_END    (1)
#define SOC_SCHARGER_DC_TOP_CFG_REG_17_da_sc_dt_grev_START     (2)
#define SOC_SCHARGER_DC_TOP_CFG_REG_17_da_sc_dt_grev_END       (3)
#define SOC_SCHARGER_DC_TOP_CFG_REG_17_da_sc_drvq4r_START      (4)
#define SOC_SCHARGER_DC_TOP_CFG_REG_17_da_sc_drvq4r_END        (6)


/*****************************************************************************
 结构名    : SOC_SCHARGER_DC_TOP_CFG_REG_18_UNION
 结构说明  : DC_TOP_CFG_REG_18 寄存器结构定义。地址偏移量:0x42，初值:0x00，宽度:8
 寄存器说明: DC_TOP_配置寄存器_18
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  da_sc_ts_mod   : 1;  /* bit[0-0]: SC 测试模式使能
                                                         0：关闭测试模式；
                                                         1：使能测试模式； */
        unsigned char  da_sc_ini_dt   : 1;  /* bit[1-1]: 预留寄存器 */
        unsigned char  da_sc_gbdt_adj : 1;  /* bit[2-2]: SC 死区时间保留寄存器。SC 特殊寄存器，不对用户开放 */
        unsigned char  da_sc_gadt_adj : 1;  /* bit[3-3]: SC 死区时间保留寄存器。SC 特殊寄存器，不对用户开放 */
        unsigned char  da_sc_freq_sh  : 1;  /* bit[4-4]: SC 在600KHz附近抖频
                                                         0：不抖频
                                                         1: 抖频 */
        unsigned char  da_sc_freq     : 3;  /* bit[5-7]: SC 开关频率
                                                         000：600KHz
                                                         001：750KHz
                                                         010: 600KHz
                                                         011: 500KHz
                                                         100: 400KHz
                                                         101：300KHz
                                                         110: 1MHz
                                                         111：1.2MHz */
    } reg;
} SOC_SCHARGER_DC_TOP_CFG_REG_18_UNION;
#endif
#define SOC_SCHARGER_DC_TOP_CFG_REG_18_da_sc_ts_mod_START    (0)
#define SOC_SCHARGER_DC_TOP_CFG_REG_18_da_sc_ts_mod_END      (0)
#define SOC_SCHARGER_DC_TOP_CFG_REG_18_da_sc_ini_dt_START    (1)
#define SOC_SCHARGER_DC_TOP_CFG_REG_18_da_sc_ini_dt_END      (1)
#define SOC_SCHARGER_DC_TOP_CFG_REG_18_da_sc_gbdt_adj_START  (2)
#define SOC_SCHARGER_DC_TOP_CFG_REG_18_da_sc_gbdt_adj_END    (2)
#define SOC_SCHARGER_DC_TOP_CFG_REG_18_da_sc_gadt_adj_START  (3)
#define SOC_SCHARGER_DC_TOP_CFG_REG_18_da_sc_gadt_adj_END    (3)
#define SOC_SCHARGER_DC_TOP_CFG_REG_18_da_sc_freq_sh_START   (4)
#define SOC_SCHARGER_DC_TOP_CFG_REG_18_da_sc_freq_sh_END     (4)
#define SOC_SCHARGER_DC_TOP_CFG_REG_18_da_sc_freq_START      (5)
#define SOC_SCHARGER_DC_TOP_CFG_REG_18_da_sc_freq_END        (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_DC_TOP_CFG_REG_19_UNION
 结构说明  : DC_TOP_CFG_REG_19 寄存器结构定义。地址偏移量:0x43，初值:0x00，宽度:8
 寄存器说明: DC_TOP_配置寄存器_19
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  da_sc_ts_patha   : 1;  /* bit[0-0]: SC 测试模式：强制打开Q1，Q4
                                                           0：Q1，Q4关闭；
                                                           1：Q1，Q4打开； */
        unsigned char  da_sc_ts_muxselb : 3;  /* bit[1-3]: SC 测试模式二级MUX，mux2 选择
                                                           000： DGND
                                                           001：CLK_PWMQ2
                                                           010: CLK_PWMQ3
                                                           011: CLK_GA_ON
                                                           100: CLK_GA_OFF
                                                           101: CLK_PATHA
                                                           110: CLK_BUF
                                                           111: SC_LVCSPLY_RDY_BUF */
        unsigned char  da_sc_ts_muxsela : 3;  /* bit[4-6]: SC 测试模式二级MUX，mux1 选择
                                                           000： DGND
                                                           001：CLK_PWMQ1
                                                           010: CLK_PWMQ4
                                                           011: CLK_GB_ON
                                                           100: CLK_GB_OFF
                                                           101: CLK_PATHB
                                                           110: CLK_600KMUXBUF
                                                           111: SC_RDY_BUF */
        unsigned char  da_sc_ts_mux     : 1;  /* bit[7-7]: SC 测试模式mux使能
                                                           0：关闭测试模式mux；
                                                           1：使能测试模式mux； */
    } reg;
} SOC_SCHARGER_DC_TOP_CFG_REG_19_UNION;
#endif
#define SOC_SCHARGER_DC_TOP_CFG_REG_19_da_sc_ts_patha_START    (0)
#define SOC_SCHARGER_DC_TOP_CFG_REG_19_da_sc_ts_patha_END      (0)
#define SOC_SCHARGER_DC_TOP_CFG_REG_19_da_sc_ts_muxselb_START  (1)
#define SOC_SCHARGER_DC_TOP_CFG_REG_19_da_sc_ts_muxselb_END    (3)
#define SOC_SCHARGER_DC_TOP_CFG_REG_19_da_sc_ts_muxsela_START  (4)
#define SOC_SCHARGER_DC_TOP_CFG_REG_19_da_sc_ts_muxsela_END    (6)
#define SOC_SCHARGER_DC_TOP_CFG_REG_19_da_sc_ts_mux_START      (7)
#define SOC_SCHARGER_DC_TOP_CFG_REG_19_da_sc_ts_mux_END        (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_DC_TOP_CFG_REG_20_UNION
 结构说明  : DC_TOP_CFG_REG_20 寄存器结构定义。地址偏移量:0x44，初值:0x44，宽度:8
 寄存器说明: DC_TOP_配置寄存器_20
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  da_vdrop_lvc_ovp_nflt_sel : 2;  /* bit[0-1]: LVC Vdrop 过压不滤波保护阈值点选择：
                                                                    00：400mV
                                                                    01：500mV
                                                                    10：600mV
                                                                    11：800mV */
        unsigned char  da_vdrop_lvc_ovp_en       : 1;  /* bit[2-2]: LVC Vdrop过压保护使能信号
                                                                    0：不使能
                                                                    1：使能 */
        unsigned char  da_vdrop_lvc_ovp_dbt      : 1;  /* bit[3-3]: LVC Vdrop过压保护debounce time：
                                                                    0：4us
                                                                    1：64us */
        unsigned char  da_vdrop_lvc_min_nflt_sel : 2;  /* bit[4-5]: LVC Vdrop min不滤波保护阈值点选择：
                                                                    00：-400mV
                                                                    01：-500mV
                                                                    10：-600mV
                                                                    11：-800mV */
        unsigned char  da_vdrop_lvc_min_en       : 1;  /* bit[6-6]: LVC Vdrop min保护使能信号
                                                                    0：不使能；
                                                                    1：使能； */
        unsigned char  da_sc_ts_pathb            : 1;  /* bit[7-7]: SC 测试模式：强制打开Q2，Q3
                                                                    0：Q2，Q3关闭；
                                                                    1：Q2，Q3打开； */
    } reg;
} SOC_SCHARGER_DC_TOP_CFG_REG_20_UNION;
#endif
#define SOC_SCHARGER_DC_TOP_CFG_REG_20_da_vdrop_lvc_ovp_nflt_sel_START  (0)
#define SOC_SCHARGER_DC_TOP_CFG_REG_20_da_vdrop_lvc_ovp_nflt_sel_END    (1)
#define SOC_SCHARGER_DC_TOP_CFG_REG_20_da_vdrop_lvc_ovp_en_START        (2)
#define SOC_SCHARGER_DC_TOP_CFG_REG_20_da_vdrop_lvc_ovp_en_END          (2)
#define SOC_SCHARGER_DC_TOP_CFG_REG_20_da_vdrop_lvc_ovp_dbt_START       (3)
#define SOC_SCHARGER_DC_TOP_CFG_REG_20_da_vdrop_lvc_ovp_dbt_END         (3)
#define SOC_SCHARGER_DC_TOP_CFG_REG_20_da_vdrop_lvc_min_nflt_sel_START  (4)
#define SOC_SCHARGER_DC_TOP_CFG_REG_20_da_vdrop_lvc_min_nflt_sel_END    (5)
#define SOC_SCHARGER_DC_TOP_CFG_REG_20_da_vdrop_lvc_min_en_START        (6)
#define SOC_SCHARGER_DC_TOP_CFG_REG_20_da_vdrop_lvc_min_en_END          (6)
#define SOC_SCHARGER_DC_TOP_CFG_REG_20_da_sc_ts_pathb_START             (7)
#define SOC_SCHARGER_DC_TOP_CFG_REG_20_da_sc_ts_pathb_END               (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_DC_TOP_CFG_REG_21_UNION
 结构说明  : DC_TOP_CFG_REG_21 寄存器结构定义。地址偏移量:0x45，初值:0x51，宽度:8
 寄存器说明: DC_TOP_配置寄存器_21
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  da_vin2vout_sc_en    : 1;  /* bit[0-0]: SCC Vdrop过压保护使能信号
                                                               0：不使能
                                                               1：使能 */
        unsigned char  da_vin2vout_sc_dbt   : 1;  /* bit[1-1]: SCC Vin-2Vout过压保护debounce time：
                                                               0：4us
                                                               1：256us */
        unsigned char  da_vdrop_lvc_ovp_sel : 5;  /* bit[2-6]: LVC Vdrop过压保护阈值点选择：1step=20mV
                                                               00000：0V
                                                               00001：20mV
                                                               00010：40mV
                                                               00011：60mV
                                                               …
                                                               10100：400mV
                                                               …
                                                               11110-11111:600mV */
        unsigned char  reserved             : 1;  /* bit[7-7]: reserved */
    } reg;
} SOC_SCHARGER_DC_TOP_CFG_REG_21_UNION;
#endif
#define SOC_SCHARGER_DC_TOP_CFG_REG_21_da_vin2vout_sc_en_START     (0)
#define SOC_SCHARGER_DC_TOP_CFG_REG_21_da_vin2vout_sc_en_END       (0)
#define SOC_SCHARGER_DC_TOP_CFG_REG_21_da_vin2vout_sc_dbt_START    (1)
#define SOC_SCHARGER_DC_TOP_CFG_REG_21_da_vin2vout_sc_dbt_END      (1)
#define SOC_SCHARGER_DC_TOP_CFG_REG_21_da_vdrop_lvc_ovp_sel_START  (2)
#define SOC_SCHARGER_DC_TOP_CFG_REG_21_da_vdrop_lvc_ovp_sel_END    (6)


/*****************************************************************************
 结构名    : SOC_SCHARGER_DC_TOP_CFG_REG_22_UNION
 结构说明  : DC_TOP_CFG_REG_22 寄存器结构定义。地址偏移量:0x46，初值:0x11，宽度:8
 寄存器说明: DC_TOP_配置寄存器_22
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  da_vin2vout_sc_min_sel      : 2;  /* bit[0-1]: LVC、SCC VDROP min保护阈值点选择：
                                                                      00：80m
                                                                      01：40mV
                                                                      10：0mV
                                                                      11：-40mV */
        unsigned char  da_vin2vout_sc_min_nflt_sel : 2;  /* bit[2-3]: SCC Vin-2Vout过压不滤波下阈值点选择：
                                                                      00：-400mV
                                                                      01：-500mV
                                                                      10：-600mV
                                                                      11：-800mV */
        unsigned char  da_vin2vout_sc_max_sel      : 2;  /* bit[4-5]: SCC Vin-2Vout过压上阈值点选择：
                                                                      00：300mV
                                                                      01：400mV
                                                                      10：500mV
                                                                      11：600mV */
        unsigned char  da_vin2vout_sc_max_nflt_sel : 2;  /* bit[6-7]: SCC Vin-2Vout过压不滤波上阈值点选择：
                                                                      00：400mV
                                                                      01：500mV
                                                                      10：600mV
                                                                      11：800mV */
    } reg;
} SOC_SCHARGER_DC_TOP_CFG_REG_22_UNION;
#endif
#define SOC_SCHARGER_DC_TOP_CFG_REG_22_da_vin2vout_sc_min_sel_START       (0)
#define SOC_SCHARGER_DC_TOP_CFG_REG_22_da_vin2vout_sc_min_sel_END         (1)
#define SOC_SCHARGER_DC_TOP_CFG_REG_22_da_vin2vout_sc_min_nflt_sel_START  (2)
#define SOC_SCHARGER_DC_TOP_CFG_REG_22_da_vin2vout_sc_min_nflt_sel_END    (3)
#define SOC_SCHARGER_DC_TOP_CFG_REG_22_da_vin2vout_sc_max_sel_START       (4)
#define SOC_SCHARGER_DC_TOP_CFG_REG_22_da_vin2vout_sc_max_sel_END         (5)
#define SOC_SCHARGER_DC_TOP_CFG_REG_22_da_vin2vout_sc_max_nflt_sel_START  (6)
#define SOC_SCHARGER_DC_TOP_CFG_REG_22_da_vin2vout_sc_max_nflt_sel_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_DC_TOP_RO_REG_23_UNION
 结构说明  : DC_TOP_RO_REG_23 寄存器结构定义。地址偏移量:0x47，初值:0x00，宽度:8
 寄存器说明: DC_TOP_只读寄存器_23
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  ad_scc_cflyp_scp : 1;  /* bit[0-0]: SC cflyp短路保护信号
                                                           0：没有发生SC cflyp短路；
                                                           1：发生SC cflyp短路； */
        unsigned char  ad_scc_cflyn_scp : 1;  /* bit[1-1]: SC cflyn短路保护信号
                                                           0：没有发生SC cflyn短路；
                                                           1：发生SC cflyn短路； */
        unsigned char  ad_sc_rdy        : 1;  /* bit[2-2]: SC1 软启动完毕 */
        unsigned char  reserved         : 5;  /* bit[3-7]: reserved */
    } reg;
} SOC_SCHARGER_DC_TOP_RO_REG_23_UNION;
#endif
#define SOC_SCHARGER_DC_TOP_RO_REG_23_ad_scc_cflyp_scp_START  (0)
#define SOC_SCHARGER_DC_TOP_RO_REG_23_ad_scc_cflyp_scp_END    (0)
#define SOC_SCHARGER_DC_TOP_RO_REG_23_ad_scc_cflyn_scp_START  (1)
#define SOC_SCHARGER_DC_TOP_RO_REG_23_ad_scc_cflyn_scp_END    (1)
#define SOC_SCHARGER_DC_TOP_RO_REG_23_ad_sc_rdy_START         (2)
#define SOC_SCHARGER_DC_TOP_RO_REG_23_ad_sc_rdy_END           (2)


/*****************************************************************************
 结构名    : SOC_SCHARGER_VERSION0_RO_REG_0_UNION
 结构说明  : VERSION0_RO_REG_0 寄存器结构定义。地址偏移量:0x49，初值:0xF6，宽度:8
 寄存器说明: VERSION0_只读寄存器_0
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  chip_id0 : 8;  /* bit[0-7]: 版本寄存器：
                                                   11110110&#45;&#45;&#45;&#45;&#45;-SchargerV600 */
    } reg;
} SOC_SCHARGER_VERSION0_RO_REG_0_UNION;
#endif
#define SOC_SCHARGER_VERSION0_RO_REG_0_chip_id0_START  (0)
#define SOC_SCHARGER_VERSION0_RO_REG_0_chip_id0_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_VERSION1_RO_REG_0_UNION
 结构说明  : VERSION1_RO_REG_0 寄存器结构定义。地址偏移量:0x4A，初值:0xF0，宽度:8
 寄存器说明: VERSION1_只读寄存器_0
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  chip_id1 : 8;  /* bit[0-7]: 版本寄存器：
                                                   11110000&#45;&#45;&#45;&#45;&#45;-SchargerV600 */
    } reg;
} SOC_SCHARGER_VERSION1_RO_REG_0_UNION;
#endif
#define SOC_SCHARGER_VERSION1_RO_REG_0_chip_id1_START  (0)
#define SOC_SCHARGER_VERSION1_RO_REG_0_chip_id1_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_VERSION2_RO_REG_0_UNION
 结构说明  : VERSION2_RO_REG_0 寄存器结构定义。地址偏移量:0x4B，初值:0x36，宽度:8
 寄存器说明: VERSION2_只读寄存器_0
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  chip_id2 : 8;  /* bit[0-7]: 芯片ID寄存器：
                                                   00110110&#45;&#45;&#45;&#45;&#45;-6 */
    } reg;
} SOC_SCHARGER_VERSION2_RO_REG_0_UNION;
#endif
#define SOC_SCHARGER_VERSION2_RO_REG_0_chip_id2_START  (0)
#define SOC_SCHARGER_VERSION2_RO_REG_0_chip_id2_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_VERSION3_RO_REG_0_UNION
 结构说明  : VERSION3_RO_REG_0 寄存器结构定义。地址偏移量:0x4C，初值:0x35，宽度:8
 寄存器说明: VERSION3_只读寄存器_0
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  chip_id3 : 8;  /* bit[0-7]: 芯片ID寄存器：
                                                   00110101&#45;&#45;&#45;&#45;&#45;-5 */
    } reg;
} SOC_SCHARGER_VERSION3_RO_REG_0_UNION;
#endif
#define SOC_SCHARGER_VERSION3_RO_REG_0_chip_id3_START  (0)
#define SOC_SCHARGER_VERSION3_RO_REG_0_chip_id3_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_VERSION4_RO_REG_0_UNION
 结构说明  : VERSION4_RO_REG_0 寄存器结构定义。地址偏移量:0x4D，初值:0x32，宽度:8
 寄存器说明: VERSION4_只读寄存器_0
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  chip_id4 : 8;  /* bit[0-7]: 芯片ID寄存器：
                                                   00110010&#45;&#45;&#45;&#45;&#45;-2 */
    } reg;
} SOC_SCHARGER_VERSION4_RO_REG_0_UNION;
#endif
#define SOC_SCHARGER_VERSION4_RO_REG_0_chip_id4_START  (0)
#define SOC_SCHARGER_VERSION4_RO_REG_0_chip_id4_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_VERSION5_RO_REG_0_UNION
 结构说明  : VERSION5_RO_REG_0 寄存器结构定义。地址偏移量:0x4E，初值:0x36，宽度:8
 寄存器说明: VERSION5_只读寄存器_0
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  chip_id5 : 8;  /* bit[0-7]: 芯片ID寄存器：
                                                   00110110&#45;&#45;&#45;&#45;&#45;-6 */
    } reg;
} SOC_SCHARGER_VERSION5_RO_REG_0_UNION;
#endif
#define SOC_SCHARGER_VERSION5_RO_REG_0_chip_id5_START  (0)
#define SOC_SCHARGER_VERSION5_RO_REG_0_chip_id5_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_BUCK_CFG_REG_0_UNION
 结构说明  : BUCK_CFG_REG_0 寄存器结构定义。地址偏移量:0x50，初值:0x2B，宽度:8
 寄存器说明: BUCK_配置寄存器_0
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  da_buck_12v_maxduty_en  : 1;  /* bit[0-0]: 12V输入时最大占空比是否使能 */
        unsigned char  da_buck_12v_maxduty_adj : 2;  /* bit[1-2]: 12V输入时最大占空比调节 */
        unsigned char  da_buck_ilimit          : 5;  /* bit[3-7]: buck平均限流环设定电流
                                                                  00000：85mA
                                                                  00001：130mA
                                                                  00010：200mA
                                                                  00011：300mA
                                                                  00100：400mA
                                                                  00101：475mA
                                                                  00110：600mA
                                                                  00111：700mA
                                                                  01000：800mA
                                                                  01001：825mA
                                                                  01010：1.0A
                                                                  01011：1.1A
                                                                  01100：1.2A
                                                                  01101：1.3A
                                                                  01110：1.4A
                                                                  01111：1.5A
                                                                  10000：1.6A
                                                                  10001：1.7A
                                                                  10010：1.8A
                                                                  10011：1.9A
                                                                  10100：2.0A
                                                                  10101：2.1A
                                                                  10110：2.2A
                                                                  10111：2.3A
                                                                  11000：2.4A
                                                                  11001：2.5A
                                                                  11010：2.6A
                                                                  11011：2.7A
                                                                  11100：2.8A
                                                                  11101：2.9A
                                                                  11110：3.0A
                                                                  11111：3.1A */
    } reg;
} SOC_SCHARGER_BUCK_CFG_REG_0_UNION;
#endif
#define SOC_SCHARGER_BUCK_CFG_REG_0_da_buck_12v_maxduty_en_START   (0)
#define SOC_SCHARGER_BUCK_CFG_REG_0_da_buck_12v_maxduty_en_END     (0)
#define SOC_SCHARGER_BUCK_CFG_REG_0_da_buck_12v_maxduty_adj_START  (1)
#define SOC_SCHARGER_BUCK_CFG_REG_0_da_buck_12v_maxduty_adj_END    (2)
#define SOC_SCHARGER_BUCK_CFG_REG_0_da_buck_ilimit_START           (3)
#define SOC_SCHARGER_BUCK_CFG_REG_0_da_buck_ilimit_END             (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_BUCK_CFG_REG_1_UNION
 结构说明  : BUCK_CFG_REG_1 寄存器结构定义。地址偏移量:0x51，初值:0x03，宽度:8
 寄存器说明: BUCK_配置寄存器_1
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  da_buck_9v_maxduty_en  : 1;  /* bit[0-0]: 9V输入时最大占空比是否使能 */
        unsigned char  da_buck_9v_maxduty_adj : 2;  /* bit[1-2]: 9V输入时最大占空比调节 */
        unsigned char  reserved               : 5;  /* bit[3-7]: reserved */
    } reg;
} SOC_SCHARGER_BUCK_CFG_REG_1_UNION;
#endif
#define SOC_SCHARGER_BUCK_CFG_REG_1_da_buck_9v_maxduty_en_START   (0)
#define SOC_SCHARGER_BUCK_CFG_REG_1_da_buck_9v_maxduty_en_END     (0)
#define SOC_SCHARGER_BUCK_CFG_REG_1_da_buck_9v_maxduty_adj_START  (1)
#define SOC_SCHARGER_BUCK_CFG_REG_1_da_buck_9v_maxduty_adj_END    (2)


/*****************************************************************************
 结构名    : SOC_SCHARGER_BUCK_CFG_REG_2_UNION
 结构说明  : BUCK_CFG_REG_2 寄存器结构定义。地址偏移量:0x52，初值:0x80，宽度:8
 寄存器说明: BUCK_配置寄存器_2
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  da_buck_antibst : 8;  /* bit[0-7]: <7> anti-reverbst模块使能
                                                          0：关闭 1：使能
                                                          <6> anti-reverbst去抖时间选择
                                                          0：32ms 1：8ms
                                                          <5:4> anti-reverbst参数调整
                                                          00:25mv
                                                          01:50mv
                                                          10:75mv
                                                          11:100mv */
    } reg;
} SOC_SCHARGER_BUCK_CFG_REG_2_UNION;
#endif
#define SOC_SCHARGER_BUCK_CFG_REG_2_da_buck_antibst_START  (0)
#define SOC_SCHARGER_BUCK_CFG_REG_2_da_buck_antibst_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_BUCK_CFG_REG_3_UNION
 结构说明  : BUCK_CFG_REG_3 寄存器结构定义。地址偏移量:0x53，初值:0x19，宽度:8
 寄存器说明: BUCK_配置寄存器_3
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  da_buck_cap2_ea    : 2;  /* bit[0-1]: 三型补偿网络补偿电容C2：
                                                             00：1.2pF
                                                             01: 2.4pF
                                                             10：3.6pF
                                                             11：4.8pF */
        unsigned char  da_buck_cap1_ea    : 3;  /* bit[2-4]: 三型补偿网络补偿电容C1：
                                                             000：10pF
                                                             001：20pF
                                                             010：30pF
                                                             011：40pF
                                                             100：50pF
                                                             101：60pF
                                                             110：70pF
                                                             111：80pF */
        unsigned char  da_buck_block_ctrl : 3;  /* bit[5-7]: 防倒流管控制 */
    } reg;
} SOC_SCHARGER_BUCK_CFG_REG_3_UNION;
#endif
#define SOC_SCHARGER_BUCK_CFG_REG_3_da_buck_cap2_ea_START     (0)
#define SOC_SCHARGER_BUCK_CFG_REG_3_da_buck_cap2_ea_END       (1)
#define SOC_SCHARGER_BUCK_CFG_REG_3_da_buck_cap1_ea_START     (2)
#define SOC_SCHARGER_BUCK_CFG_REG_3_da_buck_cap1_ea_END       (4)
#define SOC_SCHARGER_BUCK_CFG_REG_3_da_buck_block_ctrl_START  (5)
#define SOC_SCHARGER_BUCK_CFG_REG_3_da_buck_block_ctrl_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_BUCK_CFG_REG_4_UNION
 结构说明  : BUCK_CFG_REG_4 寄存器结构定义。地址偏移量:0x54，初值:0x55，宽度:8
 寄存器说明: BUCK_配置寄存器_4
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  da_buck_cap7_acl : 2;  /* bit[0-1]: acl环路补偿电容：
                                                           00：3pF
                                                           01: 6pF
                                                           10：9pF
                                                           11：12pF */
        unsigned char  da_buck_cap6_dpm : 2;  /* bit[2-3]: dpm环路补偿电容：
                                                           00：1.2pF
                                                           01: 2.4pF
                                                           10：3.6pF
                                                           11：4.8pF */
        unsigned char  da_buck_cap5_cc  : 2;  /* bit[4-5]: cc环路补偿电容：
                                                           00：3pF
                                                           01: 6pF
                                                           10：9pF
                                                           11：12pF */
        unsigned char  da_buck_cap3_ea  : 2;  /* bit[6-7]: 三型补偿网络补偿电容C3：
                                                           00：50fF
                                                           01: 100fF
                                                           10：150fF
                                                           11：200fF */
    } reg;
} SOC_SCHARGER_BUCK_CFG_REG_4_UNION;
#endif
#define SOC_SCHARGER_BUCK_CFG_REG_4_da_buck_cap7_acl_START  (0)
#define SOC_SCHARGER_BUCK_CFG_REG_4_da_buck_cap7_acl_END    (1)
#define SOC_SCHARGER_BUCK_CFG_REG_4_da_buck_cap6_dpm_START  (2)
#define SOC_SCHARGER_BUCK_CFG_REG_4_da_buck_cap6_dpm_END    (3)
#define SOC_SCHARGER_BUCK_CFG_REG_4_da_buck_cap5_cc_START   (4)
#define SOC_SCHARGER_BUCK_CFG_REG_4_da_buck_cap5_cc_END     (5)
#define SOC_SCHARGER_BUCK_CFG_REG_4_da_buck_cap3_ea_START   (6)
#define SOC_SCHARGER_BUCK_CFG_REG_4_da_buck_cap3_ea_END     (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_BUCK_CFG_REG_5_UNION
 结构说明  : BUCK_CFG_REG_5 寄存器结构定义。地址偏移量:0x55，初值:0x25，宽度:8
 寄存器说明: BUCK_配置寄存器_5
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  da_buck_dmd_ibias     : 2;  /* bit[0-1]: buck_dmd偏置电流调节 */
        unsigned char  da_buck_dmd_en        : 1;  /* bit[2-2]: 选择是否开启DMD功能0：关闭 1：开启 */
        unsigned char  da_buck_dmd_delay     : 1;  /* bit[3-3]: dmd检测NG延时：1：加入15nS */
        unsigned char  da_buck_dmd_clamp     : 1;  /* bit[4-4]: dmd比较器输出嵌位使能 0：嵌位 1：不嵌位 */
        unsigned char  da_buck_cmp_ibias_adj : 1;  /* bit[5-5]: 预留 */
        unsigned char  reserved              : 2;  /* bit[6-7]: reserved */
    } reg;
} SOC_SCHARGER_BUCK_CFG_REG_5_UNION;
#endif
#define SOC_SCHARGER_BUCK_CFG_REG_5_da_buck_dmd_ibias_START      (0)
#define SOC_SCHARGER_BUCK_CFG_REG_5_da_buck_dmd_ibias_END        (1)
#define SOC_SCHARGER_BUCK_CFG_REG_5_da_buck_dmd_en_START         (2)
#define SOC_SCHARGER_BUCK_CFG_REG_5_da_buck_dmd_en_END           (2)
#define SOC_SCHARGER_BUCK_CFG_REG_5_da_buck_dmd_delay_START      (3)
#define SOC_SCHARGER_BUCK_CFG_REG_5_da_buck_dmd_delay_END        (3)
#define SOC_SCHARGER_BUCK_CFG_REG_5_da_buck_dmd_clamp_START      (4)
#define SOC_SCHARGER_BUCK_CFG_REG_5_da_buck_dmd_clamp_END        (4)
#define SOC_SCHARGER_BUCK_CFG_REG_5_da_buck_cmp_ibias_adj_START  (5)
#define SOC_SCHARGER_BUCK_CFG_REG_5_da_buck_cmp_ibias_adj_END    (5)


/*****************************************************************************
 结构名    : SOC_SCHARGER_BUCK_CFG_REG_6_UNION
 结构说明  : BUCK_CFG_REG_6 寄存器结构定义。地址偏移量:0x56，初值:0x0E，宽度:8
 寄存器说明: BUCK_配置寄存器_6
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  da_buck_dpm_auto : 1;  /* bit[0-0]: buck DPM点自动调节功能开关
                                                           0、关闭
                                                           1、开启 */
        unsigned char  da_buck_dmd_sel  : 4;  /* bit[1-4]: DMD点选择
                                                           0000：-250mA
                                                           0001：-200mA
                                                           ……
                                                           0111：120mA
                                                           1000：200mA
                                                           ……
                                                           1111：700mA */
        unsigned char  reserved         : 3;  /* bit[5-7]: reserved */
    } reg;
} SOC_SCHARGER_BUCK_CFG_REG_6_UNION;
#endif
#define SOC_SCHARGER_BUCK_CFG_REG_6_da_buck_dpm_auto_START  (0)
#define SOC_SCHARGER_BUCK_CFG_REG_6_da_buck_dpm_auto_END    (0)
#define SOC_SCHARGER_BUCK_CFG_REG_6_da_buck_dmd_sel_START   (1)
#define SOC_SCHARGER_BUCK_CFG_REG_6_da_buck_dmd_sel_END     (4)


/*****************************************************************************
 结构名    : SOC_SCHARGER_BUCK_CFG_REG_7_UNION
 结构说明  : BUCK_CFG_REG_7 寄存器结构定义。地址偏移量:0x57，初值:0x60，宽度:8
 寄存器说明: BUCK_配置寄存器_7
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  da_buck_dt_lshs_delay : 2;  /* bit[0-1]: 下管到上管死区额外延迟5ns
                                                                高位：1 老死区控制加5nS
                                                                低位：1 新死区控制加5nS */
        unsigned char  da_buck_dt_lshs       : 1;  /* bit[2-2]: 下管到上管的死区产生方式选择
                                                                0：老死区时间
                                                                1：新死区时间(PWM) */
        unsigned char  da_buck_dt_hsls       : 1;  /* bit[3-3]: 上管到下管死区额外延迟5nS 0：不加5nS 1：加5nS */
        unsigned char  da_buck_dpm_sel       : 4;  /* bit[4-7]: DPM输入电压设定
                                                                0000: 84%
                                                                0001: 85%
                                                                0010: 86%
                                                                0011: 87%
                                                                0100: 88%
                                                                0101: 89%
                                                                0110: 90%
                                                                0111: 91%
                                                                1000: 92%
                                                                1001: 93%
                                                                1010: 94%
                                                                1011: 95%
                                                                1100~1111:96% */
    } reg;
} SOC_SCHARGER_BUCK_CFG_REG_7_UNION;
#endif
#define SOC_SCHARGER_BUCK_CFG_REG_7_da_buck_dt_lshs_delay_START  (0)
#define SOC_SCHARGER_BUCK_CFG_REG_7_da_buck_dt_lshs_delay_END    (1)
#define SOC_SCHARGER_BUCK_CFG_REG_7_da_buck_dt_lshs_START        (2)
#define SOC_SCHARGER_BUCK_CFG_REG_7_da_buck_dt_lshs_END          (2)
#define SOC_SCHARGER_BUCK_CFG_REG_7_da_buck_dt_hsls_START        (3)
#define SOC_SCHARGER_BUCK_CFG_REG_7_da_buck_dt_hsls_END          (3)
#define SOC_SCHARGER_BUCK_CFG_REG_7_da_buck_dpm_sel_START        (4)
#define SOC_SCHARGER_BUCK_CFG_REG_7_da_buck_dpm_sel_END          (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_BUCK_CFG_REG_8_UNION
 结构说明  : BUCK_CFG_REG_8 寄存器结构定义。地址偏移量:0x58，初值:0x05，宽度:8
 寄存器说明: BUCK_配置寄存器_8
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  da_buck_fullduty_en        : 1;  /* bit[0-0]: buck发生直通时处理方法：1为数周期内强制off一次，0为不处理 */
        unsigned char  da_buck_fullduty_delaytime : 4;  /* bit[1-4]: 多少时间没有开关动作视为直通 0001为4uS 0010为8uS 0100为16uS 1000为32uS  */
        unsigned char  da_buck_dt_sel             : 2;  /* bit[5-6]: 死区时间增加5nS，默认00；
                                                                     <0>低边
                                                                     <1>高边 */
        unsigned char  reserved                   : 1;  /* bit[7-7]: reserved */
    } reg;
} SOC_SCHARGER_BUCK_CFG_REG_8_UNION;
#endif
#define SOC_SCHARGER_BUCK_CFG_REG_8_da_buck_fullduty_en_START         (0)
#define SOC_SCHARGER_BUCK_CFG_REG_8_da_buck_fullduty_en_END           (0)
#define SOC_SCHARGER_BUCK_CFG_REG_8_da_buck_fullduty_delaytime_START  (1)
#define SOC_SCHARGER_BUCK_CFG_REG_8_da_buck_fullduty_delaytime_END    (4)
#define SOC_SCHARGER_BUCK_CFG_REG_8_da_buck_dt_sel_START              (5)
#define SOC_SCHARGER_BUCK_CFG_REG_8_da_buck_dt_sel_END                (6)


/*****************************************************************************
 结构名    : SOC_SCHARGER_BUCK_CFG_REG_9_UNION
 结构说明  : BUCK_CFG_REG_9 寄存器结构定义。地址偏移量:0x59，初值:0x10，宽度:8
 寄存器说明: BUCK_配置寄存器_9
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  da_buck_gap_auto         : 1;  /* bit[0-0]: buck gap值自动调节功能开关
                                                                   0、关闭
                                                                   1、开启 */
        unsigned char  da_buck_gap              : 3;  /* bit[1-3]: buck gap电压修调寄存器调节:
                                                                   000: 0
                                                                   001: -30mV
                                                                   010:-60mV
                                                                   011: -90mV
                                                                   100: 0mV
                                                                   101: +30mV
                                                                   110: +60mV
                                                                   111: +90mV */
        unsigned char  da_buck_fullduty_offtime : 2;  /* bit[4-5]: 直通后强制关闭上管时间调节寄存器 00为20nS 01为30nS 01为40nS 11为50n */
        unsigned char  reserved                 : 2;  /* bit[6-7]: reserved */
    } reg;
} SOC_SCHARGER_BUCK_CFG_REG_9_UNION;
#endif
#define SOC_SCHARGER_BUCK_CFG_REG_9_da_buck_gap_auto_START          (0)
#define SOC_SCHARGER_BUCK_CFG_REG_9_da_buck_gap_auto_END            (0)
#define SOC_SCHARGER_BUCK_CFG_REG_9_da_buck_gap_START               (1)
#define SOC_SCHARGER_BUCK_CFG_REG_9_da_buck_gap_END                 (3)
#define SOC_SCHARGER_BUCK_CFG_REG_9_da_buck_fullduty_offtime_START  (4)
#define SOC_SCHARGER_BUCK_CFG_REG_9_da_buck_fullduty_offtime_END    (5)


/*****************************************************************************
 结构名    : SOC_SCHARGER_BUCK_CFG_REG_10_UNION
 结构说明  : BUCK_CFG_REG_10 寄存器结构定义。地址偏移量:0x5A，初值:0x88，宽度:8
 寄存器说明: BUCK_配置寄存器_10
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  da_buck_hmosocp_lpf : 2;  /* bit[0-1]: buck上管ocp检测时滤抖时间调节
                                                              <1>等待PWM_H信号
                                                              <0>等待HG_ON信号 */
        unsigned char  da_buck_hmos_rise   : 3;  /* bit[2-4]: 上管上升沿驱动能力选择 */
        unsigned char  da_buck_hmos_fall   : 3;  /* bit[5-7]: 上管下降沿驱动能力选择 */
    } reg;
} SOC_SCHARGER_BUCK_CFG_REG_10_UNION;
#endif
#define SOC_SCHARGER_BUCK_CFG_REG_10_da_buck_hmosocp_lpf_START  (0)
#define SOC_SCHARGER_BUCK_CFG_REG_10_da_buck_hmosocp_lpf_END    (1)
#define SOC_SCHARGER_BUCK_CFG_REG_10_da_buck_hmos_rise_START    (2)
#define SOC_SCHARGER_BUCK_CFG_REG_10_da_buck_hmos_rise_END      (4)
#define SOC_SCHARGER_BUCK_CFG_REG_10_da_buck_hmos_fall_START    (5)
#define SOC_SCHARGER_BUCK_CFG_REG_10_da_buck_hmos_fall_END      (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_BUCK_CFG_REG_11_UNION
 结构说明  : BUCK_CFG_REG_11 寄存器结构定义。地址偏移量:0x5B，初值:0xB4，宽度:8
 寄存器说明: BUCK_配置寄存器_11
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  da_buck_lmos_rise   : 3;  /* bit[0-2]: 下管上升沿驱动能力选择 */
        unsigned char  da_buck_lmos_on_sel : 1;  /* bit[3-3]: 下管长时间不开场景下，强制开一次下管的动作的时间调整，默认为0
                                                              0:128uS开一次;
                                                              1:64uS开一次 */
        unsigned char  da_buck_lmos_on_en  : 1;  /* bit[4-4]: 下管长时间不开场景下，是否使能强制开一次下管的动作，默认值为1
                                                              0：关闭
                                                              1：开启此功能 */
        unsigned char  da_buck_lmos_fall   : 3;  /* bit[5-7]: 下管下降沿驱动能力选择 */
    } reg;
} SOC_SCHARGER_BUCK_CFG_REG_11_UNION;
#endif
#define SOC_SCHARGER_BUCK_CFG_REG_11_da_buck_lmos_rise_START    (0)
#define SOC_SCHARGER_BUCK_CFG_REG_11_da_buck_lmos_rise_END      (2)
#define SOC_SCHARGER_BUCK_CFG_REG_11_da_buck_lmos_on_sel_START  (3)
#define SOC_SCHARGER_BUCK_CFG_REG_11_da_buck_lmos_on_sel_END    (3)
#define SOC_SCHARGER_BUCK_CFG_REG_11_da_buck_lmos_on_en_START   (4)
#define SOC_SCHARGER_BUCK_CFG_REG_11_da_buck_lmos_on_en_END     (4)
#define SOC_SCHARGER_BUCK_CFG_REG_11_da_buck_lmos_fall_START    (5)
#define SOC_SCHARGER_BUCK_CFG_REG_11_da_buck_lmos_fall_END      (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_BUCK_CFG_REG_12_UNION
 结构说明  : BUCK_CFG_REG_12 寄存器结构定义。地址偏移量:0x5C，初值:0x52，宽度:8
 寄存器说明: BUCK_配置寄存器_12
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  da_buck_min_ontime_sel  : 1;  /* bit[0-0]: 最小导通时间屏蔽 */
        unsigned char  da_buck_min_ontime      : 2;  /* bit[1-2]: 最小导通时间选择 */
        unsigned char  da_buck_min_offtime_sel : 1;  /* bit[3-3]: 最小关断时间屏蔽 */
        unsigned char  da_buck_min_offtime     : 2;  /* bit[4-5]: 最小关断时间选择 */
        unsigned char  da_buck_lmosocp_lpf     : 2;  /* bit[6-7]: 预留 */
    } reg;
} SOC_SCHARGER_BUCK_CFG_REG_12_UNION;
#endif
#define SOC_SCHARGER_BUCK_CFG_REG_12_da_buck_min_ontime_sel_START   (0)
#define SOC_SCHARGER_BUCK_CFG_REG_12_da_buck_min_ontime_sel_END     (0)
#define SOC_SCHARGER_BUCK_CFG_REG_12_da_buck_min_ontime_START       (1)
#define SOC_SCHARGER_BUCK_CFG_REG_12_da_buck_min_ontime_END         (2)
#define SOC_SCHARGER_BUCK_CFG_REG_12_da_buck_min_offtime_sel_START  (3)
#define SOC_SCHARGER_BUCK_CFG_REG_12_da_buck_min_offtime_sel_END    (3)
#define SOC_SCHARGER_BUCK_CFG_REG_12_da_buck_min_offtime_START      (4)
#define SOC_SCHARGER_BUCK_CFG_REG_12_da_buck_min_offtime_END        (5)
#define SOC_SCHARGER_BUCK_CFG_REG_12_da_buck_lmosocp_lpf_START      (6)
#define SOC_SCHARGER_BUCK_CFG_REG_12_da_buck_lmosocp_lpf_END        (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_BUCK_CFG_REG_13_UNION
 结构说明  : BUCK_CFG_REG_13 寄存器结构定义。地址偏移量:0x5D，初值:0x01，宽度:8
 寄存器说明: BUCK_配置寄存器_13
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  da_buck_ocp_delay  : 2;  /* bit[0-1]: 预留 */
        unsigned char  da_buck_ocp_cap    : 1;  /* bit[2-2]: buck ocp 增大:
                                                             1：增大
                                                             0：不变 */
        unsigned char  da_buck_ocp_9_enb  : 1;  /* bit[3-3]: 关闭OCP功能
                                                             0：不关闭
                                                             1：关闭 */
        unsigned char  da_buck_ocp_300ma  : 2;  /* bit[4-5]: 预留 */
        unsigned char  da_buck_ocp_12vsel : 2;  /* bit[6-7]: 预留 */
    } reg;
} SOC_SCHARGER_BUCK_CFG_REG_13_UNION;
#endif
#define SOC_SCHARGER_BUCK_CFG_REG_13_da_buck_ocp_delay_START   (0)
#define SOC_SCHARGER_BUCK_CFG_REG_13_da_buck_ocp_delay_END     (1)
#define SOC_SCHARGER_BUCK_CFG_REG_13_da_buck_ocp_cap_START     (2)
#define SOC_SCHARGER_BUCK_CFG_REG_13_da_buck_ocp_cap_END       (2)
#define SOC_SCHARGER_BUCK_CFG_REG_13_da_buck_ocp_9_enb_START   (3)
#define SOC_SCHARGER_BUCK_CFG_REG_13_da_buck_ocp_9_enb_END     (3)
#define SOC_SCHARGER_BUCK_CFG_REG_13_da_buck_ocp_300ma_START   (4)
#define SOC_SCHARGER_BUCK_CFG_REG_13_da_buck_ocp_300ma_END     (5)
#define SOC_SCHARGER_BUCK_CFG_REG_13_da_buck_ocp_12vsel_START  (6)
#define SOC_SCHARGER_BUCK_CFG_REG_13_da_buck_ocp_12vsel_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_BUCK_CFG_REG_14_UNION
 结构说明  : BUCK_CFG_REG_14 寄存器结构定义。地址偏移量:0x5E，初值:0x15，宽度:8
 寄存器说明: BUCK_配置寄存器_14
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  da_buck_offtime_judge : 2;  /* bit[0-1]: 预留 */
        unsigned char  da_buck_ocp_vally     : 2;  /* bit[2-3]: buck 下管谷值ocp检测调节寄存器
                                                                00：2.1A
                                                                01：2.5A
                                                                10：2.9A
                                                                11：3.3A */
        unsigned char  da_buck_ocp_peak      : 2;  /* bit[4-5]: 预留 */
        unsigned char  da_buck_ocp_mode_sel  : 1;  /* bit[6-6]: 预留 */
        unsigned char  reserved              : 1;  /* bit[7-7]: reserved */
    } reg;
} SOC_SCHARGER_BUCK_CFG_REG_14_UNION;
#endif
#define SOC_SCHARGER_BUCK_CFG_REG_14_da_buck_offtime_judge_START  (0)
#define SOC_SCHARGER_BUCK_CFG_REG_14_da_buck_offtime_judge_END    (1)
#define SOC_SCHARGER_BUCK_CFG_REG_14_da_buck_ocp_vally_START      (2)
#define SOC_SCHARGER_BUCK_CFG_REG_14_da_buck_ocp_vally_END        (3)
#define SOC_SCHARGER_BUCK_CFG_REG_14_da_buck_ocp_peak_START       (4)
#define SOC_SCHARGER_BUCK_CFG_REG_14_da_buck_ocp_peak_END         (5)
#define SOC_SCHARGER_BUCK_CFG_REG_14_da_buck_ocp_mode_sel_START   (6)
#define SOC_SCHARGER_BUCK_CFG_REG_14_da_buck_ocp_mode_sel_END     (6)


/*****************************************************************************
 结构名    : SOC_SCHARGER_BUCK_CFG_REG_15_UNION
 结构说明  : BUCK_CFG_REG_15 寄存器结构定义。地址偏移量:0x5F，初值:0x48，宽度:8
 寄存器说明: BUCK_配置寄存器_15
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  da_buck_osc_frq             : 4;  /* bit[0-3]: osc振荡器频率（供参考，频率还受trim码值影响）：
                                                                      0000：0.9M
                                                                      0001：0.95M
                                                                      ……
                                                                      1000：1.4M
                                                                      1001：1.5M
                                                                      ……
                                                                      1111：1.9M */
        unsigned char  da_buck_offtime_judge_en    : 1;  /* bit[4-4]: CC环路增加4M滤波电阻 */
        unsigned char  da_buck_offtime_judge_delay : 3;  /* bit[5-7]: 预留 */
    } reg;
} SOC_SCHARGER_BUCK_CFG_REG_15_UNION;
#endif
#define SOC_SCHARGER_BUCK_CFG_REG_15_da_buck_osc_frq_START              (0)
#define SOC_SCHARGER_BUCK_CFG_REG_15_da_buck_osc_frq_END                (3)
#define SOC_SCHARGER_BUCK_CFG_REG_15_da_buck_offtime_judge_en_START     (4)
#define SOC_SCHARGER_BUCK_CFG_REG_15_da_buck_offtime_judge_en_END       (4)
#define SOC_SCHARGER_BUCK_CFG_REG_15_da_buck_offtime_judge_delay_START  (5)
#define SOC_SCHARGER_BUCK_CFG_REG_15_da_buck_offtime_judge_delay_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_BUCK_CFG_REG_16_UNION
 结构说明  : BUCK_CFG_REG_16 寄存器结构定义。地址偏移量:0x60，初值:0x3F，宽度:8
 寄存器说明: BUCK_配置寄存器_16
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  da_buck_pre_op : 8;  /* bit[0-7]:

                                                         <5:0>buck 6个环路使能控制位：
                                                          THEM.CC.CV.SYS.DPM.ACL
                                                          000000：环路全部关闭
                                                          111111：环路全部开启 */
    } reg;
} SOC_SCHARGER_BUCK_CFG_REG_16_UNION;
#endif
#define SOC_SCHARGER_BUCK_CFG_REG_16_da_buck_pre_op_START  (0)
#define SOC_SCHARGER_BUCK_CFG_REG_16_da_buck_pre_op_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_BUCK_CFG_REG_17_UNION
 结构说明  : BUCK_CFG_REG_17 寄存器结构定义。地址偏移量:0x61，初值:0x54，宽度:8
 寄存器说明: BUCK_配置寄存器_17
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  da_buck_res1_ea   : 3;  /* bit[0-2]: 三型补偿网络补偿电阻R1：
                                                            000：400K
                                                            001: 650K
                                                            010：900K
                                                            011：1100K
                                                            100：1300K
                                                            101: 1800K
                                                            110：2300K
                                                            111: 3000K */
        unsigned char  da_buck_rdy_en    : 1;  /* bit[3-3]: buck 各状态上报开关
                                                            0、关闭
                                                            1、开启 */
        unsigned char  da_buck_rc_thm    : 2;  /* bit[4-5]: thm滤波rc：
                                                            00：无滤波
                                                            01: 250k 1.2p
                                                            10：500k 2.4p
                                                            11：750k 3.6p */
        unsigned char  da_buck_q1ocp_adj : 2;  /* bit[6-7]: 预留 */
    } reg;
} SOC_SCHARGER_BUCK_CFG_REG_17_UNION;
#endif
#define SOC_SCHARGER_BUCK_CFG_REG_17_da_buck_res1_ea_START    (0)
#define SOC_SCHARGER_BUCK_CFG_REG_17_da_buck_res1_ea_END      (2)
#define SOC_SCHARGER_BUCK_CFG_REG_17_da_buck_rdy_en_START     (3)
#define SOC_SCHARGER_BUCK_CFG_REG_17_da_buck_rdy_en_END       (3)
#define SOC_SCHARGER_BUCK_CFG_REG_17_da_buck_rc_thm_START     (4)
#define SOC_SCHARGER_BUCK_CFG_REG_17_da_buck_rc_thm_END       (5)
#define SOC_SCHARGER_BUCK_CFG_REG_17_da_buck_q1ocp_adj_START  (6)
#define SOC_SCHARGER_BUCK_CFG_REG_17_da_buck_q1ocp_adj_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_BUCK_CFG_REG_18_UNION
 结构说明  : BUCK_CFG_REG_18 寄存器结构定义。地址偏移量:0x62，初值:0x55，宽度:8
 寄存器说明: BUCK_配置寄存器_18
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  da_buck_res13_dpm_pz  : 2;  /* bit[0-1]: dpm环路补偿电阻：
                                                                00：
                                                                01:
                                                                10：
                                                                11：  */
        unsigned char  da_buck_res12_cc_pz   : 2;  /* bit[2-3]: cc环路补偿电阻：
                                                                00：
                                                                01:
                                                                10：
                                                                11： */
        unsigned char  da_buck_res11_sel     : 2;  /* bit[4-5]: 备用 */
        unsigned char  da_buck_res10_cc_rout : 2;  /* bit[6-7]: fast cc regl 环路preamp Rout调节电阻：
                                                                00：35K
                                                                01: 70K
                                                                10：105K
                                                                11：140K */
    } reg;
} SOC_SCHARGER_BUCK_CFG_REG_18_UNION;
#endif
#define SOC_SCHARGER_BUCK_CFG_REG_18_da_buck_res13_dpm_pz_START   (0)
#define SOC_SCHARGER_BUCK_CFG_REG_18_da_buck_res13_dpm_pz_END     (1)
#define SOC_SCHARGER_BUCK_CFG_REG_18_da_buck_res12_cc_pz_START    (2)
#define SOC_SCHARGER_BUCK_CFG_REG_18_da_buck_res12_cc_pz_END      (3)
#define SOC_SCHARGER_BUCK_CFG_REG_18_da_buck_res11_sel_START      (4)
#define SOC_SCHARGER_BUCK_CFG_REG_18_da_buck_res11_sel_END        (5)
#define SOC_SCHARGER_BUCK_CFG_REG_18_da_buck_res10_cc_rout_START  (6)
#define SOC_SCHARGER_BUCK_CFG_REG_18_da_buck_res10_cc_rout_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_BUCK_CFG_REG_19_UNION
 结构说明  : BUCK_CFG_REG_19 寄存器结构定义。地址偏移量:0x63，初值:0x15，宽度:8
 寄存器说明: BUCK_配置寄存器_19
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  da_buck_res2_ea     : 2;  /* bit[0-1]: 三型补偿网络补偿电阻R2：
                                                              00：13K
                                                              01: 26K
                                                              10：39K
                                                              11：91K */
        unsigned char  da_buck_res15_acl_z : 2;  /* bit[2-3]: acl环路补偿电阻_零点：
                                                              00：
                                                              01:
                                                              10：
                                                              11：  */
        unsigned char  da_buck_res14_acl_p : 2;  /* bit[4-5]: acl环路补偿电阻_极点：
                                                              00：
                                                              01:
                                                              10：
                                                              11：  */
        unsigned char  reserved            : 2;  /* bit[6-7]: reserved */
    } reg;
} SOC_SCHARGER_BUCK_CFG_REG_19_UNION;
#endif
#define SOC_SCHARGER_BUCK_CFG_REG_19_da_buck_res2_ea_START      (0)
#define SOC_SCHARGER_BUCK_CFG_REG_19_da_buck_res2_ea_END        (1)
#define SOC_SCHARGER_BUCK_CFG_REG_19_da_buck_res15_acl_z_START  (2)
#define SOC_SCHARGER_BUCK_CFG_REG_19_da_buck_res15_acl_z_END    (3)
#define SOC_SCHARGER_BUCK_CFG_REG_19_da_buck_res14_acl_p_START  (4)
#define SOC_SCHARGER_BUCK_CFG_REG_19_da_buck_res14_acl_p_END    (5)


/*****************************************************************************
 结构名    : SOC_SCHARGER_BUCK_CFG_REG_20_UNION
 结构说明  : BUCK_CFG_REG_20 寄存器结构定义。地址偏移量:0x64，初值:0x39，宽度:8
 寄存器说明: BUCK_配置寄存器_20
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  da_buck_res5_acl_rout : 2;  /* bit[0-1]: acl环路preamp Rout调节电阻：
                                                                00：54K
                                                                01: 108K
                                                                10：162K
                                                                11：216K */
        unsigned char  da_buck_res4_dpm_gm   : 2;  /* bit[2-3]: dpm环路preamp跨导调节电阻：
                                                                00：3.5K
                                                                01: 5.3K
                                                                10：7.1K
                                                                11：8.9K */
        unsigned char  da_buck_res3_ea       : 3;  /* bit[4-6]: 三型补偿网络补偿电阻R3：
                                                                000：80K
                                                                001：120K
                                                                010：160K
                                                                011：200K
                                                                100：240K
                                                                101：280K
                                                                110：320K
                                                                111：360K */
        unsigned char  reserved              : 1;  /* bit[7-7]: reserved */
    } reg;
} SOC_SCHARGER_BUCK_CFG_REG_20_UNION;
#endif
#define SOC_SCHARGER_BUCK_CFG_REG_20_da_buck_res5_acl_rout_START  (0)
#define SOC_SCHARGER_BUCK_CFG_REG_20_da_buck_res5_acl_rout_END    (1)
#define SOC_SCHARGER_BUCK_CFG_REG_20_da_buck_res4_dpm_gm_START    (2)
#define SOC_SCHARGER_BUCK_CFG_REG_20_da_buck_res4_dpm_gm_END      (3)
#define SOC_SCHARGER_BUCK_CFG_REG_20_da_buck_res3_ea_START        (4)
#define SOC_SCHARGER_BUCK_CFG_REG_20_da_buck_res3_ea_END          (6)


/*****************************************************************************
 结构名    : SOC_SCHARGER_BUCK_CFG_REG_21_UNION
 结构说明  : BUCK_CFG_REG_21 寄存器结构定义。地址偏移量:0x65，初值:0x55，宽度:8
 寄存器说明: BUCK_配置寄存器_21
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  da_buck_res9_cc_gm  : 2;  /* bit[0-1]: fast cc regl 环路preamp跨导调节电阻：
                                                              00：6.5K
                                                              01: 13K
                                                              10：17K
                                                              11：30K */
        unsigned char  da_buck_res8_cv_gm  : 2;  /* bit[2-3]: cv regl 环路preamp跨导调节电阻：
                                                              00：0.9K
                                                              01: 1.8K
                                                              10：2.7K
                                                              11：3.6K */
        unsigned char  da_buck_res7_sys_gm : 2;  /* bit[4-5]: sys regl 环路preamp跨导调节电阻：
                                                              00：1.8K
                                                              01: 2.7K
                                                              10：3.6K
                                                              11：4.5K */
        unsigned char  da_buck_res6_thm_gm : 2;  /* bit[6-7]: themal regl环路preamp跨导调节电阻：
                                                              00：3.5K
                                                              01: 5.3K
                                                              10：7.1K
                                                              11：8.9K */
    } reg;
} SOC_SCHARGER_BUCK_CFG_REG_21_UNION;
#endif
#define SOC_SCHARGER_BUCK_CFG_REG_21_da_buck_res9_cc_gm_START   (0)
#define SOC_SCHARGER_BUCK_CFG_REG_21_da_buck_res9_cc_gm_END     (1)
#define SOC_SCHARGER_BUCK_CFG_REG_21_da_buck_res8_cv_gm_START   (2)
#define SOC_SCHARGER_BUCK_CFG_REG_21_da_buck_res8_cv_gm_END     (3)
#define SOC_SCHARGER_BUCK_CFG_REG_21_da_buck_res7_sys_gm_START  (4)
#define SOC_SCHARGER_BUCK_CFG_REG_21_da_buck_res7_sys_gm_END    (5)
#define SOC_SCHARGER_BUCK_CFG_REG_21_da_buck_res6_thm_gm_START  (6)
#define SOC_SCHARGER_BUCK_CFG_REG_21_da_buck_res6_thm_gm_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_BUCK_CFG_REG_22_UNION
 结构说明  : BUCK_CFG_REG_22 寄存器结构定义。地址偏移量:0x66，初值:0x00，宽度:8
 寄存器说明: BUCK_配置寄存器_22
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  da_buck_reserve1 : 8;  /* bit[0-7]: <7>预留寄存器
                                                           <6:5>saw peak
                                                           11: 2.5V
                                                           10: 2.0V
                                                           01: 1.5V
                                                           00:自动跟随vbus
                                                           <4:3>buck random 转换时间
                                                           00: 64us
                                                           01: 32us
                                                           10: 16us
                                                           11: 8 us
                                                           <2>buck random en
                                                           1: 开启
                                                           0: 关闭
                                                           <1:0>sc random 转换时间
                                                           00: 64us
                                                           01: 32us
                                                           10: 16us
                                                           11: 8 us */
    } reg;
} SOC_SCHARGER_BUCK_CFG_REG_22_UNION;
#endif
#define SOC_SCHARGER_BUCK_CFG_REG_22_da_buck_reserve1_START  (0)
#define SOC_SCHARGER_BUCK_CFG_REG_22_da_buck_reserve1_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_BUCK_CFG_REG_23_UNION
 结构说明  : BUCK_CFG_REG_23 寄存器结构定义。地址偏移量:0x67，初值:0xFF，宽度:8
 寄存器说明: BUCK_配置寄存器_23
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  da_buck_reserve2 : 8;  /* bit[0-7]: <7:4>buck ocp peak调节
                                                           1111： 3.8A
                                                           1000~1111：-5%
                                                           0001~0111：+5%
                                                           电流值只做参考，5V、9V、12V各不相同
                                                           <3>reverbst mode2 是否自动跟随dpm点设置
                                                           1：自动
                                                           0：手动
                                                           <2:0>手动reverbst mode2判断值
                                                           111: 4.85
                                                           110：4.75
                                                           101：4.7
                                                           100：4.65
                                                           011：4.6
                                                           010：4.55
                                                           001：4.5
                                                           000：4.45 */
    } reg;
} SOC_SCHARGER_BUCK_CFG_REG_23_UNION;
#endif
#define SOC_SCHARGER_BUCK_CFG_REG_23_da_buck_reserve2_START  (0)
#define SOC_SCHARGER_BUCK_CFG_REG_23_da_buck_reserve2_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_BUCK_CFG_REG_24_UNION
 结构说明  : BUCK_CFG_REG_24 寄存器结构定义。地址偏移量:0x68，初值:0x00，宽度:8
 寄存器说明: BUCK_配置寄存器_24
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  da_buck_ssmode_en      : 1;  /* bit[0-0]: buck启动时是否要等待VC再开启driver 0：不等待 1：等待 */
        unsigned char  da_buck_ss_init_dis    : 1;  /* bit[1-1]: 预留 */
        unsigned char  da_buck_sft_scp_enb    : 1;  /* bit[2-2]: buck 软启动是否开启scp功能，0为开启，1为屏蔽 */
        unsigned char  da_buck_sft_ocp_en     : 1;  /* bit[3-3]: 预留 */
        unsigned char  da_buck_sft_maxduty_en : 1;  /* bit[4-4]: 软启动时最大占空比限制使能控制 */
        unsigned char  da_buck_scp_dis        : 1;  /* bit[5-5]: buck scp使能与屏蔽
                                                                 0：不屏蔽1：屏蔽 */
        unsigned char  da_buck_saw_vally_adj  : 1;  /* bit[6-6]: saw谷值调节寄存器 */
        unsigned char  da_buck_saw_peak_adj   : 1;  /* bit[7-7]: saw峰值调节寄存器 */
    } reg;
} SOC_SCHARGER_BUCK_CFG_REG_24_UNION;
#endif
#define SOC_SCHARGER_BUCK_CFG_REG_24_da_buck_ssmode_en_START       (0)
#define SOC_SCHARGER_BUCK_CFG_REG_24_da_buck_ssmode_en_END         (0)
#define SOC_SCHARGER_BUCK_CFG_REG_24_da_buck_ss_init_dis_START     (1)
#define SOC_SCHARGER_BUCK_CFG_REG_24_da_buck_ss_init_dis_END       (1)
#define SOC_SCHARGER_BUCK_CFG_REG_24_da_buck_sft_scp_enb_START     (2)
#define SOC_SCHARGER_BUCK_CFG_REG_24_da_buck_sft_scp_enb_END       (2)
#define SOC_SCHARGER_BUCK_CFG_REG_24_da_buck_sft_ocp_en_START      (3)
#define SOC_SCHARGER_BUCK_CFG_REG_24_da_buck_sft_ocp_en_END        (3)
#define SOC_SCHARGER_BUCK_CFG_REG_24_da_buck_sft_maxduty_en_START  (4)
#define SOC_SCHARGER_BUCK_CFG_REG_24_da_buck_sft_maxduty_en_END    (4)
#define SOC_SCHARGER_BUCK_CFG_REG_24_da_buck_scp_dis_START         (5)
#define SOC_SCHARGER_BUCK_CFG_REG_24_da_buck_scp_dis_END           (5)
#define SOC_SCHARGER_BUCK_CFG_REG_24_da_buck_saw_vally_adj_START   (6)
#define SOC_SCHARGER_BUCK_CFG_REG_24_da_buck_saw_vally_adj_END     (6)
#define SOC_SCHARGER_BUCK_CFG_REG_24_da_buck_saw_peak_adj_START    (7)
#define SOC_SCHARGER_BUCK_CFG_REG_24_da_buck_saw_peak_adj_END      (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_BUCK_CFG_REG_25_UNION
 结构说明  : BUCK_CFG_REG_25 寄存器结构定义。地址偏移量:0x69，初值:0x1B，宽度:8
 寄存器说明: BUCK_配置寄存器_25
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  da_buckotg_ss_speed   : 2;  /* bit[0-1]: buck软启动时间快慢调节：
                                                                00：最慢，约1.5mS
                                                                11：最快，约0.5mS */
        unsigned char  da_buckotg_acl_downen : 1;  /* bit[2-2]: buck_otg acl op下拉电路使能：
                                                                1、开启
                                                                0、关闭 */
        unsigned char  da_buck_sysovp_en     : 1;  /* bit[3-3]: buck sys ovp功能开关
                                                                0、关闭
                                                                1、开启 */
        unsigned char  da_buck_sysmin_sel    : 2;  /* bit[4-5]: Vsys最小输出电压选择
                                                                00:3.4V
                                                                01:3.5V
                                                                10:3.6V
                                                                11:3.7V */
        unsigned char  reserved              : 2;  /* bit[6-7]: reserved */
    } reg;
} SOC_SCHARGER_BUCK_CFG_REG_25_UNION;
#endif
#define SOC_SCHARGER_BUCK_CFG_REG_25_da_buckotg_ss_speed_START    (0)
#define SOC_SCHARGER_BUCK_CFG_REG_25_da_buckotg_ss_speed_END      (1)
#define SOC_SCHARGER_BUCK_CFG_REG_25_da_buckotg_acl_downen_START  (2)
#define SOC_SCHARGER_BUCK_CFG_REG_25_da_buckotg_acl_downen_END    (2)
#define SOC_SCHARGER_BUCK_CFG_REG_25_da_buck_sysovp_en_START      (3)
#define SOC_SCHARGER_BUCK_CFG_REG_25_da_buck_sysovp_en_END        (3)
#define SOC_SCHARGER_BUCK_CFG_REG_25_da_buck_sysmin_sel_START     (4)
#define SOC_SCHARGER_BUCK_CFG_REG_25_da_buck_sysmin_sel_END       (5)


/*****************************************************************************
 结构名    : SOC_SCHARGER_BUCK_RO_REG_26_UNION
 结构说明  : BUCK_RO_REG_26 寄存器结构定义。地址偏移量:0x6A，初值:0x00，宽度:8
 寄存器说明: BUCK_只读寄存器_26
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  ad_buck_ok : 1;  /* bit[0-0]: buck工作状态指示：
                                                     0：buck没有工作；
                                                     1：buck在工作。 */
        unsigned char  reserved   : 7;  /* bit[1-7]: reserved */
    } reg;
} SOC_SCHARGER_BUCK_RO_REG_26_UNION;
#endif
#define SOC_SCHARGER_BUCK_RO_REG_26_ad_buck_ok_START  (0)
#define SOC_SCHARGER_BUCK_RO_REG_26_ad_buck_ok_END    (0)


/*****************************************************************************
 结构名    : SOC_SCHARGER_BUCK_RO_REG_27_UNION
 结构说明  : BUCK_RO_REG_27 寄存器结构定义。地址偏移量:0x6B，初值:0x00，宽度:8
 寄存器说明: BUCK_只读寄存器_27
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  ad_buck_state0 : 8;  /* bit[0-7]: <7> AVDD_OK
                                                         <6> REGN_OK_D
                                                         <5> AVDDBAS_OK
                                                         <4> buck_en
                                                         <3> EN_DELAY_60US_BAS
                                                         <2> EN_CC_CV
                                                         <1> BUCK_SSOK
                                                         <0> OTG_SSOK */
    } reg;
} SOC_SCHARGER_BUCK_RO_REG_27_UNION;
#endif
#define SOC_SCHARGER_BUCK_RO_REG_27_ad_buck_state0_START  (0)
#define SOC_SCHARGER_BUCK_RO_REG_27_ad_buck_state0_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_BUCK_RO_REG_28_UNION
 结构说明  : BUCK_RO_REG_28 寄存器结构定义。地址偏移量:0x6C，初值:0x00，宽度:8
 寄存器说明: BUCK_只读寄存器_28
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  ad_buck_state1 : 8;  /* bit[0-7]: <7> ISO_A2D
                                                         <6> REGN_OK_D
                                                         <5> AVDDBAS_OK
                                                         <4> buck_en
                                                         <3> EN_DELAY_60US_BAS
                                                         <2>
                                                         <1>
                                                         <0>  */
    } reg;
} SOC_SCHARGER_BUCK_RO_REG_28_UNION;
#endif
#define SOC_SCHARGER_BUCK_RO_REG_28_ad_buck_state1_START  (0)
#define SOC_SCHARGER_BUCK_RO_REG_28_ad_buck_state1_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_OTG_CFG_REG_0_UNION
 结构说明  : OTG_CFG_REG_0 寄存器结构定义。地址偏移量:0x6D，初值:0x27，宽度:8
 寄存器说明: OTG_配置寄存器_0
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  da_otg_hmos     : 1;  /* bit[0-0]: OTG 上管控制选择
                                                          0：强制关闭
                                                          1：正常开关 */
        unsigned char  da_otg_dmd      : 1;  /* bit[1-1]: boost dmd使能：
                                                          0: 关闭过零检测
                                                          1: 使能过零检测 */
        unsigned char  da_otg_clp_l_en : 1;  /* bit[2-2]: boost EA钳位电路（低）使能：
                                                          0: 关闭钳位电路
                                                          1: 使能钳位电路 */
        unsigned char  da_otg_clp_h_en : 1;  /* bit[3-3]: boost EA钳位电路（高）使能：
                                                          0: 关闭钳位电路
                                                          1: 使能钳位电路 */
        unsigned char  da_otg_ckmin    : 2;  /* bit[4-5]: NMOS最小导通时间
                                                          00: 40ns
                                                          01: 30ns
                                                          10: 20ns
                                                          11: 15ns */
        unsigned char  da_otg_30ma     : 1;  /* bit[6-6]: OTG 30mA轻载高效模式开关
                                                          0：关闭
                                                          1：开启 */
        unsigned char  da_com_otg_mode : 1;  /* bit[7-7]: 1：OTG普通模式，有线通道开启
                                                          0：关闭OTG有线模式（只关闭OVP管） */
    } reg;
} SOC_SCHARGER_OTG_CFG_REG_0_UNION;
#endif
#define SOC_SCHARGER_OTG_CFG_REG_0_da_otg_hmos_START      (0)
#define SOC_SCHARGER_OTG_CFG_REG_0_da_otg_hmos_END        (0)
#define SOC_SCHARGER_OTG_CFG_REG_0_da_otg_dmd_START       (1)
#define SOC_SCHARGER_OTG_CFG_REG_0_da_otg_dmd_END         (1)
#define SOC_SCHARGER_OTG_CFG_REG_0_da_otg_clp_l_en_START  (2)
#define SOC_SCHARGER_OTG_CFG_REG_0_da_otg_clp_l_en_END    (2)
#define SOC_SCHARGER_OTG_CFG_REG_0_da_otg_clp_h_en_START  (3)
#define SOC_SCHARGER_OTG_CFG_REG_0_da_otg_clp_h_en_END    (3)
#define SOC_SCHARGER_OTG_CFG_REG_0_da_otg_ckmin_START     (4)
#define SOC_SCHARGER_OTG_CFG_REG_0_da_otg_ckmin_END       (5)
#define SOC_SCHARGER_OTG_CFG_REG_0_da_otg_30ma_START      (6)
#define SOC_SCHARGER_OTG_CFG_REG_0_da_otg_30ma_END        (6)
#define SOC_SCHARGER_OTG_CFG_REG_0_da_com_otg_mode_START  (7)
#define SOC_SCHARGER_OTG_CFG_REG_0_da_com_otg_mode_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_OTG_CFG_REG_1_UNION
 结构说明  : OTG_CFG_REG_1 寄存器结构定义。地址偏移量:0x6E，初值:0x0B，宽度:8
 寄存器说明: OTG_配置寄存器_1
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  da_otg_ocp_en       : 1;  /* bit[0-0]: boost OCP动作选择：
                                                              0: 过流后不限流
                                                              1: 过流后限流 */
        unsigned char  da_otg_lmos_ocp     : 2;  /* bit[1-2]: boost 低端MOS管限流值(A)
                                                              00: 2.0
                                                              01: 2.5
                                                              10: 3.0
                                                              11: 4.0 */
        unsigned char  da_otg_lim_set      : 2;  /* bit[3-4]: boost平均限流选择：
                                                              00：1.5A
                                                              01：1A
                                                              10：1A
                                                              11：0.5A */
        unsigned char  da_otg_lim_dcoffset : 3;  /* bit[5-7]: otg限流值dc偏移,500mA档位偏移100%，1A档位偏移50%，1.5A档位偏移33%
                                                              000:无偏移
                                                              001: 偏小6.25%
                                                              010: 偏小12.5%
                                                              011: 偏小18.75%
                                                              100：无偏移
                                                              101: 偏大6.25%
                                                              110: 偏大12.5%
                                                              111: 偏大18.75%  */
    } reg;
} SOC_SCHARGER_OTG_CFG_REG_1_UNION;
#endif
#define SOC_SCHARGER_OTG_CFG_REG_1_da_otg_ocp_en_START        (0)
#define SOC_SCHARGER_OTG_CFG_REG_1_da_otg_ocp_en_END          (0)
#define SOC_SCHARGER_OTG_CFG_REG_1_da_otg_lmos_ocp_START      (1)
#define SOC_SCHARGER_OTG_CFG_REG_1_da_otg_lmos_ocp_END        (2)
#define SOC_SCHARGER_OTG_CFG_REG_1_da_otg_lim_set_START       (3)
#define SOC_SCHARGER_OTG_CFG_REG_1_da_otg_lim_set_END         (4)
#define SOC_SCHARGER_OTG_CFG_REG_1_da_otg_lim_dcoffset_START  (5)
#define SOC_SCHARGER_OTG_CFG_REG_1_da_otg_lim_dcoffset_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_OTG_CFG_REG_2_UNION
 结构说明  : OTG_CFG_REG_2 寄存器结构定义。地址偏移量:0x6F，初值:0xC3，宽度:8
 寄存器说明: OTG_配置寄存器_2
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  da_otg_ovp_en    : 1;  /* bit[0-0]: boost ovp使能：
                                                           0: 关闭过压电路（不上报）
                                                           1: 使能过压电路（上报） */
        unsigned char  da_otg_osc_ckmax : 2;  /* bit[1-2]: 振荡器放电电流（uA）
                                                           00: 13
                                                           01: 14
                                                           10: 15
                                                           11: 16  */
        unsigned char  da_otg_osc       : 4;  /* bit[3-6]: 振荡器频率调节：
                                                           0000:2.5M
                                                           0001:2.3M
                                                           ……
                                                           1000:1.4M
                                                           1001:1.3M
                                                           ……
                                                           0000:0.6M */
        unsigned char  da_otg_ocp_sys   : 1;  /* bit[7-7]: boost OCP动作选择：
                                                           0: 过流后不上报
                                                           1: 过流后上报 */
    } reg;
} SOC_SCHARGER_OTG_CFG_REG_2_UNION;
#endif
#define SOC_SCHARGER_OTG_CFG_REG_2_da_otg_ovp_en_START     (0)
#define SOC_SCHARGER_OTG_CFG_REG_2_da_otg_ovp_en_END       (0)
#define SOC_SCHARGER_OTG_CFG_REG_2_da_otg_osc_ckmax_START  (1)
#define SOC_SCHARGER_OTG_CFG_REG_2_da_otg_osc_ckmax_END    (2)
#define SOC_SCHARGER_OTG_CFG_REG_2_da_otg_osc_START        (3)
#define SOC_SCHARGER_OTG_CFG_REG_2_da_otg_osc_END          (6)
#define SOC_SCHARGER_OTG_CFG_REG_2_da_otg_ocp_sys_START    (7)
#define SOC_SCHARGER_OTG_CFG_REG_2_da_otg_ocp_sys_END      (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_OTG_CFG_REG_3_UNION
 结构说明  : OTG_CFG_REG_3 寄存器结构定义。地址偏移量:0x70，初值:0x00，宽度:8
 寄存器说明: OTG_配置寄存器_3
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  da_otg_reserve1 : 8;  /* bit[0-7]: <7>：PFM基准电压选择
                                                          0：1.05 V
                                                          1: 1.075V
                                                          <6:4>otg 输出耦合电容，每档200f */
    } reg;
} SOC_SCHARGER_OTG_CFG_REG_3_UNION;
#endif
#define SOC_SCHARGER_OTG_CFG_REG_3_da_otg_reserve1_START  (0)
#define SOC_SCHARGER_OTG_CFG_REG_3_da_otg_reserve1_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_OTG_CFG_REG_4_UNION
 结构说明  : OTG_CFG_REG_4 寄存器结构定义。地址偏移量:0x71，初值:0x00，宽度:8
 寄存器说明: OTG_配置寄存器_4
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  da_otg_reserve2 : 8;  /* bit[0-7]: 寄存器预留 */
    } reg;
} SOC_SCHARGER_OTG_CFG_REG_4_UNION;
#endif
#define SOC_SCHARGER_OTG_CFG_REG_4_da_otg_reserve2_START  (0)
#define SOC_SCHARGER_OTG_CFG_REG_4_da_otg_reserve2_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_OTG_CFG_REG_5_UNION
 结构说明  : OTG_CFG_REG_5 寄存器结构定义。地址偏移量:0x72，初值:0xA6，宽度:8
 寄存器说明: OTG_配置寄存器_5
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  da_otg_slop_set : 2;  /* bit[0-1]: boost 斜坡补偿电容选择：
                                                          00: 1.5p
                                                          01: 2p
                                                          10: 2.5p
                                                          11: 3p */
        unsigned char  da_otg_slop_en  : 1;  /* bit[2-2]: boost 斜坡补偿使能：
                                                          0: 关闭斜坡补偿
                                                          1: 使能斜坡补偿 */
        unsigned char  da_otg_skip_set : 1;  /* bit[3-3]: skip周期内Pmos的开关状态：
                                                          0: PMOS关闭
                                                          1: PMOS开启 */
        unsigned char  da_otg_scp_time : 1;  /* bit[4-4]: boost scp时间选择：
                                                          0: 1ms内VOUT不足0.85BAT，上报短路
                                                          1: 2ms内VOUT不足0.85BAT，上报短路 */
        unsigned char  da_otg_scp_en   : 1;  /* bit[5-5]: boost scp动作选择：
                                                          0: 短路后系统不自动关闭（不上报）
                                                          1: 短路后系统自动关闭 */
        unsigned char  da_otg_rf       : 2;  /* bit[6-7]: 电感电流转电压阻抗
                                                          00: 0.5Ω（实际100kΩ）
                                                          01: 0.4Ω（实际80kΩ）
                                                          10: 0.3Ω（实际60kΩ）
                                                          11: 0.2Ω（实际40kΩ） */
    } reg;
} SOC_SCHARGER_OTG_CFG_REG_5_UNION;
#endif
#define SOC_SCHARGER_OTG_CFG_REG_5_da_otg_slop_set_START  (0)
#define SOC_SCHARGER_OTG_CFG_REG_5_da_otg_slop_set_END    (1)
#define SOC_SCHARGER_OTG_CFG_REG_5_da_otg_slop_en_START   (2)
#define SOC_SCHARGER_OTG_CFG_REG_5_da_otg_slop_en_END     (2)
#define SOC_SCHARGER_OTG_CFG_REG_5_da_otg_skip_set_START  (3)
#define SOC_SCHARGER_OTG_CFG_REG_5_da_otg_skip_set_END    (3)
#define SOC_SCHARGER_OTG_CFG_REG_5_da_otg_scp_time_START  (4)
#define SOC_SCHARGER_OTG_CFG_REG_5_da_otg_scp_time_END    (4)
#define SOC_SCHARGER_OTG_CFG_REG_5_da_otg_scp_en_START    (5)
#define SOC_SCHARGER_OTG_CFG_REG_5_da_otg_scp_en_END      (5)
#define SOC_SCHARGER_OTG_CFG_REG_5_da_otg_rf_START        (6)
#define SOC_SCHARGER_OTG_CFG_REG_5_da_otg_rf_END          (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_OTG_CFG_REG_6_UNION
 结构说明  : OTG_CFG_REG_6 寄存器结构定义。地址偏移量:0x73，初值:0x15，宽度:8
 寄存器说明: OTG_配置寄存器_6
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  otg_clp_l_set   : 1;  /* bit[0-0]: boost EA输出钳位值和VRAMP直流电平值的失调
                                                          0：27mv
                                                          1：40mv */
        unsigned char  da_wl_otg_mode  : 1;  /* bit[1-1]: 1：OTG无线模式，无线通道开启
                                                          0：关闭OTG无线模式（只关闭无线开关） */
        unsigned char  da_otg_uvp_en   : 1;  /* bit[2-2]: boost uvp使能：
                                                          0: 关闭欠压电路（不上报）
                                                          1: 使能欠压电路（上报） */
        unsigned char  da_otg_switch   : 1;  /* bit[3-3]: OTG SWITCH模式开关
                                                          0：关闭SWITCH
                                                          1：开启SWITCH */
        unsigned char  da_otg_pfm_v_en : 1;  /* bit[4-4]: boost pfm_v使能：
                                                          0: 关闭pfm
                                                          1: 使能pfm */
        unsigned char  reserved        : 3;  /* bit[5-7]: reserved */
    } reg;
} SOC_SCHARGER_OTG_CFG_REG_6_UNION;
#endif
#define SOC_SCHARGER_OTG_CFG_REG_6_otg_clp_l_set_START    (0)
#define SOC_SCHARGER_OTG_CFG_REG_6_otg_clp_l_set_END      (0)
#define SOC_SCHARGER_OTG_CFG_REG_6_da_wl_otg_mode_START   (1)
#define SOC_SCHARGER_OTG_CFG_REG_6_da_wl_otg_mode_END     (1)
#define SOC_SCHARGER_OTG_CFG_REG_6_da_otg_uvp_en_START    (2)
#define SOC_SCHARGER_OTG_CFG_REG_6_da_otg_uvp_en_END      (2)
#define SOC_SCHARGER_OTG_CFG_REG_6_da_otg_switch_START    (3)
#define SOC_SCHARGER_OTG_CFG_REG_6_da_otg_switch_END      (3)
#define SOC_SCHARGER_OTG_CFG_REG_6_da_otg_pfm_v_en_START  (4)
#define SOC_SCHARGER_OTG_CFG_REG_6_da_otg_pfm_v_en_END    (4)


/*****************************************************************************
 结构名    : SOC_SCHARGER_OTG_RO_REG_7_UNION
 结构说明  : OTG_RO_REG_7 寄存器结构定义。地址偏移量:0x74，初值:0x00，宽度:8
 寄存器说明: OTG_只读寄存器_7
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  ad_otg_on    : 1;  /* bit[0-0]: boost状态指示：
                                                       0：OTG关闭
                                                       1：OTG开启 */
        unsigned char  ad_otg_en_in : 1;  /* bit[1-1]: OTG使能的指示信号：
                                                       0：OTG没有使能；
                                                       1：OTG已经使能； */
        unsigned char  reserved     : 6;  /* bit[2-7]: reserved */
    } reg;
} SOC_SCHARGER_OTG_RO_REG_7_UNION;
#endif
#define SOC_SCHARGER_OTG_RO_REG_7_ad_otg_on_START     (0)
#define SOC_SCHARGER_OTG_RO_REG_7_ad_otg_on_END       (0)
#define SOC_SCHARGER_OTG_RO_REG_7_ad_otg_en_in_START  (1)
#define SOC_SCHARGER_OTG_RO_REG_7_ad_otg_en_in_END    (1)


/*****************************************************************************
 结构名    : SOC_SCHARGER_OTG_RO_REG_8_UNION
 结构说明  : OTG_RO_REG_8 寄存器结构定义。地址偏移量:0x75，初值:0x00，宽度:8
 寄存器说明: OTG_只读寄存器_8
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  ad_otg_state0 : 8;  /* bit[0-7]: <7> otg_en_ana
                                                        <6> otg_en_reg
                                                        <5> otg_en_dig
                                                        <4> WEAK_ON
                                                        <3> PWROK
                                                        <2> SSOK
                                                        <1> PWM
                                                        <0> PWM_DE */
    } reg;
} SOC_SCHARGER_OTG_RO_REG_8_UNION;
#endif
#define SOC_SCHARGER_OTG_RO_REG_8_ad_otg_state0_START  (0)
#define SOC_SCHARGER_OTG_RO_REG_8_ad_otg_state0_END    (7)


/*****************************************************************************
 结构名    : SOC_SCHARGER_OTG_RO_REG_9_UNION
 结构说明  : OTG_RO_REG_9 寄存器结构定义。地址偏移量:0x76，初值:0x00，宽度:8
 寄存器说明: OTG_只读寄存器_9
*****************************************************************************/
#ifndef __SOC_H_FOR_ASM__
typedef union
{
    unsigned char      value;
    struct
    {
        unsigned char  ad_otg_state1 : 8;  /* bit[0-7]: <7> LVC_MODE_EN
                                                        <6> SC_MODE_EN
                                                        <5> BLK_EN
                                                        <4> DC_OK_SHIELD
                                                        <3> CHG_EN
                                                        <2> OVP_EN
                                                        <1>
                                                        <0>  */
    } reg;
} SOC_SCHARGER_OTG_RO_REG_9_UNION;
#endif
#define SOC_SCHARGER_OTG_RO_REG_9_ad_otg_state1_START  (0)
#define SOC_SCHARGER_OTG_RO_REG_9_ad_otg_state1_END    (7)






/*****************************************************************************
  8 OTHERS定义
*****************************************************************************/



/*****************************************************************************
  9 全局变量声明
*****************************************************************************/


/*****************************************************************************
  10 函数声明
*****************************************************************************/


#ifdef __cplusplus
    #if __cplusplus
        }
    #endif
#endif

#endif /* end of soc_scharger_interface.h */
