
/*******************************************************************************
 This file is part of VL53L1 Core

 Copyright (c) 2020, STMicroelectronics - All Rights Reserved

 License terms: STMicroelectronics Proprietary in accordance with licensing
 terms at www.st.com/sla0081

 STMicroelectronics confidential
 Reproduction and Communication of this document is strictly prohibited unless
 specifically authorized in writing by STMicroelectronics.

*/





#ifndef _VL53L1_NVM_DEBUG_H_
#define _VL53L1_NVM_DEBUG_H_

#include "vl53l1_ll_def.h"
#include "vl53l1_nvm_structs.h"



#ifdef __cplusplus
extern "C"
{
#endif

#ifdef VL53L1_LOG_ENABLE



void VL53L1_print_nvm_raw_data(
	uint8_t                       *pnvm_raw_data,
	uint32_t                       trace_flags);




void VL53L1_print_decoded_nvm_data(
	VL53L1_decoded_nvm_data_t *pdata,
	char                      *pprefix,
	uint32_t                   trace_flags);




void VL53L1_print_decoded_nvm_fmt_range_data(
	VL53L1_decoded_nvm_fmt_range_data_t *pdata,
	char                                *pprefix,
	uint32_t                             trace_flags);




void VL53L1_print_decoded_nvm_fmt_info(
	VL53L1_decoded_nvm_fmt_info_t *pdata,
	char                          *pprefix,
	uint32_t                       trace_flags);



void VL53L1_print_decoded_nvm_ews_info(
	VL53L1_decoded_nvm_ews_info_t *pdata,
	char                          *pprefix,
	uint32_t                       trace_flags);

#endif

#ifdef __cplusplus
}
#endif

#endif

