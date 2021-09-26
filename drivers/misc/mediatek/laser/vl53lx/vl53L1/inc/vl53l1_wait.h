
/*******************************************************************************
 This file is part of VL53L1 Core

 Copyright (c) 2020, STMicroelectronics - All Rights Reserved

 License terms: STMicroelectronics Proprietary in accordance with licensing
 terms at www.st.com/sla0081

 STMicroelectronics confidential
 Reproduction and Communication of this document is strictly prohibited unless
 specifically authorized in writing by STMicroelectronics.

*/




#ifndef _VL53L1_WAIT_H_
#define _VL53L1_WAIT_H_

#include "vl53l1_platform.h"

#ifdef __cplusplus
extern "C" {
#endif




VL53L1_Error VL53L1_wait_for_boot_completion(
	VL53L1_DEV      Dev);




VL53L1_Error VL53L1_wait_for_firmware_ready(
	VL53L1_DEV      Dev);




VL53L1_Error VL53L1_wait_for_range_completion(
	VL53L1_DEV   Dev);




VL53L1_Error VL53L1_wait_for_test_completion(
	VL53L1_DEV   Dev);






VL53L1_Error VL53L1_is_boot_complete(
	VL53L1_DEV      Dev,
	uint8_t        *pready);



VL53L1_Error VL53L1_is_firmware_ready(
	VL53L1_DEV      Dev,
	uint8_t        *pready);




VL53L1_Error VL53L1_is_new_data_ready(
	VL53L1_DEV      Dev,
	uint8_t        *pready);






VL53L1_Error VL53L1_poll_for_boot_completion(
	VL53L1_DEV      Dev,
	uint32_t        timeout_ms);




VL53L1_Error VL53L1_poll_for_firmware_ready(
	VL53L1_DEV      Dev,
	uint32_t        timeout_ms);




VL53L1_Error VL53L1_poll_for_range_completion(
	VL53L1_DEV   Dev,
	uint32_t     timeout_ms);



#ifdef __cplusplus
}
#endif

#endif

