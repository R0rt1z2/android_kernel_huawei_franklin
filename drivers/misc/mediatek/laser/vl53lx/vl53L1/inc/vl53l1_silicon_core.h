
/*******************************************************************************
 This file is part of VL53L1 Core

 Copyright (c) 2020, STMicroelectronics - All Rights Reserved

 License terms: STMicroelectronics Proprietary in accordance with licensing
 terms at www.st.com/sla0081

 STMicroelectronics confidential
 Reproduction and Communication of this document is strictly prohibited unless
 specifically authorized in writing by STMicroelectronics.

*/




#ifndef _VL53L1_SILICON_CORE_H_
#define _VL53L1_SILICON_CORE_H_

#include "vl53l1_platform.h"

#ifdef __cplusplus
extern "C" {
#endif




VL53L1_Error VL53L1_is_firmware_ready_silicon(
	VL53L1_DEV      Dev,
	uint8_t        *pready);


#ifdef __cplusplus
}
#endif

#endif

