
/******************************************************************************
 * Copyright (c) 2017, STMicroelectronics - All Rights Reserved

 This file is part of VL53L1 Core and is dual licensed,
 either 'STMicroelectronics
 Proprietary license'
 or 'BSD 3-clause "New" or "Revised" License' , at your option.

 ******************************************************************************

 'STMicroelectronics Proprietary license'

 *******************************************************************************

 License terms: STMicroelectronics Proprietary in accordance with licensing
 terms at www.st.com/sla0081

 STMicroelectronics confidential
 Reproduction and Communication of this document is strictly prohibited unless
 specifically authorized in writing by STMicroelectronics.


 *******************************************************************************

 Alternatively, VL53L1 Core may be distributed under the terms of
 'BSD 3-clause "New" or "Revised" License', in which case the following
 provisions apply instead of the ones mentioned above :

 *******************************************************************************

 License terms: BSD 3-clause "New" or "Revised" License.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:

 1. Redistributions of source code must retain the above copyright notice, this
 list of conditions and the following disclaimer.

 2. Redistributions in binary form must reproduce the above copyright notice,
 this list of conditions and the following disclaimer in the documentation
 and/or other materials provided with the distribution.

 3. Neither the name of the copyright holder nor the names of its contributors
 may be used to endorse or promote products derived from this software
 without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


 *******************************************************************************
 */

#ifndef _VL53L1_PRESET_SETUP_H_
#define _VL53L1_PRESET_SETUP_H_

#ifdef __cplusplus
extern "C"
{
#endif

/* indexes for the bare driver tuning setting API function */
enum VL53L1_Tuning_t {
	VL53L1_TUNING_VERSION = 0,
	VL53L1_TUNING_PROXY_MIN,
	VL53L1_TUNING_SINGLE_TARGET_XTALK_TARGET_DISTANCE_MM,
	VL53L1_TUNING_SINGLE_TARGET_XTALK_SAMPLE_NUMBER,
	VL53L1_TUNING_MIN_AMBIENT_DMAX_VALID,
	VL53L1_TUNING_MAX_SIMPLE_OFFSET_CALIBRATION_SAMPLE_NUMBER,
	VL53L1_TUNING_XTALK_FULL_ROI_TARGET_DISTANCE_MM,
	VL53L1_TUNING_SIMPLE_OFFSET_CALIBRATION_REPEAT,
	VL53L1_TUNING_XTALK_FULL_ROI_BIN_SUM_MARGIN,
	VL53L1_TUNING_XTALK_FULL_ROI_DEFAULT_OFFSET,
	VL53L1_TUNING_ZERO_DISTANCE_OFFSET_NON_LINEAR_FACTOR,
	VL53L1_TUNING_MAX_TUNABLE_KEY
};

/* default values for the tuning settings parameters */
#define TUNING_VERSION	0x0007

#define TUNING_PROXY_MIN -30 /* min distance in mm */
#define TUNING_SINGLE_TARGET_XTALK_TARGET_DISTANCE_MM 600
/* Target distance in mm for single target Xtalk */
#define TUNING_SINGLE_TARGET_XTALK_SAMPLE_NUMBER 50
/* Number of sample used for single target Xtalk */
#define TUNING_MIN_AMBIENT_DMAX_VALID 8
/* Minimum ambient level to state the Dmax returned by the device is valid */
#define TUNING_MAX_SIMPLE_OFFSET_CALIBRATION_SAMPLE_NUMBER 10
/* Maximum loops to perform simple offset calibration */
#define TUNING_XTALK_FULL_ROI_TARGET_DISTANCE_MM 600
/* Target distance in mm for target Xtalk from Bins method*/
#define TUNING_SIMPLE_OFFSET_CALIBRATION_REPEAT 3
/* Number of loops done during the simple offset calibration*/
#define TUNING_ZERO_DISTANCE_OFFSET_NON_LINEAR_FACTOR_DEFAULT 9
/* zero distance offset calibration non linear compensation default value */

/* The following settings are related to the fix for ticket EwokP #558410 */
#define TUNING_XTALK_FULL_ROI_BIN_SUM_MARGIN 24
/* Acceptance margin for the xtalk_shape bin_data sum computation */
#define TUNING_XTALK_FULL_ROI_DEFAULT_OFFSET 50
/* Recovery value for Xtalk compensation plane offset in kcps */
/* 50 stands for ~0.10 kcps cover glass in 7.9 format */
/* End of settings related to the fix for ticket EwokP #558410 */

#ifdef __cplusplus
}
#endif

#endif /* _VL53L1_PRESET_SETUP_H_ */
