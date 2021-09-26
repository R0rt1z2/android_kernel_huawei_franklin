/******************************************************************************
 * Copyright (c) 2019, STMicroelectronics - All Rights Reserved

 This file is part of VL53LX Core and is dual licensed,
 either 'STMicroelectronics Proprietary license'
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

 Alternatively, VL53LX Core may be distributed under the terms of
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
/**
 * @file   vl53l1_platform_ipp.c
 *
 * @brief  EwokPlus25 IPP Wrapper Layer
 */

#include "vl53l1_platform.h"
#include "vl53l1_platform_ipp.h"
#include "vl53l1_ll_def.h"
#include "vl53l1_hist_structs.h"
#include "vl53l1_hist_funcs.h"
#include "vl53l1_xtalk.h"


#define LOG_FUNCTION_START(fmt, ...) \
	_LOG_FUNCTION_START(VL53L1_TRACE_MODULE_CORE, fmt, ##__VA_ARGS__)
#define LOG_FUNCTION_END(status, ...) \
	_LOG_FUNCTION_END(VL53L1_TRACE_MODULE_CORE, status, ##__VA_ARGS__)


VL53L1_Error VL53L1_ipp_hist_process_data(
	VL53L1_DEV                         Dev,
	VL53L1_dmax_calibration_data_t    *pdmax_cal,
	VL53L1_hist_gen3_dmax_config_t    *pdmax_cfg,
	VL53L1_hist_post_process_config_t *ppost_cfg,
	VL53L1_histogram_bin_data_t       *pbins,
	VL53L1_xtalk_histogram_data_t     *pxtalk,
	uint8_t                           *pArea1,
	uint8_t                           *pArea2,
	uint8_t                           *phisto_merge_nb,
	VL53L1_range_results_t            *presults)
{

	/*
	 * IPP wrapper for histogram post processing function
	 */

	VL53L1_Error status         = VL53L1_ERROR_NONE;

	SUPPRESS_UNUSED_WARNING(Dev);

	status =
		VL53L1_hist_process_data(
			pdmax_cal,
			pdmax_cfg,
			ppost_cfg,
			pbins,
			pxtalk,
			pArea1,
			pArea2,
			presults,
			phisto_merge_nb);

	return status;
}


VL53L1_Error VL53L1_ipp_hist_ambient_dmax(
	VL53L1_DEV                         Dev,
	uint16_t                           target_reflectance,
	VL53L1_dmax_calibration_data_t    *pdmax_cal,
	VL53L1_hist_gen3_dmax_config_t    *pdmax_cfg,
	VL53L1_histogram_bin_data_t       *pbins,
	int16_t                           *pambient_dmax_mm)
{

    /*
     * IPP wrapper for histogram ambient DMAX function
     *
     * The target reflectance in percent for the DMAX calculation
	 * is set by target_reflectance input
	 *
	 * The fixed point format is 7.2
     */

    VL53L1_Error status         = VL53L1_ERROR_NONE;

	SUPPRESS_UNUSED_WARNING(Dev);

    status =
    	VL53L1_hist_ambient_dmax(
    		target_reflectance,
    		pdmax_cal,
    		pdmax_cfg,
			pbins,
			pambient_dmax_mm);

	return status;
}


VL53L1_Error VL53L1_ipp_xtalk_calibration_process_data(
	VL53L1_DEV                          Dev,
	VL53L1_xtalk_range_results_t       *pxtalk_ranges,
	VL53L1_xtalk_histogram_data_t      *pxtalk_shape,
	VL53L1_xtalk_calibration_results_t *pxtalk_cal)
{

	/*
	 * IPP wrapper for histogram post processing function
	 */

	VL53L1_Error status         = VL53L1_ERROR_NONE;

	SUPPRESS_UNUSED_WARNING(Dev);

	status =
		VL53L1_xtalk_calibration_process_data(
			pxtalk_ranges,
			pxtalk_shape,
			pxtalk_cal);

	return status;
}


