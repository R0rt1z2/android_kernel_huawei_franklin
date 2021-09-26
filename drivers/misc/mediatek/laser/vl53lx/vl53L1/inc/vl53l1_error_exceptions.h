
/*******************************************************************************
 This file is part of VL53L1 Core

 Copyright (c) 2020, STMicroelectronics - All Rights Reserved

 License terms: STMicroelectronics Proprietary in accordance with licensing
 terms at www.st.com/sla0081

 STMicroelectronics confidential
 Reproduction and Communication of this document is strictly prohibited unless
 specifically authorized in writing by STMicroelectronics.

*/




#ifndef _VL53L1_ERROR_EXCEPTIONS_H_
#define _VL53L1_ERROR_EXCEPTIONS_H_

#define IGNORE_DIVISION_BY_ZERO                                0

#define IGNORE_XTALK_EXTRACTION_NO_SAMPLE_FAIL                 0
#define IGNORE_XTALK_EXTRACTION_SIGMA_LIMIT_FAIL               0
#define IGNORE_XTALK_EXTRACTION_NO_SAMPLE_FOR_GRADIENT_WARN    0
#define IGNORE_XTALK_EXTRACTION_SIGMA_LIMIT_FOR_GRADIENT_WARN  0
#define IGNORE_XTALK_EXTRACTION_MISSING_SAMPLES_WARN           0

#define IGNORE_REF_SPAD_CHAR_NOT_ENOUGH_SPADS                  0
#define IGNORE_REF_SPAD_CHAR_RATE_TOO_HIGH                     0
#define IGNORE_REF_SPAD_CHAR_RATE_TOO_LOW                      0

#define IGNORE_OFFSET_CAL_MISSING_SAMPLES                      0
#define IGNORE_OFFSET_CAL_SIGMA_TOO_HIGH                       0
#define IGNORE_OFFSET_CAL_RATE_TOO_HIGH                        0
#define IGNORE_OFFSET_CAL_SPAD_COUNT_TOO_LOW				   0

#define IGNORE_ZONE_CAL_MISSING_SAMPLES                        0
#define IGNORE_ZONE_CAL_SIGMA_TOO_HIGH                         0
#define IGNORE_ZONE_CAL_RATE_TOO_HIGH                          0

#endif

