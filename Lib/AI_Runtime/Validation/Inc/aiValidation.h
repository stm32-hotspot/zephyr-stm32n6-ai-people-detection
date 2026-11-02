/**
 ******************************************************************************
 * @file    aiValidation.h
 * @author  STMicroelectronics
 * @brief   AI Validation application
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 - 2026 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software is licensed under terms that can be found in the LICENSE file in
 * the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

#ifndef __AI_VALIDATION_H__
#define __AI_VALIDATION_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

int aiValidationInit(void);
int aiValidationProcess(void);
void aiValidationDeInit(void);

#ifdef __cplusplus
}
#endif

#endif /* __AI_VALIDATION_H__ */
