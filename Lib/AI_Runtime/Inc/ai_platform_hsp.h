/**
  ******************************************************************************
  * @file    ai_platform_hsp.h
  * @author  STMicroelectronics
  * @brief   Implementation of AI platform hsp handler configuration
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
#ifndef AI_PLATFORM_HSP_H
#define AI_PLATFORM_HSP_H

#include "ai_platform_interface.h"
#include "hsp_api_def.h"

AI_API_DECLARE_BEGIN

AI_INTERFACE_TYPE
ai_bool ai_platform_hsp_init_default_if_needed(void);

AI_INTERFACE_TYPE
void ai_platform_hsp_deinit(void);

AI_INTERFACE_TYPE
ai_bool ai_platform_hsp_set(hsp_engine_context_t *phsp_engine_context);

AI_INTERFACE_TYPE
hsp_engine_context_t * ai_platform_hsp_get(void);

AI_INTERFACE_TYPE
ai_bool ai_platform_hsp_is_set(void);

AI_API_DECLARE_END

#endif /* AI_PLATFORM_HSP_H */
