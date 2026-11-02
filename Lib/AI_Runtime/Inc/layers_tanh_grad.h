/**
  ******************************************************************************
  * @file    layers_tanh_grad.h
  * @author  STMicroelectronics
  * @brief   header file of AI platform tanh gradient layer datatype
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
  @verbatim
  @endverbatim
  ******************************************************************************
  */

#ifndef LAYERS_TANH_GRAD_H
#define LAYERS_TANH_GRAD_H
#pragma once

#include "layers_common.h"

AI_API_DECLARE_BEGIN

typedef AI_ALIGNED_TYPE(struct, 4) ai_layer_tanh_grad_ {
  AI_LAYER_COMMON_FIELDS_DECLARE
} ai_layer_tanh_grad;

AI_INTERNAL_API
void backward_tanh(ai_layer* layer);

AI_API_DECLARE_END

#endif    /*LAYERS_TANH_GRAD_H*/
