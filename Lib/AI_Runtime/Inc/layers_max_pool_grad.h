/**
  ******************************************************************************
  * @file    layers_max_pool_grad.h
  * @author  STMicroelectronics
  * @brief   header file of AI platform max pool gradient layer datatype
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

#ifndef LAYERS_MAX_POOL_GRAD_H
#define LAYERS_MAX_POOL_GRAD_H
#pragma once

#include "layers_common.h"

AI_API_DECLARE_BEGIN

/*!
 * @struct ai_layer_max_pool_grad
 * @ingroup layers
 * @brief ai_layer_max_pool_grad layer
 */
typedef AI_ALIGNED_TYPE(struct, 4) ai_layer_max_pool_grad_ {
  AI_LAYER_COMMON_FIELDS_DECLARE
  ai_shape_2d kernel_shape; /*!< pooling kernel shape (h, w) */
  ai_shape_2d strides;      /*!< pooling strides (h, w) */
  ai_shape_2d dilations;    /*!< pooling dilations */
  ai_shape pads;            /*!< pooling pads [top, left, bottom, right] */
  ai_u8 ceil_mode;          /*!< 0: floor, 1: ceil */
  ai_u8 storage_order;      /*!< ONNX storage_order, only 0 supported */
} ai_layer_max_pool_grad;

/*!
 * @brief Computes the MaxPool input gradient using downstream gradient and mask indices.
 * @ingroup layers
 * @param layer the layer including output and input tensors
 */
AI_INTERNAL_API
void backward_max_pool(ai_layer* layer);

AI_API_DECLARE_END

#endif    /*LAYERS_MAX_POOL_GRAD_H*/
