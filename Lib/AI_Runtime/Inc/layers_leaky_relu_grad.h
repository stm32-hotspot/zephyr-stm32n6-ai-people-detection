/**
  ******************************************************************************
  * @file    layers_leaky_relu_grad.h
  * @author  STMicroelectronics
  * @brief   header file of AI platform leaky relu gradient layer datatype
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

#ifndef LAYERS_LEAKY_RELU_GRAD_H
#define LAYERS_LEAKY_RELU_GRAD_H
#pragma once

#include "layers_common.h"

/*!
 * @defgroup layers leaky_relu_grad Layer Definitions
 * @brief definition
 *
 */

AI_API_DECLARE_BEGIN

/*!
 * @struct ai_layer_leaky_relu_grad
 * @ingroup layers
 * @brief ai_layer_leaky_relu_grad layer
 */
typedef AI_ALIGNED_TYPE(struct, 4) ai_layer_leaky_relu_grad_ {
  AI_LAYER_COMMON_FIELDS_DECLARE
  ai_float alpha; /*!< negative slope */
} ai_layer_leaky_relu_grad;

/*!
 * @brief Computes the downstream gradient for LeakyReLU.
 * @ingroup layers
 * @param layer the layer including output and input tensors
 */
AI_INTERNAL_API
void backward_leaky_relu(ai_layer* layer);

AI_API_DECLARE_END

#endif    /*LAYERS_LEAKY_RELU_GRAD_H*/
