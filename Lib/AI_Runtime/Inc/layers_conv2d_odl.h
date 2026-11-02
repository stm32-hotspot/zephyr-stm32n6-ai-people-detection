/**
  ******************************************************************************
  * @file    layers_conv2dodl.h
  * @author  STMicroelectronics
  * @brief   header file of AI platform conv2dodl layer datatype
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 - 2026 STMicroelectronics.
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

#ifndef LAYERS_CONV2D_ODL_H
#define LAYERS_CONV2D_ODL_H
#pragma once

#include "layers_common.h"


#define AI_LAYER_CONV_2D_ODL_FIELDS_DECLARE \
  AI_LAYER_COMMON_FIELDS_DECLARE \
  ai_u32      groups;                   /*!< groups for separable convolution */ \
  AI_CONST ai_array*  nl_params;        /*!< array pointer to non linear parameters */ \
  ai_shape_2d filter_stride;            /*!< filter stride, how much the filter moves */ \
  ai_shape_2d dilation;                 /*!< dilation value along axis of the filter */ \
  ai_shape    filter_pad;               /*!< filter pad 4d */ \
  ai_layer_format_type in_ch_format;    /*!< Input format  (Channel 1st vs Channel last */ \
  ai_layer_format_type out_ch_format;   /*!< Output format (Channel 1st vs Channel last */ 

/*!
 * @defgroup layers CONV_2D_ODL Layer Definitions
 * @brief definition 
 *
 */

AI_API_DECLARE_BEGIN

/*!
 * @struct ai_layer_conv_2d_odl
 * @ingroup layers_conv_2d_odl
 * @brief conv_2d_odl layer
 */
typedef AI_ALIGNED_TYPE(struct, 4) ai_layer_conv_2d_odl {
  AI_LAYER_CONV_2D_ODL_FIELDS_DECLARE
} ai_layer_conv_2d_odl;

/*!
 * @brief Computes the activations of a CONV2DODL layer.
 * @ingroup layers
 * @param layer the layer including output and input tensors
 */
AI_INTERNAL_API
void forward_lite_conv2d_odl_if32of32(ai_layer* layer);

AI_API_DECLARE_END


#define AI_LAYER_CONV_2D_GRAD_FIELDS_DECLARE \
  AI_LAYER_COMMON_FIELDS_DECLARE \
  ai_u32      groups;                   /*!< groups for separable convolution */ \
  AI_CONST ai_array*  nl_params;        /*!< array pointer to non linear parameters */ \
  ai_shape_2d filter_stride;            /*!< filter stride, how much the filter moves */ \
  ai_shape_2d dilation;                 /*!< dilation value along axis of the filter */ \
  ai_shape    filter_pad;               /*!< filter pad 4d */ \
  ai_layer_format_type in_ch_format;    /*!< Input format  (Channel 1st vs Channel last */ \
  ai_layer_format_type out_ch_format;   /*!< Output format (Channel 1st vs Channel last */ 

/*!
 * @defgroup layers Conv2D Grad Layer Definitions
 * @brief definition 
 *
 */

AI_API_DECLARE_BEGIN

/*!
 * @struct ai_layer_conv_2d_grad
 * @ingroup layers_conv_2d_grad
 * @brief conv2d grad layer
 */
typedef AI_ALIGNED_TYPE(struct, 4) ai_layer_conv_2d_grad_ {
  AI_LAYER_CONV_2D_GRAD_FIELDS_DECLARE
} ai_layer_conv_2d_grad;

/*!
 * @brief Computes the back prop of conv
 * @ingroup layers
 * @param layer the layer including output and input tensors
 */
AI_INTERNAL_API
void backward_lite_conv2d_grad_if32of32(ai_layer* layer);

AI_API_DECLARE_END

#endif    /*LAYERS_CONV2DODL_H*/
