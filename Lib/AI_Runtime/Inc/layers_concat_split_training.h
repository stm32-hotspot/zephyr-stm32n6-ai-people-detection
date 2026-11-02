/**
  ******************************************************************************
  * @file    layers_concat_split_training.h
  * @author  STMicroelectronics
  * @brief   header file of AI platform concat and split training layer datatype
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

#ifndef LAYERS_CONCAT_SPLIT_TRAINING_H
#define LAYERS_CONCAT_SPLIT_TRAINING_H
#pragma once

#include "layers_common.h"

/*!
 * @defgroup layers concat_training Layer Definitions
 * @brief definition
 *
 */

AI_API_DECLARE_BEGIN

/*!
 * @struct ai_layer_concat_training
 * @ingroup layers
 * @brief ai_layer_concat_training layer
 */
typedef AI_ALIGNED_TYPE(struct, 4) ai_layer_concat_training_ {
  AI_LAYER_COMMON_FIELDS_DECLARE
  ai_shape_dimension axis;       /*!< which axis to concatenate on */
} ai_layer_concat_training;

/*!
 * @brief Computes the outputs for ConcatTraining.
 * @ingroup layers
 * @param layer the layer including output and input tensors
 */
AI_INTERNAL_API
void forward_concat_training(ai_layer* layer);


/*!
 * @struct ai_layer_split_training
 * @ingroup layers
 * @brief SplitTraining layer definition
 */

typedef AI_ALIGNED_TYPE(struct, 4) ai_layer_split_training_ {
    AI_LAYER_COMMON_FIELDS_DECLARE
    const ai_i32              outer_elems;
    const ai_i32              outer_elems_stride;
} ai_layer_split_training;

/*!
 * @brief Split network computation in N parallel branches.
 * @ingroup layers
 * @param layer the split layer
 */
AI_INTERNAL_API
void forward_split_training(ai_layer* layer);


AI_API_DECLARE_END

#endif    /*LAYERS_CONCAT_SPLIT_TRAINING_H*/
