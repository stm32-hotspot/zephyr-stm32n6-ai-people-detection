/**
  ******************************************************************************
  * @file    core_convert.h
  * @author  STMicroelectronics
  * @brief   header file of core utils routines
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2018 - 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
#ifndef CORE_CONVERT_H
#define CORE_CONVERT_H

#include "ai_platform.h"
#include "ai_platform_interface.h"

#include "core_common.h"

AI_API_DECLARE_BEGIN

/*!
 * @defgroup core_convert Core Convert Routines
 * @brief Implementation of core node format convertion routines
 *   (Q7 to float, ... etc.)
 */


/*!
 * @brief Convert tensors from float to quantized or viceversa
 * @ingroup core_convert
 * @param[in] pNode in a handler to node (layer or operator)
 */
AI_INTERNAL_API
void node_convert(ai_node *pNode);

/*!
 * @brief Convert integer tensors between QM.N formats (8/16 bits)
 * @ingroup core_convert
 * @param[in] pNode in a handler to node (layer or operator)
 */
AI_INTERNAL_API
void node_convert_fixed(ai_node *pNode);

/*!
 * @brief Convert integer tensors between signed and usigned (int8/uint8) formats
 * @ingroup core_convert
 * @param[in] pNode in a handler to node (layer or operator)
 */
AI_INTERNAL_API
void node_convert_integer(ai_node *pNode);

/*!
 * @brief Convert float tensor to binary
 * @ingroup core_convert
 * @param[in] pNode in a handler to node (layer or operator)
 */
AI_INTERNAL_API
void node_convert_if32os1(ai_node *pNode);


/*!
 * @brief Convert binary tensor to float
 * @ingroup core_convert
 * @param[in] pNode in a handler to node (layer or operator)
 */
AI_INTERNAL_API
void node_convert_is8os1(ai_node *pNode);


/*!
 * @brief Convert binary tensor to signed int 8 bit
 * @ingroup core_convert
 * @param[in] pNode in a handler to node (layer or operator)
 */
AI_INTERNAL_API
void node_convert_is1os8(ai_node *pNode);


/*!
 * @brief Convert binary tensor to signed int 16 bit
 * @ingroup core_convert
 * @param[in] pNode in a handler to node (layer or operator)
 */
AI_INTERNAL_API
void node_convert_is1os16(ai_node *pNode);


/*!
 * @brief Convert binary tensor to float
 * @ingroup core_convert
 * @param[in] pNode in a handler to node (layer or operator)
 */
AI_INTERNAL_API
void node_convert_is1of32(ai_node *pNode);


/*!
 * @brief Convert signed int 16 bit tensor to float
 * @ingroup core_convert
 * @param[in] pNode in a handler to node (layer or operator)
 */
AI_INTERNAL_API
void node_convert_is16of32(ai_node *pNode);


/*!
 * @brief Convert unsigned int 16 bit tensor to float
 * @ingroup core_convert
 * @param[in] pNode in a handler to node (layer or operator)
 */
AI_INTERNAL_API
void node_convert_iu16of32(ai_node *pNode);


/*!
 * @brief Convert float tensor to signed int 16 bit
 * @ingroup core_convert
 * @param[in] pNode in a handler to node (layer or operator)
 */
AI_INTERNAL_API
void node_convert_if32os16(ai_node *pNode);


/*!
 * @brief Convert float tensor to unsigned int 16 bit
 * @ingroup core_convert
 * @param[in] pNode in a handler to node (layer or operator)
 */
AI_INTERNAL_API
void node_convert_if32ou16(ai_node *pNode);


/*!
 * @brief Convert unsigned int 16 bit tensor to signed int 16 bit
 * @ingroup core_convert
 * @param[in] pNode in a handler to node (layer or operator)
 */
AI_INTERNAL_API
void node_convert_iu16os16(ai_node *pNode);


/*!
 * @brief Convert unsigned int 16 bit tensor to unsigned int 16 bit
 * @ingroup core_convert
 * @param[in] pNode in a handler to node (layer or operator)
 */
AI_INTERNAL_API
void node_convert_iu16ou16(ai_node *pNode);


/*!
 * @brief Convert signed int 16 bit tensor to unsigned int 16 bit
 * @ingroup core_convert
 * @param[in] pNode in a handler to node (layer or operator)
 */
AI_INTERNAL_API
void node_convert_is16ou16(ai_node *pNode);


/*!
 * @brief Convert signed int 16 bit tensor to signed int 16 bit
 * @ingroup core_convert
 * @param[in] pNode in a handler to node (layer or operator)
 */
AI_INTERNAL_API
void node_convert_is16os16(ai_node *pNode);


/*!
 * @brief Convert signed int 8 bit tensor to signed int 16 bit
 * @ingroup core_convert
 * @param[in] pNode in a handler to node (layer or operator)
 */
AI_INTERNAL_API
void node_convert_is8os16(ai_node *pNode);


/*!
 * @brief Convert a shape struct into a stride struct
 * @ingroup core_convert
 * @param[in] in a pointer to a shape to convert
 * @return a condverted stride datastruct
 */
AI_INTERNAL_API
void core_shape_to_stride(ai_stride* out, const ai_shape* in);

#ifdef AI_FLOAT16_TYPE_SUPPORTED
/*!
 * @brief Convert float tensor to float 16 bit
 * @ingroup core_convert
 * @param[in] pNode in a handler to node (layer or operator)
 */
AI_INTERNAL_API
void node_convert_if32of16(ai_node *pNode);
/*!
 * @brief Convert float16 tensor to float32
 * @ingroup core_convert
 * @param[in] pNode in a handler to node (layer or operator)
 */
AI_INTERNAL_API
void node_convert_if16of32(ai_node *pNode);

/*!
 * @brief Convert signed int8 tensor to float16
 * @ingroup core_convert
 * @param[in] pNode in a handler to node (layer or operator)
 */
AI_INTERNAL_API
void node_convert_is8of16(ai_node *pNode);

/*!
 * @brief Convert unsigned int8 tensor to float16
 * @ingroup core_convert
 * @param[in] pNode in a handler to node (layer or operator)
 */
AI_INTERNAL_API
void node_convert_iu8of16(ai_node *pNode);

/*!
 * @brief Convert float16 tensor to signed int8
 * @ingroup core_convert
 * @param[in] pNode in a handler to node (layer or operator)
 */
AI_INTERNAL_API
void node_convert_if16os8(ai_node *pNode);

/*!
 * @brief Convert float16 tensor to unsigned int8
 * @ingroup core_convert
 * @param[in] pNode in a handler to node (layer or operator)
 */
AI_INTERNAL_API
void node_convert_if16ou8(ai_node *pNode);

/*!
 * @brief Convert binary tensor to float
 * @ingroup core_convert
 * @param[in] pNode in a handler to node (layer or operator)
 */
AI_INTERNAL_API
void node_convert_is1of16(ai_node *pNode);


/*!
 * @brief Convert signed int 16 bit tensor to float
 * @ingroup core_convert
 * @param[in] pNode in a handler to node (layer or operator)
 */
AI_INTERNAL_API
void node_convert_is16of16(ai_node *pNode);


/*!
 * @brief Convert unsigned int 16 bit tensor to float
 * @ingroup core_convert
 * @param[in] pNode in a handler to node (layer or operator)
 */
AI_INTERNAL_API
void node_convert_iu16of16(ai_node *pNode);


/*!
 * @brief Convert float tensor to signed int 16 bit
 * @ingroup core_convert
 * @param[in] pNode in a handler to node (layer or operator)
 */
AI_INTERNAL_API
void node_convert_if16os16(ai_node *pNode);


/*!
 * @brief Convert float tensor to unsigned int 16 bit
 * @ingroup core_convert
 * @param[in] pNode in a handler to node (layer or operator)
 */
AI_INTERNAL_API
void node_convert_if16ou16(ai_node *pNode);

#endif /*  AI_FLOAT16_TYPE_SUPPORTED */

#endif    /*CORE_CONVERT_H*/
