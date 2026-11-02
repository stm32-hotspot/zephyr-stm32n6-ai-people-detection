/**
  ******************************************************************************
  * @file    lite_internal_apis.h
  * @author  STMicroelectronics
  * @brief
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
  */

#ifndef LITE_INTERNAL_APIS
#define LITE_INTERNAL_APIS

#include "ai_platform.h"
#include "ai_lite_interface.h"

#ifdef AI_FLOAT16_TYPE_SUPPORTED

/**
 * @brief Applies the ELU non-linearity to a float16 array.
 * @param src Input float16 array.
 * @param dst Output float16 array.
 * @param in_size Number of elements to process.
 * @param alpha Pointer to the ELU alpha parameter.
 */
LITE_API_ENTRY
void forward_lite_nl_elu_if16of16(const ai_f16* src, ai_f16* dst,
                  const ai_size in_size, const ai_f16* alpha);

/**
 * @brief Applies the ReLU non-linearity to a float16 array.
 * @param src Input float16 array.
 * @param dst Output float16 array.
 * @param in_size Number of elements to process.
 */
LITE_API_ENTRY
void forward_lite_nl_relu_if16of16(const ai_f16* src, ai_f16* dst,
                   const ai_size in_size);

/**
 * @brief Applies the hard-sigmoid non-linearity to a float16 array.
 * @param src Input float16 array.
 * @param dst Output float16 array.
 * @param in_size Number of elements to process.
 * @param alpha Pointer to the hard-sigmoid slope parameter.
 * @param beta Pointer to the hard-sigmoid offset parameter.
 */
LITE_API_ENTRY
void forward_lite_nl_hard_sigmoid_if16of16(const ai_f16* src, ai_f16* dst,
                      const ai_size in_size,
                      const ai_f16* alpha, const ai_f16* beta);

/**
 * @brief Applies the sigmoid non-linearity to a float16 array.
 * @param src Input float16 array.
 * @param dst Output float16 array.
 * @param in_size Number of elements to process.
 */
LITE_API_ENTRY
void forward_lite_nl_sigmoid_if16of16(const ai_f16* src, ai_f16* dst,
                    const ai_size in_size);

/**
 * @brief Applies the hyperbolic tangent non-linearity to a float16 array.
 * @param src Input float16 array.
 * @param dst Output float16 array.
 * @param in_size Number of elements to process.
 */
LITE_API_ENTRY
void forward_lite_nl_tanh_if16of16(const ai_f16* src, ai_f16* dst,
                   const ai_size in_size);

/**
 * @brief Clips the values of a float16 array to a specified range.
 * @param src Input float16 array.
 * @param dst Output float16 array.
 * @param in_size Number of elements to process.
 * @param min Pointer to the minimum output value.
 * @param max Pointer to the maximum output value.
 */
LITE_API_ENTRY
void forward_lite_nl_clip_if16of16(const ai_f16* src, ai_f16* dst,
                   const ai_size in_size, const ai_f16* min,
                   const ai_f16* max);

/**
 * @brief Applies the generic ReLU operation to a float16 array.
 * @param src Input float16 array.
 * @param dst Output float16 array.
 * @param in_size Number of elements to process.
 * @param params ReLU parameters: threshold, alpha, and maximum value.
 */
LITE_API_ENTRY
void forward_lite_nl_relu_generic_if16of16_kernel(
  const ai_f16* src, ai_f16* dst, const ai_i32 in_size, const ai_handle params);

#endif /* AI_FLOAT16_TYPE_SUPPORTED */

/* lite_nl_generic_float */
#define LITE_NL_ENTRY(nl_id_, nl_name_, nl_op_, nl_op_args_) \
/** \
 * @brief lite function for a templated non-linearity nl_op_. \
 * @ingroup lite_nl_generic_float \
 * @param out_ptr The pointer to output buffer. \
 * @param in_ptr The pointer to input buffer. \
 * @param in_size. The size of the input. \
 * @param params opaque handler to optional NL params (not used). \
 */ \
LITE_API_ENTRY \
void forward_lite_nl_ ## nl_name_ ## _if32of32( \
  ai_handle out_ptr, const ai_handle in_ptr, const ai_i32 in_size, const ai_handle params);

#include "lite_nl_list.h"

/**
 * @brief lite function for a float softmax non-linearity where the softmax is applied per channel.
 * @ingroup lite_nl_generic_float
 * @param output The pointer to output buffer.
 * @param input The pointer to input buffer.
 * @param in_size The size of the input.
 * @param inner_loop_cnt The size of the inner loop (elements after the selected axis)
 * @param axis_elem The elements number along the selected axis.
 */
LITE_API_ENTRY
void forward_lite_nl_softmax_if32of32(
  ai_handle out_ptr, const ai_handle in_ptr,
  const ai_size in_size, const ai_size inner_loop_cnt, const ai_size axis_elem);


/**
 * @brief lite function for a float softmax zero channel non-linearity where the softmax is applied per channel.
 * @ingroup lite_nl_generic_float
 * @param output The pointer to output buffer.
 * @param input The pointer to input buffer.
 * @param in_size. The size of the input.
 * @param channel_size The nsize of each channel.
 * @param in_channel_step
 * @param out_channel_step
 */
LITE_API_ENTRY
void forward_lite_nl_softmax_zero_channel_if32of32(
  ai_handle out_ptr, const ai_handle in_ptr, const ai_i32 in_size, const ai_size ch_size,
  const ai_i32 in_ch_step, const ai_i32 out_ch_step);

/*!
 * @typedef (*func_nl_lite)
 * @ingroup layers_nl
 * @brief Fuction pointer for generic non linear transform
 * this function pointer abstracts a generic non linear layer.
 * see @ref nl_func_tanh_array_f32 and similar as examples.
 */
  typedef void (*func_nl_lite)(ai_handle out_ptr, const ai_handle in_ptr,
                               const ai_i32 in_size, const ai_handle params);

/**
 * @brief lite function for a float gelu non-linearity.
 * @ingroup lite_nl_generic_float \
 * @param out_ptr The pointer to output buffer. \
 * @param in_ptr The pointer to input buffer. \
 * @param in_size. The size of the input. \
 * @param params opaque handler to optional NL params. \
 */
LITE_API_ENTRY
void forward_lite_nl_gelu_if32of32(
  ai_handle out_ptr, const ai_handle in_ptr, const ai_i32 in_size, const ai_handle params);

#endif /* LITE_INTERNAL_APIS */
